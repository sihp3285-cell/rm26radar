/**
 * @file pose_node.cpp
 * @brief 将二维检测投影到场地世界坐标，并形成稳定 official slot 的核心 component。
 *
 * /armor_detections 先通过相机内外参与 Open3D PLY Raycaster 得到 (world_x,world_z)
 * 和投影协方差，再送入 Tracker。Tracker 以像素/世界 Kalman、Hungarian 关联、
 * BotIdentity 与 SlotOwner 分离维护“物理轨迹”和“兵种槽位”，最终发布固定槽位
 * /world_targets。Outpost 与死亡装甲板有独立直通语义，不与普通车辆共用生命周期。
 * /pose_node/reload_calibration 可在标定节点保存新外参后更新投影，不重启主链。
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>

#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "posesolver.hpp"
#include "robot_id.hpp"
#include "tracker.hpp"
#include "tracker_message.hpp"
#include "tensorrt_detect/debug/rviz_visualizer.hpp"
#include <cuda_runtime_api.h>

enum class ProjectionMode {
    CAR,
    ARMOR
};

struct ProjectionSelectorConfig {
    bool enabled = true;
    float uncertainty_bad_std_m = 0.50f;
    float condition_bad = 30.0f;
    float uncertainty_weight = 1.2f;
    float condition_weight = 0.4f;
    float surface_weight = 1.5f;
    float region_weight = 0.5f;
    float region_outer_m = -1.5f;
    float region_inner_m = 0.0f;
    float switch_margin = 0.30f;
    int switch_confirm_frames = 3;
};

struct ProjectionSelectorState {
    ProjectionMode mode = ProjectionMode::CAR;
    ProjectionMode pending_mode = ProjectionMode::CAR;
    int pending_confirm_count = 0;
};

class PoseNode : public rclcpp::Node
{
public:
    /** 初始化 CUDA/Open3D、投影器和 Tracker，加载外参/沟区/mesh，并创建 ROS 接口。 */
    explicit PoseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("pose_node", options)
    {
        // 提前初始化 CUDA primary context，避免与 DetectNode/TensorRT 并发初始化导致 SIGSEGV
        cudaFree(0);

        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("input_topic", "/armor_detections");
        this->declare_parameter<std::string>("output_topic", "/world_targets");
        this->declare_parameter<double>("projection_pixel_sigma_px", 4.0);
        this->declare_parameter<double>("projection_finite_difference_px", 2.0);
        this->declare_parameter<double>("projection_min_world_std_m", 0.03);
        this->declare_parameter<double>("projection_max_world_std_m", 1.50);
        this->declare_parameter<double>(
            "projection_surface_discontinuity_m", 0.12);
        this->declare_parameter<bool>("projection_selector_enabled", true);
        this->declare_parameter<double>(
            "projection_uncertainty_bad_std_m", 0.50);
        this->declare_parameter<double>("projection_condition_bad", 30.0);
        this->declare_parameter<double>(
            "projection_cost_uncertainty_weight", 1.2);
        this->declare_parameter<double>(
            "projection_cost_condition_weight", 0.4);
        this->declare_parameter<double>(
            "projection_cost_surface_weight", 1.5);
        this->declare_parameter<double>(
            "projection_cost_region_weight", 0.5);
        this->declare_parameter<double>("projection_region_outer_m", -1.5);
        this->declare_parameter<double>("projection_region_inner_m", 0.0);
        this->declare_parameter<double>("projection_switch_margin", 0.30);
        this->declare_parameter<int>("projection_switch_confirm_frames", 3);
        this->declare_parameter<std::string>(
            "gully_region_path",
            "/home/delphine/rm/tensorrt10_detect/generated/gully.yaml");
        this->declare_parameter<bool>("gully_field_x_flip", false);
        // RViz 是纯旁路；总开关关闭时不创建任何 Marker/TF publisher，也不复制帧数据。
        this->declare_parameter<bool>("rviz_debug_enabled", false);
        this->declare_parameter<bool>("rviz_debug_camera", true);
        this->declare_parameter<bool>("rviz_debug_fov", true);
        this->declare_parameter<bool>("rviz_debug_rays", true);
        this->declare_parameter<bool>("rviz_debug_ray_hits", true);
        this->declare_parameter<bool>("rviz_debug_measurements", true);
        this->declare_parameter<std::string>("rviz_debug_world_frame", "world");
        this->declare_parameter<std::string>("rviz_debug_camera_frame", "camera_link");
        this->declare_parameter<std::string>(
            "rviz_debug_static_topic", "/radar/rviz/static");
        this->declare_parameter<std::string>(
            "rviz_debug_pose_topic", "/radar/rviz/pose");
        this->declare_parameter<int>("rviz_debug_image_width", 5472);
        this->declare_parameter<int>("rviz_debug_image_height", 3648);

        config_dir_ = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        projection_config_.pixel_sigma_px = static_cast<float>(std::max(
            0.1, this->get_parameter("projection_pixel_sigma_px").as_double()));
        projection_config_.finite_difference_px = static_cast<float>(std::max(
            0.5,
            this->get_parameter("projection_finite_difference_px").as_double()));
        projection_config_.minimum_world_std_m = static_cast<float>(std::max(
            0.001,
            this->get_parameter("projection_min_world_std_m").as_double()));
        projection_config_.maximum_world_std_m = static_cast<float>(std::max(
            static_cast<double>(projection_config_.minimum_world_std_m),
            this->get_parameter("projection_max_world_std_m").as_double()));
        projection_config_.surface_discontinuity_m =
            static_cast<float>(std::max(
                0.0,
                this->get_parameter(
                    "projection_surface_discontinuity_m").as_double()));
        selector_config_.enabled =
            this->get_parameter("projection_selector_enabled").as_bool();
        selector_config_.uncertainty_bad_std_m = static_cast<float>(std::max(
            0.001,
            this->get_parameter(
                "projection_uncertainty_bad_std_m").as_double()));
        selector_config_.condition_bad = static_cast<float>(std::max(
            1.0,
            this->get_parameter("projection_condition_bad").as_double()));
        selector_config_.uncertainty_weight = static_cast<float>(std::max(
            0.0,
            this->get_parameter(
                "projection_cost_uncertainty_weight").as_double()));
        selector_config_.condition_weight = static_cast<float>(std::max(
            0.0,
            this->get_parameter(
                "projection_cost_condition_weight").as_double()));
        selector_config_.surface_weight = static_cast<float>(std::max(
            0.0,
            this->get_parameter(
                "projection_cost_surface_weight").as_double()));
        selector_config_.region_weight = static_cast<float>(std::max(
            0.0,
            this->get_parameter(
                "projection_cost_region_weight").as_double()));
        selector_config_.region_outer_m = static_cast<float>(
            this->get_parameter("projection_region_outer_m").as_double());
        selector_config_.region_inner_m = static_cast<float>(
            this->get_parameter("projection_region_inner_m").as_double());
        if (!(selector_config_.region_inner_m >
              selector_config_.region_outer_m)) {
            RCLCPP_WARN(this->get_logger(),
                "projection_region_inner_m 必须大于 outer_m，使用安全值 [-1.5, 0.0] m");
            selector_config_.region_outer_m = -1.5f;
            selector_config_.region_inner_m = 0.0f;
        }
        selector_config_.switch_margin = static_cast<float>(std::max(
            0.0,
            this->get_parameter("projection_switch_margin").as_double()));
        selector_config_.switch_confirm_frames = std::max(
            1,
            static_cast<int>(this->get_parameter(
                "projection_switch_confirm_frames").as_int()));
        gully_region_path_ =
            this->get_parameter("gully_region_path").as_string();
        gully_field_x_flip_ =
            this->get_parameter("gully_field_x_flip").as_bool();
        loadGullyRegions();

        RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "反投影协方差: pixel_sigma=%.2f px diff=%.2f px "
            "world_std=[%.3f, %.3f] m surface_jump=%.3f m",
            projection_config_.pixel_sigma_px,
            projection_config_.finite_difference_px,
            projection_config_.minimum_world_std_m,
            projection_config_.maximum_world_std_m,
            projection_config_.surface_discontinuity_m);
        cfg_ = std::make_unique<Config>(config_dir_);
        pose_solver_ = std::make_unique<PoseSolver>(cfg_->camera.cameraMatrix, cfg_->camera.distCoeffs);

        TrackerParams tp;
        // Track 生命周期
        tp.max_lost_time_s = cfg_->tracker.maxLostTimeS;
        tp.max_predict_time_s = cfg_->tracker.maxPredictTimeS;
        tp.dead_retention_time_s = cfg_->tracker.deadRetentionTimeS;
        tp.min_hit = cfg_->tracker.minHit;
        tp.max_tracks = cfg_->tracker.maxTracks;
        // 物理匹配 gate
        tp.max_gate_box = cfg_->tracker.maxGateBox;
        tp.max_gate_world = cfg_->tracker.maxGateWorld;
        tp.kalman_gate_box = cfg_->tracker.kalmanGateBox;
        tp.kalman_gate_world = cfg_->tracker.kalmanGateWorld;
        tp.negative_gate_box = cfg_->tracker.negativeGateBox;
        tp.negative_gate_world = cfg_->tracker.negativeGateWorld;
        // Hungarian 匹配代价
        tp.w_box = cfg_->tracker.wBox;
        tp.w_world = cfg_->tracker.wWorld;
        tp.class_mismatch_min_penalty = cfg_->tracker.classMismatchMinPenalty;
        tp.class_mismatch_penalty = cfg_->tracker.classMismatchPenalty;
        // BotIdentity 身份稳定器
        tp.botIdentity = cfg_->tracker.botIdentity;
        // 身份更新阈值
        tp.min_identity_update_conf = cfg_->tracker.minIdentityUpdateConf;
        tp.identity_confirm_observations = cfg_->tracker.identityConfirmObservations;
        tp.identity_switch_confirm_observations = cfg_->tracker.identitySwitchConfirmObservations;
        // Official slot owner 机制
        tp.slot_bind_min_conf = cfg_->tracker.slotBindMinConf;
        tp.slot_lease_time_s = cfg_->tracker.slotLeaseTimeS;
        tp.slot_min_stability = cfg_->tracker.slotMinStability;
        tp.slot_max_switch_rate = cfg_->tracker.slotMaxSwitchRate;
        tp.max_slot_jump_dist = cfg_->tracker.maxSlotJumpDist;

        tracker_ = Tracker(tp);
        dead_target_hold_time_s_ = std::max(0.0f, cfg_->tracker.deadTargetHoldTimeS);
        RCLCPP_INFO(this->get_logger(),
            "Tracker 参数: max_lost=%.3fs max_predict=%.3fs dead_retention=%.3fs min_hit=%d max_tracks=%d | "
            "gate: box=%.1f world=%.2f kalman_box=%.3f kalman_world=%.3f negative_box=%.1f negative_world=%.2f | cost: w_box=%.2f w_world=%.2f class_pen=[%.3f, %.3f] | "
            "identity: initial=%d switch=%d observations | slot: bind=%.2f lease=%.3fs stability=%.2f switch_rate=%.2f jump=%.2f",
            tp.max_lost_time_s, tp.max_predict_time_s, tp.dead_retention_time_s,
            tp.min_hit, tp.max_tracks,
            tp.max_gate_box, tp.max_gate_world, tp.kalman_gate_box, tp.kalman_gate_world,
            tp.negative_gate_box, tp.negative_gate_world,
            tp.w_box, tp.w_world, tp.class_mismatch_min_penalty,
            tp.class_mismatch_penalty,
            tp.identity_confirm_observations, tp.identity_switch_confirm_observations,
            tp.slot_bind_min_conf, tp.slot_lease_time_s, tp.slot_min_stability,
            tp.slot_max_switch_rate, tp.max_slot_jump_dist);
        RCLCPP_INFO(this->get_logger(),
            "BotIdentity 参数: max_history=%d purge_after_lost=%.3fs min_stable=%d decay=%.3f num_classes=%d",
            tp.botIdentity.maxHistory, tp.botIdentity.purgeAfterLostTimeS,
            tp.botIdentity.minHistoryForStable, tp.botIdentity.decay, tp.botIdentity.numClasses);

        if (this->get_parameter("rviz_debug_enabled").as_bool()) {
            tensorrt_detect::debug::RvizVisualizerOptions rviz_options;
            rviz_options.pose_layer = true;
            rviz_options.camera =
                this->get_parameter("rviz_debug_camera").as_bool();
            rviz_options.fov = this->get_parameter("rviz_debug_fov").as_bool();
            rviz_options.rays = this->get_parameter("rviz_debug_rays").as_bool();
            rviz_options.ray_hits =
                this->get_parameter("rviz_debug_ray_hits").as_bool();
            rviz_options.measurements =
                this->get_parameter("rviz_debug_measurements").as_bool();
            rviz_options.world_frame =
                this->get_parameter("rviz_debug_world_frame").as_string();
            rviz_options.camera_frame =
                this->get_parameter("rviz_debug_camera_frame").as_string();
            rviz_options.static_topic =
                this->get_parameter("rviz_debug_static_topic").as_string();
            rviz_options.pose_topic =
                this->get_parameter("rviz_debug_pose_topic").as_string();
            rviz_image_width_ = std::max(
                1, static_cast<int>(this->get_parameter(
                    "rviz_debug_image_width").as_int()));
            rviz_image_height_ = std::max(
                1, static_cast<int>(this->get_parameter(
                    "rviz_debug_image_height").as_int()));
            rviz_visualizer_ =
                std::make_unique<tensorrt_detect::debug::RvizVisualizer>(
                    *this, std::move(rviz_options));
        }

        loadCalibrationAtStartup();
        updateRvizCameraCalibration();

        if (!cfg_->camera.meshPath.empty()) {
            bool mesh_ok = pose_solver_->getRaycaster().loadingMesh(cfg_->camera.meshPath);
            if (mesh_ok) {
                RCLCPP_INFO(this->get_logger(), "成功加载 3D 网格: %s", cfg_->camera.meshPath.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "加载 3D 网格失败: %s，将使用平地 fallback", cfg_->camera.meshPath.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "未配置 meshPath，将使用平地 fallback");
        }

        // 检测/世界目标都是高频实时状态：depth=10 吸收短暂 executor 抖动，
        // BestEffort 允许过载时丢旧帧，避免 Tracker 对积压历史帧产生额外延迟。
        world_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::WorldTargetArray>(output_topic_, rclcpp::QoS(10).best_effort());

        armor_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::DetectionArray>(
            input_topic_, rclcpp::QoS(10).best_effort(),
            std::bind(&PoseNode::armor_callback, this, std::placeholders::_1));

        reload_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/pose_node/reload_calibration",
            std::bind(&PoseNode::reloadCalibration, this,
                      std::placeholders::_1, std::placeholders::_2));

        if (is_calibrated_) {
            RCLCPP_INFO(this->get_logger(), "PoseNode 初始化完成，标定已就绪");
        } else {
            RCLCPP_WARN(this->get_logger(), "PoseNode 初始化完成，等待标定...");
        }
    }

private:
    /** 解析沟区多边形 YAML；失败时清空区域并让 soft prior 保守偏向车辆框。 */
    void loadGullyRegions()
    {
        gully_polygons_.clear();
        try {
            const YAML::Node root = YAML::LoadFile(gully_region_path_);
            const YAML::Node zones = root["blind_zones"];
            if (!zones || !zones.IsSequence()) {
                throw std::runtime_error("缺少 blind_zones 序列");
            }

            for (const auto& zone : zones) {
                const YAML::Node polygon_node = zone["polygon"];
                if (!polygon_node || !polygon_node.IsSequence() ||
                    polygon_node.size() < 3) {
                    continue;
                }

                std::vector<cv::Point2f> polygon;
                polygon.reserve(polygon_node.size());
                for (const auto& vertex : polygon_node) {
                    if (!vertex.IsSequence() || vertex.size() != 2) {
                        throw std::runtime_error("polygon 顶点格式无效");
                    }
                    polygon.emplace_back(
                        vertex[0].as<float>(), vertex[1].as<float>());
                }
                gully_polygons_.push_back(polygon);

                if (zone["mirror_centrally"] &&
                    zone["mirror_centrally"].as<bool>()) {
                    std::vector<cv::Point2f> mirrored;
                    mirrored.reserve(polygon.size());
                    for (const auto& point : polygon) {
                        mirrored.emplace_back(
                            28.0f - point.x, 15.0f - point.y);
                    }
                    gully_polygons_.push_back(std::move(mirrored));
                }
            }
            if (gully_polygons_.empty()) {
                throw std::runtime_error("没有有效 polygon");
            }
        } catch (const std::exception& e) {
            gully_polygons_.clear();
            RCLCPP_WARN(this->get_logger(),
                "加载沟区范围失败: %s (%s)，区域先验将保守偏向车辆框",
                gully_region_path_.c_str(), e.what());
        }
    }

    /** 返回 car 落点到最近沟区边界的有符号距离；沟内为正、沟外为负。 */
    double signedGullyDistance(const cv::Point2f& world) const
    {
        if (gully_polygons_.empty() ||
            !std::isfinite(world.x) || !std::isfinite(world.y)) {
            return -std::numeric_limits<double>::infinity();
        }

        // Raycaster 返回 world=(x,z)。这里按与地图/先验一致的 28m×15m field 约定
        // 转换后才与 canonical 盲区 polygon 比较；gully_field_x_flip 适配场地方向。
        const cv::Point2f field(
            gully_field_x_flip_ ? world.y + 14.0f : 14.0f - world.y,
            world.x + 7.5f);
        double signed_distance = -std::numeric_limits<double>::infinity();
        for (const auto& polygon : gully_polygons_) {
            signed_distance = std::max(
                signed_distance,
                cv::pointPolygonTest(polygon, field, true));
        }
        return signed_distance;
    }

    /**
     * 区域只提供 soft prior，不再参与世界坐标插值。outer/inner 分别对应
     * armor prior 约 0.1/0.9；polygon 不可用时保守偏向车辆框，但其他质量项
     * 仍可推翻该先验。
     */
    float armorRegionPrior(const cv::Point2f& car_world) const
    {
        const double signed_distance = signedGullyDistance(car_world);
        if (!std::isfinite(signed_distance)) {
            return 0.1f;
        }

        const double midpoint = 0.5 * (
            selector_config_.region_inner_m +
            selector_config_.region_outer_m);
        const double slope = 2.0 * std::log(9.0) /
            (selector_config_.region_inner_m -
             selector_config_.region_outer_m);
        const double exponent = std::clamp(
            -slope * (signed_distance - midpoint), -60.0, 60.0);
        return static_cast<float>(1.0 / (1.0 + std::exp(exponent)));
    }

    static bool finiteWorld(const WorldProjection& projection)
    {
        return std::isfinite(projection.world.x) &&
            std::isfinite(projection.world.y);
    }

    /**
     * covariance 的最大特征值平方根代表最不可靠世界方向上的标准差。
     * 非有限、非正半定或显式 invalid 的 covariance 不会得到低 cost。
     */
    static bool maximumCovarianceStd(
        const WorldProjection& projection,
        float& maximum_std)
    {
        if (!projection.covariance_valid ||
            !std::all_of(
                projection.covariance.begin(),
                projection.covariance.end(),
                [](float value) { return std::isfinite(value); })) {
            return false;
        }

        const float xx = projection.covariance[0];
        const float xz = 0.5f * (
            projection.covariance[1] + projection.covariance[2]);
        const float zz = projection.covariance[3];
        if (xx < 0.0f || zz < 0.0f ||
            xx * zz - xz * xz < -1e-6f) {
            return false;
        }
        const float discriminant = std::sqrt(std::max(
            0.0f, (xx - zz) * (xx - zz) + 4.0f * xz * xz));
        const float maximum_eigenvalue =
            0.5f * (xx + zz + discriminant);
        if (!std::isfinite(maximum_eigenvalue) ||
            maximum_eigenvalue < 0.0f) {
            return false;
        }
        maximum_std = std::sqrt(maximum_eigenvalue);
        return std::isfinite(maximum_std);
    }

    float projectionCost(
        const WorldProjection& projection,
        float region_cost) const
    {
        float uncertainty_cost = 2.0f;
        float maximum_std = 0.0f;
        if (maximumCovarianceStd(projection, maximum_std)) {
            uncertainty_cost = std::clamp(
                maximum_std / selector_config_.uncertainty_bad_std_m,
                0.0f, 2.0f);
        }

        float condition_cost = 2.0f;
        const float condition = projection.jacobian_condition_number;
        if (std::isfinite(condition) && condition >= 1.0f) {
            condition_cost = std::clamp(
                std::log1p(condition) /
                    std::log1p(selector_config_.condition_bad),
                0.0f, 2.0f);
        }
        const float surface_cost =
            projection.surface_discontinuity ? 1.0f : 0.0f;
        // Cost 越小，表示该 measurement hypothesis 的几何质量与区域先验越可信。
        // 这些初始权重不是理论最优值，仍需用真实场地数据标定。
        return selector_config_.uncertainty_weight * uncertainty_cost +
            selector_config_.condition_weight * condition_cost +
            selector_config_.surface_weight * surface_cost +
            selector_config_.region_weight * region_cost;
    }

    static bool projectionHealthy(const WorldProjection& projection)
    {
        float ignored_std = 0.0f;
        return finiteWorld(projection) &&
            maximumCovarianceStd(projection, ignored_std);
    }

    /**
     * Tracker 前没有 track_id。V1 以 team + 当前 official class 绑定少量状态；
     * 它不会假装完成物理轨迹关联，临时误分类只会切到另一份缓存状态。
     */
    static std::uint64_t projectionStateKey(
        const tensorrt_detect_msgs::msg::DetectionBox& detection)
    {
        return (static_cast<std::uint64_t>(
                    static_cast<std::uint32_t>(detection.armor_color)) << 32U) |
            static_cast<std::uint32_t>(detection.idx);
    }

    /**
     * car/armor 被视为两个完整 measurement hypotheses，而非可插值端点。
     * switch margin 防止微小 cost 波动触发切换，连续帧确认抑制边界来回翻转；
     * 当前假设的 world/covariance 明显失效、另一假设健康时则立即切换。
     * V1 刻意不使用 Kalman prediction/NIS，避免改变 Tracker 生命周期。
     */
    WorldProjection selectProjection(
        const tensorrt_detect_msgs::msg::DetectionBox& detection,
        const WorldProjection& car,
        const WorldProjection& armor)
    {
        constexpr float prior_epsilon = 1e-6f;
        const float armor_prior = std::clamp(
            armorRegionPrior(car.world), prior_epsilon, 1.0f - prior_epsilon);
        const float car_prior = 1.0f - armor_prior;
        const float car_cost = projectionCost(
            car, -std::log(car_prior + prior_epsilon));
        const float armor_cost = projectionCost(
            armor, -std::log(armor_prior + prior_epsilon));

        auto& state = selector_states_[projectionStateKey(detection)];

        if (!selector_config_.enabled) {
            // Disabled 是无插值的安全回退：优先 car；仅 car 质量失效而 armor
            // 健康时改用 armor，避免关闭实验功能后发布明显无效的测量。
            state.mode = !projectionHealthy(car) && projectionHealthy(armor)
                ? ProjectionMode::ARMOR : ProjectionMode::CAR;
            state.pending_mode = state.mode;
            state.pending_confirm_count = 0;
            return state.mode == ProjectionMode::CAR ? car : armor;
        }

        const bool current_is_car = state.mode == ProjectionMode::CAR;
        const auto& current = current_is_car ? car : armor;
        const auto& alternative = current_is_car ? armor : car;
        const float current_cost = current_is_car ? car_cost : armor_cost;
        const float alternative_cost = current_is_car ? armor_cost : car_cost;
        const ProjectionMode alternative_mode = current_is_car
            ? ProjectionMode::ARMOR : ProjectionMode::CAR;

        // 仅 world/covariance 失效触发 hard override；surface discontinuity 仍是软惩罚。
        if (!projectionHealthy(current) && projectionHealthy(alternative)) {
            state.mode = alternative_mode;
            state.pending_mode = state.mode;
            state.pending_confirm_count = 0;
        } else if (alternative_cost + selector_config_.switch_margin <
                   current_cost) {
            if (state.pending_mode != alternative_mode) {
                state.pending_mode = alternative_mode;
                state.pending_confirm_count = 1;
            } else {
                ++state.pending_confirm_count;
            }
            if (state.pending_confirm_count >=
                selector_config_.switch_confirm_frames) {
                state.mode = alternative_mode;
                state.pending_mode = state.mode;
                state.pending_confirm_count = 0;
            }
        } else {
            state.pending_mode = state.mode;
            state.pending_confirm_count = 0;
        }

        // 位置、covariance 与质量信息必须来自同一个 hypothesis，禁止交叉拼接。
        return state.mode == ProjectionMode::CAR ? car : armor;
    }

    /** 优先使用 Config 已解析外参，否则直接读取 calib_result.yaml 并更新可标定状态。 */
    void loadCalibrationAtStartup()
    {
        if (cfg_->calib.valid) {
            pose_solver_->setExtrinsic(cfg_->calib.R, cfg_->calib.T);
            is_calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "成功从 Config 加载校准结果，已设置外参");
            return;
        }

        std::filesystem::path configDir = std::filesystem::path(config_dir_);
        std::string calibPath = (configDir / "calib_result.yaml").string();
        if (!std::filesystem::exists(calibPath)) {
            RCLCPP_WARN(this->get_logger(), "未找到校准文件: %s", calibPath.c_str());
            return;
        }

        try {
            YAML::Node node = YAML::LoadFile(calibPath);
            if (!node["r"].IsSequence() || !node["t"].IsSequence()) {
                RCLCPP_WARN(this->get_logger(), "校准文件格式错误，缺少 r 或 t 数据");
                return;
            }

            std::vector<double> r_data = node["r"].as<std::vector<double>>();
            std::vector<double> t_data = node["t"].as<std::vector<double>>();
            if (r_data.size() != 9 || t_data.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "校准文件数据维度不匹配");
                return;
            }

            cv::Mat R(3, 3, CV_64F);
            cv::Mat T(3, 1, CV_64F);
            for (int i = 0; i < 9; ++i) {
                R.at<double>(i / 3, i % 3) = r_data[i];
            }
            for (int i = 0; i < 3; ++i) {
                T.at<double>(i, 0) = t_data[i];
            }

            pose_solver_->setExtrinsic(R, T);
            is_calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "成功从 %s 加载校准结果，已设置外参", calibPath.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "加载校准文件失败: %s", e.what());
        }
    }

    /** Trigger 回调：重新解析 R/T 并原子式替换 PoseSolver 外参，不重载 PLY。 */
    void reloadCalibration(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                           std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "收到重载校准请求...");

        std::filesystem::path configDir = std::filesystem::path(config_dir_);
        std::string calibPath = (configDir / "calib_result.yaml").string();

        if (!std::filesystem::exists(calibPath)) {
            response->success = false;
            response->message = "Calibration file not found: " + calibPath;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        try {
            YAML::Node node = YAML::LoadFile(calibPath);
            if (!node["r"].IsSequence() || !node["t"].IsSequence()) {
                response->success = false;
                response->message = "Invalid calibration file format";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            std::vector<double> r_data = node["r"].as<std::vector<double>>();
            std::vector<double> t_data = node["t"].as<std::vector<double>>();
            if (r_data.size() != 9 || t_data.size() != 3) {
                response->success = false;
                response->message = "Calibration data dimension mismatch";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            cv::Mat R(3, 3, CV_64F);
            cv::Mat T(3, 1, CV_64F);
            for (int i = 0; i < 9; ++i) {
                R.at<double>(i / 3, i % 3) = r_data[i];
            }
            for (int i = 0; i < 3; ++i) {
                T.at<double>(i, 0) = t_data[i];
            }

            pose_solver_->setExtrinsic(R, T);
            is_calibrated_ = true;
            updateRvizCameraCalibration();
            response->success = true;
            response->message = "Calibration reloaded successfully";
            RCLCPP_INFO(this->get_logger(), "pose_node 已重载校准结果，标定就绪");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to reload: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "重载校准失败: %s", e.what());
        }
    }

    /** 把 PoseSolver 当前同一份 R/T/K 暴露为 debug TF/Marker，不求解第二套外参。 */
    void updateRvizCameraCalibration()
    {
        if (!rviz_visualizer_ || !is_calibrated_) return;
        cv::Mat rotation;
        cv::Mat translation;
        pose_solver_->getExtrinsic(rotation, translation);
        rviz_visualizer_->setCameraCalibration(
            rotation, translation, cfg_->camera.cameraMatrix,
            rviz_image_width_, rviz_image_height_);
    }

    /** 检测回调：批量射线投影、构造测量/负观测、更新 Tracker 并发布 WorldTargetArray。 */
    void armor_callback(const tensorrt_detect_msgs::msg::DetectionArray::ConstSharedPtr msg)
    {
        if (!is_calibrated_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "标定未就绪，跳过世界坐标计算。请先完成标定。");
            return;
        }

        try {
            // 使用上游图像/检测消息的采集时间，而不是回调到达时间。
            // 零时间戳回退 ROS clock；时间倒退重置；长间隔只对 Kalman dt 限幅。
            float tracker_dt = -1.0f;
            int64_t stamp_ns = rclcpp::Time(
                msg->header.stamp, this->get_clock()->get_clock_type()).nanoseconds();
            if (stamp_ns <= 0) {
                stamp_ns = this->get_clock()->now().nanoseconds();
            }
            if (last_detection_stamp_ns_ > 0) {
                const double dt_seconds =
                    static_cast<double>(stamp_ns - last_detection_stamp_ns_) * 1e-9;
                if (!std::isfinite(dt_seconds) || dt_seconds < 0.0) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "检测时间倒退（dt=%.6f s），重置 Tracker 时间状态",
                        dt_seconds);
                    tracker_.reset();
                    selector_states_.clear();
                    cached_dead_targets_.clear();
                    last_dead_target_observed_ns_ = 0;
                } else {
                    constexpr double max_filter_dt_s = 1.0;
                    tracker_dt = static_cast<float>(std::min(dt_seconds, max_filter_dt_s));
                    if (dt_seconds > max_filter_dt_s) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), 5000,
                            "检测间隔 %.3f s，生命周期使用真实间隔，Kalman dt 限幅为 %.1f s",
                            dt_seconds, max_filter_dt_s);
                    }
                }
            }
            last_detection_stamp_ns_ = stamp_ns;

            // ---- 0. 批量预计算所有检测的世界坐标 ----
            const std::size_t detection_count = msg->detections.size();
            std::vector<cv::Rect> boxes_for_raycast;
            boxes_for_raycast.reserve(detection_count * 2);

            // car/armor 框底边是两个独立 measurement hypotheses。仍用一次 batch
            // CastRays 复用既有 5-ray/Jacobian/covariance 计算，不增加 Raycast。
            for (const auto& det : msg->detections) {
                const cv::Rect car_box(
                    det.car_x, det.car_y, det.car_width, det.car_height);
                cv::Rect armor_box(det.x, det.y, det.width, det.height);
                boxes_for_raycast.push_back(
                    car_box.width > 0 && car_box.height > 0
                        ? car_box : armor_box);
            }
            for (const auto& det : msg->detections) {
                const cv::Rect car_box(
                    det.car_x, det.car_y, det.car_width, det.car_height);
                const cv::Rect armor_box(
                    det.x, det.y, det.width, det.height);
                boxes_for_raycast.push_back(
                    armor_box.width > 0 && armor_box.height > 0
                        ? armor_box : car_box);
            }

            // 预先按 detection_count 建立默认 invalid projection，确保 raycast 返回数量
            // 异常、bbox 缺失或 detection 为空时都不会越界或 crash。
            std::vector<WorldProjection> world_projections(detection_count);
            if (!boxes_for_raycast.empty()) {
                auto frame_projection_config = projection_config_;
                frame_projection_config.capture_debug_ray_endpoint =
                    rviz_visualizer_ && rviz_visualizer_->wantsPoseFrame();
                const auto candidate_projections =
                    pose_solver_->middletoworldBatchWithUncertainty(
                        boxes_for_raycast, frame_projection_config);
                if (candidate_projections.size() == detection_count * 2) {
                    for (std::size_t i = 0; i < detection_count; ++i) {
                        const auto& det = msg->detections[i];
                        const bool car_valid =
                            det.car_width > 0 && det.car_height > 0;
                        const bool armor_valid =
                            det.width > 0 && det.height > 0;
                        const auto& car_projection = candidate_projections[i];
                        const auto& armor_projection =
                            candidate_projections[detection_count + i];

                        if (!car_valid) {
                            world_projections[i] = armor_projection;
                            auto& state = selector_states_[projectionStateKey(det)];
                            state.mode = ProjectionMode::ARMOR;
                            state.pending_mode = state.mode;
                            state.pending_confirm_count = 0;
                        } else if (!armor_valid) {
                            world_projections[i] = car_projection;
                            auto& state = selector_states_[projectionStateKey(det)];
                            state.mode = ProjectionMode::CAR;
                            state.pending_mode = state.mode;
                            state.pending_confirm_count = 0;
                        } else {
                            world_projections[i] = selectProjection(
                                det, car_projection, armor_projection);
                        }
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "批量投影返回 %zu 项，期望 %zu 项；本帧使用 invalid fallback",
                        candidate_projections.size(), detection_count * 2);
                }
            }

            // ---- 1. 解算所有检测的世界坐标，构建观测输入 ----
            // Outpost 不走 Tracker，直接透传
            // 死亡装甲板（ARMOR + is_dead）作为 Tracker 负观测，同时动态追加死亡点
            // 正常装甲板（R1~S）进入固定槽位跟踪
            std::vector<WorldMeasurement> meas;
            meas.reserve(msg->detections.size());
            std::vector<tensorrt_detect_msgs::msg::WorldTarget> dead_targets;
            tensorrt_detect_msgs::msg::WorldTarget outpost_target;
            bool has_outpost = false;
            std::size_t surface_discontinuity_count = 0;
            float maximum_projection_condition = 1.0f;
            constexpr bool ENABLE_CLASS_MARGIN = true;

            for (size_t i = 0; i < msg->detections.size(); ++i) {
                const auto& det = msg->detections[i];
                const auto& projection = world_projections[i];
                const cv::Point2f world_pos = projection.world;
                if (projection.surface_discontinuity) {
                    ++surface_discontinuity_count;
                }
                if (std::isfinite(projection.jacobian_condition_number)) {
                    maximum_projection_condition = std::max(
                        maximum_projection_condition,
                        projection.jacobian_condition_number);
                }

                // Outpost 直接透传，不进入 Tracker
                if (det.idx == robot_id::OUTPOST) {
                    outpost_target.idx      = 10;
                    outpost_target.class_id = det.idx;
                    outpost_target.team_id  = det.armor_color;
                    outpost_target.is_dead  = det.is_dead;
                    outpost_target.score    = det.confidence;
                    outpost_target.valid    = true;
                    outpost_target.bbox_x   = det.x;
                    outpost_target.bbox_y   = det.y;
                    outpost_target.bbox_w   = det.width;
                    outpost_target.bbox_h   = det.height;
                    outpost_target.world_x  = world_pos.x;
                    outpost_target.world_y  = 0.0f;
                    outpost_target.world_z  = world_pos.y;
                    const bool directly_observed = det.width > 0 && det.height > 0;
                    tracker_message::mark_direct_measurement(
                        outpost_target, directly_observed);
                    if (directly_observed) {
                        outpost_target.last_observed_time =
                            static_cast<builtin_interfaces::msg::Time>(
                                rclcpp::Time(stamp_ns, this->get_clock()->get_clock_type()));
                    }
                    has_outpost = true;
                    continue;
                }

                // 死亡装甲板仍动态发布，同时作为负观测传入 Tracker。
                if (det.idx == robot_id::ARMOR && det.is_dead) {
                    tensorrt_detect_msgs::msg::WorldTarget t;
                    t.idx      = 11 + static_cast<int>(dead_targets.size());
                    t.class_id = robot_id::ARMOR;
                    t.team_id  = robot_id::UNKNOWN;
                    t.is_dead  = true;
                    t.score    = det.confidence;
                    t.valid    = true;
                    t.bbox_x   = det.x;
                    t.bbox_y   = det.y;
                    t.bbox_w   = det.width;
                    t.bbox_h   = det.height;
                    t.world_x  = world_pos.x;
                    t.world_y  = 0.0f;
                    t.world_z  = world_pos.y;
                    t.stable_class_id  = -1;
                    t.stable_class_conf = 0.0f;
                    tracker_message::mark_direct_measurement(t, true);
                    t.last_observed_time =
                        static_cast<builtin_interfaces::msg::Time>(
                            rclcpp::Time(stamp_ns, this->get_clock()->get_clock_type()));
                    dead_targets.push_back(t);

                    WorldMeasurement negative;
                    negative.class_id = robot_id::ARMOR;
                    negative.team_id = robot_id::UNKNOWN;
                    negative.score = det.confidence;
                    negative.class_conf = det.class_conf;
                    negative.class_margin = ENABLE_CLASS_MARGIN
                        ? det.class_margin
                        : 0.0f;
                    negative.is_dead = true;
                    negative.is_negative = true;
                    negative.box = cv::Rect(det.x, det.y, det.width, det.height);
                    negative.world = world_pos;
                    negative.world_covariance = projection.covariance;
                    negative.world_covariance_valid =
                        projection.covariance_valid;
                    meas.push_back(negative);
                    continue;
                }

                WorldMeasurement m;
                m.class_id = det.idx;
                m.team_id  = det.armor_color;
                m.score    = det.confidence;
                m.class_conf = det.class_conf;
                m.class_margin = ENABLE_CLASS_MARGIN
                    ? det.class_margin
                    : 0.0f;
                m.is_dead  = det.is_dead;
                m.box      = cv::Rect(det.x, det.y, det.width, det.height);
                m.world    = world_pos;  // x=world_x, y=world_z
                m.world_covariance = projection.covariance;
                m.world_covariance_valid = projection.covariance_valid;
                meas.push_back(m);
            }

            if (surface_discontinuity_count > 0) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "检测到 %zu 个 PLY 跨高度面反投影，已增大测量协方差；"
                    "本帧最大雅可比条件数 %.2f",
                    surface_discontinuity_count,
                    maximum_projection_condition);
            }

            // 仅当 RViz 真有订阅者时，才复制本帧已经选定的投影结果；不触发额外
            // RayCast，也不访问/修改 Tracker 内部状态。
            if (rviz_visualizer_ && rviz_visualizer_->wantsPoseFrame()) {
                std::vector<tensorrt_detect::debug::PoseDebugSample> debug_samples;
                debug_samples.reserve(msg->detections.size());
                for (std::size_t i = 0; i < msg->detections.size(); ++i) {
                    const auto& projection = world_projections[i];
                    if (!projection.ray_endpoint_valid) continue;
                    const auto& detection = msg->detections[i];
                    tensorrt_detect::debug::PoseDebugSample sample;
                    sample.marker_key = static_cast<int>(i);
                    sample.class_id = detection.idx;
                    sample.team_id = detection.armor_color;
                    sample.confidence = detection.confidence;
                    sample.is_dead = detection.is_dead;
                    sample.negative_measurement =
                        detection.idx == robot_id::ARMOR && detection.is_dead;
                    sample.tracker_measurement =
                        detection.idx != robot_id::OUTPOST &&
                        !sample.negative_measurement;
                    sample.ray_endpoint = projection.ray_endpoint;
                    sample.measurement = cv::Point3f(
                        projection.world.x, 0.0f, projection.world.y);
                    debug_samples.push_back(sample);
                }
                rviz_visualizer_->publishPoseFrame(msg->header, debug_samples);
            }

            // 死亡装甲板不进入 Tracker；短暂漏检时保留最近结果，避免地图单帧闪烁。
            if (!dead_targets.empty()) {
                cached_dead_targets_ = dead_targets;
                last_dead_target_observed_ns_ = stamp_ns;
            } else if (!cached_dead_targets_.empty() &&
                       last_dead_target_observed_ns_ > 0 &&
                       stamp_ns >= last_dead_target_observed_ns_ &&
                       static_cast<double>(stamp_ns - last_dead_target_observed_ns_) * 1e-9 <=
                           dead_target_hold_time_s_) {
                dead_targets = cached_dead_targets_;
            } else {
                cached_dead_targets_.clear();
                last_dead_target_observed_ns_ = 0;
            }

            // ---- 2. Tracker 更新（正常观测 + 死亡装甲板负观测；不含 Outpost）----
            tracker_.update(meas, tracker_dt, stamp_ns);

            // ---- 3. 固定槽位 + Outpost + 动态死亡装甲板 发布 ----
            auto world_msg = std::make_unique<tensorrt_detect_msgs::msg::WorldTargetArray>();
            world_msg->header = msg->header;
            // 0-9: Tracker official slots（含 track→slot 映射 + 仲裁）；10: Outpost 透传
            world_msg->targets.resize(11);

            // 批量获取 10 个 official slot 输出（已含 track→slot 映射 + 仲裁）
            auto slot_outputs = tracker_.get_outputs();

            int valid_count = 0;
            for (int i = 0; i < Tracker::NUM_SLOTS; ++i) {
                const auto& slot = slot_outputs[i];
                auto& target = world_msg->targets[i];
                tracker_message::fill_world_target(i, slot, target);
                if (slot.valid) valid_count++;
            }

            // Outpost 直接放到索引 10
            if (has_outpost) {
                world_msg->targets[10] = outpost_target;
                valid_count++;
            } else {
                auto& target = world_msg->targets[10];
                target.idx      = 10;
                target.class_id = robot_id::OUTPOST;
                target.team_id  = robot_id::UNKNOWN;
                target.valid    = false;
                tracker_message::mark_direct_measurement(target, false);
            }

            // 动态追加死亡装甲板
            for (const auto& dt : dead_targets) {
                world_msg->targets.push_back(dt);
            }

            world_pub_->publish(std::move(world_msg));

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "接收到 %zu 个检测，固定槽位有效 %d / %d，死亡装甲板 %zu",
                msg->detections.size(), valid_count, Tracker::NUM_SLOTS, dead_targets.size());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "姿态解算回调异常: %s", e.what());
        }
    }

    std::unique_ptr<Config> cfg_; // 保存相机/Tracker/mesh 配置，覆盖 solver 生命周期。
    std::unique_ptr<PoseSolver> pose_solver_; // 持有标定矩阵和只加载一次的 Raycaster scene。
    Tracker tracker_; // 跨帧保存 PhysicalTrack、Kalman、BotIdentity 与 SlotOwner。
    bool is_calibrated_ = false; // false 时拒绝投影，避免发布看似有效的错误 world 点。
    int64_t last_detection_stamp_ns_ = 0; // 计算 Kalman dt 与检测时间倒退保护。
    std::vector<tensorrt_detect_msgs::msg::WorldTarget> cached_dead_targets_;
    int64_t last_dead_target_observed_ns_ = 0;
    float dead_target_hold_time_s_ = 0.10f;
    ProjectionUncertaintyConfig projection_config_; // 像素误差传播/数值截断配置。
    ProjectionSelectorConfig selector_config_; // V1 hypothesis cost/hysteresis 参数。
    // Tracker 前仅有 team/class，故这里是有限 official identity 缓存而非 per-track 状态。
    std::unordered_map<std::uint64_t, ProjectionSelectorState> selector_states_;
    std::vector<std::vector<cv::Point2f>> gully_polygons_; // field/canonical 米制沟区边界。
    std::string gully_region_path_;
    bool gully_field_x_flip_ = false;

    std::string config_dir_;
    std::string input_topic_;
    std::string output_topic_;

    std::unique_ptr<tensorrt_detect::debug::RvizVisualizer> rviz_visualizer_;
    int rviz_image_width_ = 5472;
    int rviz_image_height_ = 3648;

    rclcpp::Subscription<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_sub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr world_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PoseNode)

/** 非 component 调试入口；正式 launch 使用同容器组件以缩短消息传递路径。 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
