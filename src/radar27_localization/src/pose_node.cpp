#include <radar27_interfaces/msg/world_measurement_array.hpp>
#include <radar27_interfaces/msg/projection_debug_array.hpp>
#include <radar27_interfaces/msg/calibration_state.hpp>
#include <chrono>
/**
 * @file pose_node.cpp
 * @brief 将二维检测投影为世界测量；跟踪由 radar27_tracking 独立负责。
 *
 * /armor_detections -> /world_measurements。原始采集时间保持不变，坐标系改为
 * world_frame。标定版本随测量传递，使下游在重载后清除旧坐标状态。
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

#include "radar27_interfaces/msg/detection_array.hpp"
#include "radar27_interfaces/msg/detection_box.hpp"
#include <radar27_localization/config.hpp>
#include <radar27_localization/posesolver.hpp>

#include <rm_field/field_geometry.hpp>
#include <rm_field/robot_id.hpp>
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
        //声明参数
        this->declare_parameter<std::string>("config_dir",
            ".");//配置文件目录
        this->declare_parameter<std::string>("input_topic", "/armor_detections");//输入话题
        this->declare_parameter<std::string>("output_topic", "/world_measurements");//输出话题
        //以下为投影评估误差的参数
        this->declare_parameter<double>("projection_pixel_sigma_px", 4.0);
        this->declare_parameter<double>("projection_finite_difference_px", 2.0);
        this->declare_parameter<double>("projection_min_world_std_m", 0.03);
        this->declare_parameter<double>("projection_max_world_std_m", 1.50);
        //以下为投影决策选择的参数
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
        //以下为沟区先验参数
        this->declare_parameter<std::string>(
            "gully_region_path",
            "");
        this->declare_parameter<bool>("gully_field_x_flip", false);
        // Only raw debug data leaves this package; RViz formatting belongs to visualization.
        declare_parameter<bool>("rviz_debug_enabled", false);
        declare_parameter<std::string>("world_frame", "world");
        declare_parameter<int>("image_width", 5472);
        declare_parameter<int>("image_height", 3648);

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
        cfg_ = std::make_unique<LocalizationConfig>(config_dir_);
        pose_solver_ = std::make_unique<PoseSolver>(cfg_->camera.cameraMatrix, cfg_->camera.distCoeffs);

        epoch_=static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
        calibration_pub_=create_publisher<radar27_interfaces::msg::CalibrationState>("/calibration_state", rclcpp::QoS(1).reliable().transient_local());
        if (get_parameter("rviz_debug_enabled").as_bool())
            debug_pub_=create_publisher<radar27_interfaces::msg::ProjectionDebugArray>("/projection_debug",rclcpp::QoS(1).best_effort());
        declare_parameter<std::string>("calibration_path", (std::filesystem::path(config_dir_)/"calib_result.yaml").string());
        loadCalibrationAtStartup();
        publishCalibration();

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
        world_pub_ = this->create_publisher<radar27_interfaces::msg::WorldMeasurementArray>(output_topic_, rclcpp::QoS(10).best_effort());

        armor_sub_ = this->create_subscription<radar27_interfaces::msg::DetectionArray>(
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
                        double mx = 0.0, my = 0.0;
                        rm_field::mirror_field_point(
                            point.x, point.y,
                            rm_field::kDefaultFieldLength,
                            rm_field::kDefaultFieldWidth, mx, my);
                        mirrored.emplace_back(
                            static_cast<float>(mx), static_cast<float>(my));
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
        // 转换后才与 canonical 盲区 polygon 比较；公式统一来自 rm_field。
        double field_x = 0.0, field_y = 0.0;
        rm_field::world_to_field(
            world.x, world.y, /*world_z_toward_blue=*/gully_field_x_flip_,
            field_x, field_y);
        const cv::Point2f field(
            static_cast<float>(field_x), static_cast<float>(field_y));
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
        const radar27_interfaces::msg::DetectionBox& detection)
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
        const radar27_interfaces::msg::DetectionBox& detection,
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
            // Disabled 是无插值的安全回退：优先 armor；仅 armor 质量失效而 car
            // 健康时改用 car，避免关闭实验功能后发布明显无效的测量。
            state.mode = !projectionHealthy(armor) && projectionHealthy(car)
                ? ProjectionMode::CAR : ProjectionMode::ARMOR;
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

    /** 优先使用 LocalizationConfig 已解析外参，否则直接读取 calib_result.yaml 并更新可标定状态。 */
    void loadCalibrationAtStartup()
    {
        if (cfg_->calib.valid) {
            pose_solver_->setExtrinsic(cfg_->calib.R, cfg_->calib.T);
            is_calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "成功从 LocalizationConfig 加载校准结果，已设置外参");
            return;
        }

        std::filesystem::path configDir = std::filesystem::path(config_dir_);
        std::string calibPath = get_parameter("calibration_path").as_string();
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
        std::string calibPath = get_parameter("calibration_path").as_string();

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
            ++epoch_;
            selector_states_.clear();
            publishCalibration();
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
    void publishCalibration() {
        if (!is_calibrated_) return;
        cv::Mat r,t; pose_solver_->getExtrinsic(r,t);
        radar27_interfaces::msg::CalibrationState m;
        m.header.stamp=now(); m.header.frame_id=get_parameter("world_frame").as_string();
        m.version=epoch_;
        for(int i=0;i<9;++i){m.rotation[i]=r.at<double>(i/3,i%3);m.camera_matrix[i]=cfg_->camera.cameraMatrix.at<double>(i/3,i%3);}
        for(int i=0;i<3;++i)m.translation[i]=t.at<double>(i);
        m.image_width=get_parameter("image_width").as_int();
        m.image_height=get_parameter("image_height").as_int();
        calibration_pub_->publish(m);
    }

    /** 检测回调：批量射线投影、构造测量/负观测、更新 Tracker 并发布 WorldTargetArray。 */
    void armor_callback(const radar27_interfaces::msg::DetectionArray::ConstSharedPtr msg)
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
            const auto stamp_ns=rclcpp::Time(msg->header.stamp).nanoseconds();
            if (stamp_ns < last_detection_stamp_ns_) selector_states_.clear();
            last_detection_stamp_ns_=stamp_ns;
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
                    debug_pub_ && debug_pub_->get_subscription_count() > 0;
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

            auto output=std::make_unique<radar27_interfaces::msg::WorldMeasurementArray>();
            output->header=msg->header;
            output->header.frame_id=get_parameter("world_frame").as_string();
            output->calibration_version=epoch_;
            radar27_interfaces::msg::ProjectionDebugArray debug;
            debug.header=output->header;
            for (size_t i=0;i<msg->detections.size();++i) {
                const auto& det=msg->detections[i]; const auto& p=world_projections[i];
                radar27_interfaces::msg::WorldMeasurement m;
                m.detection=det; m.world_x=p.world.x; m.world_z=p.world.y;
                m.covariance=p.covariance; m.covariance_valid=p.covariance_valid;
                m.valid=std::isfinite(p.world.x) && std::isfinite(p.world.y);
                m.is_negative=det.idx==robot_id::ARMOR && det.is_dead;
                output->measurements.push_back(m);
                if(debug_pub_ && debug_pub_->get_subscription_count()>0 && p.ray_endpoint_valid) {
                    radar27_interfaces::msg::ProjectionDebugSample d;
                    d.marker_key=i; d.class_id=det.idx; d.team_id=det.armor_color;
                    d.confidence=det.confidence; d.is_dead=det.is_dead;
                    d.negative_measurement=m.is_negative; d.tracker_measurement=det.idx!=robot_id::OUTPOST && !m.is_negative;
                    d.ray_endpoint={p.ray_endpoint.x,p.ray_endpoint.y,p.ray_endpoint.z};
                    d.measurement={p.world.x,0.0f,p.world.y}; debug.samples.push_back(d);
                }
            }
            if(debug_pub_ && debug_pub_->get_subscription_count()>0) debug_pub_->publish(debug);
            world_pub_->publish(std::move(output));
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "姿态解算回调异常: %s", e.what());
        }
    }

    std::unique_ptr<LocalizationConfig> cfg_; // 保存相机/Tracker/mesh 配置，覆盖 solver 生命周期。
    std::unique_ptr<PoseSolver> pose_solver_; // 持有标定矩阵和只加载一次的 Raycaster scene。
    bool is_calibrated_ = false; // false 时拒绝投影，避免发布看似有效的错误 world 点。
    int64_t last_detection_stamp_ns_ = 0; // 计算 Kalman dt 与检测时间倒退保护。
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

    uint64_t epoch_=0;
    rclcpp::Publisher<radar27_interfaces::msg::CalibrationState>::SharedPtr calibration_pub_;
    rclcpp::Publisher<radar27_interfaces::msg::ProjectionDebugArray>::SharedPtr debug_pub_;

    rclcpp::Subscription<radar27_interfaces::msg::DetectionArray>::SharedPtr armor_sub_;
    rclcpp::Publisher<radar27_interfaces::msg::WorldMeasurementArray>::SharedPtr world_pub_;
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
