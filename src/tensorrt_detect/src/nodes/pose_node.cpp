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

#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "posesolver.hpp"
#include "robot_id.hpp"
#include "tracker.hpp"
#include "tracker_message.hpp"
#include <cuda_runtime_api.h>

class PoseNode : public rclcpp::Node
{
public:
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
        this->declare_parameter<std::string>(
            "gully_region_path",
            "/home/delphine/rm/tensorrt10_detect/generated/gully.yaml");
        this->declare_parameter<double>("gully_transition_width_m", 1.5);
        this->declare_parameter<bool>("gully_field_x_flip", false);

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
        gully_region_path_ =
            this->get_parameter("gully_region_path").as_string();
        gully_transition_width_m_ = static_cast<float>(std::max(
            0.0,
            this->get_parameter("gully_transition_width_m").as_double()));
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
        RCLCPP_INFO(this->get_logger(),
            "沟区射线切换: polygons=%zu transition=%.2f m field_x_flip=%s",
            gully_polygons_.size(), gully_transition_width_m_,
            gully_field_x_flip_ ? "true" : "false");

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

        loadCalibrationAtStartup();

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
                "加载沟区范围失败: %s (%s)，射线将始终使用车辆框",
                gully_region_path_.c_str(), e.what());
        }
    }

    float armorRayWeight(const cv::Point2f& world) const
    {
        if (gully_polygons_.empty() ||
            !std::isfinite(world.x) || !std::isfinite(world.y)) {
            return 0.0f;
        }

        const cv::Point2f field(
            gully_field_x_flip_ ? world.y + 14.0f : 14.0f - world.y,
            world.x + 7.5f);
        double signed_distance = -std::numeric_limits<double>::infinity();
        for (const auto& polygon : gully_polygons_) {
            signed_distance = std::max(
                signed_distance,
                cv::pointPolygonTest(polygon, field, true));
        }

        if (signed_distance >= 0.0) {
            return 1.0f;
        }
        if (gully_transition_width_m_ <= 0.0f) {
            return 0.0f;
        }

        const float t = std::clamp(
            1.0f + static_cast<float>(signed_distance) /
                gully_transition_width_m_,
            0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    static WorldProjection blendProjection(
        const WorldProjection& car,
        const WorldProjection& armor,
        float armor_weight)
    {
        if (armor_weight <= 0.0f) {
            return car;
        }
        if (armor_weight >= 1.0f) {
            return armor;
        }

        WorldProjection result;
        const float car_weight = 1.0f - armor_weight;
        result.world = car.world * car_weight + armor.world * armor_weight;
        result.surface_discontinuity =
            car.surface_discontinuity || armor.surface_discontinuity;
        result.jacobian_condition_number = std::max(
            car.jacobian_condition_number,
            armor.jacobian_condition_number);

        if (car.covariance_valid && armor.covariance_valid) {
            // 协方差与位置使用相同权重平滑过渡。不要把两候选的位置差
            // 再作为混合方差加入，否则 Kalman 会忽略整段过渡，直到进入
            // 沟区、权重变为 1 后才一次性跟上，看起来仍像硬切换。
            result.covariance = {
                car_weight * car.covariance[0] +
                    armor_weight * armor.covariance[0],
                car_weight * car.covariance[1] +
                    armor_weight * armor.covariance[1],
                car_weight * car.covariance[2] +
                    armor_weight * armor.covariance[2],
                car_weight * car.covariance[3] +
                    armor_weight * armor.covariance[3]};
            result.covariance_valid = true;
        } else if (car.covariance_valid) {
            result.covariance = car.covariance;
            result.covariance_valid = true;
        } else if (armor.covariance_valid) {
            result.covariance = armor.covariance;
            result.covariance_valid = true;
        }
        return result;
    }

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
            response->success = true;
            response->message = "Calibration reloaded successfully";
            RCLCPP_INFO(this->get_logger(), "pose_node 已重载校准结果，标定就绪");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to reload: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "重载校准失败: %s", e.what());
        }
    }

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

            // 前半段为车辆框，后半段为装甲板框；无效框使用另一个框回退，
            // 仍保持一次批量 CastRays。
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

            std::vector<WorldProjection> world_projections;
            if (!boxes_for_raycast.empty()) {
                const auto candidate_projections =
                    pose_solver_->middletoworldBatchWithUncertainty(
                        boxes_for_raycast, projection_config_);
                if (candidate_projections.size() == detection_count * 2) {
                    world_projections.reserve(detection_count);
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
                            world_projections.push_back(armor_projection);
                        } else if (!armor_valid) {
                            world_projections.push_back(car_projection);
                        } else {
                            world_projections.push_back(blendProjection(
                                car_projection, armor_projection,
                                armorRayWeight(car_projection.world)));
                        }
                    }
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
                    negative.class_margin = det.class_margin;
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
                m.class_margin = det.class_margin;
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

    std::unique_ptr<Config> cfg_;
    std::unique_ptr<PoseSolver> pose_solver_;
    Tracker tracker_;
    bool is_calibrated_ = false;
    int64_t last_detection_stamp_ns_ = 0;
    std::vector<tensorrt_detect_msgs::msg::WorldTarget> cached_dead_targets_;
    int64_t last_dead_target_observed_ns_ = 0;
    float dead_target_hold_time_s_ = 0.10f;
    ProjectionUncertaintyConfig projection_config_;
    std::vector<std::vector<cv::Point2f>> gully_polygons_;
    std::string gully_region_path_;
    float gully_transition_width_m_ = 1.5f;
    bool gully_field_x_flip_ = false;

    std::string config_dir_;
    std::string input_topic_;
    std::string output_topic_;

    rclcpp::Subscription<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_sub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr world_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PoseNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
