#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "posesolver.hpp"
#include "robot_id.hpp"
#include "tracker.hpp"

class PoseNode : public rclcpp::Node
{
public:
    PoseNode() : Node("pose_node")
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("input_topic", "/armor_detections");
        this->declare_parameter<std::string>("output_topic", "/world_targets");

        config_dir_ = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());

        cfg_ = std::make_unique<Config>(config_dir_);
        pose_solver_ = std::make_unique<PoseSolver>(cfg_->camera.cameraMatrix, cfg_->camera.distCoeffs);

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

        world_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::WorldTargetArray>(output_topic_, 10);

        armor_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::DetectionArray>(
            input_topic_, 10,
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

    void armor_callback(const tensorrt_detect_msgs::msg::DetectionArray::SharedPtr msg)
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
            // ---- 1. 解算所有检测的世界坐标，构建观测输入 ----
            // Outpost 不走 Tracker，直接透传
            // 死亡装甲板（ARMOR + is_dead）不走固定槽位，动态追加
            // 正常装甲板（R1~S）进入固定槽位跟踪
            std::vector<WorldMeasurement> meas;
            meas.reserve(msg->detections.size());
            std::vector<tensorrt_detect_msgs::msg::WorldTarget> dead_targets;
            tensorrt_detect_msgs::msg::WorldTarget outpost_target;
            bool has_outpost = false;

            for (const auto& det : msg->detections) {
                cv::Rect car_box(det.car_x, det.car_y, det.car_width, det.car_height);
                cv::Point2f world_pos;
                if (car_box.width > 0 && car_box.height > 0) {
                    world_pos = pose_solver_->middletoworld(car_box);
                } else {
                    cv::Rect armor_box(det.x, det.y, det.width, det.height);
                    world_pos = pose_solver_->middletoworld(armor_box);
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
                    has_outpost = true;
                    continue;
                }

                // 死亡装甲板动态追加，不走固定槽位（固定槽位没有 ARMOR 类别）
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
                    dead_targets.push_back(t);
                    continue;
                }

                WorldMeasurement m;
                m.class_id = det.idx;
                m.team_id  = det.armor_color;
                m.score    = det.confidence;
                m.is_dead  = det.is_dead;
                m.box      = cv::Rect(det.x, det.y, det.width, det.height);
                m.world    = world_pos;  // x=world_x, y=world_z
                meas.push_back(m);
            }

            // ---- 2. Tracker 更新（Kalman 平滑 + 固定槽位数据关联，不含 Outpost/死亡装甲板）----
            tracker_.update(meas);

            // ---- 3. 固定槽位 + Outpost + 动态死亡装甲板 发布 ----
            auto world_msg = std::make_shared<tensorrt_detect_msgs::msg::WorldTargetArray>();
            world_msg->header = msg->header;
            // 0-9: Tracker 固定槽位；10: Outpost 透传
            world_msg->targets.resize(11);

            int valid_count = 0;
            for (int i = 0; i < Tracker::NUM_SLOTS; ++i) {
                auto slot = tracker_.get_slot(i);
                auto& target = world_msg->targets[i];
                target.idx      = i;
                target.class_id = slot.class_id;
                target.team_id  = slot.team_id;
                target.is_dead  = slot.is_dead;
                target.score    = slot.score;
                target.valid    = slot.valid;
                target.bbox_x   = slot.smoothed_box.x;
                target.bbox_y   = slot.smoothed_box.y;
                target.bbox_w   = slot.smoothed_box.width;
                target.bbox_h   = slot.smoothed_box.height;
                target.world_x  = slot.smoothed_world.x;
                target.world_y  = 0.0f;
                target.world_z  = slot.smoothed_world.y;
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
            }

            // 动态追加死亡装甲板
            for (const auto& dt : dead_targets) {
                world_msg->targets.push_back(dt);
            }

            world_pub_->publish(*world_msg);

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

    std::string config_dir_;
    std::string input_topic_;
    std::string output_topic_;

    rclcpp::Subscription<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_sub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr world_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
