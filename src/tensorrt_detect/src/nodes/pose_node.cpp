#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "posesolver.hpp"


class PoseNode : public rclcpp::Node
{
public:
    PoseNode() : Node("pose_node")
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("input_topic", "/armor_detections");
        this->declare_parameter<std::string>("output_topic", "/world_targets");

        std::string config_dir = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());

        cfg_ = std::make_unique<Config>(config_dir);
        pose_solver_ = std::make_unique<PoseSolver>(cfg_->camera.cameraMatrix, cfg_->camera.distCoeffs);

        // 加载外参：优先使用 Config 已加载的标定结果
        if (cfg_->calib.valid) {
            pose_solver_->setExtrinsic(cfg_->calib.R, cfg_->calib.T);
            RCLCPP_INFO(this->get_logger(), "成功从 Config 加载校准结果，已设置外参");
        } else {
            std::filesystem::path configDir = std::filesystem::path(config_dir);
            std::string calibPath = (configDir / "calib_result.yaml").string();
            if (std::filesystem::exists(calibPath)) {
                try {
                    YAML::Node node = YAML::LoadFile(calibPath);
                    if (node["r"].IsSequence() && node["t"].IsSequence()) {
                        std::vector<double> r_data = node["r"].as<std::vector<double>>();
                        std::vector<double> t_data = node["t"].as<std::vector<double>>();
                        if (r_data.size() == 9 && t_data.size() == 3) {
                            cv::Mat R(3, 3, CV_64F);
                            cv::Mat T(3, 1, CV_64F);
                            for (int i = 0; i < 9; ++i) {
                                R.at<double>(i / 3, i % 3) = r_data[i];
                            }
                            for (int i = 0; i < 3; ++i) {
                                T.at<double>(i, 0) = t_data[i];
                            }
                            pose_solver_->setExtrinsic(R, T);
                            RCLCPP_INFO(this->get_logger(), "成功加载校准结果，已设置外参");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "校准文件数据维度不匹配");
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "校准文件格式错误，缺少 r 或 t 数据");
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "加载校准文件失败: %s", e.what());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "未找到校准文件: %s", calibPath.c_str());
            }
        }

        // 加载 3D mesh（用于射线碰撞）
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

        RCLCPP_INFO(this->get_logger(), "PoseNode 初始化完成，等待检测结果输入...");
    }

private:
    void armor_callback(const tensorrt_detect_msgs::msg::DetectionArray::SharedPtr msg)
    {
        try {
            auto world_msg = std::make_shared<tensorrt_detect_msgs::msg::WorldTargetArray>();
            world_msg->header = msg->header;

            for (const auto& det : msg->detections) {
                tensorrt_detect_msgs::msg::WorldTarget target;
                target.idx = det.idx;
                target.class_id = det.idx;
                target.team_id = det.armor_color;
                target.score = det.confidence;
                target.valid = true;
                target.bbox_x = det.x;
                target.bbox_y = det.y;
                target.bbox_w = det.width;
                target.bbox_h = det.height;

                // 优先使用 car_box 底部中心进行世界坐标解算
                cv::Rect car_box(det.car_x, det.car_y, det.car_width, det.car_height);
                if (car_box.width > 0 && car_box.height > 0) {
                    cv::Point2f world_pos = pose_solver_->middletoworld(car_box);
                    target.world_x = world_pos.x;
                    target.world_y = 0.0f;
                    target.world_z = world_pos.y;
                } else {
                    // 若 car_box 无效，回退到 armor box
                    cv::Rect armor_box(det.x, det.y, det.width, det.height);
                    cv::Point2f world_pos = pose_solver_->middletoworld(armor_box);
                    target.world_x = world_pos.x;
                    target.world_y = 0.0f;
                    target.world_z = world_pos.y;
                }

                world_msg->targets.push_back(target);
            }

            world_pub_->publish(*world_msg);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "接收到 %zu 个检测，发布了 %zu 个世界坐标目标",
                msg->detections.size(), world_msg->targets.size());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "姿态解算回调异常: %s", e.what());
        }
    }

    std::unique_ptr<Config> cfg_;
    std::unique_ptr<PoseSolver> pose_solver_;

    std::string input_topic_;
    std::string output_topic_;

    rclcpp::Subscription<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_sub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr world_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
