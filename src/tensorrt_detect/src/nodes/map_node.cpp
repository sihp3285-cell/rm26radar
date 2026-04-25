#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/radar_map.hpp"
#include "ConfigManager.hpp"
#include "radarmap.hpp"
#include "robot_id.hpp"

class MapNode : public rclcpp::Node
{
public:
    MapNode() : Node("map_node")
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("input_topic", "/world_targets");
        this->declare_parameter<std::string>("output_image_topic", "/map_image");
        this->declare_parameter<std::string>("output_map_topic", "/radar_map");

        std::string config_dir = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_image_topic_ = this->get_parameter("output_image_topic").as_string();
        output_map_topic_ = this->get_parameter("output_map_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布图像话题: %s", output_image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布地图话题: %s", output_map_topic_.c_str());

        cfg_ = std::make_unique<Config>(config_dir);
        radar_map_ = std::make_unique<RadarMap>(cfg_->map.mapPath, cfg_->map.isFlip);
        radar_map_->calibrate2(
            cfg_->map.race_size[0],
            cfg_->map.race_size[1],
            cfg_->map.map_size[0],
            cfg_->map.map_size[1]);

        if (!radar_map_->m_isCalibrated) {
            RCLCPP_ERROR(this->get_logger(), "RadarMap 校准失败，请检查 map.yaml 配置");
        } else {
            RCLCPP_INFO(this->get_logger(), "RadarMap 校准完成");
        }

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);
        radar_map_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::RadarMap>(output_map_topic_, 10);

        target_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::WorldTargetArray>(
            input_topic_, 10,
            std::bind(&MapNode::target_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MapNode 初始化完成，等待世界坐标输入...");
    }

private:
    void target_callback(const tensorrt_detect_msgs::msg::WorldTargetArray::SharedPtr msg)
    {
        try {
            std::vector<Mappoint> mappoints;
            auto radar_msg = std::make_shared<tensorrt_detect_msgs::msg::RadarMap>();

            // 初始化数组为 0
            for (int i = 0; i < 6; ++i) {
                radar_msg->blue_x[i] = 0.0f;
                radar_msg->blue_y[i] = 0.0f;
                radar_msg->red_x[i] = 0.0f;
                radar_msg->red_y[i] = 0.0f;
            }

            for (const auto& target : msg->targets) {
                if (!target.valid) {
                    continue;
                }

                // 过滤掉车辆检测（CAR, class_id == 0），与 standalone_main 逻辑保持一致
                if (target.class_id == robot_id::CAR) {
                    continue;
                }

                // pose_node 中 world_x 为场地 X（宽），world_z 为场地 Z（长）
                cv::Point2f world_pt(target.world_x, target.world_z);
                cv::Point2f map_pt = radar_map_->worldtomap(world_pt);

                Mappoint mp;
                mp.map_point = map_pt;
                // drawMap 内部会重新通过 robot_id 计算 label，这里传空字符串与 standalone 保持一致
                mp.label = "";
                mp.classIdx = target.class_id;
                mp.teamId = target.team_id;
                mappoints.push_back(mp);

                // 填充 RadarMap 消息
                // class_id 映射: R1=2, R2=3, R3=4, R4=5, S=6
                // RadarMap 数组: [1号, 2号, 3号, 4号, 5号, 哨兵]
                int idx = -1;
                if (target.class_id >= robot_id::R1 && target.class_id <= robot_id::R4) {
                    idx = target.class_id - robot_id::R1; // 2->0, 3->1, 4->2, 5->3
                } else if (target.class_id == robot_id::S) {
                    idx = 5; // 哨兵放在索引 5
                }

                if (idx >= 0 && idx < 6) {
                    if (target.team_id == robot_id::BLUE) {
                        radar_msg->blue_x[idx] = map_pt.x;
                        radar_msg->blue_y[idx] = map_pt.y;
                    } else if (target.team_id == robot_id::RED) {
                        radar_msg->red_x[idx] = map_pt.x;
                        radar_msg->red_y[idx] = map_pt.y;
                    }
                }
            }

            radar_msg->header = msg->header;
            radar_map_pub_->publish(*radar_msg);

            cv::Mat map_frame = radar_map_->drawMap(mappoints, cfg_->model.classNames);

            std_msgs::msg::Header header = msg->header;
            header.frame_id = "radar_map";
            auto out_msg = cv_bridge::CvImage(header, "bgr8", map_frame).toImageMsg();
            image_pub_->publish(*out_msg);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "接收到 %zu 个世界坐标目标，发布了 RadarMap 和地图图像",
                msg->targets.size());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "地图回调异常: %s", e.what());
        }
    }

    std::unique_ptr<Config> cfg_;
    std::unique_ptr<RadarMap> radar_map_;

    std::string input_topic_;
    std::string output_image_topic_;
    std::string output_map_topic_;

    rclcpp::Subscription<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::RadarMap>::SharedPtr radar_map_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
