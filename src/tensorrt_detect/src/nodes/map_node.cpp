#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/radar_map.hpp"
#include "ConfigManager.hpp"
#include "radarmap.hpp"
#include "robot_id.hpp"
#include "map_analyzer.hpp"
#include "tensorrt_detect_msgs/msg/map_tactics.hpp"

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
        this->declare_parameter<std::string>("output_tactics_topic", "/map_tactics");
        this->declare_parameter<int>("out_team_id", robot_id::RED);
        this->declare_parameter<bool>("flip_team", false);

        std::string config_dir = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_image_topic_ = this->get_parameter("output_image_topic").as_string();
        output_map_topic_ = this->get_parameter("output_map_topic").as_string();
        output_tactics_topic_ = this->get_parameter("output_tactics_topic").as_string();
        out_team_id_ = this->get_parameter("out_team_id").as_int();
        flip_team_ = this->get_parameter("flip_team").as_bool();

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
        radar_map_->setFlipTeam(flip_team_);

        if (!radar_map_->m_isCalibrated) {
            RCLCPP_ERROR(this->get_logger(), "RadarMap 校准失败，请检查 map.yaml 配置");
        } else {
            RCLCPP_INFO(this->get_logger(), "RadarMap 校准完成");
        }
        analyzer_ = std::make_unique<MapAnalyzer>(out_team_id_);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, rclcpp::QoS(1));
        radar_map_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::RadarMap>(output_map_topic_, 10);
        tactics_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::MapTactics>(output_tactics_topic_, 10);

        flip_team_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/flip_team", rclcpp::QoS(1),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                flip_team_ = msg->data;
                if (radar_map_) {
                    radar_map_->setFlipTeam(flip_team_);
                }
                RCLCPP_INFO(this->get_logger(), "阵营视角已切换为: %s", flip_team_ ? "蓝方" : "红方");
            });

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

            // 先读取前哨站状态
            bool has_outpost = false;
            bool outpost_alive = false;
            for (const auto& target : msg->targets) {
                if (target.class_id == robot_id::OUTPOST) {
                    has_outpost = true;
                    outpost_alive = !target.is_dead;
                    break;
                }
            }

            for (const auto& target : msg->targets) {
                if (!target.valid) {
                    continue;
                }
                // 过滤掉车辆检测和前哨站，它们不进入动态目标列表
                if (target.class_id == robot_id::CAR || target.class_id == robot_id::OUTPOST) {
                    continue;
                }

                // pose_node 中 world_x 为场地 X（宽），world_z 为场地 Z（长）
                cv::Point2f world_pt(target.world_x, target.world_z);
                cv::Point2f map_pt = radar_map_->worldtomap(world_pt);

                Mappoint mp;
                mp.map_point = map_pt;
                mp.label = "";
                mp.classIdx = target.class_id;
                mp.armorColor = target.team_id;
                mp.isDead = target.is_dead;
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

                if (!target.is_dead && idx >= 0 && idx < 6) {
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

            analyzer_->evaluate(msg->targets);

            auto tactics_msg = std::make_shared<tensorrt_detect_msgs::msg::MapTactics>();
            tactics_msg->header = msg->header;
            tactics_msg->engineer_on_island = analyzer_->engineer_on_island();
            tactics_msg->opponent_attack = analyzer_->opponent_attack();
            tactics_msg->our_attack = analyzer_->our_attack();
            tactics_pub_->publish(*tactics_msg);

            if (analyzer_->opponent_attack()) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 敌方大攻!");
            }
            if (analyzer_->our_attack()) {
                RCLCPP_INFO(this->get_logger(), "✅ 我方大攻!");
            }
            if (analyzer_->engineer_on_island()) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 敌方工程上岛!");
            }

                       cv::Mat map_frame = radar_map_->drawMap(mappoints, cfg_->model.classNames);

            // ========== 前哨站叠加绘制（在 drawMap 之后） ==========
            const auto& outpostPts = cfg_->map.getOutpostMapPoints(flip_team_);
            if (outpostPts.size() >= 2) {
                float x = outpostPts[0];
                float y = outpostPts[1];

                cv::Point2f pt;
                if (cfg_->map.isFlip) {
                    if (flip_team_) {
                        pt.x = 387 - y;
                        pt.y = x; 
                    } else {
                        pt.x = y;
                        pt.y = 721 - x;
                    }
                } else {
                    if (flip_team_) {
                        pt.x = y;
                        pt.y = 721 - x;
                    } else { 
                        pt.x = 387 - y;
                        pt.y = x;

                    }
                }

                if (!has_outpost) {
                    // 消息中没有前哨站信息，不绘制
                } else if (outpost_alive) {
                    cv::circle(map_frame, pt, 8, cv::Scalar(0, 215, 255), -1, cv::LINE_AA);
                    cv::circle(map_frame, pt, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                    cv::putText(map_frame, "ALIVE", cv::Point(pt.x + 16, pt.y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 4, cv::LINE_AA);
                    cv::putText(map_frame, "ALIVE", cv::Point(pt.x + 16, pt.y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
                } else {
                    cv::Scalar dead_color(0, 0, 0);
                    int len = 10;
                    cv::line(map_frame, pt + cv::Point2f(-len, -len), pt + cv::Point2f(len, len), dead_color, 3, cv::LINE_AA);
                    cv::line(map_frame, pt + cv::Point2f(len, -len), pt + cv::Point2f(-len, len), dead_color, 3, cv::LINE_AA);
                    cv::putText(map_frame, "DEAD", cv::Point(pt.x + 16, pt.y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 4, cv::LINE_AA);
                    cv::putText(map_frame, "DEAD", cv::Point(pt.x + 16, pt.y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, dead_color, 2, cv::LINE_AA);
                }
            }
            // ======================================================

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

    bool flip_team_ = false;
    std::unique_ptr<Config> cfg_;
    std::unique_ptr<RadarMap> radar_map_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flip_team_sub_;

    int out_team_id_ = robot_id::RED;
    std::string input_topic_;
    std::string output_image_topic_;
    std::string output_map_topic_;
    std::string output_tactics_topic_;

    rclcpp::Subscription<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::RadarMap>::SharedPtr radar_map_pub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::MapTactics>::SharedPtr tactics_pub_;
    std::unique_ptr<MapAnalyzer> analyzer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
