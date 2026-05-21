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
#include "tracker.hpp"

class MapNode : public rclcpp::Node
{
public:
    explicit MapNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("map_node", options)
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("input_topic", "/world_targets");
        this->declare_parameter<std::string>("output_image_topic", "/map_image");
        this->declare_parameter<std::string>("output_map_topic", "/radar_map");
        this->declare_parameter<std::string>("output_tactics_topic", "/map_tactics");
        this->declare_parameter<int>("out_team_id", robot_id::RED);
        this->declare_parameter<bool>("flip_team", false);
        this->declare_parameter<bool>("field_x_flip", false);

        std::string config_dir = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_image_topic_ = this->get_parameter("output_image_topic").as_string();
        output_map_topic_ = this->get_parameter("output_map_topic").as_string();
        output_tactics_topic_ = this->get_parameter("output_tactics_topic").as_string();
        out_team_id_ = this->get_parameter("out_team_id").as_int();
        flip_team_ = this->get_parameter("flip_team").as_bool();
        bool field_x_flip = this->get_parameter("field_x_flip").as_bool();

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
        analyzer_->setTeamByFlip(flip_team_);
        analyzer_->setFieldXFlip(!flip_team_);
        RCLCPP_INFO(this->get_logger(), "初始阵营: %s", flip_team_ ? "红方" : "蓝方");

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, rclcpp::QoS(1));
        radar_map_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::RadarMap>(output_map_topic_, 10);
        tactics_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::MapTactics>(output_tactics_topic_, 10);

        flip_team_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/flip_team", rclcpp::QoS(1),
            [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
                flip_team_ = msg->data;
                if (radar_map_) {
                    radar_map_->setFlipTeam(flip_team_);
                }
                if (analyzer_) {
                    analyzer_->setTeamByFlip(flip_team_);
                    analyzer_->setFieldXFlip(!flip_team_);
                }
                RCLCPP_INFO(this->get_logger(), "阵营视角已切换为: %s", flip_team_ ? "红方" : "蓝方");
            });

        target_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::WorldTargetArray>(
            input_topic_, rclcpp::QoS(10).best_effort(),
            std::bind(&MapNode::target_callback, this, std::placeholders::_1));
        

        RCLCPP_INFO(this->get_logger(), "MapNode 初始化完成，等待世界坐标输入...");
    }

private:
    void target_callback(const tensorrt_detect_msgs::msg::WorldTargetArray::ConstSharedPtr msg)
    {
        try {
            auto radar_msg = std::make_unique<tensorrt_detect_msgs::msg::RadarMap>();

            // 初始化数组为 0
            for (int i = 0; i < 6; ++i) {
                radar_msg->blue_x[i] = 0.0f;
                radar_msg->blue_y[i] = 0.0f;
                radar_msg->red_x[i] = 0.0f;
                radar_msg->red_y[i] = 0.0f;
            }

            // 固定槽位 + 动态死亡装甲板
            //   0-9:   固定槽位（R1~S）
            //   10:    Outpost 透传
            //   11+:   动态死亡装甲板
            bool has_outpost = false;
            bool outpost_alive = false;
            if (msg->targets.size() > 10) {
                const auto& outpost_target = msg->targets[10];
                has_outpost = outpost_target.valid;
                outpost_alive = has_outpost && !outpost_target.is_dead;
            }

            // ---- 构建 Mappoints 并填充 RadarMap ----
            std::vector<Mappoint> mappoints;

            for (size_t i = 0; i < msg->targets.size(); ++i) {
                const auto& t = msg->targets[i];
                if (!t.valid) continue;

                cv::Point2f raw_pt = radar_map_->worldtomap(cv::Point2f(t.world_x, t.world_z));

                // 优先使用 BotIdentity 稳定身份（指数加权历史），无效时回落到单帧 class_id
                int display_class = (t.stable_class_id >= 0 && t.stable_class_conf > 0.0f)
                                    ? t.stable_class_id : t.class_id;

                // 前哨站单独处理（不在 Mappoints 中绘制，后面单独叠加）
                if (t.class_id == robot_id::OUTPOST) continue;

                // 动态死亡装甲板直接绘制
                if (t.class_id == robot_id::ARMOR && t.is_dead) {
                    Mappoint mp;
                    mp.map_point = raw_pt;
                    mp.classIdx = robot_id::ARMOR;
                    mp.armorColor = robot_id::UNKNOWN;
                    mp.isDead = true;
                    if (robot_id::ARMOR >= 0 && robot_id::ARMOR < static_cast<int>(cfg_->model.classNames.size())) {
                        mp.label = cfg_->model.classNames[robot_id::ARMOR];
                    } else {
                        mp.label = "dead";
                    }
                    mappoints.push_back(mp);
                    continue;
                }

                // 固定槽位（R1~S）
                Mappoint mp;
                mp.map_point = raw_pt;
                mp.classIdx = display_class;
                mp.armorColor = t.team_id;
                mp.isDead = t.is_dead;
                if (display_class >= 0 && display_class < static_cast<int>(cfg_->model.classNames.size())) {
                    mp.label = cfg_->model.classNames[display_class];
                }
                mappoints.push_back(mp);

                // 填充 RadarMap 消息（仅固定槽位且非死亡）
                int idx = -1;
                if (display_class >= robot_id::R1 && display_class <= robot_id::R4) {
                    idx = display_class - robot_id::R1;
                } else if (display_class == robot_id::S) {
                    idx = 5;
                }
                if (!t.is_dead && idx >= 0 && idx < 6) {
                    if (t.team_id == robot_id::BLUE) {
                        radar_msg->blue_x[idx] = raw_pt.x;
                        radar_msg->blue_y[idx] = raw_pt.y;
                    } else if (t.team_id == robot_id::RED) {
                        radar_msg->red_x[idx] = raw_pt.x;
                        radar_msg->red_y[idx] = raw_pt.y;
                    }
                }
            }

            radar_msg->header = msg->header;
            radar_map_pub_->publish(std::move(radar_msg));

            analyzer_->evaluate(msg->targets);

            auto tactics_msg = std::make_unique<tensorrt_detect_msgs::msg::MapTactics>();
            tactics_msg->header = msg->header;
            tactics_msg->engineer_on_island = analyzer_->engineer_on_island();
            tactics_msg->opponent_attack = analyzer_->opponent_attack();
            tactics_msg->our_attack = analyzer_->our_attack();
            tactics_msg->opponent_near_fortress = analyzer_->opponent_near_fortress();

            tactics_pub_->publish(std::move(tactics_msg));

            if (analyzer_->opponent_attack()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "⚠️ 敌方大攻!");
            }
            if (analyzer_->our_attack()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "✅ 我方大攻!");
            }
            if (analyzer_->engineer_on_island() == 1) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "⚠️ 我方工程上岛!");
            }
            if (analyzer_->engineer_on_island() == 2) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "⚠️ 敌方工程上岛!");
            }
            if (analyzer_->opponent_near_fortress() == 1) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "⚠️ 敌方接近堡垒!");
            }

            cv::Mat map_frame = radar_map_->drawMap(mappoints, cfg_->model.classNames);

            // ========== 前哨站叠加绘制（在 drawMap 之后） ==========
            const auto& outpostPts = cfg_->map.getOutpostMapPoints(flip_team_);
            if (outpostPts.size() >= 2 && has_outpost) {
                float x = outpostPts[0];
                float y = outpostPts[1];
                if (std::isnan(x) || std::isnan(y)) return;

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

                if (outpost_alive) {
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
            auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
            cv_bridge::CvImage(header, "bgr8", map_frame).toImageMsg(*out_msg);
            image_pub_->publish(std::move(out_msg));

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "固定槽位地图输出 %zu 个目标",
                mappoints.size());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "地图回调异常: %s", e.what());
        }
    }

    bool flip_team_ = false;
    std::unique_ptr<Config> cfg_;
    std::unique_ptr<RadarMap> radar_map_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flip_team_sub_;

    int out_team_id_ = robot_id::BLUE;
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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}