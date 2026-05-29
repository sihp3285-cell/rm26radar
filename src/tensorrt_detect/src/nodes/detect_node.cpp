#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "tensorrt_detect_msgs/msg/pipeline_timing.hpp"
#include "ConfigManager.hpp"
#include "pipeline.hpp"
#include "draw.hpp"
#include "robot_id.hpp"
#include <cuda_runtime_api.h>

class DetectNode : public rclcpp::Node
{
public:
    explicit DetectNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("detect_node", options)
    {
        // 提前初始化 CUDA primary context，避免与 PoseNode/Open3D 并发初始化导致 SIGSEGV
        cudaFree(0);

        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/detected_image");
        this->declare_parameter<bool>("publish_debug_image", true);

        this->declare_parameter<int>("debug_output_max_width", 1280);

        std::string config_dir = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
        const int64_t debug_output_max_width_param =
            this->get_parameter("debug_output_max_width").as_int();
        debug_output_max_width_ = static_cast<int>(std::max<int64_t>(1, debug_output_max_width_param));

        RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());

        cfg_ = std::make_unique<Config>(config_dir);
        pipeline_ = std::make_unique<DetectPipeline>(*cfg_);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::QoS(1));
        armor_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::DetectionArray>("/armor_detections", rclcpp::QoS(10).best_effort());
        timing_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::PipelineTiming>("/pipeline_timing", rclcpp::QoS(1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, rclcpp::QoS(1),
            std::bind(&DetectNode::image_callback, this, std::placeholders::_1));

        reload_roi_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/detect_node/reload_roi",
            std::bind(&DetectNode::reloadROI, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "DetectNode 初始化完成，等待图像输入...");
    }


private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            double input_delay_ms = (this->now() - msg->header.stamp).seconds() * 1000.0;

            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;

            std::vector<Result> results = pipeline_->process(frame);

            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time_).count();
            last_time_ = now;

            double instant_fps = 1.0 / std::max(dt, 1e-6);
            fps_ = 0.9 * fps_ + 0.1 * instant_fps;

            auto armor_msg = std::make_unique<tensorrt_detect_msgs::msg::DetectionArray>();
            armor_msg->header = msg->header;   // 复用图像时间戳，方便下游同步
            armor_msg->header.frame_id = "detection";
            armor_msg->detections.reserve(results.size());

            bool hasOutpost = false;
            for (const auto& res : results) {
                if (res.idx == robot_id::CAR) {
                    continue;
                }
                if (res.box.width <= 0 || res.box.height <= 0) continue;  // 空框不画

                if (res.idx == robot_id::OUTPOST) {
                    hasOutpost = true;
                }

                tensorrt_detect_msgs::msg::DetectionBox box;
                box.idx         = res.idx;
                box.confidence  = res.confidence;
                box.x           = res.box.x;
                box.y           = res.box.y;
                box.width       = res.box.width;
                box.height      = res.box.height;
                box.armor_color = res.armorColor;
                box.is_dead     = res.isDead;
                box.car_x       = res.car_box.x;
                box.car_y       = res.car_box.y;
                box.car_width   = res.car_box.width;
                box.car_height  = res.car_box.height;
                box.world_x     = res.worldPoint.x;
                box.world_y     = res.worldPoint.y;
                box.fps         = static_cast<float>(fps_);

                armor_msg->detections.push_back(box);
            }

            // 前哨站功能启用但未在 results 中出现时，推送状态消息（空框，仅传递存活/死亡状态）
            if (cfg_->model.outpostEnabled && !hasOutpost) {
                tensorrt_detect_msgs::msg::DetectionBox statusBox;
                statusBox.idx = robot_id::OUTPOST;
                statusBox.is_dead = !pipeline_->isOutpostAlive();
                statusBox.confidence = 0.0f;
                armor_msg->detections.push_back(statusBox);
            }

            armor_pub_->publish(std::move(armor_msg));

            {
                auto timing = pipeline_->getLatestTiming();
                auto timing_msg = std::make_unique<tensorrt_detect_msgs::msg::PipelineTiming>();
                timing_msg->header = msg->header;
                timing_msg->car_ms = timing.car_ms;
                timing_msg->armor_ms = timing.armor_ms;
                timing_msg->cls_ms = timing.cls_ms;
                timing_msg->outpost_ms = timing.outpost_ms;
                timing_msg->airplane_ms = timing.airplane_ms;
                timing_msg->total_ms = timing.total_ms;
                timing_msg->end_to_end_ms = (this->now() - msg->header.stamp).seconds() * 1000.0;
                timing_msg->fps = static_cast<double>(fps_);
                timing_pub_->publish(std::move(timing_msg));
            }

            if (publish_debug_image_) {
                frame.copyTo(debug_frame_);
                drawDetect(debug_frame_, results, cfg_->model.classNames);

                if (debug_frame_.cols > debug_output_max_width_) {
                    const double debug_scale = static_cast<double>(debug_output_max_width_) /
                                               static_cast<double>(debug_frame_.cols);
                    const int target_height = static_cast<int>(debug_frame_.rows * debug_scale);
                    cv::resize(debug_frame_, debug_output_frame_, cv::Size(debug_output_max_width_, target_height));
                } else {
                    debug_output_frame_ = debug_frame_;
                }

                debug_cv_image_.header = msg->header;
                debug_cv_image_.header.frame_id = "detected_frame";
                debug_cv_image_.image = debug_output_frame_;
                auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
                debug_cv_image_.toImageMsg(*out_msg);
                image_pub_->publish(std::move(out_msg));
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "检测到 %zu 个目标，fps: %.1f，input_delay: %.2f ms",
                results.size(), fps_, input_delay_ms);
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测回调异常: %s", e.what());
        }
    }

    std::unique_ptr<Config> cfg_;
    std::unique_ptr<DetectPipeline> pipeline_;

    std::string input_topic_;
    std::string output_topic_;
    bool publish_debug_image_ = true;
    int debug_output_max_width_ = 1280;

    void reloadROI(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                   std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        try {
            std::string config_dir = this->get_parameter("config_dir").as_string();
            std::filesystem::path dir(config_dir);
            std::string outpost_yaml = (dir / "outpost_roi.yaml").string();

            YAML::Node cfg = YAML::LoadFile(outpost_yaml);
            cfg_->model.outpostEnabled = cfg["outpost_enabled"]
                                            ? cfg["outpost_enabled"].as<bool>()
                                            : false;
            if (cfg["outpost_roi"]) {
                cfg_->model.outpostRoi = cfg["outpost_roi"].as<std::vector<int>>();
            }
            cfg_->model.outpostScoreThreshold = cfg["outpost_score_threshold"]
                                                    ? cfg["outpost_score_threshold"].as<float>()
                                                    : 0.0f;
            cfg_->model.outpostMissThreshold = cfg["outpost_miss_threshold"]
                                                   ? cfg["outpost_miss_threshold"].as<int>()
                                                   : 20;

            response->success = true;
            response->message = "outpost ROI 配置已重载";
            RCLCPP_INFO(this->get_logger(), "outpost ROI 配置已重载: enabled=%s, roi=[%d,%d,%d,%d]",
                        cfg_->model.outpostEnabled ? "true" : "false",
                        cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[0] : -1,
                        cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[1] : -1,
                        cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[2] : -1,
                        cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[3] : -1);
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("重载失败: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "outpost ROI 重载失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_pub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::PipelineTiming>::SharedPtr timing_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_roi_service_;
    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
    double fps_ = 0.0;
    cv::Mat detect_input_frame_;
    cv::Mat debug_frame_;
    cv::Mat debug_output_frame_;
    cv_bridge::CvImage debug_cv_image_{std_msgs::msg::Header(), "bgr8"};
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
