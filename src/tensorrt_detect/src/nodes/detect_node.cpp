#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "pipeline.hpp"
#include "draw.hpp"

class DetectNode : public rclcpp::Node
{
public:
    DetectNode() : Node("detect_node")
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/detected_image");

        std::string config_dir = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());

        cfg_ = std::make_unique<Config>(config_dir);
        pipeline_ = std::make_unique<DetectPipeline>(*cfg_);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
        armor_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::DetectionArray>("/armor_detections", 10);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10,
            std::bind(&DetectNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "DetectNode 初始化完成，等待图像输入...");
    }


private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            std::vector<Result> results = pipeline_->process(frame);
            drawDetect(frame, results, cfg_->model.classNames);
            cv::Mat resize_frame = frame;
            int target_width = 1280;
            if(frame.cols > target_width) {
                 double scale = static_cast<double>(target_width) / frame.cols;
                 int target_height = static_cast<int>(frame.rows * scale);
                 cv::resize(frame, resize_frame, cv::Size(target_width, target_height));
            }



            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time_).count();
            last_time_ = now;

            double instant_fps = 1.0 / std::max(dt, 1e-6);
            fps_ = 0.9 * fps_ + 0.1 * instant_fps;

            cv::putText(resize_frame, cv::format("FPS: %.1f", fps_),
                        cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0, cv::Scalar(0, 255, 0), 2);

            std_msgs::msg::Header header = msg->header;
            header.frame_id = "detected_frame";

            auto out_msg = cv_bridge::CvImage(header, "bgr8", resize_frame).toImageMsg();
            image_pub_->publish(*out_msg);

            auto armor_msg = std::make_shared<tensorrt_detect_msgs::msg::DetectionArray>();
            armor_msg->header = msg->header;   // 复用图像时间戳，方便下游同步
            armor_msg->header.frame_id = "detection";

            for (const auto& res : results) {
                tensorrt_detect_msgs::msg::DetectionBox box;
                box.idx         = res.idx;
                box.confidence  = res.confidence;
                box.x           = res.box.x;
                box.y           = res.box.y;
                box.width       = res.box.width;
                box.height      = res.box.height;
                box.armor_color = res.armorColor;
                box.car_x       = res.car_box.x;
                box.car_y       = res.car_box.y;
                box.car_width   = res.car_box.width;
                box.car_height  = res.car_box.height;
                box.world_x     = res.worldPoint.x;
                box.world_y     = res.worldPoint.y;
                box.fps         = static_cast<float>(fps_);

                armor_msg->detections.push_back(box);
            }

            armor_pub_->publish(*armor_msg);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "检测到 %zu 个目标，fps: %.1f，发布了 DetectionArray 消息",
                results.size(), fps_);
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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_pub_;
    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
    double fps_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
