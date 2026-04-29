#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>

class VideoNode : public rclcpp::Node
{
public:
    VideoNode() : Node("video_node")
    {
        this->declare_parameter<std::string>("video_path", "/home/delphine/rm/car_project/test/005.mp4");
        this->declare_parameter<std::string>("topic_name", "/image_raw");
        this->declare_parameter<int>("fps", 30);

        video_path_ = this->get_parameter("video_path").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        fps_setting_ = this->get_parameter("fps").as_int();

        RCLCPP_INFO(this->get_logger(), "视频路径: %s", video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "设定帧率: %d", fps_setting_);

        cap_.open(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, rclcpp::QoS(1));

        int interval_ms = 1000 / std::max(fps_setting_, 1);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&VideoNode::timer_callback, this));

        pause_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/video_node/set_pause",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response) {
                is_paused_ = request->data;
                response->success = true;
                response->message = request->data ? "视频已暂停" : "视频已恢复";
                RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "VideoNode 初始化完成，开始发布视频帧");
    }

private:
    void timer_callback()
    {
        if (is_paused_.load()) {
            return;
        }

        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "视频播放结束，重新回到开头");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        msg->header.stamp = this->now();
        msg->header.frame_id = "video_frame";

        image_pub_->publish(*msg);
    }

    cv::VideoCapture cap_;
    std::string video_path_;
    std::string topic_name_;
    int fps_setting_;

    std::atomic<bool> is_paused_{false};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
