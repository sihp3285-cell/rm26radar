#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <chrono>

class VideoNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    cv::VideoCapture cap_;
    
    // 多线程控制变量
    std::thread capture_thread_;
    std::atomic<bool> is_running_;
    
    // 视频帧率控制
    double fps_;
    int frame_delay_ms_;

    void captureLoop()
    {   
        // 只要 ROS 正常运行且标志位为 true，就持续读取
        while(rclcpp::ok() && is_running_)
        {   
            auto start_time = std::chrono::steady_clock::now();

            cv::Mat frame;
            cap_ >> frame;

            // 【Debug 利器：循环播放】如果视频读完，把进度条拉回第 0 帧
            if(frame.empty()) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }

            // 【零拷贝构造】
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = this->now();
            
            // 注意：这里为了后续和真实相机无缝切换，通常把 frame_id 设为一样
            // 或者在 Launch 文件中统一配置，这里先用 video_frame
            msg->header.frame_id = "video_frame";

            // 将 cv::Mat 数据拷贝到 msg
            cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg(*msg);

            // 零拷贝发布
            pub_->publish(std::move(msg));

            // 【核心逻辑：帧率控制补偿】
            auto end_time = std::chrono::steady_clock::now();
            auto process_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            
            // 计算需要休眠的时间 = 理论每帧耗时 - 刚刚处理这张图的耗时
            int sleep_time = frame_delay_ms_ - process_time;
            if (sleep_time > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
            }
        }
    }

public:
    explicit VideoNode(const rclcpp::NodeOptions & options)
    : Node("video_node", options), is_running_(false)
    {
        // 1. 声明并获取参数
        this->declare_parameter<std::string>("video_path", "/home/delphine/rm/car_project/test/008.mp4");
        this->declare_parameter<std::string>("topic_name", "/image_raw");
        this->declare_parameter<int>("fps", 0);

        std::string video_path = this->get_parameter("video_path").as_string();
        std::string topic_name = this->get_parameter("topic_name").as_string();
        int fps_param = this->get_parameter("fps").as_int();

        // 2. 打开视频
        cap_.open(video_path);
        if(!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path.c_str());
            return;
        }

        // 3. 获取视频原生属性并决定 FPS
        double native_fps = cap_.get(cv::CAP_PROP_FPS);
        if (fps_param > 0) {
            fps_ = static_cast<double>(fps_param);
            RCLCPP_INFO(this->get_logger(), "使用用户指定帧率: %.2f FPS (原生帧率: %.2f)", fps_, native_fps);
        } else {
            fps_ = (native_fps > 0) ? native_fps : 30.0;
            RCLCPP_INFO(this->get_logger(), "使用视频原生帧率: %.2f FPS", fps_);
        }
        frame_delay_ms_ = static_cast<int>(1000.0 / fps_);
        
        RCLCPP_INFO(this->get_logger(), "视频加载成功 | 分辨率: %dx%d | 实际FPS: %.2f | 发布话题: %s", 
                    (int)cap_.get(cv::CAP_PROP_FRAME_WIDTH),
                    (int)cap_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    fps_,
                    topic_name.c_str());

        // 4. 创建发布者 (与项目其他节点 QoS 保持一致：Keep Last 1)
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, rclcpp::QoS(1));

        // 5. 启动读取线程
        is_running_ = true;
        capture_thread_ = std::thread(&VideoNode::captureLoop, this);
    }

    ~VideoNode() 
    {
        is_running_ = false;
        if(capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if(cap_.isOpened()) {
            cap_.release();
        }
        RCLCPP_INFO(this->get_logger(), "视频流子线程已安全关闭。");
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(VideoNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
