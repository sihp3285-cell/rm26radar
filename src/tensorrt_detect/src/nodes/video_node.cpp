/**
 * @file video_node.cpp
 * @brief 文件视频输入 component，作为工业相机的可替换数据源发布 /image_raw。
 *
 * VideoCapture 在专用线程中按参数/文件 FPS 节流读帧，再把 BGR cv::Mat
 * 转成 sensor_msgs/Image；视频播放完毕自动关闭节点（不再循环播放）。
 * cv_bridge::toImageMsg 会把像素写入 ROS 消息，发布的消息不依赖循环内局部
 * cv::Mat 的后续生命周期。Node 析构时先停止并 join 线程，再释放文件句柄，避免
 * component 被卸载后线程继续访问 this。
 */
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
    
    // 多线程控制变量，用于控制视频捕获线程的运行
    std::thread capture_thread_;// 视频捕获线程，独立于主线程运行
    std::atomic<bool> is_running_;
    
    // 视频帧率控制,保存目标帧率和每帧的理论处理时间
    double fps_;
    int frame_delay_ms_;
    bool synthetic_stamp_ = false;   // 用内容锚定的合成时间戳发布（帧号可恢复、跨运行确定）
    std::int64_t frame_index_ = 0;   // 已发布的帧计数

    /** 独立线程按目标 FPS 解码视频并发布带当前 ROS 时间的 Image；EOF 时关闭节点。 */
    void captureLoop()
    {   
        // 只要 ROS 正常运行且标志位为 true，就持续读取
        while(rclcpp::ok() && is_running_)
        {   
            auto start_time = std::chrono::steady_clock::now();

            cv::Mat frame;
            cap_ >> frame;

            // 视频播放完毕：不再循环回卷，直接关闭项目（组件容器/独立进程随之退出）
            if(frame.empty()) {
                RCLCPP_INFO(this->get_logger(), "视频播放完毕，关闭项目...");
                is_running_ = false;
                // 给 DDS 一点时间把最后一帧消息发出去，再关闭全局上下文
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                rclcpp::shutdown();
                break;
            }

            // unique_ptr 让 rclcpp 在启用 intra-process 时可以转移消息所有权；
            // 但下一步 cv_bridge 把 cv::Mat 像素填入 Image，那里仍是一次深拷贝。
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            if (synthetic_stamp_) {
                // 合成时间戳：第 k 帧固定为 (k+1)*frame_delay_ms，内容锚定、跨运行
                // 一致，下游可用它恢复视频帧号做确定性采样（见 detect_node 的
                // frame_sampling_* 参数）。仅视频模式使用；相机模式不受影响。
                msg->header.stamp = rclcpp::Time(
                    0, (frame_index_ + 1) * frame_delay_ms_ * 1000000LL);
                frame_index_++;
            } else {
                msg->header.stamp = this->now();
            }
            
            // 为了后续和真实相机无缝切换，通常把 frame_id 设为一样
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
    /** 打开视频文件、确定播放 FPS、创建 publisher 并启动解码线程。 */
    explicit VideoNode(const rclcpp::NodeOptions & options)
    : Node("video_node", options), is_running_(false)
    {
        // 1. 声明并获取参数
        this->declare_parameter<std::string>("video_path", "/home/delphine/rm/car_project/test/008.mp4");
        this->declare_parameter<std::string>("topic_name", "/image_raw");
        this->declare_parameter<int>("fps", 0);
        this->declare_parameter<bool>("synthetic_stamp", false);

        std::string video_path = this->get_parameter("video_path").as_string();
        std::string topic_name = this->get_parameter("topic_name").as_string();
        int fps_param = this->get_parameter("fps").as_int();
        synthetic_stamp_ = this->get_parameter("synthetic_stamp").as_bool();

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

        // 图像是高频实时流，depth=1 只保留最新待处理帧，防止检测速度不足时
        // 历史帧排队并累积端到端延迟。这里未显式 best_effort，使用默认 Reliable。
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, rclcpp::QoS(1));

        // 5. 启动读取线程
        is_running_ = true;
        capture_thread_ = std::thread(&VideoNode::captureLoop, this);
    }

    /** 请求解码线程停止并 join，保证 VideoCapture 不在析构后继续访问。 */
    ~VideoNode() 
    {
        is_running_ = false;
        if(capture_thread_.joinable()) {
            capture_thread_.join();
        }
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "视频流子线程已安全关闭。");
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(VideoNode)

/** 非 component 调试入口；正式 launch 与 Detect/Pose/Map 一同装入组件容器。 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}