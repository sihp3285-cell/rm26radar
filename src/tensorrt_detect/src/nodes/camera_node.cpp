#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <stdexcept>

#include "CamreaExmple.hpp"  // rb26SDK 的 PUBLIC include 目录已导出

namespace tensorrt_detect
{

class CameraNode : public rclcpp::Node
{
private:
    // ── 相机后端选择 ──
    enum class Brand {
#ifdef RB26SDK_HAS_HIK
        Hik,
#endif
#ifdef RB26SDK_HAS_DAHENG
        Daheng,
#endif
    };
    Brand brand_;
#ifdef RB26SDK_HAS_HIK
    std::unique_ptr<sdk::CameraExmple<sdk::HikCamera>>     hik_;
#endif
#ifdef RB26SDK_HAS_DAHENG
    std::unique_ptr<sdk::CameraExmple<sdk::DahengCamera>>  daheng_;
#endif

    // ── ROS 发布者 ──
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    // ── 多线程 ──
    std::thread capture_thread_;
    std::thread record_thread_;
    std::atomic<bool> is_running_;

    // ── 内录缓冲 ──
    std::queue<cv::Mat> record_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    cv::VideoWriter writer_;
    static constexpr size_t MAX_QUEUE_SIZE = 60;

    // ── 状态标志 ──
    bool camera_opened_   = false;  // CameraInit() 成功，设备已打开并取流
    bool sdk_initialized_ = false;  // CameraSDKInit() 已调用成功

    // ── 参数 ──
    bool enable_recording_ = false;
    int frame_width_  = 0;
    int frame_height_ = 0;

    // ── 帧率统计 ──
    std::chrono::steady_clock::time_point last_fps_print_;
    int frame_count_since_print_ = 0;

    // ──────────────────────────────────────────────
    // 录制线程
    // ──────────────────────────────────────────────
    void recordLoop()
    {
        while (is_running_ || !record_queue_.empty()) {
            cv::Mat frame_to_write;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] {
                    return !record_queue_.empty() || !is_running_;
                });

                if (record_queue_.empty() && !is_running_) break;

                frame_to_write = std::move(record_queue_.front());
                record_queue_.pop();
            }

            if (!frame_to_write.empty() && writer_.isOpened()) {
                writer_.write(frame_to_write);
            }
        }
        if (writer_.isOpened()) writer_.release();
        RCLCPP_INFO(this->get_logger(), "录制已停止，文件已安全关闭。");
    }

    // ──────────────────────────────────────────────
    // 统一的取帧接口（隐藏品牌差异）
    // ──────────────────────────────────────────────
    cv::Mat grabFrame()
    {
        switch (brand_) {
#ifdef RB26SDK_HAS_HIK
        case Brand::Hik:    return hik_->getFrame(false, false);
#endif
#ifdef RB26SDK_HAS_DAHENG
        case Brand::Daheng: return daheng_->getFrame(false, false);
#endif
        }
        return {};
    }

    // ──────────────────────────────────────────────
    // 主采集 + 发布线程
    // ──────────────────────────────────────────────
    void captureLoop()
    {
        if (!camera_opened_) {
            RCLCPP_WARN(this->get_logger(), "captureLoop 启动但相机未打开，立即退出。");
            return;
        }
        while (rclcpp::ok() && is_running_) {
            cv::Mat frame = grabFrame();
            if (frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // 1. 内录
            if (enable_recording_) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (record_queue_.size() < MAX_QUEUE_SIZE) {
                    record_queue_.push(frame.clone());
                }
                queue_cv_.notify_one();
            }

            // 2. 发布 ROS 消息（零拷贝）
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp    = this->now();
            msg->header.frame_id = frame_id_;
            cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg(*msg);
            pub_->publish(std::move(msg));

            // 3. 帧率日志（每 5 秒输出一次）
            frame_count_since_print_++;
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_fps_print_).count();
            if (elapsed >= 5.0) {
                double fps = frame_count_since_print_ / elapsed;
                RCLCPP_DEBUG(this->get_logger(), "采集帧率: %.1f fps", fps);
                frame_count_since_print_ = 0;
                last_fps_print_ = now;
            }
        }
    }

    // ──────────────────────────────────────────────
    // 工具：时间字符串
    // ──────────────────────────────────────────────
    static std::string getCurrentTimeString()
    {
        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    // ── 保存参数变量 ──
    std::string frame_id_;

public:
    explicit CameraNode(const rclcpp::NodeOptions &options)
        : Node("camera_node", options), is_running_(false)
    {
        // ──────── 1. 声明并获取参数 ────────
        // 相机通用参数
        this->declare_parameter<std::string>("camera_brand",        "hik");
        this->declare_parameter<std::string>("camera_sn",           "DA7831910");
        this->declare_parameter<bool>       ("auto_white_balance",  true);
        this->declare_parameter<int>        ("exposure_time",       10000);
        this->declare_parameter<double>     ("gain",                0.7);
        this->declare_parameter<double>     ("gamma",               0.3);

        // 输出话题
        this->declare_parameter<std::string>("topic_name", "/image_raw");
        this->declare_parameter<std::string>("frame_id",  "camera_frame");

        // 内录
        this->declare_parameter<bool>       ("enable_recording", false);
        this->declare_parameter<std::string>("record_path",
            "/home/delphine/rm/recording/");

        // ──────── 读取参数 ────────
        std::string camera_brand = this->get_parameter("camera_brand").as_string();
        std::string sn           = this->get_parameter("camera_sn").as_string();
        bool auto_wb             = this->get_parameter("auto_white_balance").as_bool();
        int  exposure            = this->get_parameter("exposure_time").as_int();
        double gain_val          = this->get_parameter("gain").as_double();
        double gamma_val         = this->get_parameter("gamma").as_double();

        std::string topic        = this->get_parameter("topic_name").as_string();
        frame_id_                = this->get_parameter("frame_id").as_string();

        enable_recording_ = this->get_parameter("enable_recording").as_bool();
        std::string record_base  = this->get_parameter("record_path").as_string();

        // ──────── 2. 选择并初始化相机 ────────
        bool init_ok = false;

#ifdef RB26SDK_HAS_HIK
        if (camera_brand == "hik" || camera_brand == "Hik") {
            brand_ = Brand::Hik;
            hik_ = std::make_unique<sdk::CameraExmple<sdk::HikCamera>>();
            sdk_initialized_ = sdk::CameraExmple<sdk::HikCamera>::CameraSDKInit();
            if (sdk_initialized_) {
                init_ok = hik_->CameraInit(
                    const_cast<char*>(sn.c_str()),
                    auto_wb, exposure, gain_val, gamma_val);
            }
            if (init_ok) camera_opened_ = true;

        } else
#endif
#ifdef RB26SDK_HAS_DAHENG
        if (camera_brand == "daheng" || camera_brand == "Daheng") {
            brand_ = Brand::Daheng;
            daheng_ = std::make_unique<sdk::CameraExmple<sdk::DahengCamera>>();
            sdk_initialized_ = sdk::CameraExmple<sdk::DahengCamera>::CameraSDKInit();
            if (sdk_initialized_) {
                init_ok = daheng_->CameraInit(
                    const_cast<char*>(sn.c_str()),
                    auto_wb, exposure, gain_val, gamma_val);
            }
            if (init_ok) camera_opened_ = true;
        } else
#endif
        {
            RCLCPP_FATAL(this->get_logger(),
                "未知 camera_brand: '%s' (支持 hik / daheng)", camera_brand.c_str());
            throw std::runtime_error("CameraNode: 未知 camera_brand");
        }

        if (!init_ok) {
            RCLCPP_FATAL(this->get_logger(),
                "相机初始化失败！brand=%s sn=%s | sdk_ok=%d camera_open=%d",
                camera_brand.c_str(), sn.c_str(), sdk_initialized_, camera_opened_);
            throw std::runtime_error("CameraNode: 相机初始化失败");
        }

        // 读取实际分辨率
        switch (brand_) {
#ifdef RB26SDK_HAS_HIK
        case Brand::Hik:
            frame_width_  = hik_->sensorWidth;
            frame_height_ = hik_->sensorHeight;
            break;
#endif
#ifdef RB26SDK_HAS_DAHENG
        case Brand::Daheng:
            frame_width_  = daheng_->sensorWidth;
            frame_height_ = daheng_->sensorHeight;
            break;
#endif
        }

        RCLCPP_INFO(this->get_logger(),
            "相机就绪 | brand=%s sn=%s | 分辨率: %dx%d | 曝光=%dus gain=%.2f gamma=%.2f",
            camera_brand.c_str(), sn.c_str(),
            frame_width_, frame_height_, exposure, gain_val, gamma_val);

        // ──────── 3. 初始化内录（可选） ────────
        if (enable_recording_) {
            if (!record_base.empty() && record_base.back() != '/')
                record_base += "/";

            std::string full_path = record_base + "cam_" +
                                    getCurrentTimeString() + ".mp4";

            RCLCPP_INFO(this->get_logger(),
                "初始化 GStreamer 录制: %dx%d @30fps → %s",
                frame_width_, frame_height_, full_path.c_str());

            std::string pipeline =
                "appsrc ! videoconvert ! video/x-raw, format=I420 ! "
                "x264enc bitrate=15000 speed-preset=ultrafast tune=zerolatency ! "
                "h264parse ! mp4mux ! filesink location=" + full_path;

            writer_.open(pipeline, cv::CAP_GSTREAMER, 0, 30.0,
                         cv::Size(frame_width_, frame_height_));

            if (!writer_.isOpened()) {
                RCLCPP_WARN(this->get_logger(),
                    "GStreamer 打开失败，降级为原生 MP4 录制");
                writer_.open(full_path,
                    cv::VideoWriter::fourcc('m','p','4','v'), 30.0,
                    cv::Size(frame_width_, frame_height_));
            }

            if (writer_.isOpened()) {
                RCLCPP_INFO(this->get_logger(),
                    "内录已开启: %s", full_path.c_str());
                record_thread_ = std::thread(&CameraNode::recordLoop, this);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "所有录制管道均开启失败！关闭本次内录功能。");
                enable_recording_ = false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "内录已禁用 (enable_recording=false)");
        }

        // ──────── 4. 创建发布者 & 启动采集线程 ────────
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic, rclcpp::QoS(1));

        if (camera_opened_) {
            is_running_ = true;
            last_fps_print_ = std::chrono::steady_clock::now();
            capture_thread_ = std::thread(&CameraNode::captureLoop, this);
            RCLCPP_INFO(this->get_logger(),
                "CameraNode 已启动，发布话题: %s (frame_id: %s)",
                topic.c_str(), frame_id_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(),
                "CameraNode 未启动采集线程（相机未打开），仅创建话题: %s", topic.c_str());
        }
    }

    ~CameraNode() override
    {
        // 1. 停止采集循环
        is_running_ = false;

        // 2. 停止录制线程（如果开启了录制）
        if (enable_recording_) {
            queue_cv_.notify_all();
            if (record_thread_.joinable()) {
                record_thread_.join();
                RCLCPP_DEBUG(this->get_logger(), "录制线程已结束。");
            }
        }

        // 3. 停止采集线程（如果已启动）
        if (capture_thread_.joinable()) {
            capture_thread_.join();
            RCLCPP_DEBUG(this->get_logger(), "采集线程已结束。");
        }

        // 4. 释放相机资源（unique_ptr 析构 → SDK 析构函数检查 cap_init 后安全释放）
        //    - 相机已打开：正常执行 StopGrabbing → CloseDevice → DestroyHandle → Finalize
        //    - 相机未打开：cap_init=false，跳过设备级清理，仅 Finalize/CloseLib
#ifdef RB26SDK_HAS_HIK
        hik_.reset();
#endif
#ifdef RB26SDK_HAS_DAHENG
        daheng_.reset();
#endif

        RCLCPP_INFO(this->get_logger(),
            "CameraNode 已安全关闭 | camera_was_open=%d sdk_was_init=%d",
            camera_opened_, sdk_initialized_);
    }
};

} // namespace tensorrt_detect

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_detect::CameraNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<tensorrt_detect::CameraNode>(rclcpp::NodeOptions());
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "[CameraNode] 致命错误，进程退出: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
