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
#include <filesystem>
#include <iostream>
#include <string>
#include <cstddef>
#include <algorithm>
#include <cmath>

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
    std::unique_ptr<sdk::CameraExmple<sdk::HikCamera>> hik_;
#endif

#ifdef RB26SDK_HAS_DAHENG
    std::unique_ptr<sdk::CameraExmple<sdk::DahengCamera>> daheng_;
#endif

    // ── ROS 发布者 ──
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    // ── 多线程 ──
    std::thread capture_thread_;
    std::thread record_thread_;
    std::atomic<bool> is_running_{false};

    // ── 内录缓冲 ──
    std::queue<cv::Mat> record_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    cv::VideoWriter writer_;

    // 录制队列最大长度（运行时可配，默认值仅作兜底）。
    // 5472x3648 的一帧 BGR 大约 60MB，队列过大时内存快速上涨。
    size_t max_queue_size_ = 8;

    // ── 状态标志 ──
    bool camera_opened_   = false;
    bool sdk_initialized_ = false;

    // ── 参数 ──
    std::atomic<bool> enable_recording_{false};
    int frame_width_  = 0;
    int frame_height_ = 0;

    // ── GStreamer MP4 录制参数 ──
    std::string record_file_path_;
    double record_fps_ = 30.0;
    int record_bitrate_kbps_ = 15000;
    cv::Size record_frame_size_;
    std::atomic<size_t> dropped_record_frames_{0};

    // ── 帧率统计 ──
    std::chrono::steady_clock::time_point last_fps_print_;
    int frame_count_since_print_ = 0;

    // ── ROS frame_id ──
    std::string frame_id_;

private:
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

    // ──────────────────────────────────────────────
    // 工具：GStreamer pipeline 字符串中的路径转义
    // ──────────────────────────────────────────────
    static std::string quoteGstString(const std::string& s)
    {
        std::string out;
        out.reserve(s.size() + 2);
        out.push_back('"');

        for (char c : s) {
            if (c == '\\' || c == '"') {
                out.push_back('\\');
            }
            out.push_back(c);
        }

        out.push_back('"');
        return out;
    }

    // ──────────────────────────────────────────────
    // 工具：保证编码尺寸为偶数
    //
    // H.264 / MP4 编码通常更喜欢偶数宽高。
    // 如果相机输出奇数宽高，直接写入可能失败。
    // ──────────────────────────────────────────────
    static cv::Size makeEvenSize(const cv::Size& size)
    {
        int w = size.width;
        int h = size.height;

        if (w % 2 != 0) {
            --w;
        }

        if (h % 2 != 0) {
            --h;
        }

        return cv::Size(w, h);
    }

    // ──────────────────────────────────────────────
    // 工具：统一转为 CV_8UC3 BGR
    //
    // GStreamer appsrc caps 声明的是 video/x-raw,format=BGR。
    // 因此 writer_.write() 进去的 Mat 必须是 CV_8UC3 BGR。
    // ──────────────────────────────────────────────
    static cv::Mat normalizeToBgr8(const cv::Mat& src)
    {
        if (src.empty()) {
            return {};
        }

        cv::Mat src8;

        if (src.depth() == CV_8U) {
            src8 = src;
        } else {
            src.convertTo(src8, CV_8U);
        }

        cv::Mat bgr;

        if (src8.channels() == 1) {
            cv::cvtColor(src8, bgr, cv::COLOR_GRAY2BGR);
        } else if (src8.channels() == 3) {
            bgr = src8;
        } else if (src8.channels() == 4) {
            cv::cvtColor(src8, bgr, cv::COLOR_BGRA2BGR);
        } else {
            return {};
        }

        if (!bgr.isContinuous()) {
            bgr = bgr.clone();
        }

        return bgr;
    }

    // ──────────────────────────────────────────────
    // 初始化 OpenCV GStreamer Writer
    //
    // 注意：
    //   1. 使用 cv::CAP_GSTREAMER；
    //   2. 不做 FFmpeg fallback，GStreamer 失败就直接报错；
    //   3. 第一帧到达后，按真实帧尺寸初始化 writer；
    //   4. appsrc 后明确声明 BGR / width / height / framerate。
    // ──────────────────────────────────────────────
    bool openGStreamerWriter(const cv::Size& first_frame_size)
    {
        if (record_file_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "录制文件路径为空，无法初始化 GStreamer Writer。");
            return false;
        }

        cv::Size even_size = makeEvenSize(first_frame_size);

        if (even_size.width <= 0 || even_size.height <= 0) {
            RCLCPP_ERROR(this->get_logger(),
                "非法录制尺寸: input=%dx%d even=%dx%d",
                first_frame_size.width,
                first_frame_size.height,
                even_size.width,
                even_size.height);
            return false;
        }

        record_frame_size_ = even_size;

        int fps_int = static_cast<int>(std::round(record_fps_));
        fps_int = std::clamp(fps_int, 1, 240);

        int key_int_max = std::max(1, fps_int);

        std::string pipeline =
            "appsrc is-live=true block=true format=time do-timestamp=true ! "
            "video/x-raw,format=BGR,width=" + std::to_string(record_frame_size_.width) +
            ",height=" + std::to_string(record_frame_size_.height) +
            ",framerate=" + std::to_string(fps_int) + "/1 ! "
            "queue max-size-buffers=4 leaky=downstream ! "
            "videoconvert n-threads=4 ! "
            "video/x-raw,format=I420 ! "
            "x264enc bitrate=" + std::to_string(record_bitrate_kbps_) +
            " speed-preset=ultrafast tune=zerolatency key-int-max=" + std::to_string(key_int_max) +
            " bframes=0 ! "
            "video/x-h264,profile=baseline ! "
            "h264parse config-interval=-1 ! "
            "mp4mux faststart=true ! "
            "filesink location=" + quoteGstString(record_file_path_) + " sync=false";

        RCLCPP_INFO(this->get_logger(),
            "初始化 GStreamer MP4 Writer | size=%dx%d | fps=%d/1 | bitrate=%d kbps | path=%s",
            record_frame_size_.width,
            record_frame_size_.height,
            fps_int,
            record_bitrate_kbps_,
            record_file_path_.c_str());

        RCLCPP_INFO(this->get_logger(),
            "GStreamer pipeline: %s",
            pipeline.c_str());

        bool ok = writer_.open(
            pipeline,
            cv::CAP_GSTREAMER,
            0,
            static_cast<double>(fps_int),
            record_frame_size_,
            true
        );

        if (ok && writer_.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "GStreamer MP4 Writer 打开成功。");
            return true;
        }

        RCLCPP_ERROR(this->get_logger(),
            "GStreamer MP4 Writer 打开失败。请检查 OpenCV GStreamer backend、x264enc、h264parse、mp4mux。");

        return false;
    }

    // ──────────────────────────────────────────────
    // 清空录制队列
    // ──────────────────────────────────────────────
    void clearRecordQueue()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::queue<cv::Mat> empty;
        std::swap(record_queue_, empty);
    }

    // ──────────────────────────────────────────────
    // 录制线程
    //
    // 生产者：captureLoop()
    // 消费者：recordLoop()
    //
    // 逻辑：
    //   1. 等待采集线程推入 BGR8 帧；
    //   2. 第一帧到达后，按真实尺寸打开 GStreamer writer；
    //   3. 写入剩余所有帧；
    //   4. 退出时 release writer，保证 MP4 正常 finalize。
    // ──────────────────────────────────────────────
    void recordLoop()
    {
        size_t written_frames = 0;

        while (true) {
            cv::Mat frame_to_write;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);

                queue_cv_.wait(lock, [this] {
                    return !record_queue_.empty() || !is_running_.load();
                });

                if (record_queue_.empty() && !is_running_.load()) {
                    break;
                }

                if (record_queue_.empty()) {
                    continue;
                }

                frame_to_write = std::move(record_queue_.front());
                record_queue_.pop();
            }

            if (frame_to_write.empty()) {
                continue;
            }

            // 第一帧到达后再初始化 GStreamer writer。
            if (!writer_.isOpened()) {
                if (!openGStreamerWriter(frame_to_write.size())) {
                    enable_recording_.store(false);
                    clearRecordQueue();
                    break;
                }
            }

            // 尺寸必须和 GStreamer caps / writer 初始化尺寸一致。
            if (frame_to_write.size() != record_frame_size_) {
                cv::resize(frame_to_write, frame_to_write, record_frame_size_);
            }

            // 兜底：确保仍然是 BGR8。
            if (frame_to_write.depth() != CV_8U || frame_to_write.channels() != 3) {
                frame_to_write = normalizeToBgr8(frame_to_write);

                if (frame_to_write.empty()) {
                    continue;
                }

                if (frame_to_write.size() != record_frame_size_) {
                    cv::resize(frame_to_write, frame_to_write, record_frame_size_);
                }
            }

            if (!frame_to_write.isContinuous()) {
                frame_to_write = frame_to_write.clone();
            }

            writer_.write(frame_to_write);
            ++written_frames;
        }

        if (writer_.isOpened()) {
            writer_.release();
        }

        RCLCPP_INFO(this->get_logger(),
            "录制已停止，文件已安全关闭 | written=%zu dropped=%zu path=%s",
            written_frames,
            dropped_record_frames_.load(),
            record_file_path_.c_str());
    }

    // ──────────────────────────────────────────────
    // 统一取帧接口
    // ──────────────────────────────────────────────
    cv::Mat grabFrame()
    {
        switch (brand_) {
#ifdef RB26SDK_HAS_HIK
        case Brand::Hik:
            return hik_->getFrame(false, false);
#endif

#ifdef RB26SDK_HAS_DAHENG
        case Brand::Daheng:
            return daheng_->getFrame(false, false);
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

        bool printed_frame_info = false;

        while (rclcpp::ok() && is_running_.load()) {
            cv::Mat frame = grabFrame();

            if (frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            cv::Mat bgr_frame = normalizeToBgr8(frame);

            if (bgr_frame.empty()) {
                RCLCPP_WARN(this->get_logger(),
                    "获取到不支持的相机帧格式: size=%dx%d channels=%d depth=%d type=%d",
                    frame.cols,
                    frame.rows,
                    frame.channels(),
                    frame.depth(),
                    frame.type());
                continue;
            }

            if (!printed_frame_info) {
                RCLCPP_INFO(this->get_logger(),
                    "相机实际输出帧: %dx%d channels=%d depth=%d type=%d | SDK声明分辨率: %dx%d",
                    bgr_frame.cols,
                    bgr_frame.rows,
                    bgr_frame.channels(),
                    bgr_frame.depth(),
                    bgr_frame.type(),
                    frame_width_,
                    frame_height_);
                printed_frame_info = true;
            }

            // 1. 内录：推入 BGR8 帧
            if (enable_recording_.load()) {
                bool pushed = false;

                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);

                    if (record_queue_.size() < max_queue_size_) {
                        record_queue_.push(bgr_frame.clone());
                        pushed = true;
                    }
                }

                if (pushed) {
                    queue_cv_.notify_one();
                } else {
                    size_t dropped = ++dropped_record_frames_;

                    if (dropped == 1 || dropped % 300 == 0) {
                        RCLCPP_WARN(this->get_logger(),
                            "录制队列已满，丢弃录制帧 | dropped=%zu queue_max=%zu",
                            dropped,
                            max_queue_size_);
                    }
                }
            }

            // 2. 发布 ROS 图像
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp    = this->now();
            msg->header.frame_id = frame_id_;

            cv_bridge::CvImage(msg->header, "bgr8", bgr_frame).toImageMsg(*msg);
            pub_->publish(std::move(msg));

            // 3. 帧率日志
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

public:
    explicit CameraNode(const rclcpp::NodeOptions &options)
        : Node("camera_node", options)
    {
        // ──────── 1. 声明参数 ────────

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
        this->declare_parameter<std::string>("record_path", "/home/delphine/rm/recording/");
        this->declare_parameter<double>     ("record_fps", 30.0);
        this->declare_parameter<int>        ("record_bitrate_kbps", 15000);
        this->declare_parameter<int>        ("max_record_queue_size", 8);

        // ──────── 2. 读取参数 ────────
        std::string camera_brand = this->get_parameter("camera_brand").as_string();
        std::string sn           = this->get_parameter("camera_sn").as_string();
        bool auto_wb             = this->get_parameter("auto_white_balance").as_bool();
        int  exposure            = this->get_parameter("exposure_time").as_int();
        double gain_val          = this->get_parameter("gain").as_double();
        double gamma_val         = this->get_parameter("gamma").as_double();

        std::string topic        = this->get_parameter("topic_name").as_string();
        frame_id_                = this->get_parameter("frame_id").as_string();

        enable_recording_.store(this->get_parameter("enable_recording").as_bool());
        std::string record_base  = this->get_parameter("record_path").as_string();
        record_fps_              = this->get_parameter("record_fps").as_double();
        record_bitrate_kbps_     = this->get_parameter("record_bitrate_kbps").as_int();

        if (record_fps_ <= 1.0 || record_fps_ > 240.0) {
            RCLCPP_WARN(this->get_logger(),
                "record_fps=%.2f 不合理，自动改为 30.0", record_fps_);
            record_fps_ = 30.0;
        }

        if (record_bitrate_kbps_ < 1000 || record_bitrate_kbps_ > 200000) {
            RCLCPP_WARN(this->get_logger(),
                "record_bitrate_kbps=%d 不合理，自动改为 15000",
                record_bitrate_kbps_);
            record_bitrate_kbps_ = 15000;
        }

        int max_q = this->get_parameter("max_record_queue_size").as_int();
        if (max_q < 1 || max_q > 64) {
            RCLCPP_WARN(this->get_logger(),
                "max_record_queue_size=%d 不合理，自动改为 8", max_q);
            max_queue_size_ = 8;
        } else {
            max_queue_size_ = static_cast<size_t>(max_q);
        }

        // ──────── 3. 初始化相机 ────────
        bool init_ok = false;

#ifdef RB26SDK_HAS_HIK
        if (camera_brand == "hik" || camera_brand == "Hik") {
            brand_ = Brand::Hik;
            hik_ = std::make_unique<sdk::CameraExmple<sdk::HikCamera>>();

            sdk_initialized_ = sdk::CameraExmple<sdk::HikCamera>::CameraSDKInit();

            if (sdk_initialized_) {
                init_ok = hik_->CameraInit(
                    const_cast<char*>(sn.c_str()),
                    auto_wb,
                    exposure,
                    gain_val,
                    gamma_val
                );
            }

            if (init_ok) {
                camera_opened_ = true;
            }

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
                    auto_wb,
                    exposure,
                    gain_val,
                    gamma_val
                );
            }

            if (init_ok) {
                camera_opened_ = true;
            }

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
                camera_brand.c_str(),
                sn.c_str(),
                sdk_initialized_,
                camera_opened_);

            throw std::runtime_error("CameraNode: 相机初始化失败");
        }

        // 读取 SDK 声明分辨率，仅用于日志。
        // 真正录制尺寸会在第一帧到达后，以实际帧尺寸为准。
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
            "相机就绪 | brand=%s sn=%s | SDK声明分辨率: %dx%d | 曝光=%dus gain=%.2f gamma=%.2f",
            camera_brand.c_str(),
            sn.c_str(),
            frame_width_,
            frame_height_,
            exposure,
            gain_val,
            gamma_val);

        // ──────── 4. 准备 GStreamer MP4 内录路径 ────────
        //
        // 注意：
        //   这里只准备路径，不启动录制线程，也不打开 writer。
        //   录制线程必须等 is_running_=true 后再启动。
        //   writer 必须等第一帧到达后再打开。
        //
        if (enable_recording_.load()) {
            try {
                if (record_base.empty()) {
                    record_base = ".";
                }

                std::filesystem::path record_dir(record_base);
                std::filesystem::create_directories(record_dir);

                record_file_path_ =
                    (record_dir / ("cam_" + getCurrentTimeString() + ".mp4")).string();

                RCLCPP_INFO(this->get_logger(),
                    "GStreamer MP4 内录准备就绪 | fps=%.2f | bitrate=%d kbps | path=%s",
                    record_fps_,
                    record_bitrate_kbps_,
                    record_file_path_.c_str());

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                    "创建录制目录或初始化录制路径失败: %s", e.what());
                enable_recording_.store(false);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "内录已禁用 (enable_recording=false)");
        }

        // ──────── 5. 创建发布者 ────────
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic,
            rclcpp::QoS(1)
        );

        // ──────── 6. 启动线程 ────────
        //
        // 关键顺序：
        //   1. is_running_ = true
        //   2. 启动 record_thread_
        //   3. 启动 capture_thread_
        //
        // 之前 written=0 的问题，就是 record_thread_ 启动时 is_running_ 还是 false。
        //
        if (camera_opened_) {
            is_running_.store(true);
            last_fps_print_ = std::chrono::steady_clock::now();

            if (enable_recording_.load()) {
                record_thread_ = std::thread(&CameraNode::recordLoop, this);
                RCLCPP_INFO(this->get_logger(), "录制线程已启动。");
            }

            capture_thread_ = std::thread(&CameraNode::captureLoop, this);

            RCLCPP_INFO(this->get_logger(),
                "CameraNode 已启动，发布话题: %s (frame_id: %s)",
                topic.c_str(),
                frame_id_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(),
                "CameraNode 未启动采集线程（相机未打开），仅创建话题: %s",
                topic.c_str());
        }
    }

    ~CameraNode() override
    {
        // 1. 通知线程停止
        is_running_.store(false);
        queue_cv_.notify_all();

        // 2. 先停采集线程
        //
        // capture_thread_ 是生产者。
        // 必须先停止生产者，防止录制线程退出后还有新帧入队。
        //
        if (capture_thread_.joinable()) {
            capture_thread_.join();
            RCLCPP_DEBUG(this->get_logger(), "采集线程已结束。");
        }

        // 3. 再停录制线程
        //
        // recordLoop 会在 is_running_=false 后继续消费队列里的剩余帧。
        // 队列清空后 release writer，MP4 文件才会有完整索引。
        //
        queue_cv_.notify_all();

        if (record_thread_.joinable()) {
            record_thread_.join();
            RCLCPP_DEBUG(this->get_logger(), "录制线程已结束。");
        }

        // 4. 释放相机资源
#ifdef RB26SDK_HAS_HIK
        hik_.reset();
#endif

#ifdef RB26SDK_HAS_DAHENG
        daheng_.reset();
#endif

        RCLCPP_INFO(this->get_logger(),
            "CameraNode 已安全关闭 | camera_was_open=%d sdk_was_init=%d",
            camera_opened_,
            sdk_initialized_);
    }
};

} // namespace tensorrt_detect

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_detect::CameraNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        // 从编译期注入的配置文件路径加载所有参数。
        // 命令行仍可通过 --ros-args --params-file <other.yaml> 覆盖。
        rclcpp::NodeOptions options;

#ifdef CAMERA_PARAMS_FILE
        // 如果配置文件存在则自动加载（源码树开发模式），
        // 安装后的正式部署请使用 launch 文件或 --ros-args --params-file。
        if (std::filesystem::exists(CAMERA_PARAMS_FILE)) {
            options.arguments(
                {"--ros-args", "--params-file", CAMERA_PARAMS_FILE});
        }
#endif

        auto node = std::make_shared<tensorrt_detect::CameraNode>(options);

        rclcpp::spin(node);

    } catch (const std::exception& e) {
        std::cerr << "[CameraNode] 致命错误，进程退出: "
                  << e.what()
                  << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}