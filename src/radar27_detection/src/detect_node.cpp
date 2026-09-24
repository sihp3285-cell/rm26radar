/**
 * @file detect_node.cpp
 * @brief 相机/视频 -> 内部有界帧缓冲 -> 检测；原图仅供工具按需读取。
 * 采集、推理、原图/调试图发布和录制独立执行。所有输入像素只读共享，
 * ROI 重载与推理互斥；退出先终止采集和推理，再释放 SDK、模型与配置。
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <optional>
#include <limits>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <cuda_runtime_api.h>

#include "radar27_interfaces/msg/detection_array.hpp"
#include "radar27_interfaces/msg/detection_box.hpp"
#include "radar27_interfaces/msg/pipeline_timing.hpp"
#include <radar27_detection/config.hpp>
#include <radar27_detection/pipeline.hpp>
#include <radar27_detection/draw.hpp>
#include <radar27_detection/input/video_source.hpp>
#include <radar27_detection/input/camera_source.hpp>
#include <radar27_detection/frame_buffer.hpp>
#include <radar27_detection/recorder.hpp>
#include <rm_field/robot_id.hpp>

class DetectNode : public rclcpp::Node
{
    using Clock = std::chrono::steady_clock;
    using Frame = radar27_detection::input::Frame;
    using ReadStatus = radar27_detection::input::ReadStatus;
    struct Packet { Frame frame; std_msgs::msg::Header header; };

public:
    explicit DetectNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("detect_node", options)
    {
        // Initialize GPU before loading models; the detection thread selects the same device.
        checkCuda(cudaFree(nullptr), "CUDA initialization");
        checkCuda(cudaGetDevice(&cuda_device_), "cudaGetDevice");
        const auto config_dir = declare_parameter<std::string>("config_dir", ".");
        const auto roi_path = declare_parameter<std::string>("roi_path",
            (std::filesystem::path(config_dir) / "outpost_roi.yaml").string());
        const auto model_dir = declare_parameter<std::string>("model_dir", "");
        mode_ = declare_parameter<std::string>("input.mode", "video");
        frame_id_ = declare_parameter<std::string>("input.frame_id", "camera_frame");
        sampling_step_ = positiveInt("input.sampling_step", 1);
        const auto playback = declare_parameter<std::string>("video.playback_mode", "realtime");
        if (mode_ != "camera" && mode_ != "video") throw std::invalid_argument("input.mode must be camera or video");
        if (playback != "realtime" && playback != "sequential") throw std::invalid_argument("video.playback_mode must be realtime or sequential");
        sequential_ = mode_ == "video" && playback == "sequential";
        synthetic_stamp_ = declare_parameter<bool>("video.synthetic_stamp", false);
        shutdown_on_eof_ = declare_parameter<bool>("video.shutdown_on_eof", true);
        const auto video_path = declare_parameter<std::string>("video.path", "");
        const auto video_fps = declare_parameter<double>("video.fps", 20.0);

        radar27_detection::input::CameraSourceConfig camera;
        camera.brand = declare_parameter<std::string>("camera.brand", "hik");
        camera.serial_number = declare_parameter<std::string>("camera.sn", "DA7831910");
        camera.auto_white_balance = declare_parameter<bool>("camera.auto_white_balance", true);
        camera.exposure_time_us = positiveInt("camera.exposure_time", 6000);
        camera.gain = declare_parameter<double>("camera.gain", 0.7);
        camera.gamma = declare_parameter<double>("camera.gamma", 0.3);
        camera.timeout_ms = positiveInt("camera.timeout_ms", 1000);
        camera.flip = declare_parameter<bool>("camera.flip", false);
        camera.mirror = declare_parameter<bool>("camera.mirror", false);

        publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
        debug_output_max_width_ = positiveInt("debug_output_max_width", 1280);
        const auto output_topic = declare_parameter<std::string>("output_topic", "/detected_image");
        raw_enabled_ = declare_parameter<bool>("raw_output.enabled", true);
        raw_only_subscribers_ = declare_parameter<bool>("raw_output.only_with_subscribers", true);
        const auto raw_topic = declare_parameter<std::string>("raw_output.topic", "/image_raw");
        const double raw_fps = declare_parameter<double>("raw_output.max_fps", 5.0);
        if (!std::isfinite(raw_fps) || raw_fps <= 0.0 || raw_fps > 1000.0)
            throw std::invalid_argument("raw_output.max_fps must be in (0,1000]");
        raw_period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / raw_fps));
        const bool record = declare_parameter<bool>("recording.enabled", false);
        const auto record_dir = declare_parameter<std::string>("recording.path", "recordings");
        radar27_detection::RecorderConfig recording;
        recording.fps = declare_parameter<double>("recording.fps", 20.0);
        recording.queue_size = positiveInt("recording.queue_size", 8);
        recording.codec = declare_parameter<std::string>("recording.codec", "mp4v");

        cfg_ = std::make_unique<DetectionConfig>(config_dir, roi_path);
        if (!model_dir.empty()) {
            for (auto* path : {&cfg_->model.modelPath, &cfg_->model.armorModelPath,
                              &cfg_->model.classifyModelPath})
                if (!path->empty()) *path = (std::filesystem::path(model_dir) / std::filesystem::path(*path).filename()).string();
        }
        pipeline_ = std::make_unique<DetectPipeline>(*cfg_);
        if (mode_ == "video") {
            source_ = std::make_unique<radar27_detection::input::VideoSource>(
                radar27_detection::input::VideoSourceConfig{video_path, video_fps});
        } else {
            source_ = std::make_unique<radar27_detection::input::CameraSource>(camera);
        }
        source_->open();
        if (mode_ == "video") {
            source_fps_ = static_cast<radar27_detection::input::VideoSource*>(source_.get())->fps();
            video_period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / source_fps_));
            if (video_period_ <= Clock::duration::zero()) throw std::invalid_argument("Video FPS too high");
        }
        if (record) {
            const auto suffix = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            recording.path = (std::filesystem::path(record_dir) /
                (mode_ + "_" + std::to_string(suffix) + ".mp4")).string();
            recorder_ = std::make_unique<radar27_detection::Recorder>(recording);
        }

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, rclcpp::QoS(1));
        if (raw_enabled_) raw_pub_ = create_publisher<sensor_msgs::msg::Image>(raw_topic, rclcpp::QoS(1));
        armor_pub_ = create_publisher<radar27_interfaces::msg::DetectionArray>("/armor_detections", rclcpp::QoS(10).best_effort());
        timing_pub_ = create_publisher<radar27_interfaces::msg::PipelineTiming>("/pipeline_timing", rclcpp::QoS(1));
        reload_roi_service_ = create_service<std_srvs::srv::Trigger>("/detect_node/reload_roi",
            std::bind(&DetectNode::reloadROI, this, std::placeholders::_1, std::placeholders::_2));
        auto pause_callback = [this](std_srvs::srv::SetBool::Request::SharedPtr request,
                                     std_srvs::srv::SetBool::Response::SharedPtr response) {
            setPause(request, response);
        };
        pause_service_ = create_service<std_srvs::srv::SetBool>("/detect_node/set_pause", pause_callback);
        legacy_pause_service_ = create_service<std_srvs::srv::SetBool>("/video_node/set_pause", pause_callback);
        status_service_ = create_service<std_srvs::srv::Trigger>("/detect_node/input_status",
            [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
                std::lock_guard<std::mutex> lock(control_mutex_);
                std::ostringstream status;
                status << "mode=" << mode_ << " paused=" << paused_ << " eof=" << eof_.load()
                       << " done=" << detection_done_.load() << " acquired=" << acquired_.load()
                       << " processed=" << processed_.load() << " sampled_out=" << sampled_out_.load()
                       << " overwritten=" << frames_.overwritten() << " error=" << input_error_;
                if (recorder_) status << " recorded=" << recorder_->written() << " record_dropped=" << recorder_->dropped()
                                     << " record_error=" << recorder_->error();
                response->success = !failed_;
                response->message = status.str();
            });
        // Start only once construction is complete and the executor is spinning.
        startup_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
            startup_timer_->cancel();
            try { startWorkers(); } catch (const std::exception& error) {
                fail(error.what());
                stopWorkers();
                rclcpp::shutdown(get_node_base_interface()->get_context());
            }
        });
        completion_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
            if (failed_ || (shutdown_on_eof_ && detection_done_ && eof_)) {
                completion_timer_->cancel();
                rclcpp::shutdown(get_node_base_interface()->get_context());
            }
        });
        RCLCPP_INFO(get_logger(), "内部输入已就绪: mode=%s playback=%s sampling_step=%d",
                    mode_.c_str(), playback.c_str(), sampling_step_);
    }

    ~DetectNode() override { stopWorkers(); }
    bool failed() const { return failed_.load(); }

private:
    int positiveInt(const std::string& name, int fallback)
    {
        auto value = declare_parameter<std::int64_t>(name, fallback);
        if (value < 1 || value > std::numeric_limits<int>::max())
            throw std::invalid_argument(name + " must be a positive int");
        return static_cast<int>(value);
    }
    static void checkCuda(cudaError_t code, const char* operation)
    {
        if (code != cudaSuccess) throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(code));
    }
    void startWorkers()
    {
        running_ = true;
        debug_running_ = true;
        debug_worker_ = std::thread(&DetectNode::debugWorkerLoop, this);
        if (raw_enabled_) raw_worker_ = std::thread(&DetectNode::rawLoop, this);
        detect_worker_ = std::thread(&DetectNode::detectLoop, this);
        capture_worker_ = std::thread(&DetectNode::captureLoop, this);
    }
    void stopWorkers()
    {
        if (startup_timer_) startup_timer_->cancel();
        if (completion_timer_) completion_timer_->cancel();
        running_ = false;
        control_cv_.notify_all();
        raw_cv_.notify_all();
        frames_.close(true);
        if (capture_worker_.joinable()) capture_worker_.join();
        if (detect_worker_.joinable()) detect_worker_.join();
        if (raw_worker_.joinable()) raw_worker_.join();
        debug_running_ = false;
        debug_cv_.notify_all();
        if (debug_worker_.joinable()) debug_worker_.join();
        if (recorder_) recorder_->finish();
        if (source_) source_->close();
    }
    void fail(const std::string& message)
    {
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            input_error_ = message;
            failed_ = true;
            reading_ = false;
        }
        running_ = false;
        frames_.close(true);
        control_cv_.notify_all();
        raw_cv_.notify_all();
        RCLCPP_ERROR(get_logger(), "输入/检测停止: %s", message.c_str());
    }
    void setPause(const std_srvs::srv::SetBool::Request::SharedPtr request,
                  std_srvs::srv::SetBool::Response::SharedPtr response)
    {
        if (mode_ != "video") {
            response->success = false;
            response->message = "相机输入不支持视频暂停";
            return;
        }
        std::unique_lock<std::mutex> lock(control_mutex_);
        paused_ = request->data;
        control_cv_.notify_all();
        const bool acknowledged = !paused_ || control_cv_.wait_for(lock, std::chrono::seconds(1),
            [this] { return !reading_ || !running_; });
        response->success = acknowledged && !failed_;
        response->message = acknowledged ? (paused_ ? "paused (in-flight detection may finish)" : "playing")
                                         : "pause requested; current decode has not finished";
    }
    void captureLoop()
    {
        try {
            const auto video_epoch_ns = now().nanoseconds();
            auto next_read = Clock::now();
            while (running_ && rclcpp::ok(get_node_base_interface()->get_context())) {
                {
                    std::unique_lock<std::mutex> lock(control_mutex_);
                    if (paused_) {
                        control_cv_.wait(lock, [this] { return !running_ || !paused_; });
                        next_read = Clock::now();  // no catch-up burst after resume
                    }
                    if (!running_) break;
                    if (mode_ == "video" && !sequential_) {
                        control_cv_.wait_until(lock, next_read, [this] { return !running_ || paused_; });
                        if (!running_) break;
                        if (paused_) continue;
                    }
                    reading_ = true;
                }
                struct ReadCompletion {
                    DetectNode* node;
                    ~ReadCompletion() {
                        std::lock_guard<std::mutex> lock(node->control_mutex_);
                        node->reading_ = false;
                        node->control_cv_.notify_all();
                    }
                } read_completion{this};
                auto read = source_->read();
                if (!running_) break;
                if (read.status == ReadStatus::Timeout) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待相机图像超时");
                    continue;
                }
                if (read.status == ReadStatus::End) { eof_ = true; break; }
                if (read.status != ReadStatus::Ok) throw std::runtime_error(read.message);
                Packet packet;
                packet.frame = std::move(read.frame);
                packet.header.frame_id = frame_id_;
                if (mode_ == "video" && (synthetic_stamp_ || sequential_)) {
                    const std::int64_t epoch = synthetic_stamp_
                        ? static_cast<std::int64_t>(std::llround(1e9 / source_fps_)) : video_epoch_ns;
                    packet.header.stamp = rclcpp::Time(epoch + packet.frame.source_time_ns, get_clock()->get_clock_type());
                } else {
                    packet.header.stamp = now();
                }
                ++acquired_;
                {
                    std::lock_guard<std::mutex> lock(raw_mutex_);
                    latest_raw_ = packet;  // owning Mat reference, no full-frame copy
                }
                if (recorder_) {
                    recorder_->submit(packet.frame.image);
                    const auto error = recorder_->error();
                    if (!error.empty()) RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "录制已停止: %s", error.c_str());
                }
                if (packet.frame.sequence % static_cast<std::uint64_t>(sampling_step_) == 0) {
                    if (!frames_.push(std::move(packet), sequential_)) break;
                } else ++sampled_out_;
                if (mode_ == "video" && !sequential_) {
                    next_read = std::max(next_read + video_period_, Clock::now());
                }
            }
            frames_.close();  // EOF drains the final pending frame
            if (recorder_) recorder_->finish();
        } catch (const std::exception& error) { fail(error.what()); }
    }
    void detectLoop()
    {
        try {
            checkCuda(cudaSetDevice(cuda_device_), "Detection thread CUDA device");
            while (auto packet = frames_.pop()) {
                if (!running_) break;
                processFrame(*packet);
                ++processed_;
            }
            detection_done_ = true;
            RCLCPP_INFO(get_logger(), "检测输入结束: acquired=%lu processed=%lu sampled=%lu overwritten=%lu",
                static_cast<unsigned long>(acquired_.load()), static_cast<unsigned long>(processed_.load()),
                static_cast<unsigned long>(sampled_out_.load()), static_cast<unsigned long>(frames_.overwritten()));
        } catch (const std::exception& error) { fail(error.what()); detection_done_ = true; }
    }
    void rawLoop()
    {
        std::unique_lock<std::mutex> wait_lock(raw_wait_mutex_);
        while (running_) {
            raw_cv_.wait_for(wait_lock, raw_period_, [this] { return !running_; });
            if (!running_) break;
            if (raw_only_subscribers_ && raw_pub_->get_subscription_count() == 0) continue;
            std::optional<Packet> packet;
            {
                std::lock_guard<std::mutex> lock(raw_mutex_);
                packet = latest_raw_;
            }
            if (!packet) continue;
            try {
                auto message = std::make_unique<sensor_msgs::msg::Image>();
                cv_bridge::CvImage(packet->header, "bgr8", packet->frame.image).toImageMsg(*message);
                raw_pub_->publish(std::move(message));
            } catch (const std::exception& error) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "原图发布失败: %s", error.what());
            }
        }
    }
    void processFrame(const Packet& packet)
    {
        // ROI updates occur strictly between frames, also when input is paused/EOF.
        std::lock_guard<std::mutex> pipeline_lock(pipeline_mutex_);
        const cv::Mat& frame = packet.frame.image;
        const double input_delay_ms = std::chrono::duration<double, std::milli>(Clock::now() - packet.frame.received_at).count();
        // First/reset frame has no source interval; do not substitute wall time
        // spent loading engines or waiting while the video was paused.
        float elapsed_s = 0.0f;
        const auto stamp_ns = packet.frame.source_time_ns;
        if (last_source_stamp_) {
            if (stamp_ns >= *last_source_stamp_) elapsed_s = static_cast<float>((stamp_ns - *last_source_stamp_) * 1e-9);
            else pipeline_->resetTimeState();
        }
        last_source_stamp_ = stamp_ns;
        std::vector<Result> results = pipeline_->process(frame, elapsed_s);

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time_).count();
        last_time_ = now;

        double instant_fps = 1.0 / std::max(dt, 1e-6);
        fps_ = 0.9 * fps_ + 0.1 * instant_fps;

        auto armor_msg = std::make_unique<radar27_interfaces::msg::DetectionArray>();
        armor_msg->header = packet.header;   // 复用图像时间戳，方便下游同步

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

            radar27_interfaces::msg::DetectionBox box;
            box.idx         = res.idx;
            box.confidence  = res.confidence;
            box.class_conf  = res.class_conf;
            box.class_margin = res.class_margin;
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

            armor_msg->detections.push_back(box);
        }

        // 前哨站功能启用但未在 results 中出现时，推送状态消息（空框，仅传递存活/死亡状态）
        if (cfg_->model.outpostEnabled && !hasOutpost) {
            radar27_interfaces::msg::DetectionBox statusBox;
            statusBox.idx = robot_id::OUTPOST;
            statusBox.is_dead = !pipeline_->isOutpostAlive();
            statusBox.confidence = 0.0f;
            statusBox.class_conf = -1.0f;
            statusBox.class_margin = -1.0f;
            armor_msg->detections.push_back(statusBox);
        }

        // publish(unique_ptr) 把消息 ownership 交给 rclcpp；启用 intra-process
        // 时可直接转交订阅者，调用后不得再访问 armor_msg。
        armor_pub_->publish(std::move(armor_msg));

        {
            auto timing = pipeline_->getLatestTiming();
            auto timing_msg = std::make_unique<radar27_interfaces::msg::PipelineTiming>();
            timing_msg->header = packet.header;
            timing_msg->car_ms = timing.car_ms;
            timing_msg->armor_ms = timing.armor_ms;
            timing_msg->cls_ms = timing.cls_ms;
            timing_msg->outpost_ms = timing.outpost_ms;
            timing_msg->total_ms = timing.total_ms;
            timing_msg->end_to_end_ms = std::chrono::duration<double, std::milli>(Clock::now() - packet.frame.received_at).count();
            timing_msg->fps = static_cast<double>(fps_);
            timing_pub_->publish(std::move(timing_msg));
        }

        if (publish_debug_image_ && image_pub_->get_subscription_count() > 0) {
            // 异步调试图：只把缩小帧与结果交给后台 worker（最新帧替换语义），
            // 绘制/序列化/发布交给后台线程；缩放仍有成本，实时模式帧选择取决于负载。
            cv::Mat small;
            double scale_x = 1.0, scale_y = 1.0;
            if (frame.cols > debug_output_max_width_) {
                scale_x = static_cast<double>(debug_output_max_width_) / frame.cols;
                scale_y = scale_x;  // 等比缩放
                int target_h = std::max(1, static_cast<int>(frame.rows * scale_y));
                cv::resize(frame, small, cv::Size(debug_output_max_width_, target_h));
            } else {
                // 调试绘制会修改像素，必须复制，避免影响录制和原图缓存。
                frame.copyTo(small);
            }
            {
                std::lock_guard<std::mutex> lock(debug_mutex_);
                debug_pending_frame_ = std::move(small);
                debug_pending_results_ = results;
                debug_pending_scale_x_ = scale_x;
                debug_pending_scale_y_ = scale_y;
                debug_pending_header_ = packet.header;
                debug_dirty_ = true;
            }
            debug_cv_.notify_one();
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            10000,
            "检测到 %zu 个目标，fps: %.1f，input_delay: %.2f ms",
            results.size(), fps_, input_delay_ms);
    }
    void debugWorkerLoop()
    {
        for (;;) {
            cv::Mat frame;
            std::vector<Result> results;
            double sx, sy;
            std_msgs::msg::Header header;
            {
                std::unique_lock<std::mutex> lock(debug_mutex_);
                debug_cv_.wait(lock, [this] { return !debug_running_ || debug_dirty_; });
                if (!debug_dirty_) break;
                frame = std::move(debug_pending_frame_);
                results = std::move(debug_pending_results_);
                sx = debug_pending_scale_x_; sy = debug_pending_scale_y_;
                header = debug_pending_header_;
                debug_dirty_ = false;
            }
            try {
                drawDetect(frame, results, cfg_->model.classNames, sx, sy);
                auto message = std::make_unique<sensor_msgs::msg::Image>();
                header.frame_id = "detected_frame";
                cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*message);
                image_pub_->publish(std::move(message));
            } catch (const std::exception& error) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "调试图失败: %s", error.what());
            }
        }
    }
    void reloadROI(const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        try {
            const auto yaml = YAML::LoadFile(get_parameter("roi_path").as_string());
            const bool enabled = yaml["outpost_enabled"] ? yaml["outpost_enabled"].as<bool>() : false;
            const auto roi = yaml["outpost_roi"] ? yaml["outpost_roi"].as<std::vector<int>>() : std::vector<int>{};
            const float score = yaml["outpost_score_threshold"] ? yaml["outpost_score_threshold"].as<float>() : 0.0f;
            const float timeout = yaml["outpost_miss_timeout_s"] ? yaml["outpost_miss_timeout_s"].as<float>() : 1.0f;
            if ((enabled && (roi.size() != 4 || roi[0] < 0 || roi[1] < 0 || roi[2] <= 0 || roi[3] <= 0)) ||
                (!roi.empty() && roi.size() != 4) || !std::isfinite(score) || score < 0 || score > 1 ||
                !std::isfinite(timeout) || timeout <= 0) throw std::invalid_argument("Invalid outpost ROI/score/timeout");
            {
                std::lock_guard<std::mutex> lock(pipeline_mutex_);
                cfg_->model.outpostEnabled = enabled;
                cfg_->model.outpostRoi = roi;
                cfg_->model.outpostScoreThreshold = score;
                cfg_->model.outpostMissTimeoutS = timeout;
                pipeline_->resetTimeState();
                last_source_stamp_.reset();
            }
            response->success = true;
            response->message = "outpost ROI 配置已重载";
        } catch (const std::exception& error) {
            response->success = false;
            response->message = std::string("重载失败: ") + error.what();
        }
    }

    std::unique_ptr<DetectionConfig> cfg_;
    std::unique_ptr<DetectPipeline> pipeline_;
    std::unique_ptr<radar27_detection::input::FrameSource> source_;
    std::unique_ptr<radar27_detection::Recorder> recorder_;
    radar27_detection::FrameBuffer<Packet> frames_;
    std::string mode_, frame_id_;
    bool sequential_ = false, synthetic_stamp_ = false, shutdown_on_eof_ = false;
    bool raw_enabled_ = true, raw_only_subscribers_ = true;
    bool publish_debug_image_ = true;
    int sampling_step_ = 1, debug_output_max_width_ = 1280, cuda_device_ = 0;
    double source_fps_ = 0.0, fps_ = 0.0;
    Clock::duration video_period_{}, raw_period_{};
    Clock::time_point last_time_ = Clock::now();
    std::optional<std::int64_t> last_source_stamp_;
    std::mutex pipeline_mutex_;
    std::atomic<bool> running_{false}, failed_{false}, eof_{false}, detection_done_{false};
    std::atomic<std::uint64_t> acquired_{0}, processed_{0}, sampled_out_{0};
    std::thread capture_worker_, detect_worker_, raw_worker_, debug_worker_;
    std::mutex control_mutex_, raw_mutex_, raw_wait_mutex_, debug_mutex_;
    std::condition_variable control_cv_, raw_cv_, debug_cv_;
    bool paused_ = false, reading_ = false;
    std::string input_error_;
    std::optional<Packet> latest_raw_;
    std::atomic<bool> debug_running_{false};
    bool debug_dirty_ = false;
    cv::Mat debug_pending_frame_;
    std::vector<Result> debug_pending_results_;
    double debug_pending_scale_x_ = 1.0, debug_pending_scale_y_ = 1.0;
    std_msgs::msg::Header debug_pending_header_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_, raw_pub_;
    rclcpp::Publisher<radar27_interfaces::msg::DetectionArray>::SharedPtr armor_pub_;
    rclcpp::Publisher<radar27_interfaces::msg::PipelineTiming>::SharedPtr timing_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_roi_service_, status_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_, legacy_pause_service_;
    rclcpp::TimerBase::SharedPtr startup_timer_, completion_timer_;
};

#ifndef RADAR27_DETECT_NO_MAIN
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<DetectNode>();
        rclcpp::spin(node);
        result = node->failed() ? 1 : 0;
    } catch (const std::exception& error) {
        std::cerr << "[DetectNode] " << error.what() << std::endl;
        result = 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return result;
}
#endif
