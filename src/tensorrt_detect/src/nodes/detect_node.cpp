/**
 * @file detect_node.cpp
 * @brief 主感知 component：把 /image_raw 转成检测数组、调试图和阶段耗时。
 *
 * 数据链：sensor_msgs/Image -> cv_bridge 共享视图 -> DetectPipeline（车辆、装甲板、
 * 兵种、前哨站/无人机）-> /armor_detections；调试分支另发布 /detected_image，
 * /pipeline_timing 给 Qt 展示。本节点不做世界坐标或跨帧跟踪，这两项由 PoseNode
 * 负责。/detect_node/reload_roi 只重读前哨站 ROI 配置，不重建 TensorRT engine。
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>
#include <cstdint>
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
    /** 加载 Config/DetectPipeline，创建图像订阅、三个 publisher 和 ROI 重载服务。 */
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
        this->declare_parameter<bool>("frame_sampling_enabled", false);
        this->declare_parameter<int>("frame_sampling_step", 1);
        this->declare_parameter<int>("frame_sampling_period_ms", 50);

        std::string config_dir = this->get_parameter("config_dir").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
        const int64_t debug_output_max_width_param =
            this->get_parameter("debug_output_max_width").as_int();
        debug_output_max_width_ = static_cast<int>(std::max<int64_t>(1, debug_output_max_width_param));
        frame_sampling_enabled_ = this->get_parameter("frame_sampling_enabled").as_bool();
        frame_sampling_step_ = static_cast<int>(std::max<std::int64_t>(1, this->get_parameter("frame_sampling_step").as_int()));
        frame_sampling_period_ms_ = static_cast<int>(std::max<std::int64_t>(1, this->get_parameter("frame_sampling_period_ms").as_int()));

        RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());

        cfg_ = std::make_unique<Config>(config_dir);
        pipeline_ = std::make_unique<DetectPipeline>(*cfg_);

        // 调试图只需要最新帧，depth=1 避免 UI 慢时反压主感知。检测数组 depth=10
        // 且 BestEffort：允许实时链在订阅方来不及处理时丢旧样本，不为逐帧可靠性
        // 牺牲时延。PipelineTiming 同样是“最新状态”语义。
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

        // 异步调试图 worker：UI 开/关不影响主链（借鉴 radar2026）
        debug_running_ = true;
        debug_worker_ = std::thread(&DetectNode::debugWorkerLoop, this);

        RCLCPP_INFO(this->get_logger(), "DetectNode 初始化完成，等待图像输入...");
    }

    ~DetectNode() override
    {
        debug_running_ = false;
        debug_cv_.notify_all();
        if (debug_worker_.joinable()) {
            debug_worker_.join();
        }
    }


private:
    /** 图像订阅回调：共享读取输入、执行推理、转换 DetectionArray，并可选发布调试图/耗时。 */
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            double input_delay_ms = (this->now() - msg->header.stamp).seconds() * 1000.0;

            // toCvShare 在编码兼容时让 cv::Mat 指向消息像素而不复制；cv_ptr/msg
            // 共同保证该内存在本回调内有效。任何需要跨回调保存的图像都必须 clone。
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;

            float elapsed_s = -1.0f;
            int64_t stamp_ns = rclcpp::Time(
                msg->header.stamp, this->get_clock()->get_clock_type()).nanoseconds();
            if (stamp_ns <= 0) {
                stamp_ns = this->get_clock()->now().nanoseconds();
            }
            // 确定性帧采样：内容锚定的合成时间戳下，帧号 = stamp/period - 1。
            // 只处理目标步进的帧，未命中直接返回（不推进 dt 状态，保证相邻
            // 处理帧的 elapsed_s 恰好是 step*period）。
            if (frame_sampling_enabled_) {
                const std::int64_t frame_idx =
                    stamp_ns / (frame_sampling_period_ms_ * 1000000LL) - 1;
                if (frame_idx % frame_sampling_step_ != 0) {
                    return;
                }
            }
            if (last_image_stamp_ns_ > 0) {
                const double stamp_dt_s =
                    static_cast<double>(stamp_ns - last_image_stamp_ns_) * 1e-9;
                if (std::isfinite(stamp_dt_s) && stamp_dt_s >= 0.0) {
                    elapsed_s = static_cast<float>(stamp_dt_s);
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "图像时间倒退（dt=%.6f s），重置检测管线时间状态",
                        stamp_dt_s);
                    pipeline_->resetTimeState();
                }
            }
            last_image_stamp_ns_ = stamp_ns;

            std::vector<Result> results = pipeline_->process(frame, elapsed_s);

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
                tensorrt_detect_msgs::msg::DetectionBox statusBox;
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
                // 异步调试图：只把缩小帧与结果交给后台 worker（最新帧替换语义），
                // 绘制/序列化/发布完全脱离检测回调关键路径，UI 开/关不影响主链
                // 吞吐与帧选择（借鉴 radar2026 netdetector 的线程化发布）。
                cv::Mat small;
                double scale_x = 1.0, scale_y = 1.0;
                if (frame.cols > debug_output_max_width_) {
                    scale_x = static_cast<double>(debug_output_max_width_) / frame.cols;
                    scale_y = scale_x;  // 等比缩放
                    int target_h = static_cast<int>(frame.rows * scale_y);
                    cv::resize(frame, small, cv::Size(debug_output_max_width_, target_h));
                } else {
                    // 必须 deep copy：frame 数据来自 cv_bridge::toCvShare（不拥有数据），
                    // 回调结束后 ROS 消息销毁，数据即失效，浅拷贝会导致悬空指针
                    frame.copyTo(small);
                }
                {
                    std::lock_guard<std::mutex> lock(debug_mutex_);
                    debug_pending_frame_ = std::move(small);
                    debug_pending_results_ = results;
                    debug_pending_scale_x_ = scale_x;
                    debug_pending_scale_y_ = scale_y;
                    debug_pending_header_ = msg->header;
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
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测回调异常: %s", e.what());
        }
    }

    /** 后台线程：绘制检测框并发布 /detected_image（最新帧替换，不阻塞检测回调）。 */
    void debugWorkerLoop()
    {
        while (debug_running_) {
            cv::Mat frame;
            std::vector<Result> results;
            double sx = 1.0, sy = 1.0;
            std_msgs::msg::Header header;
            {
                std::unique_lock<std::mutex> lock(debug_mutex_);
                debug_cv_.wait_for(lock, std::chrono::milliseconds(250),
                                   [this]() { return !debug_running_ || debug_dirty_; });
                if (!debug_running_) break;
                if (!debug_dirty_) continue;
                frame = debug_pending_frame_;
                results = debug_pending_results_;
                sx = debug_pending_scale_x_;
                sy = debug_pending_scale_y_;
                header = debug_pending_header_;
                debug_dirty_ = false;
            }
            drawDetect(frame, results, cfg_->model.classNames, sx, sy);
            try {
                auto cv_img = cv_bridge::CvImage(header, "bgr8", frame);
                cv_img.header.frame_id = "detected_frame";
                auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
                cv_img.toImageMsg(*out_msg);
                image_pub_->publish(std::move(out_msg));
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                      "调试图发布异常: %s", e.what());
            }
        }
    }

    // Config 必须比 DetectPipeline 活得久：pipeline 内部保存 cfg_ 的引用，并据此
    // 读取模型开关/ROI。成员按声明逆序析构，因此此处先声明 Config、后声明 pipeline。
    std::unique_ptr<Config> cfg_;
    std::unique_ptr<DetectPipeline> pipeline_;

    std::string input_topic_;
    std::string output_topic_;
    bool publish_debug_image_ = true;
    int debug_output_max_width_ = 1280;

    /** Trigger 回调：仅从 outpost_roi.yaml 更新 Config 中 ROI，不重建 TensorRT 模型。 */
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
            cfg_->model.outpostMissTimeoutS = cfg["outpost_miss_timeout_s"]
                                                  ? cfg["outpost_miss_timeout_s"].as<float>()
                                                  : 1.0f;

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

    // 确定性帧采样：配合 video_node 的 synthetic_stamp，只处理帧号满足
    // (帧号 % frame_sampling_step_ == 0) 的帧，使观察序列与负载/UI 无关。
    bool frame_sampling_enabled_ = false;
    int frame_sampling_step_ = 1;
    int frame_sampling_period_ms_ = 50;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_pub_;
    rclcpp::Publisher<tensorrt_detect_msgs::msg::PipelineTiming>::SharedPtr timing_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_roi_service_;
    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
    double fps_ = 0.0;
    cv::Mat detect_input_frame_;
    // 异步调试图 worker（最新帧替换语义）
    std::thread debug_worker_;
    std::mutex debug_mutex_;
    std::condition_variable debug_cv_;
    std::atomic<bool> debug_running_{false};
    std::atomic<bool> debug_dirty_{false};
    cv::Mat debug_pending_frame_;
    std::vector<Result> debug_pending_results_;
    double debug_pending_scale_x_ = 1.0;
    double debug_pending_scale_y_ = 1.0;
    std_msgs::msg::Header debug_pending_header_;
    int64_t last_image_stamp_ns_ = 0;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)

/** 非 component 调试入口；正式 launch 使用注册在文件末尾的组件工厂。 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}