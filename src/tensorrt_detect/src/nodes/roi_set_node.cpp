#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <atomic>
#include <chrono>
#include <thread>

#include "mouseback.hpp"

class ROISetNode : public rclcpp::Node
{
public:
    ROISetNode() : Node("roi_set_node")
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("image_topic", "/image_raw");
        this->declare_parameter<std::string>("outpost_roi_path",
            "/home/delphine/rm/tensorrt10_detect/configs/outpost_roi.yaml");
        this->declare_parameter<bool>("auto_set_roi", true);
        this->declare_parameter<int>("auto_set_delay_sec", 3);

        config_dir_ = this->get_parameter("config_dir").as_string();
        image_topic_ = this->get_parameter("image_topic").as_string();
        outpost_roi_path_ = this->get_parameter("outpost_roi_path").as_string();
        auto_set_roi_ = this->get_parameter("auto_set_roi").as_bool();
        auto_set_delay_sec_ = this->get_parameter("auto_set_delay_sec").as_int();

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/roi_set/start",
            std::bind(&ROISetNode::startROISet, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "ROISetNode 初始化完成。提供服务: /roi_set/start");
        RCLCPP_INFO(this->get_logger(), "图像话题: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "ROI 保存路径: %s", outpost_roi_path_.c_str());

        if (auto_set_roi_ && !isROIValid()) {
            RCLCPP_WARN(this->get_logger(),
                "outpost_roi.yaml 无效或为空，将在 %d 秒后自动进入 ROI 框定...",
                auto_set_delay_sec_);
            auto_set_timer_ = this->create_wall_timer(
                std::chrono::seconds(auto_set_delay_sec_),
                [this]() {
                    auto_set_timer_->cancel();
                    if (!isROIValid() && !is_setting_.load()) {
                        RCLCPP_INFO(this->get_logger(), "自动进入 ROI 框定流程...");
                        auto [success, msg] = doROISet();
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "自动 ROI 框定成功: %s", msg.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "自动 ROI 框定失败: %s", msg.c_str());
                        }
                    }
                });
        }
    }

private:
    bool isROIValid()
    {
        if (!std::filesystem::exists(outpost_roi_path_)) {
            return false;
        }
        try {
            YAML::Node node = YAML::LoadFile(outpost_roi_path_);
            if (!node["outpost_roi"]) {
                return false;
            }
            auto roi = node["outpost_roi"].as<std::vector<int>>();
            return roi.size() == 4;
        } catch (...) {
            return false;
        }
    }

    bool isCalibValid()
    {
        std::filesystem::path dir(config_dir_);
        std::string calib_yaml = (dir / "calib_result.yaml").string();
        if (!std::filesystem::exists(calib_yaml)) {
            return false;
        }
        try {
            YAML::Node node = YAML::LoadFile(calib_yaml);
            if (!node["r"] || !node["t"]) {
                return false;
            }
            auto r = node["r"].as<std::vector<double>>();
            auto t = node["t"].as<std::vector<double>>();
            return (r.size() == 9 && t.size() == 3);
        } catch (...) {
            return false;
        }
    }

    void startROISet(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                     std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "收到手动 ROI 框定请求");
        auto [success, msg] = doROISet();
        response->success = success;
        response->message = msg;
    }

    std::pair<bool, std::string> doROISet()
    {
        if (is_setting_.exchange(true)) {
            return {false, "ROI 框定正在进行中，请勿重复触发"};
        }
        auto guard = [this](bool*) { is_setting_ = false; };
        std::unique_ptr<bool, decltype(guard)> scope_guard(nullptr, guard);

        // ========== 1. 若标定无效，先触发相机标定 ==========
        if (!isCalibValid()) {
            RCLCPP_WARN(this->get_logger(), "相机标定无效，先触发相机标定...");
            auto [calib_ok, calib_msg] = callCalibrationStart();
            if (!calib_ok) {
                if (calib_msg.find("进行中") != std::string::npos) {
                    RCLCPP_INFO(this->get_logger(), "相机标定正在进行中，等待完成...");
                    if (!waitForCalibValid(std::chrono::seconds(60))) {
                        return {false, "等待相机标定完成超时"};
                    }
                } else {
                    return {false, "相机标定触发失败: " + calib_msg};
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "相机标定完成: %s", calib_msg.c_str());
            }
        }

        // ========== 2. 抓帧 ==========
        static int temp_node_counter = 0;
        std::string temp_node_name = "_roi_capture_" + std::to_string(++temp_node_counter);
        auto temp_node = std::make_shared<rclcpp::Node>(temp_node_name);

        std::promise<cv::Mat> frame_promise;
        auto frame_future = frame_promise.get_future();

        auto image_sub = temp_node->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::QoS(1),
            [&](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                    frame_promise.set_value(cv_ptr->image.clone());
                } catch (const std::exception& e) {
                    frame_promise.set_exception(std::current_exception());
                }
            });

        rclcpp::executors::SingleThreadedExecutor temp_executor;
        temp_executor.add_node(temp_node);

        RCLCPP_INFO(this->get_logger(), "等待图像（最多10秒）...");
        auto status = temp_executor.spin_until_future_complete(
            frame_future, std::chrono::seconds(10));

        image_sub.reset();
        temp_executor.remove_node(temp_node);

        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            return {false, "等待图像超时（10秒），请检查图像源是否正常发布到 " + image_topic_};
        }

        cv::Mat captured_frame;
        try {
            captured_frame = frame_future.get();
        } catch (const std::exception& e) {
            return {false, std::string("图像处理失败: ") + e.what()};
        }

        RCLCPP_INFO(this->get_logger(), "成功捕获图像，分辨率: %dx%d",
                    captured_frame.cols, captured_frame.rows);

        // ========== 3. 暂停视频 ==========
        struct VideoPauseGuard {
            ROISetNode* node;
            bool active;
            ~VideoPauseGuard() {
                if (active && node) {
                    node->callVideoPause(false);
                }
            }
        };
        VideoPauseGuard vp_guard{this, true};
        if (!callVideoPause(true)) {
            RCLCPP_WARN(this->get_logger(), "暂停视频失败，将继续框定");
        }

        // ========== 4. 手动框选 ROI（两点模式） ==========
        MouseBack mouseBack("ROISet", 2);
        auto points = mouseBack.getPoints(captured_frame);

        if (points.size() < 2) {
            return {false, "ROI 框定被取消（点数不足）"};
        }

        int x1 = static_cast<int>(points[0].x);
        int y1 = static_cast<int>(points[0].y);
        int x2 = static_cast<int>(points[1].x);
        int y2 = static_cast<int>(points[1].y);
        int x = std::min(x1, x2);
        int y = std::min(y1, y2);
        int w = std::abs(x1 - x2);
        int h = std::abs(y1 - y2);
        cv::Rect roi(x, y, w, h);

        // 边界检查
        const cv::Rect imgBound(0, 0, captured_frame.cols, captured_frame.rows);
        roi = roi & imgBound;
        if (roi.width <= 0 || roi.height <= 0) {
            return {false, "ROI 完全超出图像边界"};
        }

        RCLCPP_INFO(this->get_logger(),
            "ROI 框定结果: [%d, %d, %d, %d]", roi.x, roi.y, roi.width, roi.height);

        // ========== 5. 保存结果 ==========
        if (!saveROIResult(roi)) {
            return {false, "保存 ROI 结果失败"};
        }

        // ========== 6. 通知 detect_node 重载 ==========
        if (!callDetectNodeReload()) {
            return {false, "ROI 结果已保存，但 detect_node 重载失败"};
        }

        return {true, "ROI 框定成功，结果: [" + std::to_string(roi.x) + ", " +
                           std::to_string(roi.y) + ", " + std::to_string(roi.width) + ", " +
                           std::to_string(roi.height) + "]"};
    }

    bool waitForCalibValid(std::chrono::seconds timeout)
    {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < timeout) {
            if (isCalibValid()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        return isCalibValid();
    }

    std::pair<bool, std::string> callCalibrationStart()
    {
        auto client = this->create_client<std_srvs::srv::Trigger>("/calibration/start");
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            return {false, "calibrate_node 的 /calibration/start 服务未上线"};
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(60));

        if (status == std::future_status::timeout) {
            return {false, "调用 /calibration/start 超时"};
        }

        auto result = future.get();
        return {result->success, result->message};
    }

    bool callVideoPause(bool pause)
    {
        auto client = this->create_client<std_srvs::srv::SetBool>("/video_node/set_pause");
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_WARN(this->get_logger(),
                "video_node 的 set_pause 服务未上线（等待 3 秒超时）");
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = pause;
        auto future = client->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(2));

        if (status == std::future_status::timeout) {
            RCLCPP_WARN(this->get_logger(), "调用 video_node set_pause 超时");
            return false;
        }

        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "视频控制: %s", result->message.c_str());
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "视频控制失败: %s", result->message.c_str());
            return false;
        }
    }

    bool callDetectNodeReload()
    {
        auto client = this->create_client<std_srvs::srv::Trigger>("/detect_node/reload_roi");
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(),
                "detect_node 的 reload_roi 服务未上线（等待 3 秒超时）");
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(5));

        if (status == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "调用 detect_node reload_roi 超时");
            return false;
        }

        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "detect_node 重载成功: %s", result->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "detect_node 重载失败: %s", result->message.c_str());
            return false;
        }
    }

    bool saveROIResult(const cv::Rect& roi)
    {
        try {
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "outpost_enabled" << YAML::Value << true;
            out << YAML::Key << "outpost_roi" << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << roi.x << roi.y << roi.width << roi.height;
            out << YAML::EndSeq;
            out << YAML::Key << "outpost_score_threshold" << YAML::Value << 0.4f;
            out << YAML::Key << "outpost_miss_threshold" << YAML::Value << 20;
            out << YAML::EndMap;

            std::ofstream fout(outpost_roi_path_);
            if (!fout.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开文件写入: %s", outpost_roi_path_.c_str());
                return false;
            }
            fout << out.c_str();
            fout.close();
            RCLCPP_INFO(this->get_logger(), "ROI 结果已保存到: %s", outpost_roi_path_.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "保存 YAML 失败: %s", e.what());
            return false;
        }
    }

    std::string config_dir_;
    std::string image_topic_;
    std::string outpost_roi_path_;
    bool auto_set_roi_ = true;
    int auto_set_delay_sec_ = 3;

    std::atomic<bool> is_setting_{false};
    rclcpp::TimerBase::SharedPtr auto_set_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROISetNode>();
    if (!node) {
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
