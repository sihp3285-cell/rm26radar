#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <atomic>

#include "mouseback.hpp"

class CalibrateNode : public rclcpp::Node
{
public:
    CalibrateNode() : Node("calibrate_node")
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("image_topic", "/image_raw");
        this->declare_parameter<std::string>("calib_result_path",
            "/home/delphine/rm/tensorrt10_detect/configs/calib_result.yaml");
        this->declare_parameter<double>("reprojection_threshold", 10.0);
        this->declare_parameter<bool>("auto_calibrate", true);
        this->declare_parameter<int>("auto_calibrate_delay_sec", 2);

        config_dir_ = this->get_parameter("config_dir").as_string();
        image_topic_ = this->get_parameter("image_topic").as_string();
        calib_result_path_ = this->get_parameter("calib_result_path").as_string();
        reprojection_threshold_ = this->get_parameter("reprojection_threshold").as_double();
        auto_calibrate_ = this->get_parameter("auto_calibrate").as_bool();
        auto_calibrate_delay_sec_ = this->get_parameter("auto_calibrate_delay_sec").as_int();

        if (!loadCameraConfig()) {
            RCLCPP_ERROR(this->get_logger(), "加载 camera.yaml 失败，标定节点无法正常启动");
            return;
        }

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibration/start",
            std::bind(&CalibrateNode::startCalibrate, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "CalibrateNode 初始化完成。提供服务: /calibration/start");
        RCLCPP_INFO(this->get_logger(), "图像话题: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "重投影误差阈值: %.2f px", reprojection_threshold_);

        // 检查标定文件有效性，若无效且开启自动标定，则延迟后自动进入标定
        if (auto_calibrate_ && !isCalibFileValid()) {
            RCLCPP_WARN(this->get_logger(),
                "标定配置文件无效或为空，将在 %d 秒后自动进入标定...",
                auto_calibrate_delay_sec_);
            auto_calibrate_timer_ = this->create_wall_timer(
                std::chrono::seconds(auto_calibrate_delay_sec_),
                [this]() {
                    auto_calibrate_timer_->cancel();
                    if (!isCalibFileValid() && !is_calibrating_.load()) {
                        RCLCPP_INFO(this->get_logger(), "自动进入标定流程...");
                        auto [success, msg] = doCalibration();
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "自动标定成功: %s", msg.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "自动标定失败: %s", msg.c_str());
                        }
                    }
                });
        }
    }

private:
    bool loadCameraConfig()
    {
        try {
            std::filesystem::path dir(config_dir_);
            std::string camera_yaml = (dir / "camera.yaml").string();
            YAML::Node cfg = YAML::LoadFile(camera_yaml);

            std::vector<double> cam_data = cfg["cameraMatrix"].as<std::vector<double>>();
            if (cam_data.size() != 9) {
                RCLCPP_ERROR(this->get_logger(), "cameraMatrix 长度必须为 9");
                return false;
            }
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; ++i) {
                camera_matrix_.at<double>(i / 3, i % 3) = cam_data[i];
            }

            std::vector<double> dist_data = cfg["distCoeffs"].as<std::vector<double>>();
            dist_coeffs_ = cv::Mat(1, static_cast<int>(dist_data.size()), CV_64F);
            for (size_t i = 0; i < dist_data.size(); ++i) {
                dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_data[i];
            }

            require_points_num_ = cfg["requirePointsNum"].as<int>();
            auto wp = cfg["worldPoints"].as<std::vector<std::vector<float>>>();
            world_points_.clear();
            for (const auto& p : wp) {
                if (p.size() != 3) {
                    RCLCPP_ERROR(this->get_logger(), "worldPoints 每个点必须有 3 个元素");
                    return false;
                }
                world_points_.emplace_back(p[0], p[1], p[2]);
            }

            if (static_cast<int>(world_points_.size()) != require_points_num_) {
                RCLCPP_WARN(this->get_logger(),
                    "worldPoints 数量 (%zu) 与 requirePointsNum (%d) 不一致",
                    world_points_.size(), require_points_num_);
            }
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "解析 camera.yaml 失败: %s", e.what());
            return false;
        }
    }

    bool isCalibFileValid()
    {
        if (!std::filesystem::exists(calib_result_path_)) {
            return false;
        }
        try {
            YAML::Node node = YAML::LoadFile(calib_result_path_);
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

    void startCalibrate(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "收到手动标定请求");
        auto [success, msg] = doCalibration();
        response->success = success;
        response->message = msg;
    }

    std::pair<bool, std::string> doCalibration()
    {
        if (is_calibrating_.exchange(true)) {
            return {false, "标定正在进行中，请勿重复触发"};
        }

        auto guard = [this](bool*) { is_calibrating_ = false; };
        bool guard_token = false;
        std::unique_ptr<bool, decltype(guard)> scope_guard(&guard_token, guard);

        RCLCPP_INFO(this->get_logger(), "开始标定流程，尝试捕获图像...");

        // ========== 1. 使用临时节点 + 独立 Executor 获取一帧图像 ==========
        // 避免与主 Executor 的回调调度互相干扰
        static int temp_node_counter = 0;
        std::string temp_node_name = "_calib_capture_" + std::to_string(++temp_node_counter);
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
        RCLCPP_INFO(this->get_logger(), "操作提示: 左键选点, 空格=撤销上一点, Q=取消标定");

        // 获取图像后暂停视频，标定结束（无论成败）自动恢复
        struct VideoPauseGuard {
            CalibrateNode* node;
            bool active;
            ~VideoPauseGuard() {
                if (active && node) {
                    node->callVideoPause(false);
                }
            }
        };
        VideoPauseGuard vp_guard{this, true};
        if (!callVideoPause(true)) {
            RCLCPP_WARN(this->get_logger(), "暂停视频失败，将继续标定");
        }

        // ========== 2. 手动标定 + 重投影误差检查 ==========
        std::vector<cv::Point2f> image_points;
        cv::Mat R, T;
        double mean_error = 0.0;
        bool success = false;

        while (rclcpp::ok()) {
            MouseBack mouseBack("Calibrate", require_points_num_);
            image_points = mouseBack.getPoints(captured_frame);

            if (image_points.size() < static_cast<size_t>(require_points_num_)) {
                return {false, "标定被取消（点数不足）"};
            }

            // 计算外参和重投影误差
            cv::Mat rvec, tvec;
            bool pnp_ok = cv::solvePnP(world_points_, image_points,
                                       camera_matrix_, dist_coeffs_,
                                       rvec, tvec, false,
                                       cv::SOLVEPNP_ITERATIVE);
            if (!pnp_ok) {
                RCLCPP_ERROR(this->get_logger(), "solvePnP 失败，请重新标定");
                continue;
            }

            std::vector<cv::Point2f> projected;
            cv::projectPoints(world_points_, rvec, tvec,
                              camera_matrix_, dist_coeffs_, projected);

            double total_err = 0.0;
            for (size_t i = 0; i < image_points.size(); ++i) {
                double dx = image_points[i].x - projected[i].x;
                double dy = image_points[i].y - projected[i].y;
                total_err += std::sqrt(dx * dx + dy * dy);
            }
            mean_error = total_err / image_points.size();

            RCLCPP_INFO(this->get_logger(),
                "重投影误差: %.3f px (阈值: %.2f px)", mean_error, reprojection_threshold_);

            if (mean_error <= reprojection_threshold_) {
                cv::Mat R_mat;
                cv::Rodrigues(rvec, R_mat);
                R = R_mat.t();
                T = -R * tvec;
                success = true;
                break;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "误差过大，需要重新标定。请在弹出的窗口中重新点击标定点。");
            }
        }

        if (!success) {
            return {false, "标定失败，未达到精度要求"};
        }

        // ========== 3. 保存标定结果 ==========
        if (!saveCalibResult(image_points, R, T)) {
            return {false, "保存标定结果失败"};
        }

        // ========== 4. 调用 pose_node reload ==========
        if (!callPoseNodeReload()) {
            return {false, "标定结果已保存，但 pose_node 重载失败"};
        }

        // 等待 map pipeline 稳定（map 正常显示后再恢复视频）
        RCLCPP_INFO(this->get_logger(), "等待 map pipeline 稳定...");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        return {true, "标定成功，重投影误差: " + std::to_string(mean_error) + " px"};
    }

    bool saveCalibResult(const std::vector<cv::Point2f>& imagePoints,
                         const cv::Mat& R, const cv::Mat& T)
    {
        try {
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "image_points" << YAML::Value << YAML::BeginSeq;
            for (const auto& pt : imagePoints) {
                out << YAML::Flow << YAML::BeginSeq << pt.x << pt.y << YAML::EndSeq;
            }
            out << YAML::EndSeq;
            out << YAML::Key << "r" << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (int i = 0; i < 9; ++i) {
                out << R.at<double>(i / 3, i % 3);
            }
            out << YAML::EndSeq;
            out << YAML::Key << "t" << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (int i = 0; i < 3; ++i) {
                out << T.at<double>(i, 0);
            }
            out << YAML::EndSeq;
            out << YAML::EndMap;

            std::ofstream fout(calib_result_path_);
            if (!fout.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开文件写入: %s", calib_result_path_.c_str());
                return false;
            }
            fout << out.c_str();
            fout.close();
            RCLCPP_INFO(this->get_logger(), "标定结果已保存到: %s", calib_result_path_.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "保存 YAML 失败: %s", e.what());
            return false;
        }
    }

    bool callVideoPause(bool pause)
    {
        static int temp_counter = 0;
        auto temp_node = std::make_shared<rclcpp::Node>("_calib_vp_" + std::to_string(++temp_counter));

        auto client = temp_node->create_client<std_srvs::srv::SetBool>("/video_node/set_pause");
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_WARN(this->get_logger(),
                "video_node 的 set_pause 服务未上线（等待 3 秒超时）");
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = pause;
        auto future = client->async_send_request(request);

        rclcpp::executors::SingleThreadedExecutor temp_exec;
        temp_exec.add_node(temp_node);
        auto status = temp_exec.spin_until_future_complete(future, std::chrono::seconds(2));
        temp_exec.remove_node(temp_node);

        if (status != rclcpp::FutureReturnCode::SUCCESS) {
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

    bool callPoseNodeReload()
    {
        static int temp_counter = 0;
        auto temp_node = std::make_shared<rclcpp::Node>("_calib_reload_" + std::to_string(++temp_counter));

        auto client = temp_node->create_client<std_srvs::srv::Trigger>("/pose_node/reload_calibration");
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(),
                "pose_node 的 reload 服务未上线（等待 3 秒超时）");
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        rclcpp::executors::SingleThreadedExecutor temp_exec;
        temp_exec.add_node(temp_node);
        auto status = temp_exec.spin_until_future_complete(future, std::chrono::seconds(5));
        temp_exec.remove_node(temp_node);

        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "调用 pose_node reload 超时");
            return false;
        }

        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "pose_node 重载成功: %s", result->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "pose_node 重载失败: %s", result->message.c_str());
            return false;
        }
    }

    std::string config_dir_;
    std::string image_topic_;
    std::string calib_result_path_;
    double reprojection_threshold_ = 5.0;
    bool auto_calibrate_ = true;
    int auto_calibrate_delay_sec_ = 2;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point3f> world_points_;
    int require_points_num_ = 6;

    std::atomic<bool> is_calibrating_{false};
    rclcpp::TimerBase::SharedPtr auto_calibrate_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrateNode>();
    // 使用多线程 Executor：标定流程在 service callback 中阻塞执行，
    // 需要另一个线程来处理 pose_node reload 等嵌套 service 调用的响应
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
