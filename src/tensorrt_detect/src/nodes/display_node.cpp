// 测试用节点，用于水平拼接显示检测图像和小地图
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

class DisplayNode : public rclcpp::Node
{
public:
    DisplayNode() : Node("display_node")
    {
        // ===== 参数声明 =====
        this->declare_parameter<std::string>("topic", "/detected_image");
        this->declare_parameter<std::string>("window_name", "Video & Radar");
        this->declare_parameter<int>("window_width", 1920);
        this->declare_parameter<int>("window_height", 720);

        this->declare_parameter<std::string>("map_topic", "/map_image");

        topic_ = this->get_parameter("topic").as_string();
        window_name_ = this->get_parameter("window_name").as_string();
        int win_w = this->get_parameter("window_width").as_int();
        int win_h = this->get_parameter("window_height").as_int();

        map_topic_ = this->get_parameter("map_topic").as_string();

        // ===== 创建 OpenCV 窗口 =====
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name_, win_w, win_h);

        // ===== 订阅图像话题 =====
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_, 10,
            std::bind(&DisplayNode::image_callback, this, std::placeholders::_1));

        map_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            map_topic_, 10,
            std::bind(&DisplayNode::map_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "显示节点启动 | 订阅主图像: %s | 订阅地图: %s | 按 Q/ESC 退出",
            topic_.c_str(), map_topic_.c_str());
    }

    ~DisplayNode()
    {
        cv::destroyWindow(window_name_);
    }

    // 供主循环调用：获取最新帧并显示
    bool show_latest()
    {
        cv::Mat frame;
        cv::Mat map_frame;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!latest_frame_.empty()) {
                frame = latest_frame_.clone();
            }
            if (!latest_map_frame_.empty()) {
                map_frame = latest_map_frame_.clone();
            }
        }

        if (frame.empty() && map_frame.empty()) {
            // 两者都还没收到，继续等待
            cv::waitKey(1);
            return true;
        }

        cv::Mat combined;
        if (!frame.empty() && !map_frame.empty()) {
            // 将地图缩放到与主图像相同高度，保持宽高比
            int targetHeight = frame.rows;
            int targetWidth = static_cast<int>(map_frame.cols *
                                               (static_cast<float>(targetHeight) / map_frame.rows));
            cv::Mat resized_map;
            cv::resize(map_frame, resized_map, cv::Size(targetWidth, targetHeight));
            cv::hconcat(std::vector<cv::Mat>{frame, resized_map}, combined);
        } else if (!frame.empty()) {
            combined = frame.clone();
        } else {
            combined = map_frame.clone();
        }

        cv::imshow(window_name_, combined);
        int key = cv::waitKey(1);

        if (key == 'q' || key == 'Q' || key == 27) {
            RCLCPP_INFO(this->get_logger(), "收到退出键，关闭显示...");
            return false;
        }
        return true;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_frame_ = cv_ptr->image.clone();
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "收到图像: %d x %d",
                latest_frame_.cols,
                latest_frame_.rows
            );
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
        }
    }

    void map_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_map_frame_ = cv_ptr->image.clone();
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "收到小地图: %d x %d",
                latest_map_frame_.cols,
                latest_map_frame_.rows
            );
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "地图 cv_bridge 转换失败: %s", e.what());
        }
    }

    std::string topic_;
    std::string window_name_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    cv::Mat latest_frame_;

    std::string map_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr map_sub_;
    cv::Mat latest_map_frame_;

    std::mutex mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayNode>();

    // 使用单线程 executor，在循环里交替 spin_some 和 waitKey
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        // 处理回调（收图）
        executor.spin_some(std::chrono::milliseconds(10));

        // 刷新显示（必须在主线程）
        if (!node->show_latest()) break;
    }

    rclcpp::shutdown();
    return 0;
}
