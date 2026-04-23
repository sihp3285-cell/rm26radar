//测试用节点，用于显示检测到的图像
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
        this->declare_parameter<std::string>("window_name", "Detection View");
        this->declare_parameter<int>("window_width", 1280);
        this->declare_parameter<int>("window_height", 720);

        topic_ = this->get_parameter("topic").as_string();
        window_name_ = this->get_parameter("window_name").as_string();
        int win_w = this->get_parameter("window_width").as_int();
        int win_h = this->get_parameter("window_height").as_int();

        // ===== 创建 OpenCV 窗口（和你原来 UI 类一样） =====
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name_, win_w, win_h);

        // ===== 订阅图像话题 =====
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_, 10,
            std::bind(&DisplayNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "显示节点启动 | 订阅: %s | 按 Q/ESC 退出", topic_.c_str());
    }

    ~DisplayNode()
    {
        cv::destroyWindow(window_name_);
    }

    // 供主循环调用：获取最新帧并显示
    bool show_latest()
    {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (latest_frame_.empty()) return true;  // 还没收到图，继续等
            frame = latest_frame_.clone();
        }

        cv::imshow(window_name_, frame);
        int key = cv::waitKey(1);

        if (key == 'q' || key == 'Q' || key == 27) {
            RCLCPP_INFO(this->get_logger(), "收到退出键，关闭显示...");
            return false;  // 通知主循环退出
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
            2000,
            "收到图像: %d x %d",
            latest_frame_.cols,
            latest_frame_.rows
        );
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
    }
}

    std::string topic_;
    std::string window_name_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    cv::Mat latest_frame_;
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
