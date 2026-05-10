#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QCloseEvent>
#include <QMutex>
#include <QMutexLocker>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/map_tactics.hpp"

class QtDisplayNode;

/**
 * @brief Qt 主窗口：左侧视频、右侧小地图、底部状态栏
 */
class DisplayWindow : public QMainWindow
{
public:
    explicit DisplayWindow(QWidget *parent = nullptr)
        : QMainWindow(parent)
    {
        auto *central = new QWidget(this);
        setCentralWidget(central);

        auto *main_layout = new QVBoxLayout(central);
        main_layout->setContentsMargins(8, 8, 8, 8);
        main_layout->setSpacing(6);

        // 顶部：状态栏（放大字体，醒目显示）
        status_label_ = new QLabel("FPS: --  |  Delay: -- ms", this);
        status_label_->setStyleSheet(
            "color: #00ff88; background-color: #0d0d0d; font-size: 25px; "
            "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
            "padding: 10px 16px; border-radius: 4px;");
        main_layout->addWidget(status_label_);

        // 中部：视频 + 地图 水平排列
        auto *content_layout = new QHBoxLayout();
        content_layout->setSpacing(8);

        video_label_ = new QLabel(this);
        video_label_->setMinimumSize(640, 480);
        video_label_->setAlignment(Qt::AlignCenter);
        video_label_->setStyleSheet("background-color: #1a1a1a; border-radius: 4px;");
        content_layout->addWidget(video_label_, 3);  // 视频占 3 份

        map_label_ = new QLabel(this);
        map_label_->setMinimumSize(320, 480);
        map_label_->setAlignment(Qt::AlignCenter);
        map_label_->setStyleSheet("background-color: #1a1a1a; border-radius: 4px;");
        content_layout->addWidget(map_label_, 1);    // 地图占 1 份

        main_layout->addLayout(content_layout, 1);

        // 底部：阵营切换按钮
        auto *bottom_layout = new QHBoxLayout();
        bottom_layout->setSpacing(8);
        bottom_layout->addStretch(1);

        team_button_ = new QPushButton("蓝方视角", this);
        team_button_->setCheckable(true);
        team_button_->setCursor(Qt::PointingHandCursor);
        team_button_->setStyleSheet(
            "QPushButton {"
            "  background-color: #0066cc;"
            "  color: white;"
            "  font-size: 16px;"
            "  font-weight: bold;"
            "  font-family: 'Microsoft YaHei', 'Consolas', monospace;"
            "  padding: 8px 24px;"
            "  border-radius: 4px;"
            "  border: none;"
            "}"
            "QPushButton:hover { background-color: #aa0000; }"
            "QPushButton:checked {"
            "  background-color: #cc0000;"
            "}"
            "QPushButton:checked:hover { background-color: #0055aa; }");
        connect(team_button_, &QPushButton::toggled, this, [this](bool checked) {
            team_button_->setText(checked ? "红方视角" : "蓝方视角");
            if (team_flip_cb_) team_flip_cb_(checked);
        });
        bottom_layout->addWidget(team_button_);

        main_layout->addLayout(bottom_layout);

        setWindowTitle("TensorRT Detect Qt Display");
        resize(1280, 720);
    }

    void setNode(QtDisplayNode *node) { node_ = node; }

    void refresh()
    {
        if (!node_) return;
        updateFromNode();
    }

    void updateVideo(const cv::Mat &cv_img)
    {
        if (cv_img.empty()) return;
        QPixmap pixmap = cvMatToQPixmap(cv_img);
        video_label_->setPixmap(pixmap.scaled(
            video_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    void updateMap(const cv::Mat &cv_img)
    {
        if (cv_img.empty()) return;
        QPixmap pixmap = cvMatToQPixmap(cv_img);
        map_label_->setPixmap(pixmap.scaled(
            map_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    void updateStatus(double fps, double delay_ms, bool outpost_alive,
                       bool engineer_on_island, bool opponent_attack, bool our_attack)
    {
        QString outpost_text = outpost_alive
            ? QStringLiteral("前哨站: 存活")
            : QStringLiteral("前哨站: 摧毁");

        QStringList tactics;
        if (engineer_on_island) tactics << "敌方工程上岛";
        if (opponent_attack)    tactics << "敌方大攻";
        if (our_attack)         tactics << "我方大攻";
        QString tactics_text = tactics.isEmpty() ? QStringLiteral("战术: 正常") : tactics.join(" | ");

        QString text = QString("FPS: %1  |  Delay: %2 ms  |  %3  |  %4")
                           .arg(fps, 0, 'f', 1)
                           .arg(delay_ms, 0, 'f', 2)
                           .arg(outpost_text)
                           .arg(tactics_text);
        status_label_->setText(text);

        // 根据威胁程度改变状态栏背景色
        if (opponent_attack || engineer_on_island) {
            status_label_->setStyleSheet(
                "color: #ff4444; background-color: #1a0505; font-size: 22px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 10px 16px; border-radius: 4px; font-weight: bold;");
        } else if (our_attack) {
            status_label_->setStyleSheet(
                "color: #44ff44; background-color: #051a05; font-size: 22px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 10px 16px; border-radius: 4px; font-weight: bold;");
        } else {
            status_label_->setStyleSheet(
                "color: #00ff88; background-color: #0d0d0d; font-size: 22px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 10px 16px; border-radius: 4px;");
        }
    }

protected:
    void closeEvent(QCloseEvent *event) override
    {
        if (close_cb_) close_cb_();
        QMainWindow::closeEvent(event);
    }

public:
    void setCloseCallback(std::function<void()> cb) { close_cb_ = std::move(cb); }
    void setTeamFlipCallback(std::function<void(bool)> cb) { team_flip_cb_ = std::move(cb); }

private:
    void updateFromNode();  // 实现在 QtDisplayNode 定义之后

    QLabel *video_label_{nullptr};
    QLabel *map_label_{nullptr};
    QLabel *status_label_{nullptr};
    QPushButton *team_button_{nullptr};
    QtDisplayNode *node_{nullptr};
    std::function<void()> close_cb_;
    std::function<void(bool)> team_flip_cb_;

    static QPixmap cvMatToQPixmap(const cv::Mat &cv_img)
    {
        cv::Mat rgb;
        if (cv_img.channels() == 3)
            cv::cvtColor(cv_img, rgb, cv::COLOR_BGR2RGB);
        else if (cv_img.channels() == 4)
            cv::cvtColor(cv_img, rgb, cv::COLOR_BGRA2RGB);
        else
            rgb = cv_img;
        QImage img(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
        return QPixmap::fromImage(img.copy());
    }
};

/**
 * @brief ROS2 节点：订阅图像和地图图像，驱动 Qt 窗口刷新
 */
class QtDisplayNode : public rclcpp::Node
{
public:
    explicit QtDisplayNode(DisplayWindow *window)
        : Node("qt_display_node"), window_(window)
    {
        this->declare_parameter<std::string>("video_topic", "/detected_image");
        this->declare_parameter<std::string>("map_image_topic", "/map_image");
        this->declare_parameter<std::string>("armor_topic", "/armor_detections");

        video_topic_ = this->get_parameter("video_topic").as_string();
        map_image_topic_ = this->get_parameter("map_image_topic").as_string();
        armor_topic_ = this->get_parameter("armor_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "视频话题: %s", video_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "地图图像话题: %s", map_image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "检测话题: %s", armor_topic_.c_str());

        // 订阅检测图像
        video_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            video_topic_, rclcpp::QoS(1),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

                    double delay_ms = (this->now() - msg->header.stamp).seconds() * 1000.0;

                    // 计算 FPS（指数移动平均）
                    auto now = std::chrono::steady_clock::now();
                    double dt = std::chrono::duration<double>(now - last_time_).count();
                    last_time_ = now;
                    double instant_fps = 1.0 / std::max(dt, 1e-6);
                    fps_ = 0.9 * fps_ + 0.1 * instant_fps;

                    QMutexLocker lock(&mutex_);
                    latest_frame_ = cv_ptr->image.clone();
                    latest_fps_ = fps_;
                    latest_delay_ms_ = delay_ms;
                } catch (const cv_bridge::Exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "视频 cv_bridge 失败: %s", e.what());
                }
            });

        // 订阅地图图像消息（直接显示，与 standalone 模式一致）
        map_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            map_image_topic_, rclcpp::QoS(1),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                    QMutexLocker lock(&mutex_);
                    latest_map_ = cv_ptr->image.clone();
                } catch (const cv_bridge::Exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "地图 cv_bridge 失败: %s", e.what());
                }
            });

        // 订阅检测结果，提取前哨站状态
        armor_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::DetectionArray>(
            armor_topic_, rclcpp::QoS(10),
            [this](const tensorrt_detect_msgs::msg::DetectionArray::SharedPtr msg) {
                bool alive = false;
                bool found = false;
                for (const auto& det : msg->detections) {
                    if (det.idx == 7) {  // OUTPOST
                        found = true;
                        alive = !det.is_dead;
                        break;
                    }
                }
                if (found) {
                    latest_outpost_alive_ = alive;
                }
            });

        // 订阅战术分析消息
        tactics_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::MapTactics>(
            "/map_tactics", rclcpp::QoS(10),
            [this](const tensorrt_detect_msgs::msg::MapTactics::SharedPtr msg) {
                QMutexLocker lock(&mutex_);
                latest_engineer_on_island_ = msg->engineer_on_island;
                latest_opponent_attack_ = msg->opponent_attack;
                latest_our_attack_ = msg->our_attack;
            });

        // 发布阵营翻转话题
        team_flip_pub_ = this->create_publisher<std_msgs::msg::Bool>("/flip_team", rclcpp::QoS(1).reliable());
    }

    void publishTeamFlip(bool is_blue_team)
    {
        std_msgs::msg::Bool msg;
        msg.data = is_blue_team;
        team_flip_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "发布阵营切换: %s", is_blue_team ? "红方视角" : "蓝方视角");
    }

    // 供 DisplayWindow 在主线程调用，安全取出最新数据
    void fetchData(cv::Mat &frame, cv::Mat &map, double &fps, double &delay_ms, bool &outpost_alive,
                   bool &engineer_on_island, bool &opponent_attack, bool &our_attack)
    {
        QMutexLocker lock(&mutex_);
        frame = latest_frame_.clone();
        map = latest_map_.clone();
        fps = latest_fps_;
        delay_ms = latest_delay_ms_;
        outpost_alive = latest_outpost_alive_;
        engineer_on_island = latest_engineer_on_island_;
        opponent_attack = latest_opponent_attack_;
        our_attack = latest_our_attack_;
    }

private:
    DisplayWindow *window_{nullptr};
    QMutex mutex_;

    std::string video_topic_;
    std::string map_image_topic_;
    std::string armor_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr map_sub_;
    rclcpp::Subscription<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_sub_;
    rclcpp::Subscription<tensorrt_detect_msgs::msg::MapTactics>::SharedPtr tactics_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr team_flip_pub_;

    cv::Mat latest_frame_;
    cv::Mat latest_map_;
    double latest_fps_{0.0};
    double latest_delay_ms_{0.0};
    bool latest_outpost_alive_ = false;
    bool latest_engineer_on_island_ = false;
    bool latest_opponent_attack_ = false;
    bool latest_our_attack_ = false;

    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
    double fps_{0.0};
};

// DisplayWindow 的刷新实现：从节点取数据并更新 UI
void DisplayWindow::updateFromNode()
{
    if (!node_) return;
    cv::Mat frame, map;
    double fps = 0.0, delay = 0.0;
    bool outpost_alive = false;
    bool engineer_on_island = false, opponent_attack = false, our_attack = false;
    node_->fetchData(frame, map, fps, delay, outpost_alive,
                     engineer_on_island, opponent_attack, our_attack);
    updateVideo(frame);
    updateMap(map);
    updateStatus(fps, delay, outpost_alive, engineer_on_island, opponent_attack, our_attack);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    app.setStyleSheet(R"(
        QMainWindow { background-color: #2b2b2b; }
        QWidget { background-color: #2b2b2b; }
    )");

    DisplayWindow window;
    window.show();

    auto node = std::make_shared<QtDisplayNode>(&window);
    window.setNode(node.get());
    window.setCloseCallback([]() {
        if (rclcpp::ok()) rclcpp::shutdown();
    });
    window.setTeamFlipCallback([node](bool is_blue_team) {
        node->publishTeamFlip(is_blue_team);
    });

    // Qt 定时器在主线程驱动 UI 刷新（30 FPS）
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, &window, &DisplayWindow::refresh);
    timer.start(33);  // 33 ms ≈ 30 FPS

    std::thread ros_thread([node]() {
        rclcpp::spin(node);
        QApplication::quit();
    });

    int ret = app.exec();
    if (rclcpp::ok()) rclcpp::shutdown();
    ros_thread.join();
    return ret;
}
