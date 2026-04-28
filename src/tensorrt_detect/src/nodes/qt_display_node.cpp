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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/radar_map.hpp"
#include "ConfigManager.hpp"
#include "radarmap.hpp"
#include "robot_id.hpp"

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

        // 上部分：视频 + 地图 水平排列
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

        // 底部状态栏
        status_label_ = new QLabel("FPS: --  |  Delay: -- ms", this);
        status_label_->setStyleSheet(
            "color: #00ff88; background-color: #0d0d0d; font-size: 14px; "
            "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
            "padding: 6px 12px; border-radius: 4px;");
        main_layout->addWidget(status_label_);

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

    void updateStatus(double fps, double delay_ms)
    {
        QString text = QString("FPS: %1  |  Delay: %2 ms")
                           .arg(fps, 0, 'f', 1)
                           .arg(delay_ms, 0, 'f', 2);
        status_label_->setText(text);
    }

protected:
    void closeEvent(QCloseEvent *event) override
    {
        if (close_cb_) close_cb_();
        QMainWindow::closeEvent(event);
    }

public:
    void setCloseCallback(std::function<void()> cb) { close_cb_ = std::move(cb); }

private:
    void updateFromNode();  // 实现在 QtDisplayNode 定义之后

    QLabel *video_label_{nullptr};
    QLabel *map_label_{nullptr};
    QLabel *status_label_{nullptr};
    QtDisplayNode *node_{nullptr};
    std::function<void()> close_cb_;

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
 * @brief ROS2 节点：订阅图像和地图坐标，驱动 Qt 窗口刷新
 */
class QtDisplayNode : public rclcpp::Node
{
public:
    explicit QtDisplayNode(DisplayWindow *window)
        : Node("qt_display_node"), window_(window)
    {
        this->declare_parameter<std::string>("config_dir",
            "/home/delphine/rm/tensorrt10_detect/configs");
        this->declare_parameter<std::string>("video_topic", "/detected_image");
        this->declare_parameter<std::string>("radar_map_topic", "/radar_map");

        std::string config_dir = this->get_parameter("config_dir").as_string();
        video_topic_ = this->get_parameter("video_topic").as_string();
        radar_map_topic_ = this->get_parameter("radar_map_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "QtDisplayNode 配置目录: %s", config_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "视频话题: %s", video_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "地图话题: %s", radar_map_topic_.c_str());

        // 加载配置，初始化 RadarMap（用于根据坐标重新绘制地图，只显示兵种）
        cfg_ = std::make_unique<Config>(config_dir);
        radar_map_ = std::make_unique<RadarMap>(cfg_->map.mapPath, cfg_->map.isFlip);
        radar_map_->calibrate2(
            cfg_->map.race_size[0],
            cfg_->map.race_size[1],
            cfg_->map.map_size[0],
            cfg_->map.map_size[1]);

        if (!radar_map_->m_isCalibrated) {
            RCLCPP_ERROR(this->get_logger(), "RadarMap 校准失败");
        } else {
            RCLCPP_INFO(this->get_logger(), "RadarMap 校准完成");
        }

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

        // 订阅 RadarMap 坐标消息（而非地图图像），以便自己绘制并优化文字
        radar_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::RadarMap>(
            radar_map_topic_, 10,
            [this](const tensorrt_detect_msgs::msg::RadarMap::SharedPtr msg) {
                std::vector<Mappoint> mappoints;

                auto add_points = [&](const float xs[6],
                                      const float ys[6],
                                      int team_id) {
                    for (size_t i = 0; i < 6; ++i) {
                        if (xs[i] == 0.0f && ys[i] == 0.0f) continue;  // 未检测到
                        Mappoint mp;
                        mp.map_point = cv::Point2f(xs[i], ys[i]);
                        mp.label = "";
                        mp.teamId = team_id;
                        // 索引 0~3 对应 R1~R4，索引 4 对应 DEAD，索引 5 对应 S
                        if (i < 4) mp.classIdx = robot_id::R1 + static_cast<int>(i);
                        else if (i == 4) mp.classIdx = robot_id::DEAD;
                        else if (i == 5) mp.classIdx = robot_id::S;
                        else continue;
                        mappoints.push_back(mp);
                    }
                };

                add_points(msg->blue_x.data(), msg->blue_y.data(), robot_id::BLUE);
                add_points(msg->red_x.data(), msg->red_y.data(), robot_id::RED);

                cv::Mat map_frame = radar_map_->drawMap(mappoints, cfg_->model.classNames, true);

                QMutexLocker lock(&mutex_);
                latest_map_ = map_frame.clone();
            });
    }

    // 供 DisplayWindow 在主线程调用，安全取出最新数据
    void fetchData(cv::Mat &frame, cv::Mat &map, double &fps, double &delay_ms)
    {
        QMutexLocker lock(&mutex_);
        frame = latest_frame_.clone();
        map = latest_map_.clone();
        fps = latest_fps_;
        delay_ms = latest_delay_ms_;
    }

private:
    DisplayWindow *window_{nullptr};
    QMutex mutex_;

    std::string video_topic_;
    std::string radar_map_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_sub_;
    rclcpp::Subscription<tensorrt_detect_msgs::msg::RadarMap>::SharedPtr radar_sub_;

    std::unique_ptr<Config> cfg_;
    std::unique_ptr<RadarMap> radar_map_;

    cv::Mat latest_frame_;
    cv::Mat latest_map_;
    double latest_fps_{0.0};
    double latest_delay_ms_{0.0};

    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
    double fps_{0.0};
};

// DisplayWindow 的刷新实现：从节点取数据并更新 UI
void DisplayWindow::updateFromNode()
{
    if (!node_) return;
    cv::Mat frame, map;
    double fps = 0.0, delay = 0.0;
    node_->fetchData(frame, map, fps, delay);
    updateVideo(frame);
    updateMap(map);
    updateStatus(fps, delay);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // 全局暗色样式
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
