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
#include <QKeyEvent>
#include <QMutex>
#include <QMutexLocker>
#include <QPushButton>
#include <QMessageBox>
#include <QSurfaceFormat>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <thread>
#include <atomic>
#include <string>

#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/map_tactics.hpp"
#include "tensorrt_detect_msgs/msg/pipeline_timing.hpp"

class QtDisplayNode;

// ============================================================================
// GLVideoWidget — GPU 加速的视频渲染控件，替代 QLabel::setPixmap
// 数据路径：cv::Mat BGR → glTexImage2D (DMA) → shader BGR→RGB swizzle → GPU 缩放
// 相比 QLabel 路径消除 3 次全帧 CPU 拷贝 + CPU 双线性缩放
// ============================================================================
class GLVideoWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    explicit GLVideoWidget(QWidget *parent = nullptr)
        : QOpenGLWidget(parent)
    {
        setMinimumSize(640, 480);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        // 配置最小化的 surface format：只做 2D 纹理渲染
        QSurfaceFormat fmt;
        fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
        fmt.setSwapInterval(1);          // vsync on
        fmt.setDepthBufferSize(0);
        fmt.setStencilBufferSize(0);
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);  // 兼容旧 GLSL 语法
        setFormat(fmt);
    }

    /// 主线程调用，拷入最新帧并触发 GPU 渲染
    void setFrame(const cv::Mat &frame)
    {
        if (frame.empty()) return;

        const int w = frame.cols;
        const int h = frame.rows;
        const int ch = frame.channels();
        const size_t data_size = frame.total() * frame.elemSize();

        frame_data_.resize(data_size);
        std::memcpy(frame_data_.data(), frame.data, data_size);
        frame_width_ = w;
        frame_height_ = h;
        frame_channels_ = ch;
        frame_updated_ = true;

        update();   // 调度 paintGL()
    }

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();

        // ---- 编译 shader ----
        // vertex：传入 NDC 坐标，输出纹理坐标
        // OpenCV 原点在左上，OpenGL 纹理原点在左下 — 翻转 Y 轴修正
        static const char *vert_src =
            "attribute vec2 aPos;                 \n"
            "varying   vec2 vTexCoord;            \n"
            "void main() {                        \n"
            "    gl_Position = vec4(aPos, 0.0, 1.0);\n"
            "    vTexCoord = vec2(aPos.x * 0.5 + 0.5, 1.0 - (aPos.y * 0.5 + 0.5));\n"
            "}";

        // fragment：BGR→RGB 通道交换在 GPU 上完成（零成本）
        static const char *frag_src =
            "varying vec2 vTexCoord;              \n"
            "uniform sampler2D uTex;              \n"
            "void main() {                        \n"
            "    vec4 c = texture2D(uTex, vTexCoord);\n"
            "    gl_FragColor = c.bgra;           \n"   // R↔B 交换，A 保持 1
            "}";

        shader_.addShaderFromSourceCode(QOpenGLShader::Vertex, vert_src);
        shader_.addShaderFromSourceCode(QOpenGLShader::Fragment, frag_src);
        shader_.link();
        shader_.bind();
        shader_.setUniformValue("uTex", 0);  // 纹理单元 0

        // ---- 创建纹理 ----
        glGenTextures(1, &tex_id_);
        glBindTexture(GL_TEXTURE_2D, tex_id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);      // 缩小时 GPU 双线性
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);      // 放大时 GPU 双线性
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // ---- 创建 quad VBO ----
        quad_vbo_.create();
        quad_vbo_.bind();
        // 初始化为全屏 quad（NDC），后续 paintGL 根据宽高比动态更新
        const float initial_quad[] = { -1, -1,   1, -1,   1, 1,   -1, 1 };
        quad_vbo_.allocate(initial_quad, sizeof(initial_quad));

        glClearColor(0.102f, 0.102f, 0.102f, 1.0f);  // #1a1a1a — 与旧 QSS 背景一致
    }

    void paintGL() override
    {
        glClear(GL_COLOR_BUFFER_BIT);

        if (!frame_updated_ || frame_data_.empty()) return;
        if (this->width() <= 0 || this->height() <= 0) return;

        const int w = frame_width_;
        const int h = frame_height_;
        const int ch = frame_channels_;

        // ---- 纹理上传 ----
        shader_.bind();
        glBindTexture(GL_TEXTURE_2D, tex_id_);

        // BGR 数据以 GL_RGB 格式上传，shader 里 .bgra 交换通道即可得到正确 RGB
        // glPixelStorei 处理 BGR 3 字节宽度可能不被 4 整除的字节对齐问题
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        if (tex_w_ != w || tex_h_ != h) {
            // 尺寸变化：重新分配纹理存储
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0,
                         GL_RGB, GL_UNSIGNED_BYTE, frame_data_.data());
            tex_w_ = w;
            tex_h_ = h;
        } else {
            // 尺寸不变：DMA 更新子区域，避免重新分配
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h,
                            GL_RGB, GL_UNSIGNED_BYTE, frame_data_.data());
        }

        // ---- 计算保持宽高比的 quad 顶点 ----
        const float frame_aspect = static_cast<float>(w) / static_cast<float>(h);
        const float widget_aspect = static_cast<float>(this->width())
                                    / static_cast<float>(this->height());
        float qw, qh;
        if (frame_aspect > widget_aspect) {
            qw = 1.0f;
            qh = widget_aspect / frame_aspect;
        } else {
            qw = frame_aspect / widget_aspect;
            qh = 1.0f;
        }

        const float vertices[] = {
            -qw, -qh,    qw, -qh,    qw,  qh,   -qw,  qh
        };
        quad_vbo_.bind();
        quad_vbo_.write(0, vertices, sizeof(vertices));

        shader_.enableAttributeArray("aPos");
        shader_.setAttributeBuffer("aPos", GL_FLOAT, 0, 2);

        glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

        frame_updated_ = false;
    }

private:
    QOpenGLShaderProgram shader_;
    QOpenGLBuffer quad_vbo_;
    GLuint tex_id_ = 0;
    int tex_w_ = 0, tex_h_ = 0;      // 当前 GPU 纹理尺寸，用于判断是否需要 glTexImage2D

    std::vector<uint8_t> frame_data_;
    int frame_width_ = 0;
    int frame_height_ = 0;
    int frame_channels_ = 0;
    bool frame_updated_ = false;
};

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

        // 顶部状态栏：Timing + 前哨站 + 战术 同行显示
        auto *top_bar_layout = new QHBoxLayout();
        top_bar_layout->setSpacing(8);

        status_label_ = new QLabel("car=-- armor=-- cls=-- output=-- airplane=-- total=-- e2e=-- disp=-- fps=--", this);
        status_label_->setStyleSheet(
            "color: #00ff88; background-color: #0d0d0d; font-size: 20px; "
            "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
            "padding: 8px 16px; border-radius: 4px;");
        top_bar_layout->addWidget(status_label_, 4);

        outpost_label_ = new QLabel("前哨站: --", this);
        outpost_label_->setStyleSheet(
            "color: #00ff88; background-color: #0d0d0d; font-size: 20px; "
            "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
            "padding: 8px 16px; border-radius: 4px; font-weight: bold;");
        outpost_label_->setAlignment(Qt::AlignCenter);
        outpost_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        top_bar_layout->addWidget(outpost_label_, 1);

        tactics_label_ = new QLabel("战术: --", this);
        tactics_label_->setStyleSheet(
            "color: #00ff88; background-color: #0d0d0d; font-size: 20px; "
            "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
            "padding: 8px 16px; border-radius: 4px; font-weight: bold;");
        tactics_label_->setAlignment(Qt::AlignCenter);
        tactics_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        top_bar_layout->addWidget(tactics_label_, 2);

        main_layout->addLayout(top_bar_layout);

        // 中部：视频 + 地图 水平排列
        auto *content_layout = new QHBoxLayout();
        content_layout->setSpacing(8);

        video_label_ = new GLVideoWidget(this);
        content_layout->addWidget(video_label_, 3);  // 视频占 3 份，GPU 渲染

        map_label_ = new QLabel(this);
        map_label_->setMinimumSize(320, 480);
        map_label_->setAlignment(Qt::AlignCenter);
        map_label_->setStyleSheet("background-color: #1a1a1a; border-radius: 4px;");
        content_layout->addWidget(map_label_, 1);    // 地图占 1 份

        main_layout->addLayout(content_layout, 1);

        // 底部：操作按钮栏
        auto *bottom_layout = new QHBoxLayout();
        bottom_layout->setSpacing(8);
        bottom_layout->addStretch(1);

        // --- 重新标定按钮 ---
        calibrate_button_ = new QPushButton("重新标定", this);
        calibrate_button_->setFocusPolicy(Qt::NoFocus);
        calibrate_button_->setCursor(Qt::PointingHandCursor);
        calibrate_button_->setStyleSheet(
            "QPushButton {"
            "  background-color: #e67e22;"
            "  color: white;"
            "  font-size: 16px;"
            "  font-weight: bold;"
            "  font-family: 'Microsoft YaHei', 'Consolas', monospace;"
            "  padding: 8px 24px;"
            "  border-radius: 4px;"
            "  border: none;"
            "}"
            "QPushButton:hover { background-color: #d35400; }"
            "QPushButton:disabled { background-color: #666666; color: #999999; }");
        connect(calibrate_button_, &QPushButton::clicked, this, [this]() {
            if (calibrate_cb_) {
                setCalibrateButtonsEnabled(false);
                calibrate_cb_();
            }
        });
        bottom_layout->addWidget(calibrate_button_);

        // --- 重新框定 ROI 按钮 ---
        roi_button_ = new QPushButton("重新框定 ROI", this);
        roi_button_->setFocusPolicy(Qt::NoFocus);
        roi_button_->setCursor(Qt::PointingHandCursor);
        roi_button_->setStyleSheet(
            "QPushButton {"
            "  background-color: #8e44ad;"
            "  color: white;"
            "  font-size: 16px;"
            "  font-weight: bold;"
            "  font-family: 'Microsoft YaHei', 'Consolas', monospace;"
            "  padding: 8px 24px;"
            "  border-radius: 4px;"
            "  border: none;"
            "}"
            "QPushButton:hover { background-color: #732d91; }"
            "QPushButton:disabled { background-color: #666666; color: #999999; }");
        connect(roi_button_, &QPushButton::clicked, this, [this]() {
            if (roi_cb_) {
                setCalibrateButtonsEnabled(false);
                roi_cb_();
            }
        });
        bottom_layout->addWidget(roi_button_);

        // --- 阵营切换按钮 ---
        team_button_ = new QPushButton("蓝方视角", this);
        team_button_->setCheckable(true);
        team_button_->setFocusPolicy(Qt::NoFocus);
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

        // 底部操作结果提示栏
        op_status_label_ = new QLabel("", this);
        op_status_label_->setStyleSheet(
            "color: #aaaaaa; background-color: #0d0d0d; font-size: 14px; "
            "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
            "padding: 4px 12px; border-radius: 4px;");
        op_status_label_->setAlignment(Qt::AlignCenter);
        main_layout->addWidget(op_status_label_);

        setWindowTitle("TensorRT Detect Qt Display");
        resize(1280, 720);
    }

    void setNode(QtDisplayNode *node) { node_ = node; }

    void refresh()
    {
        if (!node_) return;
        if (paused_) return;
        updateFromNode();
    }

    void updateVideo(const cv::Mat &cv_img)
    {
        // GPU 路径：单次 memcpy + DMA 纹理上传，shader 里完成 BGR→RGB + 缩放
        video_label_->setFrame(cv_img);
    }

    void updateMap(const cv::Mat &cv_img)
    {
        if (cv_img.empty()) return;
        QPixmap pixmap = cvMatToQPixmap(cv_img);
        map_label_->setPixmap(pixmap.scaled(
            map_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    void updateStatus(const tensorrt_detect_msgs::msg::PipelineTiming& timing, bool outpost_alive,
                       bool engineer_on_island, bool opponent_attack, bool our_attack, bool opponent_near_fortress,
                       double display_latency_ms)
    {
        // 更新基础状态栏（time 日志各阶段耗时）
        QString text = QString("car=%1 armor=%2 cls=%3 output=%4 airplane=%5 total=%6 e2e=%7 disp=%8 fps=%9")
                           .arg(timing.car_ms, 0, 'f', 1)
                           .arg(timing.armor_ms, 0, 'f', 1)
                           .arg(timing.cls_ms, 0, 'f', 1)
                           .arg(timing.outpost_ms, 0, 'f', 1)
                           .arg(timing.airplane_ms, 0, 'f', 1)
                           .arg(timing.total_ms, 0, 'f', 1)
                           .arg(timing.end_to_end_ms, 0, 'f', 1)
                           .arg(display_latency_ms, 0, 'f', 1)
                           .arg(timing.fps, 0, 'f', 1);
        status_label_->setText(text);

        // 更新前哨站状态
        QString outpost_text = outpost_alive
            ? QStringLiteral("前哨站: 存活")
            : QStringLiteral("前哨站: 摧毁");
        outpost_label_->setText(outpost_text);

        if (outpost_alive) {
            outpost_label_->setStyleSheet(
                "color: #00ff88; background-color: #0d0d0d; font-size: 20px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 8px 16px; border-radius: 4px; font-weight: bold;");
        } else {
            outpost_label_->setStyleSheet(
                "color: #ff4444; background-color: #1a0505; font-size: 20px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 8px 16px; border-radius: 4px; font-weight: bold;");
        }

        // 更新战术状态
        QStringList tactics;
        if (engineer_on_island==1) tactics << "我方工程上岛";
        if (engineer_on_island==2) tactics << "敌方工程上岛";
        if (opponent_attack)    tactics << "敌方大攻";
        if (our_attack)         tactics << "我方大攻";
        if (opponent_near_fortress == 1) tactics << "敌方接近堡垒!";

        QString tactics_text = tactics.isEmpty() ? QStringLiteral("战术: 正常") : tactics.join(" | ");
        tactics_label_->setText(tactics_text);

        // 根据威胁程度改变战术栏背景色
        if (opponent_attack || engineer_on_island == 2 || opponent_near_fortress == 1) {
            tactics_label_->setStyleSheet(
                "color: #ff4444; background-color: #1a0505; font-size: 20px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 8px 16px; border-radius: 4px; font-weight: bold;");
        } else if (our_attack || engineer_on_island == 1) {
            tactics_label_->setStyleSheet(
                "color: #44ff44; background-color: #051a05; font-size: 20px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 8px 16px; border-radius: 4px; font-weight: bold;");
        } else {
            tactics_label_->setStyleSheet(
                "color: #00ff88; background-color: #0d0d0d; font-size: 20px; "
                "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
                "padding: 8px 16px; border-radius: 4px; font-weight: bold;");
        }
    }

    /**
     * @brief 设置操作结果提示（线程安全，可通过 invokeMethod 跨线程调用）
     */
    void showOperationStatus(const QString &text, bool success)
    {
        QString style = success
            ? "color: #00ff88; background-color: #0d1a0d; font-size: 14px; "
              "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
              "padding: 4px 12px; border-radius: 4px;"
            : "color: #ff4444; background-color: #1a0505; font-size: 14px; "
              "font-family: 'Microsoft YaHei', 'Consolas', monospace; "
              "padding: 4px 12px; border-radius: 4px;";
        op_status_label_->setText(text);
        op_status_label_->setStyleSheet(style);
    }

    void setCalibrateButtonsEnabled(bool enabled)
    {
        calibrate_button_->setEnabled(enabled);
        roi_button_->setEnabled(enabled);
    }

protected:
    void closeEvent(QCloseEvent *event) override
    {
        if (close_cb_) close_cb_();
        QMainWindow::closeEvent(event);
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        if (event->key() == Qt::Key_Space) {
            paused_ = !paused_;
            if (pause_cb_) {
                pause_cb_(paused_);
            }
            QString status = paused_
                ? QStringLiteral("已暂停 (按空格恢复)")
                : QStringLiteral("已恢复 (按空格暂停)");
            showOperationStatus(status, !paused_);
        } else {
            QMainWindow::keyPressEvent(event);
        }
    }

public:
    void setCloseCallback(std::function<void()> cb) { close_cb_ = std::move(cb); }
    void setTeamFlipCallback(std::function<void(bool)> cb) { team_flip_cb_ = std::move(cb); }
    void setCalibrateCallback(std::function<void()> cb) { calibrate_cb_ = std::move(cb); }
    void setROICallback(std::function<void()> cb) { roi_cb_ = std::move(cb); }
    void setPauseCallback(std::function<void(bool)> cb) { pause_cb_ = std::move(cb); }

private:
    void updateFromNode();  // 实现在 QtDisplayNode 定义之后

    GLVideoWidget *video_label_{nullptr};
    QLabel *map_label_{nullptr};
    QLabel *status_label_{nullptr};
    QLabel *outpost_label_{nullptr};
    QLabel *tactics_label_{nullptr};
    QLabel *op_status_label_{nullptr};
    QPushButton *team_button_{nullptr};
    QPushButton *calibrate_button_{nullptr};
    QPushButton *roi_button_{nullptr};
    QtDisplayNode *node_{nullptr};
    std::function<void()> close_cb_;
    std::function<void(bool)> team_flip_cb_;
    std::function<void()> calibrate_cb_;
    std::function<void()> roi_cb_;
    std::function<void(bool)> pause_cb_;
    bool paused_ = false;

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
                    QMutexLocker lock(&mutex_);
                    latest_frame_ = cv_ptr->image.clone();
                    latest_display_latency_ms_ = (this->now() - msg->header.stamp).seconds() * 1000.0;
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
            armor_topic_, rclcpp::QoS(10).best_effort(),
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
                latest_opponent_near_fortress_ = msg->opponent_near_fortress;
            });

        // 订阅 pipeline 耗时统计
        timing_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::PipelineTiming>(
            "/pipeline_timing", rclcpp::QoS(1),
            [this](const tensorrt_detect_msgs::msg::PipelineTiming::SharedPtr msg) {
                QMutexLocker lock(&mutex_);
                latest_timing_ = *msg;
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

    /**
     * @brief 异步调用 /video_node/set_pause 服务，在后台线程执行
     */
    void setVideoPauseAsync(bool pause)
    {
        std::thread([this, pause]() {
            auto client = this->create_client<std_srvs::srv::SetBool>("/video_node/set_pause");
            if (!client->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_WARN(this->get_logger(), "暂停服务未上线: /video_node/set_pause");
                return;
            }
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = pause;
            auto future = client->async_send_request(request);
            auto status = future.wait_for(std::chrono::seconds(2));
            if (status == std::future_status::timeout) {
                RCLCPP_WARN(this->get_logger(), "暂停服务调用超时");
                return;
            }
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "视频暂停结果: %s - %s",
                        result->success ? "成功" : "失败", result->message.c_str());
        }).detach();
    }

    /**
     * @brief 异步调用 ROS2 service（在后台线程执行，不阻塞 Qt 主线程）
     * @param service_name 服务名称
     * @param operation_name 操作名称（用于日志和 UI 提示）
     */
    void callServiceAsync(const std::string &service_name, const std::string &operation_name)
    {
        if (is_operation_running_.exchange(true)) {
            RCLCPP_WARN(this->get_logger(), "%s 正在进行中，请勿重复触发", operation_name.c_str());
            return;
        }

        // 在后台线程执行 service 调用
        std::thread([this, service_name, operation_name]() {
            RCLCPP_INFO(this->get_logger(), "发起 %s 请求...", operation_name.c_str());

            auto client = this->create_client<std_srvs::srv::Trigger>(service_name);

            // 等待服务上线（最多 5 秒）
            if (!client->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "%s 服务未上线: %s", operation_name.c_str(), service_name.c_str());
                QMetaObject::invokeMethod(window_, [this, operation_name]() {
                    window_->showOperationStatus(
                        QString::fromStdString(operation_name + " 失败: 服务未上线"), false);
                    window_->setCalibrateButtonsEnabled(true);
                });
                is_operation_running_ = false;
                return;
            }

            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

            // 标定/框定可能需要较长时间（用户手动点选），给予 5 分钟超时
            auto future = client->async_send_request(request);
            auto status = future.wait_for(std::chrono::seconds(300));

            if (status == std::future_status::timeout) {
                RCLCPP_ERROR(this->get_logger(), "%s 调用超时", operation_name.c_str());
                QMetaObject::invokeMethod(window_, [this, operation_name]() {
                    window_->showOperationStatus(
                        QString::fromStdString(operation_name + " 超时"), false);
                    window_->setCalibrateButtonsEnabled(true);
                });
                is_operation_running_ = false;
                return;
            }

            auto result = future.get();
            bool success = result->success;
            std::string msg = result->message;

            RCLCPP_INFO(this->get_logger(), "%s 结果: %s - %s",
                        operation_name.c_str(), success ? "成功" : "失败", msg.c_str());

            // 回到 Qt 主线程更新 UI
            QMetaObject::invokeMethod(window_, [this, operation_name, success, msg]() {
                QString status_text = success
                    ? QString::fromStdString(operation_name + " 成功: " + msg)
                    : QString::fromStdString(operation_name + " 失败: " + msg);
                window_->showOperationStatus(status_text, success);
                window_->setCalibrateButtonsEnabled(true);
            });

            is_operation_running_ = false;
        }).detach();
    }

    // 供 DisplayWindow 在主线程调用，安全取出最新数据
    void fetchData(cv::Mat &frame, cv::Mat &map,
                   tensorrt_detect_msgs::msg::PipelineTiming &timing,
                   bool &outpost_alive,
                   bool &engineer_on_island, bool &opponent_attack, bool &our_attack, bool &opponent_near_fortress,
                   double &display_latency_ms)
    {
        QMutexLocker lock(&mutex_);
        frame = latest_frame_.clone();
        map = latest_map_.clone();
        timing = latest_timing_;
        outpost_alive = latest_outpost_alive_;
        engineer_on_island = latest_engineer_on_island_;
        opponent_attack = latest_opponent_attack_;
        our_attack = latest_our_attack_;
        opponent_near_fortress = latest_opponent_near_fortress_;
        display_latency_ms = latest_display_latency_ms_.load();
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
    rclcpp::Subscription<tensorrt_detect_msgs::msg::PipelineTiming>::SharedPtr timing_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr team_flip_pub_;

    cv::Mat latest_frame_;
    cv::Mat latest_map_;
    tensorrt_detect_msgs::msg::PipelineTiming latest_timing_;
    std::atomic<double> latest_display_latency_ms_{0.0};
    bool latest_outpost_alive_ = false;
    bool latest_engineer_on_island_ = false;
    bool latest_opponent_attack_ = false;
    bool latest_our_attack_ = false;
    bool latest_opponent_near_fortress_ = false;

    std::atomic<bool> is_operation_running_{false};
};

// DisplayWindow 的刷新实现：从节点取数据并更新 UI
void DisplayWindow::updateFromNode()
{
    if (!node_) return;
    cv::Mat frame, map;
    tensorrt_detect_msgs::msg::PipelineTiming timing;
    bool outpost_alive = false;
    bool engineer_on_island = false, opponent_attack = false, our_attack = false, opponent_near_fortress = false;
    double display_latency_ms = 0.0;
    node_->fetchData(frame, map, timing, outpost_alive,
                     engineer_on_island, opponent_attack, our_attack, opponent_near_fortress,
                     display_latency_ms);
    updateVideo(frame);
    updateMap(map);
    updateStatus(timing, outpost_alive, engineer_on_island, opponent_attack, our_attack, opponent_near_fortress,
                 display_latency_ms);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 设置 OpenGL surface 格式（必须在 QApplication 创建之前）
    QSurfaceFormat gl_fmt;
    gl_fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    gl_fmt.setSwapInterval(1);       // vsync
    gl_fmt.setDepthBufferSize(0);    // 2D 视频无需深度缓冲
    gl_fmt.setStencilBufferSize(0);
    gl_fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
    QSurfaceFormat::setDefaultFormat(gl_fmt);

    QApplication app(argc, argv);

    // 全局样式只影响非 GL 的普通 widget，GLVideoWidget 自行用 glClear 处理背景
    app.setStyleSheet(R"(
        QMainWindow { background-color: #2b2b2b; }
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

    // 重新标定按钮 → 调用 /calibration/start 服务
    window.setCalibrateCallback([node]() {
        node->callServiceAsync("/calibration/start", "相机标定");
    });

    // 重新框定 ROI 按钮 → 调用 /roi_set/start 服务
    window.setROICallback([node]() {
        node->callServiceAsync("/roi_set/start", "ROI 框定");
    });

    // 空格键暂停 / 恢复 → 调用 /video_node/set_pause 服务
    window.setPauseCallback([node](bool paused) {
        node->setVideoPauseAsync(paused);
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
