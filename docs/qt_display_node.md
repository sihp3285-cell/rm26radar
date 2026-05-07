# `qt_display_node.cpp` 逐行讲解

这份文件是 **Qt 可视化显示节点**，与 `display_node.cpp` 功能类似，但使用 Qt5 框架代替 OpenCV 的 `cv::imshow`，提供更美观、更稳定的 GUI。

```text
video_node
   ↓  /image_raw
detect_node
   ↓  /detected_image, /armor_detections
pose_node
   ↓  /world_targets
map_node
   ↓  /map_image, /radar_map
qt_display_node  ← 订阅 /detected_image、/map_image、/armor_detections
```

相比 `display_node`，`qt_display_node` 增加了：

1. **Qt5 GUI 框架**：更美观的界面、更流畅的刷新
2. **阵营切换按钮**：点击按钮发布 `/flip_team` 话题，动态切换红蓝视角
3. **前哨站状态显示**：订阅 `/armor_detections`，实时显示"前哨站：存活/摧毁"

---

# 一、头文件部分

```cpp
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
```

---

### Qt5 相关头文件

| 头文件 | 用途 |
|--------|------|
| `QApplication` | Qt 应用程序入口 |
| `QMainWindow` | 主窗口 |
| `QLabel` | 显示图像（通过 `setPixmap`） |
| `QHBoxLayout/VBoxLayout` | 水平/垂直布局管理 |
| `QTimer` | 定时器驱动 UI 刷新 |
| `QMutex` | 线程同步（ROS 回调线程 ↔ Qt 主线程） |
| `QPushButton` | 阵营切换按钮 |

### ROS2 相关头文件

与 `display_node` 相同，但额外引入了：

```cpp
#include "tensorrt_detect_msgs/msg/detection_array.hpp"
```

用于订阅 `/armor_detections`，提取前哨站状态。

---

# 二、`DisplayWindow`：Qt 主窗口

```cpp
class DisplayWindow : public QMainWindow
```

继承自 `QMainWindow`，包含三个区域：

1. **顶部**：状态栏（FPS + 延迟 + 前哨站状态）
2. **左侧**：视频图像 (`video_label_`)
3. **右侧**：小地图 (`map_label_`)
4. **底部**：阵营切换按钮

---

### 布局结构

```text
+--------------------------------------+
| status_label_                        |
| (FPS + Delay + 前哨站状态)           |
+--------------------------------------+
+------------------+------------------+
|                  |                  |
|   video_label_   |   map_label_     |
|   (检测图像)      |   (小地图)        |
|                  |                  |
+------------------+------------------+
|                              [红方视角] |
+--------------------------------------+
```

---

### `updateStatus`：更新顶部状态栏

```cpp
void updateStatus(double fps, double delay_ms, bool outpost_alive)
{
    QString outpost_text = outpost_alive
        ? QStringLiteral("前哨站: 存活")
        : QStringLiteral("前哨站: 摧毁");
    QString text = QString("FPS: %1  |  Delay: %2 ms  |  %3")
                       .arg(fps, 0, 'f', 1)
                       .arg(delay_ms, 0, 'f', 2)
                       .arg(outpost_text);
    status_label_->setText(text);
}
```

状态栏显示三部分信息：

* **FPS**：当前帧率
* **Delay**：图像传输延迟
* **前哨站状态**："存活"（绿色语义）或 "摧毁"（红色语义）

---

# 三、`QtDisplayNode`：ROS2 节点

```cpp
class QtDisplayNode : public rclcpp::Node
```

---

### 订阅三个话题

```cpp
// 1. 检测图像
video_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    video_topic_, rclcpp::QoS(1), ...);

// 2. 地图图像
map_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    map_image_topic_, rclcpp::QoS(1), ...);

// 3. 检测结果（新增）
amor_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::DetectionArray>(
    armor_topic_, rclcpp::QoS(10), ...);
```

---

### 前哨站状态提取

```cpp
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
```

---

#### 为什么只取 `idx == 7`？

`robot_id.hpp` 中定义：

```cpp
enum ClassId {
    CAR   = 0,
    ARMOR = 1,
    R1    = 2,
    R2    = 3,
    R3    = 4,
    R4    = 5,
    S     = 6,
    OUTPOST = 7,
};
```

遍历 `/armor_detections` 消息，找到 `idx == 7` 的检测框，根据 `is_dead` 判断前哨站是否存活。

#### 为什么用 `found` 标志？

如果当前帧没有前哨站检测结果（比如 `runOutpostDetect` 返回空），保持上一帧状态，避免状态闪烁。

---

### `fetchData`：供 Qt 主线程安全取数据

```cpp
void fetchData(cv::Mat &frame, cv::Mat &map, double &fps, double &delay_ms, bool &outpost_alive)
{
    QMutexLocker lock(&mutex_);
    frame = latest_frame_.clone();
    map = latest_map_.clone();
    fps = latest_fps_;
    delay_ms = latest_delay_ms_;
    outpost_alive = latest_outpost_alive_;
}
```

使用 `QMutexLocker`（RAII 锁）保护共享数据，避免 ROS 回调线程和 Qt 主线程竞争。

---

# 四、`updateFromNode`：Qt 定时器驱动的刷新

```cpp
void DisplayWindow::updateFromNode()
{
    if (!node_) return;
    cv::Mat frame, map;
    double fps = 0.0, delay = 0.0;
    bool outpost_alive = false;
    node_->fetchData(frame, map, fps, delay, outpost_alive);
    updateVideo(frame);
    updateMap(map);
    updateStatus(fps, delay, outpost_alive);
}
```

`QTimer` 每 33ms（约 30 FPS）触发一次，调用 `updateFromNode`：

1. 从 ROS 节点取最新数据
2. 更新视频显示
3. 更新地图显示
4. 更新状态栏（含前哨站状态）

---

# 五、阵营切换按钮

```cpp
team_button_ = new QPushButton("红方视角", this);
team_button_->setCheckable(true);
connect(team_button_, &QPushButton::toggled, this, [this](bool checked) {
    team_button_->setText(checked ? "蓝方视角" : "红方视角");
    if (team_flip_cb_) team_flip_cb_(checked);
});
```

* 未按下：红方视角，按钮红色
* 按下：蓝方视角，按钮蓝色
* 触发 `team_flip_cb_` → 发布 `/flip_team` 话题

---

# 六、`main` 函数：ROS2 + Qt 联合事件循环

```cpp
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    DisplayWindow window;
    window.show();

    auto node = std::make_shared<QtDisplayNode>(&window);
    window.setNode(node.get());

    // Qt 定时器驱动 UI 刷新
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, &window, &DisplayWindow::refresh);
    timer.start(33);  // 30 FPS

    // ROS2 在独立线程中 spin
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
        QApplication::quit();
    });

    int ret = app.exec();
    rclcpp::shutdown();
    ros_thread.join();
    return ret;
}
```

---

### 线程模型

```text
Qt 主线程          ROS2 线程
    │                  │
    │ ←─ QTimer 33ms ─→│
    │    fetchData     │
    │    updateStatus  │
    │                  │ ←─ 图像回调 ── updateVideo
    │                  │ ←─ 检测回调 ── updateOutpostStatus
```

* **Qt 主线程**：负责 GUI 刷新、事件处理
* **ROS2 线程**：负责订阅回调、话题通信
* **同步**：`QMutex` 保护共享数据

---

# 七、从 `qt_display_node` 学到的设计要点

## 1. ROS2 + Qt 的线程分离

ROS2 的 `spin` 放在独立线程，`QApplication::exec()` 放在主线程，两者通过 `QMutex` 同步数据。

## 2. 定时器驱动刷新

不用 `spin_some` 轮询，而是用 `QTimer` 定时触发 UI 更新，代码更简洁、刷新更稳定。

## 3. 状态栏扩展

```cpp
void updateStatus(double fps, double delay_ms, bool outpost_alive);
```

状态栏不只是显示性能指标，还可以显示业务状态（前哨站存活/摧毁）。扩展参数比修改全局变量更清晰。

## 4. 订阅多路数据做融合

`qt_display_node` 同时订阅：

* `/detected_image` → 视频
* `/map_image` → 地图
* `/armor_detections` → 前哨站状态

从多路独立话题中提取所需信息，在 UI 层做融合显示，是 ROS2 可视化节点的典型模式。
