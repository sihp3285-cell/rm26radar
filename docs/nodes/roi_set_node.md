# `roi_set_node.cpp` 逐行讲解

这份文件实现了 **前哨站 ROI 框定节点**，负责在系统启动时（或手动触发时）引导用户框选前哨站的检测区域（Region of Interest）。

在你当前 ROS2 视觉链路里的位置：

```text
video_node
   ↓  /image_raw
   ├── detect_node（使用 outpost_roi 做前哨站检测）
   ├── calibrate_node（相机标定）
   └── roi_set_node  ← 你在这里（ROI 框定）
         ├── /roi_set/start 服务
         ├── /calibration/start 客户端
         ├── /video_node/set_pause 客户端
         └── /detect_node/reload_roi 客户端
```

`roi_set_node` 是系统的**工具节点**，不参与实时检测流水线，而是在初始化阶段和维护阶段提供交互式配置能力。

---

# 一、核心职责

1. **自动检测**：启动时检查 `outpost_roi.yaml` 是否有效，无效则自动进入框定流程
2. **依赖检查**：若相机标定无效，先自动触发 `/calibration/start` 完成标定
3. **抓帧**：从 `/image_raw` 话题获取一帧图像
4. **暂停视频**：通过 `/video_node/set_pause` 暂停视频播放
5. **交互框选**：弹出 OpenCV 窗口，用户两点框选 ROI
6. **保存结果**：把 ROI 写入 `outpost_roi.yaml`
7. **通知重载**：通过 `/detect_node/reload_roi` 通知 `detect_node` 重载配置

---

# 二、头文件部分

```cpp
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
```

---

### `<atomic>`

`is_setting_` 使用 `std::atomic<bool>` 保证线程安全。ROI 框定流程可能在 service 回调线程和 timer 回调线程中被并发触发，需要原子操作防止重入。

---

### `<thread>`

`waitForCalibValid()` 中使用 `std::this_thread::sleep_for()` 做轮询等待。

---

### `"mouseback.hpp"`

`MouseBack` 类封装了 OpenCV 的鼠标交互逻辑，支持多点选取。在 `calibrate_node` 中也使用了同样的交互方式。

---

# 三、构造函数

```cpp
ROISetNode() : Node("roi_set_node")
```

---

## 3.1 参数声明

```cpp
this->declare_parameter<std::string>("config_dir",
    "/home/delphine/rm/tensorrt10_detect/configs");
this->declare_parameter<std::string>("image_topic", "/image_raw");
this->declare_parameter<std::string>("outpost_roi_path",
    "/home/delphine/rm/tensorrt10_detect/configs/outpost_roi.yaml");
this->declare_parameter<bool>("auto_set_roi", true);
this->declare_parameter<int>("auto_set_delay_sec", 3);
```

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `config_dir` | configs 目录 | 配置文件根目录（用于查找 `calib_result.yaml`） |
| `image_topic` | `/image_raw` | 抓帧用的图像话题 |
| `outpost_roi_path` | `outpost_roi.yaml` 路径 | ROI 保存文件路径 |
| `auto_set_roi` | `true` | 是否在 ROI 无效时自动进入框定 |
| `auto_set_delay_sec` | 3 | 自动框定前的延迟秒数（等待系统稳定） |

---

## 3.2 服务注册

```cpp
start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/roi_set/start",
    std::bind(&ROISetNode::startROISet, this,
              std::placeholders::_1, std::placeholders::_2));
```

提供 `/roi_set/start` 服务，用户可通过命令行手动触发：

```bash
ros2 service call /roi_set/start std_srvs/srv/Trigger
```

---

## 3.3 自动框定逻辑

```cpp
if (auto_set_roi_ && !isROIValid()) {
    RCLCPP_WARN(this->get_logger(),
        "outpost_roi.yaml 无效或为空，将在 %d 秒后自动进入 ROI 框定...",
        auto_set_delay_sec_);
    auto_set_timer_ = this->create_wall_timer(
        std::chrono::seconds(auto_set_delay_sec_),
        [this]() {
            auto_set_timer_->cancel();
            if (!isROIValid() && !is_setting_.load()) {
                auto [success, msg] = doROISet();
            }
        });
}
```

---

### 延迟触发的工程意义

延迟 3 秒是给系统初始化留缓冲时间：

* `video_node` 需要时间打开相机/视频文件
* `detect_node` 需要时间加载 TensorRT 模型
* `calibrate_node` 需要时间检查标定文件

如果立即触发框定，可能因为图像源还没就绪而失败。

---

### `isROIValid()` 检查

```cpp
bool isROIValid() {
    if (!std::filesystem::exists(outpost_roi_path_)) return false;
    try {
        YAML::Node node = YAML::LoadFile(outpost_roi_path_);
        if (!node["outpost_roi"]) return false;
        auto roi = node["outpost_roi"].as<std::vector<int>>();
        return roi.size() == 4;
    } catch (...) { return false; }
}
```

检查 `outpost_roi.yaml` 中是否有 `outpost_roi` 字段且为 4 元素数组 `[x, y, w, h]`。

---

# 四、核心流程：`doROISet()`

这是整个节点的核心方法，包含完整的 ROI 框定流程。

---

## 4.1 防重入保护

```cpp
if (is_setting_.exchange(true)) {
    return {false, "ROI 框定正在进行中，请勿重复触发"};
}
auto guard = [this](bool*) { is_setting_ = false; };
std::unique_ptr<bool, decltype(guard)> scope_guard(nullptr, guard);
```

`std::atomic::exchange(true)` 原子地把 `is_setting_` 从 `false` 改为 `true`，并返回旧值。如果旧值已经是 `true`，说明有另一个线程正在执行框定。

`scope_guard` 是一个 RAII 式的作用域守卫，确保 `is_setting_` 在函数退出时（无论是正常返回还是异常退出）都会被重置为 `false`。

---

## 4.2 标定依赖检查

```cpp
if (!isCalibValid()) {
    RCLCPP_WARN(this->get_logger(), "相机标定无效，先触发相机标定...");
    auto [calib_ok, calib_msg] = callCalibrationStart();
    if (!calib_ok) {
        if (calib_msg.find("进行中") != std::string::npos) {
            if (!waitForCalibValid(std::chrono::seconds(60))) {
                return {false, "等待相机标定完成超时"};
            }
        } else {
            return {false, "相机标定触发失败: " + calib_msg};
        }
    }
}
```

---

### 流程逻辑

1. 检查 `calib_result.yaml` 是否有效
2. 无效 → 调用 `/calibration/start` 服务触发标定
3. 如果标定服务返回"进行中" → 轮询等待（最多 60 秒）
4. 如果标定服务返回其他错误 → 返回失败

这体现了**节点间协作**的设计：`roi_set_node` 不自己做标定，而是委托 `calibrate_node` 完成。

---

## 4.3 抓帧

```cpp
auto temp_node = std::make_shared<rclcpp::Node>(temp_node_name);
auto image_sub = temp_node->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::QoS(1),
    [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        frame_promise.set_value(cv_ptr->image.clone());
    });

rclcpp::executors::SingleThreadedExecutor temp_executor;
temp_executor.add_node(temp_node);
auto status = temp_executor.spin_until_future_complete(
    frame_future, std::chrono::seconds(10));
```

---

### 临时节点模式

创建一个临时 ROS2 节点来订阅图像，获取一帧后立即销毁。这种模式的好处：

* 不与 `video_node` 的发布频率耦合
* 不占用长期订阅资源
* 超时机制（10 秒）保证不会永久阻塞

---

### `std::promise` / `std::future` 同步模式

```cpp
std::promise<cv::Mat> frame_promise;
auto frame_future = frame_promise.get_future();
```

`promise` 在回调中设置值，`future` 在主线程中等待。这是一种标准的**异步转同步**模式。

---

## 4.4 暂停视频

```cpp
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
callVideoPause(true);
```

---

### RAII 守卫模式

`VideoPauseGuard` 在构造时暂停视频，在析构时恢复视频。无论函数从哪个出口退出（正常返回、异常、超时），析构函数都会执行，确保视频不会被永久暂停。

---

## 4.5 手动框选

```cpp
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
```

两点框选模式：左键选第一个点（左上角），再选第二个点（右下角），自动计算矩形。

---

## 4.6 保存结果

```cpp
bool saveROIResult(const cv::Rect& roi)
{
    YAML::Node node;
    if (std::filesystem::exists(outpost_roi_path_)) {
        node = YAML::LoadFile(outpost_roi_path_);  // 保留原有字段
    }
    node["outpost_roi"] = std::vector<int>{roi.x, roi.y, roi.width, roi.height};
    // ... 写入文件 ...
}
```

---

### 增量更新策略

如果 `outpost_roi.yaml` 已存在，先加载再更新 `outpost_roi` 字段，保留其他字段（如 `outpost_enabled`、`outpost_score_threshold`、`outpost_miss_threshold`）。

---

## 4.7 通知 `detect_node` 重载

```cpp
bool callDetectNodeReload()
{
    auto client = temp_node->create_client<std_srvs::srv::Trigger>("/detect_node/reload_roi");
    if (!client->wait_for_service(std::chrono::seconds(10))) {
        return false;
    }
    // ... 发送请求 ...
}
```

等待 `detect_node` 的 `/detect_node/reload_roi` 服务上线（最多 10 秒），然后发送重载请求。`detect_node` 收到请求后会重新读取 `outpost_roi.yaml`，更新 `DetectPipeline` 中的前哨站 ROI 配置。

---

# 五、`main` 函数

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROISetNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
```

---

### 为什么用 `MultiThreadedExecutor`？

ROI 框定流程在 service callback 中**阻塞执行**（弹出 OpenCV 窗口等待用户操作），期间还需要调用其他节点的服务（`/calibration/start`、`/video_node/set_pause`、`/detect_node/reload_roi`）。

如果用 `SingleThreadedExecutor`，service callback 阻塞时无法处理嵌套 service 调用的响应，会死锁。

`MultiThreadedExecutor(2)` 使用 2 个线程：一个处理阻塞的框定流程，另一个处理嵌套 service 的响应。

---

# 六、从 roi_set_node 学到的设计要点

## 1. 工具节点的定位

`roi_set_node` 不参与实时检测流水线，只在初始化和维护阶段工作。它体现了 ROS2 的**松耦合**思想：

* 可以随时启动或停止，不影响其他节点
* 通过服务调用与 `calibrate_node`、`detect_node`、`video_node` 协作
* 使用完毕后可以安全关闭

## 2. RAII 资源管理

代码中大量使用 RAII 模式：

* `scope_guard`：确保 `is_setting_` 被重置
* `VideoPauseGuard`：确保视频被恢复

这是 C++ 中最优雅的资源管理方式，比 `try/catch/finally` 更安全。

## 3. 临时节点模式

抓帧、调用服务时创建临时节点，用完即销毁。避免了长期占用资源，也让代码更自包含。

## 4. 多线程 Executor 的必要性

当 service callback 中需要调用其他 service 时，必须使用多线程 Executor。这是 ROS2 中一个常见的陷阱：单线程 Executor 下嵌套 service 调用会死锁。

## 5. 依赖链的自动编排

`roi_set_node` 自动处理了复杂的依赖链：

```text
检查标定 → 无效则触发标定 → 等待标定完成
                            → 检查 ROI → 无效则框定
                                        → 暂停视频 → 抓帧 → 框选
                                        → 恢复视频 → 保存 → 通知重载
```

整个链路无需人工干预，实现了"一键初始化"的用户体验。
