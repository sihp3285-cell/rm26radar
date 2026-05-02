# `calibrate_node.cpp` 逐行讲解

这份文件是**标定控制节点**，在你当前 ROS2 视觉链路里的位置是：

```text
video_node
   ↓  /image_raw
detect_node
   ↓  /armor_detections
pose_node  ← 通过 /pose_node/reload_calibration 服务通知
   ↓  /world_targets
map_node
   ↓  /map_image, /radar_map
qt_display_node
```

`calibrate_node` 是系统的**标定控制层**。它不负责任何视觉算法，只负责一件事：

> **检测标定状态 → 必要时触发手动标定 → 保存结果 → 通知 pose_node 重载**

在 ROS2 架构中，这种节点叫做 **Configuration / Calibration Node**：

* 不处理常规数据流
* 提供人机交互接口（OpenCV UI + ROS2 Service）
* 协调多个节点的配置状态

---

# 一、头文件部分

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
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <atomic>

#include "mouseback.hpp"
```

---

### 核心头文件

* `<std_srvs/srv/trigger.hpp>`：`/calibration/start` 服务的类型
* `<std_srvs/srv/set_bool.hpp>`：调用 `/video_node/set_pause` 服务的类型
* `<yaml-cpp/yaml.h>`：保存标定结果到 `calib_result.yaml`
* `"mouseback.hpp"`：OpenCV 交互式标定工具（鼠标点击收集像素坐标）

---

# 二、节点定位与职责

`calibrate_node` 的核心职责可以概括为五步：

1. **启动检测**：检查 `calib_result.yaml` 是否存在且有效
2. **自动标定**：无效时延迟 N 秒后自动进入手动标定
3. **图像获取**：创建临时 ROS2 节点 + 独立 Executor，同步获取一帧图像
4. **交互标定**：弹出 OpenCV 窗口，用户点击标定点，计算外参
5. **结果保存**：验证重投影误差 → 写入 YAML → 通知 `pose_node` 重载

---

# 三、参数系统

```cpp
this->declare_parameter<std::string>("config_dir", "/home/delphine/rm/tensorrt10_detect/configs");
this->declare_parameter<std::string>("image_topic", "/image_raw");
this->declare_parameter<std::string>("calib_result_path", ".../calib_result.yaml");
this->declare_parameter<double>("reprojection_threshold", 10.0);
this->declare_parameter<bool>("auto_calibrate", true);
this->declare_parameter<int>("auto_calibrate_delay_sec", 2);
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_dir` | `.../configs` | 配置目录，用于读取 `camera.yaml` |
| `image_topic` | `/image_raw` | 标定时订阅的图像话题 |
| `calib_result_path` | `.../calib_result.yaml` | 标定结果保存路径 |
| `reprojection_threshold` | `10.0` | 重投影误差阈值（像素），超过则强制重新标定 |
| `auto_calibrate` | `true` | 启动时是否自动检测标定文件并触发标定 |
| `auto_calibrate_delay_sec` | `2` | 自动标定前的延迟秒数，给图像源留出启动时间 |

---

# 四、启动时自动检测

```cpp
if (auto_calibrate_ && !isCalibFileValid()) {
    RCLCPP_WARN(..., "标定配置文件无效或为空，将在 %d 秒后自动进入标定...", ...);
    auto_calibrate_timer_ = this->create_wall_timer(..., [this]() {
        auto_calibrate_timer_->cancel();
        if (!isCalibFileValid() && !is_calibrating_.load()) {
            auto [success, msg] = doCalibration();
            // ...
        }
    });
}
```

---

### `isCalibFileValid()`

快速预检标定文件有效性：

1. 文件是否存在
2. 是否能解析出 `r`（9 个元素）和 `t`（3 个元素）

任一条件不满足 → 返回 `false`，触发自动标定。

---

### 为什么用一次性 Timer？

`create_wall_timer` 创建定时器，`auto_calibrate_timer_->cancel()` 确保只执行一次。

延迟 2 秒的原因是：`video_node` 和 `calibrate_node` 在 launch 中同时启动，`video_node` 需要一点时间打开视频文件并开始发布。如果 `calibrate_node` 立即开始标定，可能收不到图像。

---

# 五、核心方法：`doCalibration()`

这是 `calibrate_node` 的干活函数。无论自动标定还是手动触发，最终都走到这里。

---

## 5.1 并发保护

```cpp
if (is_calibrating_.exchange(true)) {
    return {false, "标定正在进行中，请勿重复触发"};
}
```

`std::atomic<bool>` 防止并发标定。如果用户快速点击两次按钮，第二次会被拒绝。

---

## 5.2 图像获取：临时节点 + 独立 Executor

```cpp
static int temp_node_counter = 0;
std::string temp_node_name = "_calib_capture_" + std::to_string(++temp_node_counter);
auto temp_node = std::make_shared<rclcpp::Node>(temp_node_name);

std::promise<cv::Mat> frame_promise;
auto frame_future = frame_promise.get_future();

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

### 为什么用临时节点？

`calibrate_node` 本身被主 Executor（`rclcpp::spin`）管理。如果在 `doCalibration` 中直接创建 subscription 并阻塞等待，会占用主 Executor 的回调线程，导致 subscription 回调无法执行，形成死锁。

**临时节点的优势**：

* 完全独立于主 Executor
* 自己的 `SingleThreadedExecutor` 主动驱动消息接收
* `spin_until_future_complete` 阻塞直到收到图像或超时
* 不干扰主节点的 service/timer 调度

---

## 5.3 视频暂停控制

```cpp
struct VideoPauseGuard {
    CalibrateNode* node;
    bool active;
    ~VideoPauseGuard() {
        if (active && node) node->callVideoPause(false);
    }
};
VideoPauseGuard vp_guard{this, true};
callVideoPause(true);
```

**RAII 设计**：`VideoPauseGuard` 在析构时自动恢复视频，确保无论标定成功、失败还是被取消，视频都不会永久暂停。

`callVideoPause` 内部通过 ROS2 Service Client 调用 `video_node` 的 `/video_node/set_pause`：

```cpp
auto client = create_client<std_srvs::srv::SetBool>("/video_node/set_pause");
auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
request->data = pause;  // true=暂停, false=恢复
auto future = client->async_send_request(request);
```

---

## 5.4 手动标定：MouseBack

```cpp
MouseBack mouseBack("Calibrate", require_points_num_);
image_points = mouseBack.getPoints(captured_frame);
```

`MouseBack` 是项目中的交互式标定工具：

* 弹出 OpenCV 窗口显示 `captured_frame`
* 用户用鼠标左键点击标定点（红圈 + 序号显示）
* 按空格键撤销上一个点
* 按 `q` 取消标定
* 收集到足够点数后自动返回

---

## 5.5 PnP 解算与重投影误差

```cpp
cv::solvePnP(world_points_, image_points, camera_matrix_, dist_coeffs_,
             rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

cv::projectPoints(world_points_, rvec, tvec,
                  camera_matrix_, dist_coeffs_, projected);

double total_err = 0.0;
for (size_t i = 0; i < image_points.size(); ++i) {
    double dx = image_points[i].x - projected[i].x;
    double dy = image_points[i].y - projected[i].y;
    total_err += std::sqrt(dx * dx + dy * dy);
}
double mean_error = total_err / image_points.size();
```

---

### 重投影误差是什么？

用 `solvePnP` 解算出相机外参后，再用 `projectPoints` 把 3D 世界点投影回 2D 图像。比较投影点和用户实际点击点的平均像素距离：

* 误差 **≤ 10px** → 认为标定合格，进入保存流程
* 误差 **> 10px** → 提示用户重新点击标定点，循环重来

这是标定精度的**闭环验证**，防止脏数据进入系统。

---

## 5.6 保存标定结果

```cpp
YAML::Emitter out;
out << YAML::BeginMap;
out << YAML::Key << "image_points" << YAML::Value << YAML::BeginSeq;
for (const auto& pt : imagePoints) {
    out << YAML::Flow << YAML::BeginSeq << pt.x << pt.y << YAML::EndSeq;
}
out << YAML::EndSeq;
out << YAML::Key << "r" << YAML::Value << YAML::Flow << YAML::BeginSeq;
for (int i = 0; i < 9; ++i) out << R.at<double>(i / 3, i % 3);
out << YAML::EndSeq;
out << YAML::Key << "t" << YAML::Value << YAML::Flow << YAML::BeginSeq;
for (int i = 0; i < 3; ++i) out << T.at<double>(i, 0);
out << YAML::EndSeq;
out << YAML::EndMap;
```

保存格式与 `ConfigManager::loadCalibConfig()` 和 `pose_node` 的加载逻辑完全兼容：

```yaml
image_points: [[x1, y1], [x2, y2], ...]
r: [r11, r12, ..., r33]
t: [t1, t2, t3]
```

---

## 5.7 通知 pose_node 重载

```cpp
bool callPoseNodeReload()
{
    auto client = create_client<std_srvs::srv::Trigger>("/pose_node/reload_calibration");
    if (!client->wait_for_service(std::chrono::seconds(3))) {
        return false;  // pose_node 服务未上线
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    auto result = future.get();
    return result->success;
}
```

标定完成后，通过 ROS2 Service 调用 `pose_node` 的 `/pose_node/reload_calibration`，让 `pose_node` 重新读取 `calib_result.yaml` 并更新外参。

---

## 5.8 等待 map 稳定后恢复视频

```cpp
std::this_thread::sleep_for(std::chrono::seconds(2));
```

`pose_node` 重载后立即开始发布 `/world_targets`，`map_node` 收到后绘制地图并发布 `/map_image`。延迟 2 秒确保 `qt_display_node` 已经显示出正常的地图，再恢复视频播放。

---

# 六、手动触发接口：/calibration/start

```cpp
start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/calibration/start",
    std::bind(&CalibrateNode::startCalibrate, this,
              std::placeholders::_1, std::placeholders::_2));
```

提供 `/calibration/start` 服务，支持用户随时手动触发重新标定：

```bash
ros2 service call /calibration/start std_srvs/srv/Trigger {}
```

---

# 七、完整标定时序图

```text
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│calibrate_node│     │ video_node  │     │ pose_node   │     │  User (OpenCV)│
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │                   │
       │  callVideoPause(true)                 │                   │
       │──────────────────→│                   │                   │
       │                   │ 停止发布 /image_raw│                   │
       │                   │                   │                   │
       │ 创建临时节点订阅 /image_raw            │                   │
       │──────────────────────────────────────→│                   │
       │ 收到一帧图像       │                   │                   │
       │←──────────────────────────────────────│                   │
       │                   │                   │                   │
       │ 弹出 MouseBack 窗口                   │                   │
       │────────────────────────────────────────────────────────────→│
       │                   │                   │ 用户点击标定点      │
       │                   │                   │←──────────────────│
       │                   │                   │                   │
       │ 计算 PnP + 重投影误差                 │                   │
       │ 误差 ≤ 10px        │                   │                   │
       │                   │                   │                   │
       │ 保存 calib_result.yaml                │                   │
       │                   │                   │                   │
       │ 调用 /pose_node/reload_calibration    │                   │
       │──────────────────────────────────────→│                   │
       │                   │ 重新加载外参       │                   │
       │                   │ is_calibrated_=true│                   │
       │                   │                   │                   │
       │ 等待 2 秒          │                   │                   │
       │                   │                   │                   │
       │  VideoPauseGuard 析构                 │                   │
       │  callVideoPause(false)                │                   │
       │──────────────────→│                   │                   │
       │                   │ 恢复发布 /image_raw│                   │
```

---

# 八、从这份代码里学到的设计要点

## 1. 临时节点模式

```cpp
auto temp_node = std::make_shared<rclcpp::Node>("_calib_capture_1");
rclcpp::executors::SingleThreadedExecutor temp_executor;
temp_executor.add_node(temp_node);
temp_executor.spin_until_future_complete(future, timeout);
```

在 ROS2 中，如果主 Executor 已经被 `spin`，需要同步等待消息时，**创建临时节点 + 独立 Executor** 是最干净的方案，避免了回调组、线程数等复杂配置。

## 2. RAII 资源管理

```cpp
struct VideoPauseGuard {
    ~VideoPauseGuard() { node->callVideoPause(false); }
};
```

用 RAII 确保资源（视频播放状态）在任何退出路径下都能正确恢复，避免函数中途 `return` 导致视频永久暂停。

## 3. 精度闭环验证

```cpp
solvePnP(...) → projectPoints(...) → 比较误差 → 合格才保存
```

标定不是"算完就完"，而是有明确的精度指标和重试机制，防止不精准的外参污染下游节点。

## 4. 配置即代码

标定结果保存为 YAML，与 `ConfigManager` 和 `pose_node` 的加载逻辑共享同一套格式，形成"保存-加载-使用"的闭环。

## 5. 服务协调而非直接耦合

`calibrate_node` 不直接修改 `pose_node` 的内部状态，而是通过 `/pose_node/reload_calibration` 服务通知它。这保持了节点间的边界和解耦。
