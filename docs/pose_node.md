# `pose_node.cpp` 逐行讲解

这份文件是**位姿解算与跟踪节点**，在你当前 ROS2 视觉链路里的位置是：

```text
video_node
   ↓  /image_raw
detect_node
   ↓  /armor_detections  ← pose_node 订阅这里
pose_node  ← 你在这里
   ├── PoseSolver（像素→世界坐标）
   ├── Tracker（固定槽位多目标跟踪 + Kalman 平滑）
   ↓  /world_targets      ← pose_node 发布这里
map_node
   ↓  /map_image, /radar_map, /map_tactics
qt_display_node
```

`pose_node` 是系统的**几何变换 + 跟踪层**。它的职责比早期版本更重：

1. 订阅 `detect_node` 输出的结构化检测结果 `/armor_detections`
2. 用相机内参、外参、PnP 解算、射线碰撞，把每个目标的像素坐标解算成世界坐标
3. **通过固定槽位 Tracker 做多目标跟踪和 Kalman 平滑**（新增）
4. **前哨站直接透传，不走 Tracker**（新增）
5. **死亡装甲板动态追加，不走固定槽位**（新增）
6. 发布 `WorldTargetArray` 给 `map_node` 绘制小地图

---

# 一、头文件部分

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "posesolver.hpp"
#include "robot_id.hpp"
#include "tracker.hpp"        // ← 新增：固定槽位跟踪器
```

---

### 新增依赖：`"tracker.hpp"`

引入 `Tracker` 类和 `WorldMeasurement` 结构体。`Tracker` 内部使用 `KalmanFilterBox`、`KalmanFilter2d` 和 `HungarianAlgorithm`，但 `pose_node` 只需要直接调用 `Tracker` 的接口。

---

# 二、构造函数（与旧版相同的部分）

构造函数的前半部分与旧版一致：声明参数、初始化 `Config` 和 `PoseSolver`、加载标定、加载 3D Mesh、创建 Pub/Sub/Service。这些不再赘述，详见旧版文档。

---

# 三、核心回调：`armor_callback`（重大更新）

```cpp
void armor_callback(const tensorrt_detect_msgs::msg::DetectionArray::SharedPtr msg)
```

这是 `pose_node` 的干活函数。每收到一帧 `DetectionArray`，就执行一次。**当前版本的核心变化在于回调内部的处理流程**。

---

## 3.1 标定状态检查（与旧版相同）

```cpp
if (!is_calibrated_) {
    RCLCPP_WARN_THROTTLE(..., "标定未就绪，跳过世界坐标计算...");
    return;
}
```

---

## 3.2 遍历检测结果，分三路处理（核心变化）

```cpp
std::vector<WorldMeasurement> meas;
meas.reserve(msg->detections.size());
std::vector<tensorrt_detect_msgs::msg::WorldTarget> dead_targets;
tensorrt_detect_msgs::msg::WorldTarget outpost_target;
bool has_outpost = false;

for (const auto& det : msg->detections) {
    // 世界坐标解算
    cv::Rect car_box(det.car_x, det.car_y, det.car_width, det.car_height);
    cv::Point2f world_pos;
    if (car_box.width > 0 && car_box.height > 0) {
        world_pos = pose_solver_->middletoworld(car_box);
    } else {
        cv::Rect armor_box(det.x, det.y, det.width, det.height);
        world_pos = pose_solver_->middletoworld(armor_box);
    }
```

---

### 世界坐标解算（与旧版一致）

优先使用 `car_box`（车辆框）做解算，回退到 `armor_box`（装甲板框）。`PoseSolver::middletoworld` 执行 PnP + 射线碰撞。

---

### 三路分流

每个检测结果根据其类型进入不同的处理路径：

```text
                    ┌─ idx == OUTPOST  → outpost_target（直接透传）
检测结果 → 解算世界坐标 ─┤
                    ├─ idx == ARMOR && is_dead → dead_targets（动态追加）
                    └─ 正常 R1~S → meas（进入 Tracker）
```

---

#### 路径 1：前哨站直接透传

```cpp
if (det.idx == robot_id::OUTPOST) {
    outpost_target.idx      = 10;
    outpost_target.class_id = det.idx;
    outpost_target.team_id  = det.armor_color;
    outpost_target.is_dead  = det.is_dead;
    outpost_target.score    = det.confidence;
    outpost_target.valid    = true;
    outpost_target.world_x  = world_pos.x;
    outpost_target.world_y  = 0.0f;
    outpost_target.world_z  = world_pos.y;
    has_outpost = true;
    continue;
}
```

前哨站是**固定建筑**，不需要跟踪和 Kalman 平滑。它在场地中的位置不变，直接透传即可。

前哨站放在 `WorldTargetArray` 的**索引 10**（前 10 个是 Tracker 固定槽位）。

---

#### 路径 2：死亡装甲板动态追加

```cpp
if (det.idx == robot_id::ARMOR && det.is_dead) {
    tensorrt_detect_msgs::msg::WorldTarget t;
    t.idx      = 11 + static_cast<int>(dead_targets.size());
    t.class_id = robot_id::ARMOR;
    t.team_id  = robot_id::UNKNOWN;
    t.is_dead  = true;
    t.valid    = true;
    t.world_x  = world_pos.x;
    t.world_y  = 0.0f;
    t.world_z  = world_pos.y;
    dead_targets.push_back(t);
    continue;
}
```

死亡装甲板没有固定的"槽位"（它不是 R1~S 中的任何一个），所以**动态追加**到 `WorldTargetArray` 的末尾（索引 11+）。

---

#### 路径 3：正常装甲板进入 Tracker

```cpp
WorldMeasurement m;
m.class_id = det.idx;
m.team_id  = det.armor_color;
m.score    = det.confidence;
m.is_dead  = det.is_dead;
m.box      = cv::Rect(det.x, det.y, det.width, det.height);
m.world    = world_pos;  // x=world_x, y=world_z
meas.push_back(m);
```

把检测结果填充为 `WorldMeasurement` 结构体，喂给 Tracker。

---

## 3.3 Tracker 更新

```cpp
tracker_.update(meas);
```

一行调用，内部完成：

1. **Predict**：所有 ACTIVE/LOST 槽位的 Kalman 滤波器预测下一状态
2. **Associate**：匈牙利匹配把 `meas` 分配给对应槽位（基于 `team_id` + `class_id` + 距离门限）
3. **Update**：匹配成功的槽位用观测更新 Kalman；未匹配的槽位增加 miss 计数

详见 `docs/core_tracker.md`。

---

## 3.4 构建输出消息（固定槽位 + Outpost + 死亡装甲板）

```cpp
auto world_msg = std::make_shared<tensorrt_detect_msgs::msg::WorldTargetArray>();
world_msg->header = msg->header;
world_msg->targets.resize(11);  // 0-9: Tracker 固定槽位; 10: Outpost
```

---

### 固定槽位输出

```cpp
int valid_count = 0;
for (int i = 0; i < Tracker::NUM_SLOTS; ++i) {
    auto slot = tracker_.get_slot(i);
    auto& target = world_msg->targets[i];
    target.idx      = i;
    target.class_id = slot.class_id;
    target.team_id  = slot.team_id;
    target.is_dead  = slot.is_dead;
    target.score    = slot.score;
    target.valid    = slot.valid;
    target.bbox_x   = slot.smoothed_box.x;
    target.bbox_y   = slot.smoothed_box.y;
    target.bbox_w   = slot.smoothed_box.width;
    target.bbox_h   = slot.smoothed_box.height;
    target.world_x  = slot.smoothed_world.x;
    target.world_y  = 0.0f;
    target.world_z  = slot.smoothed_world.y;
    if (slot.valid) valid_count++;
}
```

---

#### 固定槽位索引约定

| 索引 | 内容 |
|------|------|
| 0~4 | Red R1~R4, Red S |
| 5~9 | Blue R1~R4, Blue S |
| 10 | Outpost（前哨站） |
| 11+ | 动态死亡装甲板 |

---

#### `smoothed_box` 和 `smoothed_world`

注意这里使用的是 `slot.smoothed_box` 和 `slot.smoothed_world`，即经过 **Kalman 滤波平滑后的坐标**，而不是原始检测框。

这是 Tracker 的核心价值：即使检测模型逐帧输出有噪声，经过 Kalman 平滑后的坐标在地图上会更稳定。

---

#### `valid` 的含义

`slot.valid = (state != DEAD) && (hit_count >= min_hit)`

* DEAD 槽位：`valid = false`，`map_node` 不会在地图上绘制
* 首次激活但 hit_count < 2：`valid = false`，防止误检输出
* 正常跟踪中：`valid = true`

---

### Outpost 透传

```cpp
if (has_outpost) {
    world_msg->targets[10] = outpost_target;
    valid_count++;
} else {
    auto& target = world_msg->targets[10];
    target.idx      = 10;
    target.class_id = robot_id::OUTPOST;
    target.team_id  = robot_id::UNKNOWN;
    target.valid    = false;
}
```

如果当前帧没有检测到前哨站，索引 10 仍然存在，但 `valid = false`。

---

### 动态追加死亡装甲板

```cpp
for (const auto& dt : dead_targets) {
    world_msg->targets.push_back(dt);
}
```

死亡装甲板追加到数组末尾，索引从 11 开始。`WorldTargetArray` 的大小是动态的。

---

## 3.5 发布

```cpp
world_pub_->publish(*world_msg);
```

---

## 3.6 日志

```cpp
RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 10000,
    "接收到 %zu 个检测，固定槽位有效 %d / %d，死亡装甲板 %zu",
    msg->detections.size(), valid_count, Tracker::NUM_SLOTS, dead_targets.size());
```

每 10 秒打印一次：

* 输入检测数量
* 有效槽位数 / 总槽位数（10）
* 死亡装甲板数量

---

# 四、成员变量

```cpp
std::unique_ptr<Config> cfg_;
std::unique_ptr<PoseSolver> pose_solver_;
Tracker tracker_;                    // ← 新增：固定槽位跟踪器
bool is_calibrated_ = false;

std::string config_dir_;
std::string input_topic_;
std::string output_topic_;

rclcpp::Subscription<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_sub_;
rclcpp::Publisher<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr world_pub_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
```

---

### `Tracker tracker_`

新增的成员变量，使用默认参数构造（`max_miss=4`, `min_hit=2`, `max_gate_box=200`）。

`tracker_` 的生命周期与 `pose_node` 一致，内部维护 10 个固定槽位的 Kalman 状态。

---

# 五、`main` 函数

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

经典四步法，与旧版相同。

---

# 六、完整数据流回顾

```text
detect_node ──/armor_detections──→ pose_node ──/world_targets──→ map_node
                                      │
                                      ├── PoseSolver（PnP + 射线碰撞）
                                      ├── Tracker（固定槽位跟踪 + Kalman 平滑）
                                      │     ├── KalmanFilterBox（像素框平滑）
                                      │     ├── KalmanFilter2d（世界坐标平滑）
                                      │     └── HungarianAlgorithm（数据关联）
                                      ├── Outpost 直接透传
                                      └── 死亡装甲板动态追加
```

输出的 `WorldTargetArray` 包含：

| 索引 | 内容 | 来源 |
|------|------|------|
| 0~4 | Red R1~R4, Red S | Tracker 固定槽位（Kalman 平滑） |
| 5~9 | Blue R1~R4, Blue S | Tracker 固定槽位（Kalman 平滑） |
| 10 | Outpost | 直接透传（无平滑） |
| 11+ | 死亡装甲板 | 动态追加（无平滑） |

---

# 七、从这份代码里学到的设计要点

## 1. 三路分流的设计

同一个回调中，根据检测类型走不同路径：

* **前哨站**：固定建筑，不需要跟踪 → 直接透传
* **死亡装甲板**：无法预分配槽位 → 动态追加
* **正常装甲板**：需要跟踪和平滑 → 进入 Tracker

这种分流设计比"统一处理"更精确，避免了不必要的 Kalman 更新和数据关联开销。

## 2. Tracker 的引入时机

Tracker 被放在 `pose_node` 而不是 `detect_node`，原因是：

* Tracker 需要**世界坐标**（`WorldMeasurement.world`），这只有 `pose_node` 才能提供
* `detect_node` 只有像素坐标，无法做世界坐标系下的 Kalman 平滑

## 3. 固定索引的约定

`WorldTargetArray` 的前 11 个元素使用**固定索引**（0~10），下游 `map_node` 可以直接用索引访问，不需要遍历查找。

这种设计牺牲了灵活性（数组大小固定为至少 11），但换来了极高的查询效率和代码简洁性。

## 4. 配置降级策略（与旧版一致）

标定加载的 fallback 链：`Config → calib_result.yaml → 等待标定`。

## 5. Header 血缘链（与旧版一致）

```cpp
world_msg->header = msg->header;
```

时间戳沿消息链传递，支持下游时序对齐。

## 6. 算法回退策略（与旧版一致）

```cpp
if (car_box 有效) {
    // 用更稳定的车辆底部中心
} else {
    // 回退到装甲板中心
}
```

优先用高质量数据，降级时也不放弃。
