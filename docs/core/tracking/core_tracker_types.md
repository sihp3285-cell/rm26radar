# `tracker_types.hpp` 逐行讲解

这份文件定义了 **Tracker 子系统的公共数据结构**，是 `tracker.hpp`、`kalman.hpp` 和 `pose_node.cpp` 共享的类型基础。

在你的系统里，Tracker 的位置是：

```text
detect_node
   ↓  /armor_detections
pose_node
   ├── PoseSolver（像素→世界坐标）
   ├── Tracker（固定槽位多目标跟踪 + Kalman 平滑）  ← 这个模块
   ↓  /world_targets
map_node
```

把类型定义独立到一个头文件里，是为了避免循环依赖：`tracker.hpp` 需要 `WorldMeasurement`，`pose_node.cpp` 也需要 `WorldMeasurement` 和 `TrackedTarget`，如果都塞在 `tracker.hpp` 里，耦合太紧。

---

# 一、头文件保护与依赖

```cpp
#pragma once
#include <opencv2/opencv.hpp>
```

只依赖 OpenCV，用于 `cv::Rect` 和 `cv::Point2f`。

---

# 二、`TrackState` 枚举

```cpp
enum class TrackState {
    ACTIVE = 0,    // 正常跟踪中
    PREDICTED = 1, // 短暂丢失，卡尔曼外推中（仍显示在地图上）
    LOST   = 2,    // 丢失较久，不再对外输出
    DEAD   = 3     // 超过最大丢失帧，已删除
};
```

---

### 四状态生命周期

每个跟踪槽位的状态机（从三状态扩展为四状态）：

```text
        ┌──────────────────┐
        │      DEAD        │ ← 初始状态 / 超时删除
        │   (未激活/死亡)    │
        └────────┬─────────┘
                 │ 首次匹配到检测
                 ↓
        ┌──────────────────┐
        │     ACTIVE       │ ← 正常跟踪
        │   (持续匹配)      │
        └────────┬─────────┘
                 │ 连续 miss ≤ max_predict
                 ↓
        ┌──────────────────┐
        │    PREDICTED     │ ← 短暂丢失，卡尔曼外推
        │  (仍对外输出)     │    对外输出 valid=true
        └────────┬─────────┘
                 │ 连续 miss > max_predict
                 ↓
        ┌──────────────────┐
        │      LOST        │ ← 丢失较久
        │  (不对外输出)     │    对外输出 valid=false
        └────────┬─────────┘
                 │ miss_count > max_miss
                 ↓
        ┌──────────────────┐
        │      DEAD        │ ← 超时，释放槽位
        └──────────────────┘
```

> **关键变化**：PREDICTED 是新增的中间状态。当目标短暂丢失（miss_count ≤ max_predict）时，槽位进入 PREDICTED 而非 LOST。PREDICTED 状态下卡尔曼滤波器继续外推，目标仍然显示在地图上（valid=true）。只有当丢失超过 max_predict 帧后才进入 LOST，此时不再对外输出（valid=false）。

---

### 为什么用 `enum class` 而不是普通 `enum`？

`enum class` 是 C++11 引入的**强类型枚举**：

* 不会隐式转换为 `int`
* 不会污染命名空间
* 必须用 `TrackState::ACTIVE` 访问，代码更清晰

---

# 三、`WorldMeasurement` 结构体

```cpp
struct WorldMeasurement {
    int class_id = 0;
    int team_id = 0;
    float score = 0.0f;
    bool is_dead = false;
    cv::Rect box;          // 像素框 [x, y, w, h]
    cv::Point2f world;     // world.x = world_x, world.y = world_z
};
```

---

### 用途

这是 **Tracker 的单帧输入**。`pose_node` 在 `armor_callback` 中，对每个 `DetectionBox` 做完世界坐标解算后，填充一个 `WorldMeasurement`，然后喂给 `Tracker::update()`。

---

### 字段说明

| 字段 | 含义 | 来源 |
|------|------|------|
| `class_id` | 机器人类别（R1~R4、S） | `det.idx`（经过 `runClassify` 映射后） |
| `team_id` | 队伍（RED/BLUE） | `det.armor_color` |
| `score` | 检测置信度 | `det.confidence` |
| `is_dead` | 是否为死亡装甲板 | `det.is_dead` |
| `box` | 像素检测框 | `cv::Rect(det.x, det.y, det.width, det.height)` |
| `world` | 世界坐标 | `PoseSolver::middletoworld()` 的返回值 |

---

### `world` 坐标的约定

```cpp
cv::Point2f world;  // world.x = world_x, world.y = world_z
```

注意 `world.y` 存储的是 **场地 Z 坐标**（长度方向），而不是场地 Y（高度）。这与 `WorldTarget` 消息中的 `world_x` / `world_z` 对应。

`world_y`（高度）始终为 0（平地假设），所以这里不需要存储。

---

# 四、`TrackedTarget` 结构体

```cpp
struct TrackedTarget {
    int track_id = 0;
    int team_id = 0;
    int class_id = 0;
    int hit_count = 0;
    int miss_count = 0;
    TrackState state = TrackState::ACTIVE;

    cv::Rect smoothed_box;
    cv::Point2f smoothed_world;
    bool is_dead = false;
    float score = 0.0f;
};
```

---

### 用途

这是 **Tracker 的单目标输出**。注意当前版本的 `pose_node` 实际上并没有直接使用 `TrackedTarget`，而是通过 `Tracker::SlotOutput` 获取槽位状态。`TrackedTarget` 保留为通用数据结构，可能在独立测试或未来的扩展中使用。

---

### 关键字段

| 字段 | 含义 |
|------|------|
| `track_id` | 全局唯一跟踪 ID（固定槽位架构中等于槽位索引） |
| `smoothed_box` | 经过 Kalman 滤波平滑后的像素框 |
| `smoothed_world` | 经过 Kalman 滤波平滑后的世界坐标 |
| `hit_count` | 连续命中次数（达到 `min_hit` 才对外输出） |
| `miss_count` | 连续丢失次数（超过 `max_miss` 判定为 DEAD） |

---

### `smoothed_box` vs `box`

`WorldMeasurement.box` 是原始检测框（可能有噪声、跳变）。

`TrackedTarget.smoothed_box` 是 Kalman 滤波后的框，更平滑、更稳定。在低帧率或遮挡场景下，Kalman 预测可以填补漏检帧的空缺。

---

# 五、从这份文件学到的设计要点

## 1. 类型分离

把公共数据结构放在独立头文件里，是 C++ 项目中避免循环依赖的标准做法：

```text
tracker_types.hpp   ← 纯数据结构，无逻辑
    ↑           ↑
tracker.hpp     kalman.hpp   ← 算法实现
    ↑
pose_node.cpp   ← 业务逻辑
```

## 2. 输入输出对称设计

`WorldMeasurement`（输入）和 `TrackedTarget`（输出）的字段高度对称：

* 输入有 `box` → 输出有 `smoothed_box`
* 输入有 `world` → 输出有 `smoothed_world`
* 输入有 `class_id` / `team_id` → 输出原样保留

这种对称性让数据流清晰可读：进什么、出什么，一目了然。

## 3. 状态枚举的设计

三状态（ACTIVE / LOST / DEAD）足以覆盖多目标跟踪的所有生命周期场景。状态数量太少会导致逻辑不够表达，太多会增加复杂度。三状态是工程实践中最常用的折中。
