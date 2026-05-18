# `tracker.hpp` + `tracker.cpp` 逐行讲解

这两份文件实现了 **固定槽位多目标跟踪器**（Fixed-Slot Multi-Object Tracker），是 `pose_node` 内部最核心的算法模块。

在系统中的位置：

```text
detect_node
   ↓  /armor_detections
pose_node
   ├── PoseSolver（像素→世界坐标）
   ├── Tracker（固定槽位多目标跟踪 + Kalman 平滑）  ← 这里
   │     ├── KalmanFilterBox（像素框平滑）
   │     ├── KalmanFilter2d（世界坐标平滑）
   │     └── HungarianAlgorithm（贪心数据关联）
   ↓  /world_targets
map_node
```

---

# 一、设计思想：为什么需要 Tracker？

## 1.1 没有 Tracker 时的问题

直接把 `detect_node` 每帧的检测结果发给 `map_node`，会出现：

* **ID 跳变**：同一辆车在不同帧被分类为不同机器人（R1→R3→R1）
* **坐标抖动**：检测框逐帧噪声导致地图上的点跳来跳去
* **遮挡丢失**：车辆被遮挡一帧，地图上就消失了
* **多检污染**：一个检测框同时匹配到多个目标

## 1.2 固定槽位的核心思想

**每个兵种预分配一个永久槽位（座位）**。无论检测到与否，每个兵种在内存中永远有一个对应的"位置"。

```text
槽位 0: Red R1  ← 有且仅有一个
槽位 1: Red R2
槽位 2: Red R3
槽位 3: Red R4
槽位 4: Red S
槽位 5: Blue R1
槽位 6: Blue R2
槽位 7: Blue R3
槽位 8: Blue R4
槽位 9: Blue S
```

每个槽位内部维护独立的 Kalman 滤波器，状态只有三种：`ACTIVE`（跟踪中）、`LOST`（短暂丢失）、`DEAD`（未激活/已超时）。

**注：Outpost（前哨站）不走 Tracker，由 `pose_node` 直接透传。**

---

# 第二部分：`tracker.hpp`

## 一、`TrackerParams` 结构体

```cpp
struct TrackerParams {
    int max_miss = 4;               // 连续丢失多少帧后标记为 DEAD
    int min_hit = 2;                // 最少命中次数才对外输出
    float max_gate_box = 200.0f;    // 像素框中心距离门限
};
```

---

### 参数说明

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `max_miss` | 4 | 连续漏检 4 帧后，槽位从 LOST 变为 DEAD |
| `min_hit` | 2 | 至少连续命中 2 次，槽位才对外输出（防误检） |
| `max_gate_box` | 200 像素 | 匹配门限：检测框与槽位预测位置的距离超过 200 像素则拒绝匹配 |

---

### `max_miss = 4` 的工程含义

在 30fps 下，4 帧 = 133ms。一辆车被遮挡 133ms 后就会标记为 DEAD。

这个值需要在**灵敏度**和**稳定性**之间平衡：

* 太小（如 1）：遮挡一帧就丢失，地图闪烁
* 太大（如 20）：目标已经离开视野，但地图上还显示旧位置

4 帧是 RoboMaster 场景下的经验值。

---

### `min_hit = 2` 的工程含义

只命中 1 次就输出，可能是误检。连续命中 2 次才输出，可以过滤掉大部分偶发误检。

代价是目标首次出现时会有 1 帧的延迟。

---

## 二、`Tracker` 类

```cpp
class Tracker {
public:
    static constexpr int NUM_SLOTS = 10;
    static constexpr int SLOT_RED_R1   = 0;
    // ... (共 10 个槽位)
```

---

### 固定槽位布局

| 槽位索引 | 阵营 | 类别 |
|---------|------|------|
| 0 | RED | R1 |
| 1 | RED | R2 |
| 2 | RED | R3 |
| 3 | RED | R4 |
| 4 | RED | S |
| 5 | BLUE | R1 |
| 6 | BLUE | R2 |
| 7 | BLUE | R3 |
| 8 | BLUE | R4 |
| 9 | BLUE | S |

**注意**：Outpost（前哨站）不走 Tracker，由 `pose_node` 直接透传到 `WorldTargetArray` 的索引 10。

---

### 公共接口

```cpp
explicit Tracker(const TrackerParams& params = TrackerParams());
void update(const std::vector<WorldMeasurement>& detections);
SlotOutput get_slot(int idx) const;
void reset();
```

| 方法 | 作用 |
|------|------|
| `update()` | 输入新一帧观测，更新所有槽位状态 |
| `get_slot()` | 获取指定槽位的当前输出状态 |
| `reset()` | 重置所有槽位到 DEAD 状态 |

---

### `SlotOutput` 结构体

```cpp
struct SlotOutput {
    int slot_idx = 0;
    int team_id = 0;
    int class_id = 0;
    bool valid = false;          // 非 DEAD 且满足 min_hit
    TrackState state = TrackState::LOST;
    cv::Rect smoothed_box;
    cv::Point2f smoothed_world;
    bool is_dead = false;
    float score = 0.0f;
};
```

`pose_node` 通过 `get_slot(i)` 遍历 10 个槽位，把 `SlotOutput` 填充到 `WorldTargetArray` 的前 10 个元素中。

---

### `valid` 字段的判断逻辑

```cpp
out.valid = (s.state != TrackState::DEAD) && (s.hit_count >= params_.min_hit);
```

两个条件必须同时满足：

1. 槽位不是 DEAD（至少是 ACTIVE 或 LOST）
2. 历史命中次数 ≥ `min_hit`（防止误检输出）

---

## 三、`Slot` 内部结构

```cpp
struct Slot {
    int slot_idx = 0;
    int team_id = 0;
    int class_id = 0;
    int hit_count = 0;
    int miss_count = 0;
    TrackState state = TrackState::DEAD;
    bool initialized = false;

    KalmanFilterBox kf_box;   // 8维像素框滤波
    KalmanFilter2d kf_world;  // 4维世界坐标滤波

    cv::Rect last_box;
    cv::Point2f last_world;
    float last_score = 0.0f;
    bool last_is_dead = false;
};
```

每个槽位拥有**独立的两个 Kalman 滤波器**：

* `kf_box`：平滑像素框位置（8 维：位置 + 速度）
* `kf_world`：平滑世界坐标位置（4 维：位置 + 速度）

---

# 第三部分：`tracker.cpp`

## 一、构造函数

```cpp
Tracker::Tracker(const TrackerParams& params) : params_(params) {
    slots_.resize(NUM_SLOTS);

    auto init_slot = [this](int idx, int team, int cls) {
        slots_[idx].slot_idx = idx;
        slots_[idx].team_id  = team;
        slots_[idx].class_id = cls;
        slots_[idx].state    = TrackState::DEAD;
        slots_[idx].initialized = false;
        slots_[idx].hit_count = 0;
        slots_[idx].miss_count = 0;
    };

    init_slot(SLOT_RED_R1, robot_id::RED, robot_id::R1);
    // ... 初始化所有 10 个槽位
}
```

---

### 构造时"焊死"属性

`team_id` 和 `class_id` 在构造时确定，之后**永远不会改变**。

这是固定槽位的核心设计：Red R1 的槽位永远是 Red R1，不可能变成 Blue R2。即使某帧检测模型把 Blue R2 错误分类为 Red R1，Tracker 的匈牙利匹配会因为其他字段（如位置距离）不匹配而拒绝这次关联。

---

## 二、`update` 方法（核心算法）

### Step 1: Predict（预测）

```cpp
for (auto& slot : slots_) {
    if (!slot.initialized || slot.state == TrackState::DEAD) continue;
    if (slot.last_is_dead) continue;  // 死亡车辆冻结显示

    auto box_pred = slot.kf_box.predict();
    slot.last_box = cv::Rect(
        static_cast<int>(box_pred[0] - box_pred[2] / 2.0f),
        static_cast<int>(box_pred[1] - box_pred[3] / 2.0f),
        static_cast<int>(box_pred[2]),
        static_cast<int>(box_pred[3])
    );
    auto world_pred = slot.kf_world.predict();
    slot.last_world = cv::Point2f(world_pred[0], world_pred[1]);
}
```

---

#### 死亡车辆冻结

```cpp
if (slot.last_is_dead) continue;
```

被标记为"死亡"的车辆，Kalman 滤波器不再 predict，位置保持在最后已知坐标。这样在地图上，死亡车辆会"定格"在被摧毁的位置，不会因为没有新观测而漂移。

---

#### `box_pred` 的坐标转换

Kalman 滤波器内部存储的是 `[cx, cy, w, h]`（中心点 + 宽高），但 `last_box` 需要 `[x, y, w, h]`（左上角 + 宽高），所以要做转换：

```cpp
x = cx - w/2
y = cy - h/2
```

---

### Step 2: 匈牙利匹配

```cpp
int n_rows = NUM_SLOTS;  // 10
int n_cols = static_cast<int>(detections.size());

radar_core::tracker::HungarianAlgorithm hungarian;
std::vector<std::vector<float>> cost_matrix(n_rows, std::vector<float>(n_cols, 1e6f));

for (int r = 0; r < n_rows; ++r) {
    const auto& slot = slots_[r];
    for (int c = 0; c < n_cols; ++c) {
        const auto& det = detections[c];
        // 硬过滤：team/class 必须严格匹配槽位
        if (det.team_id != slot.team_id || det.class_id != slot.class_id) {
            cost_matrix[r][c] = 1e6f;
            continue;
        }
        // 门限过滤
        float gate = (slot.initialized && slot.state != TrackState::DEAD)
                         ? params_.max_gate_box : 1e6f;
        float d = box_center_distance(slot.last_box, det.box);
        if (d >= gate) {
            cost_matrix[r][c] = 1e6f;
            continue;
        }
        cost_matrix[r][c] = d;
    }
}
```

---

#### 代价矩阵的三层过滤

| 层级 | 条件 | 代价 |
|------|------|------|
| 第 1 层 | `team_id` 或 `class_id` 不匹配 | `1e6f`（硬拒绝） |
| 第 2 层 | 像素距离 ≥ `max_gate_box` | `1e6f`（门限拒绝） |
| 第 3 层 | 通过以上两层 | 实际像素距离（合法代价） |

---

#### DEAD/未初始化槽位的特殊处理

```cpp
float gate = (slot.initialized && slot.state != TrackState::DEAD)
                 ? params_.max_gate_box : 1e6f;
```

对于 DEAD 或未初始化的槽位，门限放宽到 `1e6f`（即无门限），允许它们在任意距离上首次匹配/复活。

这解决了一个关键问题：如果目标刚进入视野（DEAD → ACTIVE），它的位置和槽位的"默认位置"可能差很远。如果不放宽门限，首次激活永远无法成功。

---

### Step 3: 匹配结果处理

```cpp
std::vector<int> assignment;
hungarian.Solve(cost_matrix, assignment);

for (int r = 0; r < n_rows; ++r) {
    auto& slot = slots_[r];
    int c = assignment[r];
    float gate_check = (slot.initialized && slot.state != TrackState::DEAD)
                           ? params_.max_gate_box : 1e6f;
    bool matched = (c >= 0 && c < n_cols && cost_matrix[r][c] < gate_check);

    if (matched) {
        // 匹配成功 → Kalman 更新 + 状态更新
        const auto& det = detections[c];

        std::vector<float> box_meas = {
            det.box.x + det.box.width  / 2.0f,
            det.box.y + det.box.height / 2.0f,
            static_cast<float>(det.box.width),
            static_cast<float>(det.box.height)
        };

        if (!slot.initialized || slot.state == TrackState::DEAD) {
            // 首次激活或 DEAD 复活：重置 Kalman
            slot.kf_box.reset(box_meas);
            slot.kf_world.reset({det.world.x, det.world.y});
            slot.initialized = true;
        } else {
            // 正常更新
            auto box_upd = slot.kf_box.update(box_meas);
            slot.last_box = cv::Rect(...);
            auto world_upd = slot.kf_world.update({det.world.x, det.world.y});
            slot.last_world = cv::Point2f(world_upd[0], world_upd[1]);
        }

        slot.hit_count++;
        slot.miss_count = 0;
        slot.state = TrackState::ACTIVE;
        slot.last_score = det.score;
        slot.last_is_dead = det.is_dead;
        det_matched[c] = true;
    } else {
        // 未匹配
        if (slot.last_is_dead) {
            // 死亡车辆冻结：不增加 miss
        } else {
            slot.miss_count++;
            slot.state = (slot.miss_count > params_.max_miss)
                             ? TrackState::DEAD : TrackState::LOST;
        }
    }
}
```

---

#### 首次激活 vs 正常更新

* **首次激活**（`!initialized || DEAD`）：`reset()` 重置 Kalman 滤波器，以当前观测为基准
* **正常更新**（ACTIVE/LOST）：`update()` 用观测修正预测

---

#### 死亡车辆的 miss 处理

```cpp
if (slot.last_is_dead) {
    // 死亡车辆冻结：不增加 miss，保持 ACTIVE
}
```

死亡车辆被标记为 `is_dead = true` 后，即使后续帧没有匹配到检测，也不会增加 `miss_count`。这让死亡车辆在地图上**永远定格**在最后已知位置。

---

## 三、`get_slot` 方法

```cpp
Tracker::SlotOutput Tracker::get_slot(int idx) const {
    SlotOutput out;
    if (idx < 0 || idx >= NUM_SLOTS) return out;

    const auto& s = slots_[idx];
    out.slot_idx  = s.slot_idx;
    out.team_id   = s.team_id;
    out.class_id  = s.class_id;
    out.state     = s.state;
    out.smoothed_box   = s.last_box;
    out.smoothed_world = s.last_world;
    out.is_dead   = s.last_is_dead;
    out.score     = s.last_score;
    out.valid = (s.state != TrackState::DEAD) && (s.hit_count >= params_.min_hit);
    return out;
}
```

---

### `valid` 的语义

`valid = true` 需要同时满足：

1. `state != DEAD`：槽位处于跟踪状态
2. `hit_count >= min_hit`：连续命中次数达标

首次激活后的前 `min_hit - 1` 帧，虽然槽位不是 DEAD，但 `valid` 仍是 `false`。这避免了误检输出。

---

# 第四部分：从 Tracker 学到的设计要点

## 1. 固定槽位 vs 动态 Track ID

传统多目标跟踪（如 SORT、DeepSORT）使用动态 Track ID：

* 新目标出现 → 分配新 ID
* 目标消失 → ID 过期回收
* 如果 ID 管理出错 → 同一目标有多个 ID，或不同目标共享 ID

固定槽位彻底消除了 ID 管理问题：每个兵种永远只有一个槽位，`team_id` 和 `class_id` 在构造时"焊死"。

## 2. Kalman 预测 + 数据关联的解耦

Tracker 把"预测"和"关联"分成两个独立步骤：

1. **Predict**：每个槽位的 Kalman 滤波器独立预测
2. **Associate**：匈牙利匹配把检测分配给槽位
3. **Update**：根据匹配结果更新 Kalman

这是 SORT（Simple Online and Realtime Tracking）框架的标准流程。

## 3. 死亡车辆的冻结策略

死亡车辆的特殊处理（不 predict、不增加 miss）是一个精心设计的业务逻辑：

* 如果死亡车辆继续 predict，Kalman 会外推它继续移动，但实际车辆已经停了
* 如果死亡车辆增加 miss，超过 `max_miss` 后会变为 DEAD，从地图上消失

冻结策略让死亡车辆在地图上保持最后位置，提供视觉反馈。

## 4. 门限的分层设计

* **未初始化/DEAD 槽位**：无门限（`1e6f`），允许任意距离首次匹配
* **ACTIVE/LOST 槽位**：有门限（`max_gate_box`），防止远距离误匹配

这种分层设计保证了首次激活的成功率，同时避免了跟踪过程中的误匹配。
