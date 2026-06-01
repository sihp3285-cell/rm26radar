# 多目标跟踪系统 (Multi-Object Tracking System)

## 1. 系统架构

本项目采用 **Kalman Filter + Hungarian Matching + BotIdentity 身份稳定** 的三层跟踪架构：

```
WorldMeasurements (每帧检测结果)
    │
    ├── 1. Kalman Predict ──→ 所有活跃 track 预测下一帧状态
    │
    ├── 2. Hungarian Match ──→ tracks × detections 代价矩阵求解
    │    ├── team_id 硬过滤 (不同队伍 cost=∞)
    │    ├── box_center_distance (空间距离主导)
    │    └── class_mismatch_penalty (跨兵种惩罚)
    │
    ├── 3. Kalman Update ──→ 匹配到的 track 用观测更新
    │    ├── 新 track 创建 (未匹配观测 → new PhysicalTrack)
    │    └── 未匹配 track 状态机推进 (ACTIVE→PREDICTED→LOST→DEAD)
    │
    └── 4. Output ──→ 10 个 official slot + Outpost + 死亡装甲板
         └── arbitrate_outputs() 同队同兵种仲裁
```

**文件结构**：

| 文件 | 职责 |
|---|---|
| [kalman.hpp/cpp](../src/tensorrt_detect/include/tensorrt_detect/core/kalman.hpp) | Kalman 滤波器实现 (Box 8D + World 4D) |
| [tracker.hpp/cpp](../src/tensorrt_detect/include/tensorrt_detect/core/tracker.hpp) | Tracker 核心：匹配、状态机、输出 |
| [tracker_types.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/tracker_types.hpp) | 公共数据结构 |
| [bot_identity.hpp/cpp](../src/tensorrt_detect/include/tensorrt_detect/core/bot_identity.hpp) | BotIdentity 身份稳定器 |
| [robot_id.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/robot_id.hpp) | RoboMaster ID 枚举 |
| [hungarian.hpp/cpp](../src/tensorrt_detect/include/tensorrt_detect/core/hungarian.hpp) | 匈牙利算法求解器 |

---

## 2. Kalman 滤波器 (Eigen 实现)

### 2.1 KalmanFilterBox — 像素空间 8 维 Kalman

```cpp
// kalman.hpp:9-56
class KalmanFilterBox {
    // 状态向量: [cx, cy, w, h, vx, vy, vw, vh]  (8×1)
    Eigen::Matrix<float, 8, 1> x;
    Eigen::Matrix<float, 8, 8> P;  // 误差协方差
    Eigen::Matrix<float, 8, 8> F;  // 状态转移矩阵
    Eigen::Matrix<float, 4, 8> H;  // 观测矩阵 (仅观测位置+尺寸)
    Eigen::Matrix<float, 4, 4> R;  // 测量噪声协方差
    Eigen::Matrix<float, 8, 8> Q;  // 过程噪声协方差
};
```

**运动模型（带持续白噪声过程噪声）**：

```
F = | I(4×4)   dt·I(4×4) |     H = | I(4×4)   0(4×4) |
    | 0(4×4)   I(4×4)    |

Q = | dt³/3·q²·I   dt²/2·q²·I |   (continuous white noise model)
    | dt²/2·q²·I   dt·q²·I    |
```

**默认参数**：`dt=1.0`, `q_std=2.0`, `r_std=1.0`

**防跳变保护**：
```cpp
// kalman.cpp — 位置状态跳变检测
if (std::abs(x(0) - bbox[0]) > 100.0f ||   // cx 跳变 > 100px
    std::abs(x(1) - bbox[1]) > 100.0f) {   // cy 跳变 > 100px
    reset(bbox);  // 重置滤波器，避免发散
}
```

### 2.2 KalmanFilter2d — 世界坐标 4 维 Kalman

```cpp
// kalman.hpp:62-115
class KalmanFilter2d {
    // 状态向量: [x, z, vx, vz]  (4×1)
    // 注意：y=0 平面（地面），所以世界坐标用 (x, z) 表示
    Eigen::Matrix<float, 4, 1> x;
    Eigen::Matrix<float, 4, 4> P, F, Q;
    Eigen::Matrix<float, 2, 4> H;   // 2D 观测
    Eigen::Matrix<float, 2, 2> R;
};
```

**防跳变保护**：位置跳变 > 5m 时自动重置。

**默认参数**：`q_std=2.0`, `r_std=1.0`, `dt=0.1`

### 2.3 设计要点

- **固定大小矩阵**：使用 `Eigen::Matrix<float, N, N>` 而非 `Eigen::MatrixXf`，全部栈分配，零动态内存
- **每次 predict 更新 Q 矩阵**：通过 `updateQ(dt)` 按当前时间步长重新计算过程噪声
- **EIGEN_MAKE_ALIGNED_OPERATOR_NEW**：确保 Eigen 内存对齐，可在 STL 容器中使用

---

## 3. 匈牙利匹配

### 3.1 代价矩阵构建

```cpp
// tracker.cpp:164-194 — 构建 tracks × detections 代价矩阵
for (int r = 0; r < n_rows; ++r) {           // 遍历每个 track
    const auto& track = tracks_[r];
    auto [track_stable_cls, _] = track.bot_id.getStableClass();

    for (int c = 0; c < n_cols; ++c) {        // 遍历每个 detection
        const auto& det = detections[c];

        // 1. team_id 硬过滤 — 不同队伍直接 reject
        if (det.team_id != track.team_id) {
            cost_matrix[r][c] = 1e6f;
            continue;
        }

        // 2. 空间距离（主度量）
        float gate = (track.initialized && track.state != TrackState::DEAD)
                         ? params_.max_gate_box  // 默认 200px
                         : 1e6f;                 // 新 track: 无门限
        float d = box_center_distance(track.last_box, det.box);
        float cost = d;

        // 3. 跨兵种惩罚 — 辅助项，仅在已有稳定身份时生效
        if (track_stable_cls >= 0 && det.class_id != track_stable_cls) {
            cost += params_.class_mismatch_penalty;  // 默认 300px 等效距离
        }

        // 4. 门限过滤
        if (cost >= gate) {
            cost_matrix[r][c] = 1e6f;
            continue;
        }
        cost_matrix[r][c] = cost;
    }
}
```

**匹配策略哲学**：
- **空间距离主导** — track 的物理位置是最可靠的关联依据
- **兵种辅助** — `class_mismatch_penalty` 是一个微弱偏置，在空间上足够接近的情况下才影响匹配
- **渐进发现** — 初期没有稳定兵种时（`track_stable_cls < 0`），完全由空间距离决定

### 3.2 贪心求解

```cpp
// hungarian.cpp — "匈牙利" 算法（实际使用迭代最小成本匹配）
// 对于小规模匹配（track 数 ≤ 20），贪心算法 ≈ Kuhn-Munkres
// 且速度 ~10× 更快，适合实时系统
```
项目使用 `HungarianAlgorithm::Solve()` 完成成本矩阵 → 最优匹配的求解。

---

## 4. Tracker 核心状态机

### 4.1 状态定义

```cpp
// tracker_types.hpp
enum class TrackState {
    ACTIVE,      // 本帧匹配到观测，置信度高
    PREDICTED,   // 短暂丢失（≤ max_predict 帧），仍输出 Kalman 预测
    LOST,        // 超出预测期但未超时，仅内部维护
    DEAD         // 超出 max_miss 帧或清理，不再跟踪
};
```

### 4.2 状态转移

```
                  ┌───────────────────────────────────┐
                  │                                    │
     [创建] ──→ ACTIVE ──(未匹配)──→ PREDICTED ──(未匹配)──→ LOST ──(未匹配)──→ DEAD
                  │  ↑                  │  ↑               │                     │
                  │  └──(匹配到)────────┘  └──(匹配到)─────┘                     │
                  └──────────────────────────────────────────────────────────────┘
                                           (匹配到时恢复 ACTIVE)

     死亡车辆冻结: dead track 匹配到后保持 DEAD 状态，不做 Kalman 更新
```

### 4.3 核心更新循环

```cpp
// tracker.cpp:141-323
void Tracker::update(const std::vector<WorldMeasurement>& detections) {

    // Step 1: Kalman predict — 所有活跃 track
    for (auto& track : tracks_) {
        if (track.initialized && track.state != DEAD && !track.last_is_dead) {
            auto box_pred = track.kf_box.predict();
            track.last_box = ...;    // 预测位置 → 匹配时的参考位置
            auto world_pred = track.kf_world.predict();
            track.last_world = ...;
        }
    }

    // Step 2: 匈牙利匹配 — 构建代价矩阵 + 求解
    HungarianAlgorithm hungarian;
    std::vector<std::vector<float>> cost_matrix(n_rows, ...);
    // ... 代价矩阵构建 (见 §3.1)
    hungarian.Solve(cost_matrix, assignment);

    // Step 3: 处理匹配结果
    for (int r = 0; r < n_rows; ++r) {
        int c = assignment[r];
        if (matched) {
            // 匹配到: Kalman update + BotIdentity update
            track.kf_box.update(box_meas);      // 像素框
            track.kf_world.update(world_meas);  // 世界坐标
            track.bot_id.update(det.class_id, det.score);
            track.state = ACTIVE;  // 恢复活跃
            track.miss_count = 0;
        } else {
            // 未匹配: 推进丢失状态机
            track.miss_count++;
            if (track.miss_count > max_miss)      track.state = DEAD;
            else if (track.miss_count <= max_predict) track.state = PREDICTED;
            else                                    track.state = LOST;
        }
    }

    // Step 4: 新 track 创建 (未匹配的 detection)
    for (each unmatched detection) {
        PhysicalTrack track;
        track.team_id = det.team_id;
        track.kf_box.reset(box_meas);      // 初始化 Kalman
        track.kf_world.reset(world_meas);
        track.bot_id.update(det.class_id, det.score);
        track.state = ACTIVE;
        tracks_.push_back(track);
    }

    // Step 5: 清理 DEAD 超时 track
    tracks_.erase(/* miss_count > max_miss + 2 */);
}
```

### 4.4 跟踪参数配置

```yaml
# configs/tracker.yaml
max_miss: 4                # 连续丢失 4 帧后标记 DEAD
max_predict: 2             # 丢失 2 帧内保持 PREDICTED（仍输出）
min_hit: 2                 # 最少命中 2 次才对外输出（抑制假阳性）
max_gate_box: 200.0        # 像素框中心距离门限
class_mismatch_penalty: 300.0  # 跨兵种匹配惩罚（像素等效距离）
max_tracks: 20             # 最大同时跟踪的 PhysicalTrack 数量
```

---

## 5. PhysicalTrack — 单目标物理跟踪实例

```cpp
// tracker.hpp:73-90
struct PhysicalTrack {
    int track_id = -1;           // 全局唯一 ID
    int team_id = UNKNOWN;       // 首次匹配设定，之后不变
    int hit_count = 0;           // 累计命中次数
    int miss_count = 0;          // 连续丢失次数
    TrackState state = DEAD;
    bool initialized = false;    // Kalman 是否已初始化

    KalmanFilterBox kf_box;   // 8D 像素框 Kalman
    KalmanFilter2d kf_world;  // 4D 世界坐标 Kalman
    BotIdentity bot_id;       // 身份轨迹池

    cv::Rect last_box;         // 最新平滑后的像素框
    cv::Point2f last_world;    // 最新平滑后的世界坐标
    cv::Point2f detected_world;// ACTIVE 时优先使用原始检测坐标
    float last_score = 0.0f;
    bool last_is_dead = false;
};
```

**双 Kalman 跟踪** 的设计理由：
- `kf_box` (8D) 在像素空间跟踪，提供平滑的边界框输出，对后续分类和显示友好
- `kf_world` (4D) 在世界坐标空间跟踪，消除透视畸变，提供更准确的运动估计
- 两个滤波器独立运行，互不干扰

---

## 6. BotIdentity — 身份轨迹池

[bot_identity.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/bot_identity.hpp) / [bot_identity.cpp](../src/tensorrt_detect/src/core/bot_identity.cpp)

### 6.1 设计思想

分类器输出是**逐帧独立**的，可能因视角、遮挡、模型噪声而产生帧间类别翻转。BotIdentity 维护跨帧历史，通过**指数衰减加权投票**平滑分类输出：

```
稳定身份 = 加权的历史类别频率分布
权重 = decay^age  (越近的观测权重越大)
```

### 6.2 算法

```cpp
// bot_identity.cpp — getStableClass() 核心逻辑
std::pair<int, float> BotIdentity::getStableClass() const {
    if (history_.size() < minHistoryForStable_) {
        return {-1, 0.0};  // 历史不足，返回无效身份
    }

    std::vector<float> class_weights(numClasses_, 0.0f);
    float total_weight = 0.0f;
    int age = 0;

    // 从最旧到最新遍历，越旧权重越低
    for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
        float w = std::pow(decay_, age);  // decay^age
        class_weights[it->class_id] += w * it->class_conf;
        total_weight += w;
        ++age;
    }

    // 归一化：每个类别的归一化权重
    auto max_it = std::max_element(class_weights.begin(), class_weights.end());
    float max_weight = *max_it;
    int best_class = std::distance(class_weights.begin(), max_it);

    float normalized_conf = total_weight > 0 ? max_weight / total_weight : 0.0f;
    return {best_class, normalized_conf};
}
```

### 6.3 配置参数

```cpp
struct BotIdentityConfig {
    int maxHistory = 50;            // 最大保留历史观测数
    int purgeThreshold = 30;        // 超过此数时清理最旧的一半
    int minHistoryForStable = 8;    // 最少累积观测才输出稳定身份
    float decay = 0.97f;            // 指数衰减因子
    int numClasses = 9;             // CAR=0, ARMOR=1, R1=2, ..., AIRPLANE=8
};
```

**参数调优指南**：
- `decay=0.97`：约 22 帧权重降至一半（`0.97^22 ≈ 0.5`），@30fps ≈ 0.7s 半衰期
- `minHistoryForStable=8`：需要 8 次观测才开始输出身份，有效抑制头几帧的不稳定分类
- `maxHistory=50`：保留约 1.7s 的历史，单帧噪声不会影响整体分布

### 6.4 生命周期

```
track 创建
    │
    ├── update() ──→ 添加新观测到 history_ (FIFO)
    │                  age++ for all existing observations
    │                  if history_.size() > purgeThreshold: 清理最旧的一半
    │
    ├── markLost() ──→ 漏检计数累加，触发 purge 检查
    │
    └── reset() ──→ 清空所有历史
```

---

## 7. 输出映射系统

### 7.1 Official Slot 架构

```cpp
// tracker.hpp:29-40
static constexpr int NUM_SLOTS = 10;
static constexpr int SLOT_RED_R1   = 0;
static constexpr int SLOT_RED_R2   = 1;
static constexpr int SLOT_RED_R3   = 2;
static constexpr int SLOT_RED_R4   = 3;
static constexpr int SLOT_RED_S    = 4;
static constexpr int SLOT_BLUE_R1  = 5;
static constexpr int SLOT_BLUE_R2  = 6;
static constexpr int SLOT_BLUE_R3  = 7;
static constexpr int SLOT_BLUE_R4  = 8;
static constexpr int SLOT_BLUE_S   = 9;
```

**10 个固定槽位** 对应 RoboMaster 比赛的官方队伍编制（红方 R1-R4+S，蓝方 R1-R4+S）。每个槽位的 `(team, class)` 是 **nominal** 的，实际内容通过 `(team, stable_class) → slot` 映射动态填充。

### 7.2 Track → Slot 映射

```cpp
// tracker.cpp:75-131 — get_outputs()
// 遍历所有 track，按 (team_id, stable_class) 映射到 official slot
// 如果多个 track 竞争同一个 slot，保留 stable_class_conf 最高的
// 未被占据的 slot 保持 invalid
```

### 7.3 同队同兵种仲裁

```cpp
// tracker.cpp:47-72 — arbitrate_outputs()
// 按 (team_id << 32 | stable_class_id) 分组
// 每组只保留 stable_class_conf 最高的一个
// 其他 suppressed（valid=false, stable_class_id=-1）
```

### 7.4 特殊目标处理

```cpp
// pose_node.cpp:237-254 — Outpost 直接透传（idx=10，不走 Tracker）
// pose_node.cpp:257-275 — 死亡装甲板动态追加（idx=11+）
```

---

## 8. 世界测量结构

```cpp
// tracker_types.hpp
struct WorldMeasurement {
    int class_id;      // 分类输出（R1-R4/S/ARMOR 等）
    int team_id;       // RED/BLUE/UNKNOWN
    float score;       // 置信度
    bool is_dead;      // 是否为死亡目标
    cv::Rect box;      // 像素边界框（装甲板）
    cv::Point2f world; // 世界坐标 (x, z) — y=0 地面
};

struct TrackedTarget {
    int track_id;
    TrackState state;
    cv::Rect smoothed_box;    // Kalman 平滑后的像素框
    cv::Point2f smoothed_world; // Kalman 平滑后的世界坐标
    int class_id;
    int team_id;
    bool is_dead;
    float score;
};
```

---

## 9. 完整数据流示例

```
Frame N 检测结果:
  detection[0]: team=RED, class=R1, box=(100,200,30,30), world=(1.2, 3.4)
  detection[1]: team=BLUE, class=R3, box=(500,300,25,25), world=(4.5, 2.1)
  (outpost 未检测到)

Track 列表 (上一帧):
  PhysicalTrack[0]: team=RED, last_box=(98,198,32,32), last_world=(1.1,3.3)
                    bot_id.getStableClass() = {R1, 0.85}
  PhysicalTrack[1]: team=BLUE, last_box=(510,305,24,24), last_world=(4.6,2.0)
                    bot_id.getStableClass() = {R3, 0.72}

匈牙利匹配:
  Cost Matrix (2×2):
           det[0]  det[1]
  track[0]   15    1e6     ← team 不匹配 → reject
  track[1]  1e6     20     ← team 不匹配 → reject

  结果: track[0]⇔det[0] (cost=15, R1 class 匹配, 无惩罚)
        track[1]⇔det[1] (cost=20, R3 class 匹配, 无惩罚)

  Kalman Update:
    track[0]: kf_box.update([100,200,30,30]), kf_world.update([1.2,3.4])
              bot_id.update(R1, 0.95)
              state=ACTIVE, miss_count=0
    track[1]: kf_box.update([500,300,25,25]), kf_world.update([4.5,2.1])
              bot_id.update(R3, 0.88)
              state=ACTIVE, miss_count=0

输出映射:
  Slot[0] (RED R1): track[0] → valid, class_id=R1, world=(1.18,3.37)
  Slot[7] (BLUE R3): track[1] → valid, class_id=R3, world=(4.48,2.06)
  其余 slot: invalid
  Slot[10] (Outpost): 未检测到 → outpost 状态保持 (Deathtimer 推进)
```
