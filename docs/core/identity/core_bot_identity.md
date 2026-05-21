# `bot_identity.hpp` + `bot_identity.cpp` 逐行讲解

这两份文件实现了 **单个物理机器人的身份轨迹池**（Bot Identity Pool），是 Tracker 子系统中解决"分类抖动"问题的关键模块。

在系统中的位置：

```text
pose_node
   ├── PoseSolver（像素→世界坐标）
   ├── Tracker（固定槽位多目标跟踪）
   │     ├── KalmanFilterBox（像素框平滑）
   │     ├── KalmanFilter2d（世界坐标平滑）
   │     ├── HungarianAlgorithm（数据关联）
   │     └── BotIdentity（跨帧身份稳定化）   ← 这里
   ↓  /world_targets
map_node
```

---

# 一、设计思想：为什么需要 BotIdentity？

## 1.1 分类模型的抖动问题

第三阶段分类模型（`classifyModel_`）对每一帧装甲板做独立推理，输出类别 ID。但分类模型不是 100% 准确的：

* 某帧分类为 R3，下一帧可能分类为 R4
* 光照变化、运动模糊、遮挡都可能导致分类结果跳变
* 单帧分类置信度可能只有 0.4，不足以做出可靠判断

如果直接把单帧分类结果用于地图显示，机器人标签会在 R3 和 R4 之间来回跳动，严重影响操作员判断。

## 1.2 BotIdentity 的核心思想

**用时间换精度**：跨多帧累积分类历史，用指数加权投票选出最稳定的类别。

```text
帧 1: 分类=R3, 置信度=0.6
帧 2: 分类=R3, 置信度=0.7
帧 3: 分类=R4, 置信度=0.3  ← 偶尔错一帧
帧 4: 分类=R3, 置信度=0.8
帧 5: 分类=R3, 置信度=0.9

BotIdentity 输出：stable_class_id = R3, stable_conf = 0.85
```

即使偶尔一帧分类错误，加权投票后 R3 的累积分数仍然远高于 R4，最终输出稳定为 R3。

---

# 第二部分：`bot_identity.hpp`

## 一、常量定义

```cpp
static constexpr int MAX_HISTORY = 30;
static constexpr int PURGE_THRESHOLD = 30;
static constexpr float DECAY = 0.95f;
static constexpr int NUM_CLASSES = 9;
```

---

### 常量说明

| 常量 | 值 | 含义 |
|------|------|------|
| `MAX_HISTORY` | 30 | 历史队列最大长度（保存最近 30 帧观测） |
| `PURGE_THRESHOLD` | 30 | 连续丢失 30 帧后清空历史 |
| `DECAY` | 0.95 | 指数衰减因子（越新的观测权重越高） |
| `NUM_CLASSES` | 9 | 类别总数（CAR, ARMOR, R1~R4, S, OUTPOST, AIRPLANE） |

---

### `DECAY = 0.95` 的含义

最新一帧的权重是 `0.95⁰ = 1.0`，倒数第二帧的权重是 `0.95¹ = 0.95`，倒数第三帧 `0.95² = 0.9025`，以此类推。

30 帧前的观测权重衰减到 `0.95²⁹ ≈ 0.215`，仍然是当前帧权重的约 21.5%。这意味着：

* **新观测影响大，但不会瞬间覆盖旧历史**
* **偶尔的分类错误很快被后续正确帧"稀释"**
* **如果机器人真的换了身份（如重喷涂），约 10~15 帧后新身份会占主导**

---

### `PURGE_THRESHOLD = 30` 的含义

如果一个槽位连续 30 帧未匹配到检测（`markLost` 调用 30 次），BotIdentity 会清空历史。

在 30fps 下，30 帧 = 1 秒。这意味着：

* 目标离开视野 1 秒后，身份历史被清除
* 下次重新出现时，从零开始积累新的身份历史
* 避免了"幽灵记忆"——用旧目标的分类历史污染新目标

---

## 二、`Observation` 结构体

```cpp
struct Observation {
    int class_id;
    float class_conf;
};
```

单帧观测：类别 ID + 该帧的分类置信度。

---

## 三、`history_` 队列

```cpp
std::deque<Observation> history_;
```

使用 `std::deque`（双端队列）存储最近 30 帧的观测历史。

为什么用 `deque` 而不是 `vector`？

* `deque` 支持 O(1) 的头部删除（`pop_front`），适合滑动窗口
* `vector` 的头部删除是 O(n)，需要搬移所有后续元素
* 30 帧的窗口很小，两者性能差异可忽略，但 `deque` 的语义更清晰

---

## 四、`lost_counter_`

```cpp
int lost_counter_ = 0;
```

连续未匹配帧计数器。每次 `update()` 时清零，每次 `markLost()` 时递增。

---

## 五、公共接口

```cpp
void update(int class_id, float class_conf);
void markLost();
void reset();
bool empty() const;
bool shouldPurge() const;
int getLostCounter() const;
size_t getHistorySize() const;
std::pair<int, float> getStableClass() const;
```

| 方法 | 作用 |
|------|------|
| `update()` | 收到新观测时调用，添加到历史队列 |
| `markLost()` | 本帧未匹配时调用，递增丢失计数器 |
| `reset()` | 强制清空历史（首次激活/DEAD 复活时调用） |
| `empty()` | 历史队列是否为空 |
| `shouldPurge()` | 是否已超过清除阈值 |
| `getLostCounter()` | 当前连续丢失帧数 |
| `getHistorySize()` | 历史队列当前长度 |
| `getStableClass()` | **核心方法**：返回稳定的类别 ID 和归一化置信度 |

---

# 第三部分：`bot_identity.cpp`

## 一、`update` 方法

```cpp
void BotIdentity::update(int class_id, float class_conf) {
    lost_counter_ = 0;
    history_.push_back({class_id, class_conf});
    if (history_.size() > MAX_HISTORY) {
        history_.pop_front();
    }
}
```

收到新观测时：

1. **清零丢失计数器**：目标"活着"，重置丢失状态
2. **追加到队尾**：新观测放在最后（最新）
3. **滑动窗口**：如果队列超过 30 帧，删除最老的一帧

---

## 二、`markLost` 方法

```cpp
void BotIdentity::markLost() {
    lost_counter_++;
    if (lost_counter_ >= PURGE_THRESHOLD) {
        reset();
    }
}
```

本帧未匹配时：

1. **递增丢失计数器**
2. **自动清除**：超过 30 帧未匹配，清空历史（避免幽灵记忆）

注意：`markLost` **不会删除历史中的观测**。即使连续丢失 5 帧，历史队列中仍然保留之前的观测。只有超过 `PURGE_THRESHOLD`（30 帧）才彻底清空。

这意味着：

* 短暂遮挡（< 30 帧）后重新出现，身份历史仍然有效
* 长时间离开（≥ 30 帧）后重新出现，身份历史已清空，从零开始

---

## 三、`reset` 方法

```cpp
void BotIdentity::reset() {
    history_.clear();
    lost_counter_ = 0;
}
```

强制清空所有历史。在 Tracker 中，以下情况会调用 `reset`：

* 首次激活（`!initialized || DEAD`）
* DEAD 复活

---

## 四、`getStableClass` 方法（核心算法）

```cpp
std::pair<int, float> BotIdentity::getStableClass() const {
    if (history_.empty()) {
        return {-1, 0.0f};
    }

    std::vector<float> scores(NUM_CLASSES, 0.0f);
    float weight_sum = 0.0f;

    int N = static_cast<int>(history_.size());
    for (int i = 0; i < N; ++i) {
        // i=0 是最老的, i=N-1 是最新的
        float weight = std::pow(DECAY, N - 1 - i);
        const auto& obs = history_[i];
        if (obs.class_id >= 0 && obs.class_id < NUM_CLASSES) {
            scores[obs.class_id] += obs.class_conf * weight;
        }
        weight_sum += weight;
    }

    int best_id = -1;
    float best_score = 0.0f;
    for (int c = 0; c < NUM_CLASSES; ++c) {
        if (scores[c] > best_score) {
            best_score = scores[c];
            best_id = c;
        }
    }

    float normalized_conf = weight_sum > 0.0f ? best_score / weight_sum : 0.0f;
    return {best_id, normalized_conf};
}
```

---

### 算法步骤

#### Step 1: 指数加权累积

```cpp
for (int i = 0; i < N; ++i) {
    float weight = std::pow(DECAY, N - 1 - i);
    scores[obs.class_id] += obs.class_conf * weight;
    weight_sum += weight;
}
```

遍历所有历史观测，对每个类别的分数做加权累加：

```text
weight = 0.95^(N-1-i)
```

* `i = N-1`（最新）→ `weight = 0.95⁰ = 1.0`
* `i = N-2`（倒数第二）→ `weight = 0.95¹ = 0.95`
* `i = 0`（最老）→ `weight = 0.95^(N-1)`

每个观测对 `scores[class_id]` 的贡献是 `class_conf × weight`。

同时累加 `weight_sum`，用于后续归一化。

---

#### Step 2: 找最大类别

```cpp
for (int c = 0; c < NUM_CLASSES; ++c) {
    if (scores[c] > best_score) {
        best_score = scores[c];
        best_id = c;
    }
}
```

遍历所有类别，找出累积分数最高的类别。

---

#### Step 3: 归一化置信度

```cpp
float normalized_conf = weight_sum > 0.0f ? best_score / weight_sum : 0.0f;
```

归一化到 [0, 1] 范围：

```text
normalized_conf = best_score / weight_sum
```

如果所有观测都是同一个类别且置信度都是 1.0，`normalized_conf` 会趋近于 1.0。如果历史中有多个类别混杂，`normalized_conf` 会降低。

---

### 算法示例

假设某槽位的 BotIdentity 历史如下（最新在右）：

```text
帧:    1     2     3     4     5
类别:  R3    R3    R4    R3    R3
置信度: 0.6   0.7   0.3   0.8   0.9
```

计算过程：

```text
权重: 0.95⁴=0.815  0.95³=0.857  0.95²=0.903  0.95¹=0.950  0.95⁰=1.000

R3: 0.6×0.815 + 0.7×0.857 + 0.8×0.950 + 0.9×1.000 = 0.489+0.600+0.760+0.900 = 2.749
R4: 0.3×0.903 = 0.271

weight_sum = 0.815+0.857+0.903+0.950+1.000 = 4.525

stable_class_id = R3
normalized_conf = 2.749 / 4.525 = 0.608
```

R3 的累积分数（2.749）远高于 R4（0.271），输出稳定的 R3 身份，归一化置信度 0.608。

---

# 第四部分：BotIdentity 在 Tracker 中的集成

## 1. 更新时机

在 `Tracker::update()` 中：

```cpp
// 匹配成功时
slot.bot_id.update(det.class_id, det.score);

// 未匹配时
slot.bot_id.markLost();

// 首次激活/DEAD 复活时
slot.bot_id.reset();
```

---

## 2. 输出接口

在 `Tracker::get_slot()` 中：

```cpp
auto [stable_cls, stable_conf] = s.bot_id.getStableClass();
out.stable_class_id  = stable_cls;
out.stable_class_conf = stable_conf;
```

`SlotOutput` 新增两个字段：

| 字段 | 含义 |
|------|------|
| `stable_class_id` | BotIdentity 投票出的稳定类别 ID |
| `stable_class_conf` | 归一化后的稳定置信度 |

---

## 3. 与 `class_id` 的区别

| 字段 | 来源 | 特点 |
|------|------|------|
| `class_id` | 构造时焊死的槽位类别 | 固定不变，用于数据关联 |
| `stable_class_id` | BotIdentity 跨帧投票 | 随时间收敛，用于显示 |

---

# 第五部分：从 BotIdentity 学到的设计要点

## 1. 时间换精度

单帧分类精度有限，但 30 帧的历史投票可以显著降低抖动。这是**时序融合（Temporal Fusion）**的经典应用。

## 2. 指数加权 vs 简单平均

指数加权比简单平均更好，因为它：

* **对新观测更敏感**：快速响应真实的身份变化
* **对旧观测有遗忘**：不被早期错误持续影响
* **无需手动设置窗口长度**：衰减自然实现了"软窗口"

## 3. 滑动窗口 + 衰减的组合

`MAX_HISTORY = 30` 限制了内存占用，`DECAY = 0.95` 控制了遗忘速度。两者配合：

* 队列满时丢弃最老的观测（硬限制）
* 队列内的观测按时间衰减（软权重）

## 4. 自动清除机制

`PURGE_THRESHOLD` 防止了"幽灵记忆"：目标离开视野足够久后，旧身份历史自动清除，下次出现时从零开始。

## 5. 与卡尔曼滤波器的互补

| 模块 | 处理对象 | 作用 |
|------|---------|------|
| KalmanFilterBox | 像素框位置 | 空间平滑（位置不跳） |
| KalmanFilter2d | 世界坐标位置 | 空间平滑（坐标不抖） |
| BotIdentity | 类别 ID | 时间平滑（身份不跳） |

三者共同构成了 Tracker 的**三维平滑体系**：像素空间平滑 + 世界空间平滑 + 语义空间平滑。
