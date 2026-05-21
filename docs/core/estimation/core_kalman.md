# `kalman.hpp` + `kalman.cpp` 逐行讲解

这两份文件实现了 **两个独立的卡尔曼滤波器**，分别用于像素框平滑和世界坐标平滑。它们是 Tracker 子系统的核心信号处理组件。

在系统中的位置：

```text
pose_node
   ├── PoseSolver（像素→世界坐标）
   ├── Tracker（固定槽位多目标跟踪）
   │     ├── KalmanFilterBox（像素框平滑）   ← 这里
   │     ├── KalmanFilter2d（世界坐标平滑）   ← 这里
   │     └── HungarianAlgorithm（数据关联）
   ↓  /world_targets
map_node
```

---

# 第一部分：`kalman.hpp`

## 一、依赖

```cpp
#include <Eigen/Dense>
#include <vector>
```

使用 Eigen 库做矩阵运算。Eigen 是 C++ 中最广泛使用的线性代数库，特点是**模板化、编译期确定大小、无动态内存分配**。

---

## 二、`KalmanFilterBox` 类

### 2.1 概述

8 维状态空间的卡尔曼滤波器，专门用于**像素框**的平滑和预测。

```cpp
// 状态向量: [cx, cy, w, h, vx, vy, vw, vh]
//         位置 4 维 + 速度 4 维
```

---

### 2.2 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`

```cpp
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
```

Eigen 的固定大小矩阵（如 `Matrix<float, 8, 8>`）要求 16 字节内存对齐。这个宏重载了 `operator new`，确保动态分配时满足对齐要求。

如果不加这个宏，在某些平台上使用 `new KalmanFilterBox()` 可能会触发段错误（SIMD 指令访问未对齐内存）。

---

### 2.3 成员矩阵

```cpp
Eigen::Matrix<float, 8, 1> x;  // 状态向量
Eigen::Matrix<float, 8, 8> P;  // 误差协方差
Eigen::Matrix<float, 8, 8> F;  // 状态转移矩阵
Eigen::Matrix<float, 4, 8> H;  // 观测矩阵 (4维观测)
Eigen::Matrix<float, 4, 4> R;  // 测量噪声协方差
Eigen::Matrix<float, 8, 8> Q;  // 过程噪声协方差
```

---

#### 矩阵维度与含义

| 矩阵 | 大小 | 含义 |
|------|------|------|
| `x` | 8×1 | 状态向量 `[cx, cy, w, h, vx, vy, vw, vh]` |
| `P` | 8×8 | 状态估计的不确定性（误差协方差） |
| `F` | 8×8 | 状态转移矩阵（匀速运动模型） |
| `H` | 4×8 | 观测矩阵（只观测位置，不直接观测速度） |
| `R` | 4×4 | 测量噪声（检测框的抖动） |
| `Q` | 8×8 | 过程噪声（加速度的不确定性） |

---

#### 为什么用 `float` 而不是 `double`？

对于实时视觉跟踪，`float` 的精度完全足够，而且：

* 占用内存更少（矩阵小一半）
* SIMD 运算更快
* GPU 管线通常也用 `float`

---

#### 为什么是固定大小矩阵？

Eigen 的固定大小矩阵（如 `Matrix<float, 8, 8>`）在**栈上分配**，不需要 `new`/`malloc`。

* 没有堆分配开销
* 没有内存碎片
* 编译器可以做更好的优化（循环展开、SIMD）

对于每帧都要调用的滤波器，这个优化是有意义的。

---

### 2.4 公共接口

```cpp
KalmanFilterBox(float dt = 1.0f, float q_std = 2.0f, float r_std = 1.0f);
std::vector<float> predict(float dt = -1.0f);
std::vector<float> update(const std::vector<float>& bbox);
void reset(const std::vector<float>& initial_bbox = {});
std::vector<float> get_state() const;
```

| 方法 | 作用 |
|------|------|
| `predict()` | 预测下一状态，返回预测的 `[cx, cy, w, h]` |
| `update()` | 用观测修正预测，返回修正后的 `[cx, cy, w, h]` |
| `reset()` | 重置滤波器到初始状态 |
| `get_state()` | 获取完整 8 维状态 |

---

## 三、`KalmanFilter2d` 类

### 3.1 概述

4 维状态空间的卡尔曼滤波器，专门用于**世界坐标**的平滑和预测。

```cpp
// 状态向量: [x, y, vx, vy]
//         位置 2 维 + 速度 2 维
```

---

### 3.2 成员矩阵

```cpp
Eigen::Matrix<float, 4, 1> x;  // 状态向量
Eigen::Matrix<float, 4, 4> P;
Eigen::Matrix<float, 4, 4> F;
Eigen::Matrix<float, 2, 4> H;  // 观测矩阵 (2维观测)
Eigen::Matrix<float, 2, 2> R;
Eigen::Matrix<float, 4, 4> Q;
```

与 `KalmanFilterBox` 结构对称，只是维度从 8 降到了 4（去掉了宽度和高度）。

---

### 3.3 额外的接口

```cpp
std::vector<float> get_position() const;
std::vector<float> get_velocity() const;
```

比 `KalmanFilterBox` 多了 `get_velocity()`，因为世界坐标系下的速度估计对决策节点可能有用（比如判断敌方车辆的移动方向）。

---

# 第二部分：`kalman.cpp`

## 一、`KalmanFilterBox` 构造函数

```cpp
KalmanFilterBox::KalmanFilterBox(float dt, float q_std, float r_std)
    : dt_(dt), q_std_(q_std), r_std_(r_std)
{
    x.setZero();
    P = Eigen::Matrix<float, 8, 8>::Identity() * 100.0f;
    F = Eigen::Matrix<float, 8, 8>::Identity();
    for(int i=0; i<4; i++) F(i, i+4) = dt;

    H.setZero();
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1; H(3, 3) = 1;

    R = Eigen::Matrix<float, 4, 4>::Identity() * (r_std * r_std);
    Q.setZero();

    reset();
}
```

---

### 状态转移矩阵 `F`

```cpp
F = Identity();
for(int i=0; i<4; i++) F(i, i+4) = dt;
```

展开后：

```text
F = [1  0  0  0  dt 0  0  0 ]    cx_new  = cx + vx * dt
    [0  1  0  0  0  dt 0  0 ]    cy_new  = cy + vy * dt
    [0  0  1  0  0  0  dt 0 ]    w_new   = w  + vw * dt
    [0  0  0  1  0  0  0  dt]    h_new   = h  + vh * dt
    [0  0  0  0  1  0  0  0 ]    vx_new  = vx
    [0  0  0  0  0  1  0  0 ]    vy_new  = vy
    [0  0  0  0  0  0  1  0 ]    vw_new  = vw
    [0  0  0  0  0  0  0  1 ]    vh_new  = vh
```

这就是**匀速运动模型（Constant Velocity Model）**：假设目标在两帧之间做匀速直线运动。

---

### 观测矩阵 `H`

```cpp
H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1; H(3, 3) = 1;
```

我们只能**直接观测** `[cx, cy, w, h]`（检测框的位置和大小），无法直接观测速度 `[vx, vy, vw, vh]`。

速度信息是通过**多帧位置变化**隐式推断出来的——这正是卡尔曼滤波器的核心价值。

---

### 初始协方差 `P`

```cpp
P = Identity() * 100.0f;
```

初始不确定性设得很大（100），表示"刚开始时我们对状态一无所知"。这会让前几帧的卡尔曼增益 `K` 更大（更信任观测），滤波器会快速收敛。

---

## 二、`updateQ` 方法

```cpp
void KalmanFilterBox::updateQ(float dt) {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt / 2.0f;
    float dt4 = dt2 * dt2 / 4.0f;

    Q.setZero();
    for (int i = 0; i < 4; i++) {
        Q(i, i) = dt4;
        Q(i, i+4) = dt3;
        Q(i+4, i) = dt3;
        Q(i+4, i+4) = dt2;
    }
    Q *= (q_std_ * q_std_);
}
```

---

### 连续时间白噪声模型（Continuous White Noise Model）

这是从连续时间随机过程离散化得到的 `Q` 矩阵。数学推导基于：

> 假设加速度是一个零均值白噪声过程，其功率谱密度为 `q_std²`

对于每个维度（cx, cy, w, h），Q 的 2×2 子块为：

```text
Q_i = q² * [dt⁴/4   dt³/2]
            [dt³/2   dt²   ]
```

其中 `q = q_std`。这种建模方式来自港科大（HKUST）的 VINS-Mono 项目，在工程中被广泛验证。

---

### 为什么要在 `predict` 里重新计算 `Q`？

```cpp
std::vector<float> KalmanFilterBox::predict(float dt) {
    float use_dt = (dt > 0) ? dt : dt_;
    for(int i=0; i<4; i++) F(i, i+4) = use_dt;
    updateQ(use_dt);
    ...
}
```

因为 `dt` 可能每帧不同（比如帧率波动、丢帧），`Q` 必须随 `dt` 动态调整。

如果 `dt` 翻倍，`Q` 的位置分量会增大 16 倍（`dt⁴`），速度分量增大 4 倍（`dt²`）。这意味着长时间没观测时，不确定性增长更快，卡尔曼增益会更大，滤波器更信任新的观测——这正是我们期望的行为。

---

## 三、`KalmanFilterBox::predict`

```cpp
std::vector<float> KalmanFilterBox::predict(float dt) {
    float use_dt = (dt > 0) ? dt : dt_;
    for(int i=0; i<4; i++) F(i, i+4) = use_dt;
    updateQ(use_dt);

    x = F * x;
    P = F * P * F.transpose() + Q;
    return {x(0), x(1), x(2), x(3)};
}
```

标准卡尔曼预测步骤：

1. 更新 `F` 和 `Q` 中的 `dt`
2. `x = F * x`：状态预测（匀速外推）
3. `P = F * P * Fᵀ + Q`：协方差预测（不确定性增大）
4. 返回预测的位置 `[cx, cy, w, h]`

---

## 四、`KalmanFilterBox::update`

```cpp
std::vector<float> KalmanFilterBox::update(const std::vector<float>& bbox) {
    // 防跳变逻辑
    if (std::abs(x(0) - bbox[0]) > 100.0f || std::abs(x(1) - bbox[1]) > 100.0f) {
        reset(bbox);
        return bbox;
    }

    Eigen::Matrix<float, 4, 1> z;
    z << bbox[0], bbox[1], bbox[2], bbox[3];

    Eigen::Matrix<float, 4, 4> S = H * P * H.transpose() + R;
    Eigen::Matrix<float, 8, 4> K = P * H.transpose() * S.inverse();

    x = x + K * (z - H * x);
    P = (Eigen::Matrix<float, 8, 8>::Identity() - K * H) * P;

    return {x(0), x(1), x(2), x(3)};
}
```

---

### 防跳变逻辑（关键工程技巧）

```cpp
if (std::abs(x(0) - bbox[0]) > 100.0f || std::abs(x(1) - bbox[1]) > 100.0f) {
    reset(bbox);
    return bbox;
}
```

如果检测框中心与预测位置偏差超过 100 像素，直接**重置滤波器**，以观测值为准。

为什么要这样做？

在 5K 分辨率下，一个目标在相邻帧之间不可能跳动 100 像素以上。如果发生了，说明：

* 检测模型 ID 错乱（同一个 track_id 对应了不同目标）
* 检测框异常跳变
* 滤波器已经发散

此时继续用卡尔曼更新会导致**状态被污染**（滤波器跟踪到错误的位置），直接 reset 是最安全的。

---

### 标准卡尔曼更新步骤

```cpp
S = H * P * Hᵀ + R    // 新息协方差
K = P * Hᵀ * S⁻¹       // 卡尔曼增益
x = x + K * (z - H*x)  // 状态更新
P = (I - K*H) * P       // 协方差更新
```

* `z - H*x`：**新息（Innovation）**，即观测与预测的差异
* `K`：卡尔曼增益，决定"更信任观测还是更信任预测"
  * `R` 小（测量噪声小）→ `K` 大 → 更信任观测
  * `P` 小（预测很确定）→ `K` 小 → 更信任预测

---

## 五、`KalmanFilter2d`

`KalmanFilter2d` 的结构与 `KalmanFilterBox` 完全对称，只是从 8 维降到了 4 维：

| 维度 | KalmanFilterBox | KalmanFilter2d |
|------|----------------|---------------|
| 状态 | [cx, cy, w, h, vx, vy, vw, vh] | [x, y, vx, vy] |
| 观测 | [cx, cy, w, h] | [x, y] |
| 场景 | 像素框平滑 | 世界坐标平滑 |

---

### `KalmanFilter2d` 的防跳变阈值

```cpp
float jump_dist = std::hypot(x(0) - pos[0], x(1) - pos[1]);
if (jump_dist > 5.0f) {
    reset(pos);
    return pos;
}
```

阈值是 **5 米**（世界坐标单位），而不是 100 像素。

在 RoboMaster 场地（28m × 15m）上，一辆车在相邻帧之间不可能移动 5 米以上。如果发生了，说明世界坐标解算出错（比如 PnP 解算到了错误的位置）。

---

# 第三部分：从 Kalman 模块学到的设计要点

## 1. 匀速运动模型

假设目标做匀速直线运动是最简单、最常用的运动模型。对于 RoboMaster 比赛中车速约 3~5 m/s、帧率 30~60 fps 的场景，相邻帧间位移约 5~17 cm，匀速假设非常合理。

## 2. 两个独立滤波器的分离

为什么不用一个 10 维滤波器（像素框 4 + 世界坐标 2 + 速度 4）同时处理？

* **坐标系不同**：像素坐标和世界坐标是非线性关系，不能用线性卡尔曼同时处理
* **噪声特性不同**：像素框的噪声取决于检测模型（约几像素），世界坐标的噪声取决于 PnP 解算精度（约几十厘米）
* **更新频率可能不同**：如果以后某些帧只有像素框检测但 PnP 解算失败，可以只更新 `KalmanFilterBox`

## 3. 工程级防跳变

教科书上的卡尔曼滤波器没有防跳变逻辑。但在实际工程中，检测模型的输出经常有异常值（ID 切换、误检、漏检），如果不做保护，一个异常观测就能让滤波器发散，后续所有帧都跟着偏。

`reset` 而不是 `ignore` 是一个深思熟虑的选择：

* 如果用 `ignore`（丢弃异常观测），滤波器会继续用旧状态预测，可能已经偏了
* 如果用 `reset`（以异常观测为准），虽然信任了一次"可疑的"观测，但至少滤波器从正确的位置重新开始，不会累积误差

## 4. 固定大小矩阵的性能优势

在每帧跟踪中，`predict` 和 `update` 各被调用 NUM_SLOTS（10）次。如果用 `Eigen::MatrixXf`（动态大小），每次调用都会有堆分配。用固定大小矩阵后，所有计算都在栈上完成，零堆分配。
