# 11 Eigen 与数学计算

> Eigen 是 C++ 的轻量级线性代数模板库，本项目使用它实现卡尔曼滤波的矩阵运算。由于 Eigen 采用**表达式模板**技术，其固定大小矩阵在编译期确定维度，运行时性能接近手写循环，且栈分配无动态内存开销。

---

## 11.1 Eigen 基础

### 11.1.1 固定大小矩阵

```cpp
#include <Eigen/Dense>

// 8x8 浮点矩阵（栈分配）
Eigen::Matrix<float, 8, 8> A;

// 8x1 列向量
Eigen::Matrix<float, 8, 1> x;

// 4x4 矩阵
Eigen::Matrix<float, 4, 4> P;

// 动态大小矩阵（堆分配，本项目尽量避免）
Eigen::MatrixXf B(10, 10);
```

### 11.1.2 基本运算

```cpp
// 矩阵乘法
Eigen::Matrix<float, 8, 8> F, P, FPFT;
FPFT = F * P * F.transpose();

// 矩阵求逆
Eigen::Matrix<float, 4, 4> S_inv = S.inverse();

// 矩阵加法
P = P_pred + Q;

// 元素访问
x(0) = cx;    // 向量
A(i, j) = v;  // 矩阵
```

### 11.1.3 初始化

```cpp
// 零矩阵
P = Eigen::Matrix<float, 8, 8>::Zero();

// 单位矩阵
I = Eigen::Matrix<float, 8, 8>::Identity();

// 逐元素赋值
F << 1, 0, 0, 0, dt, 0,  0,  0,
     0, 1, 0, 0, 0,  dt, 0,  0,
     // ...
```

---

## 11.2 对齐问题

Eigen 的固定大小向量化类型（如 `Eigen::Matrix4f`）默认使用 SIMD 指令（SSE/AVX），需要 **16 字节（或 32 字节）对齐**。

### 11.2.1 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

```cpp
class KalmanFilterBox {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 重载 new/delete 保证对齐
    
    Eigen::Matrix<float, 8, 1> x;
    Eigen::Matrix<float, 8, 8> P;
    // ...
};
```

**不加此宏的后果**：如果类被 `std::vector` 存储，可能因内存未对齐导致 **SIGSEGV**。

### 11.2.2 含有 Eigen 成员的类的存储

```cpp
// 错误：std::vector 可能不对齐
std::vector<KalmanFilterBox> filters;

// 正确：使用 Eigen 的对齐分配器
std::vector<KalmanFilterBox, Eigen::aligned_allocator<KalmanFilterBox>> filters;

// 本项目中的解决方案：Tracker 的 slots_ 是单个数组（非 vector）
std::vector<Slot> slots_;  // Slot 内含 KalmanFilterBox
```

由于 `std::vector` 在 C++17 中会自动处理对齐（当类型有对齐的 `operator new` 时），加上 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 通常足够。

---

## 11.3 卡尔曼滤波的 Eigen 实现

### 11.3.1 预测步骤

```cpp
std::vector<float> KalmanFilterBox::predict(float dt) {
    if (dt > 0) {
        // 更新时间步长
        F(0, 4) = dt; F(1, 5) = dt;
        F(2, 6) = dt; F(3, 7) = dt;
        updateQ(dt);
    }
    
    // x_pred = F * x
    x = F * x;
    
    // P_pred = F * P * F^T + Q
    P = F * P * F.transpose() + Q;
    
    return {x(0), x(1), x(2), x(3)};  // [cx, cy, w, h]
}
```

### 11.3.2 更新步骤

```cpp
std::vector<float> KalmanFilterBox::update(const std::vector<float>& bbox) {
    // 观测向量 z = [cx, cy, w, h]
    Eigen::Matrix<float, 4, 1> z;
    z << bbox[0], bbox[1], bbox[2], bbox[3];
    
    // 残差 y = z - H * x
    Eigen::Matrix<float, 4, 1> y = z - H * x;
    
    // 残差协方差 S = H * P * H^T + R
    Eigen::Matrix<float, 4, 4> S = H * P * H.transpose() + R;
    
    // 卡尔曼增益 K = P * H^T * S^-1
    Eigen::Matrix<float, 8, 4> K = P * H.transpose() * S.inverse();
    
    // 更新状态 x = x + K * y
    x = x + K * y;
    
    // 更新协方差 P = (I - K * H) * P
    Eigen::Matrix<float, 8, 8> I = Eigen::Matrix<float, 8, 8>::Identity();
    P = (I - K * H) * P;
    
    return {x(0), x(1), x(2), x(3)};
}
```

---

## 11.4 性能优势

| 特性 | Eigen 固定矩阵 | 手写循环 | std::vector |
|:---|:---|:---|:---|
| 内存分配 | **栈分配**（无 malloc） | 栈/堆 | 堆分配 |
| SIMD 向量化 | **自动**（编译期展开） | 需手写 intrinsics | 无 |
| 表达式模板 | **避免临时对象** | 手动优化 | 无 |
| 代码可读性 | 高（类似数学公式） | 低 | 中 |

---

## 11.5 本章小结

| 技术点 | 说明 | 关键代码 |
|:---|:---|:---|
| 固定大小矩阵 | 编译期确定维度，栈分配 | `Eigen::Matrix<float, 8, 8>` |
| 表达式模板 | 链式运算不产生临时对象 | `P = F * P * F.transpose() + Q` |
| 对齐宏 | 防止 SIMD 未对齐崩溃 | `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` |
| 矩阵运算 | 预测/更新的完整实现 | `predict()` / `update()` |
| 求逆 | 小矩阵直接求逆 | `S.inverse()` |
