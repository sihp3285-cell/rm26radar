# `posesolver.hpp` + `posesolver.cpp` 逐行讲解

这两份文件是项目的**相机位姿解算核心**，负责把图像上的像素坐标转换成场地坐标系中的世界坐标。

涉及的计算机视觉理论：

* **相机模型**：针孔相机模型、内参矩阵 `K`、畸变系数 `D`
* **PnP（Perspective-n-Point）**：通过 3D-2D 对应点求解相机外参 `R` 和 `T`
* **坐标变换**：像素坐标 → 归一化平面 → 相机坐标系 → 世界坐标系
* **射线碰撞（Ray Casting）**：从相机光心发射射线，与场地地面/Mesh 求交点

---

# 第一部分：`posesolver.hpp`

## 一、头文件保护

```cpp
#ifndef __POSESOLVER_HPP__
#define __POSESOLVER_HPP__
```

标准的 include guard。

---

## 二、引入的头文件

```cpp
#include <opencv2/opencv.hpp>
#include <vector>
#include "raycaster.hpp"
```

---

### `<opencv2/opencv.hpp>`

OpenCV 全功能头。这里主要用到：

* `cv::Mat`：矩阵存储（内参 `K`、畸变 `D`、旋转 `R`、平移 `T`）
* `cv::Point2f` / `cv::Point3f`：二维/三维点
* `cv::solvePnP`：PnP 解算
* `cv::Rodrigues`：旋转向量 ↔ 旋转矩阵 转换
* `cv::undistortPoints`：像素坐标去畸变

---

### `"raycaster.hpp"`

`Raycaster` 类封装了射线与 3D Mesh/地面的碰撞检测。`PoseSolver` 内部持有一个 `Raycaster` 实例。

---

## 三、`PoseSolver` 类

```cpp
class PoseSolver
```

这是项目的**几何核心类**。它的生命周期分为两个阶段：

1. **初始化阶段**：加载内参 `K` 和畸变 `D`
2. **标定阶段**：通过 PnP 求解外参 `R` 和 `T`，或者从文件直接加载

完成这两步后，就可以调用 `middletoworld()` 做像素坐标到世界坐标的转换。

---

## 四、私有成员

```cpp
private:
    cv::Mat K, D;  // 相机内参矩阵和畸变系数
    cv::Mat R, T;  // 相机旋转矩阵和平移向量
    bool isPoseEstimated = false;
    Raycaster raycaster_;
```

---

### `K`（Camera Matrix / 内参矩阵）

3×3 矩阵，描述相机内部几何特性：

```text
K = | fx  0  cx |
    | 0   fy cy |
    | 0   0   1 |
```

* `fx`, `fy`：焦距（像素单位）
* `cx`, `cy`：主点（图像中心，通常接近 `(width/2, height/2)`）

内参在相机出厂后基本不变，通过**相机标定（Camera Calibration）**获得。

---

### `D`（Distortion Coefficients / 畸变系数）

描述镜头的畸变模型。通常是 1×4、1×5 或 1×8 的向量，包含：

* `k1, k2, k3`：径向畸变系数（桶形/枕形畸变）
* `p1, p2`：切向畸变系数（镜头与传感器不平行导致）

去畸变公式（径向部分）：

```text
x_distorted = x(1 + k1·r² + k2·r⁴ + k3·r⁶)
y_distorted = y(1 + k1·r² + k2·r⁴ + k3·r⁶)
```

其中 `r² = x² + y²`。

---

### `R`（Rotation Matrix / 旋转矩阵）

3×3 矩阵，描述**世界坐标系到相机坐标系**的旋转关系。

注意 OpenCV 的 `solvePnP` 默认输出的是**相机坐标系相对于世界坐标系**的位姿。代码里做了转置处理，得到的是**世界到相机的旋转**。

---

### `T`（Translation Vector / 平移向量）

3×1 向量，描述相机在世界坐标系中的位置。

---

### `isPoseEstimated`

标志位。`true` 表示外参已经求解/加载完成，可以调用 `middletoworld()`；`false` 表示还没标定，调用会返回 `(0, 0)`。

这是**防御性编程**：避免在没标定的情况下做坐标转换，产生无意义的错误结果。

---

### `raycaster_`

`Raycaster` 实例。负责从相机光心发射射线，与地面或 3D Mesh 求交点。

---

## 五、公共接口

```cpp
public:
    PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat);
    void calibrate(const std::vector<cv::Point3f>& objectPoints,
                   const std::vector<cv::Point2f>& imagePoints);
    void setExtrinsic(const cv::Mat& R_in, const cv::Mat& T_in);
    void getExtrinsic(cv::Mat& R_out, cv::Mat& T_out) const;
    cv::Point2f middletoworld(const cv::Rect& box);
    Raycaster& getRaycaster() { return raycaster_; }
    const Raycaster& getRaycaster() const { return raycaster_; }
```

---

### `PoseSolver`

构造函数。接收相机内参矩阵 `camMat` 和畸变系数 `disMat`，保存到 `K` 和 `D`。

---

### `calibrate`

通过 PnP 算法标定外参。输入：

* `objectPoints`：3D 世界坐标点（如场地上的标定板角点）
* `imagePoints`：对应的 2D 图像坐标点（用户鼠标点击获取）

---

### `setExtrinsic` / `getExtrinsic`

直接设置/获取外参。用于从文件加载标定结果，或保存标定结果到文件。

---

### `middletoworld`

**核心转换函数**。输入一个检测框 `cv::Rect`，输出世界坐标 `cv::Point2f`。

注意：它用的是检测框的**底部中心点**（`box.x + width/2, box.y + height`），因为车辆底部中心最接近地面，做射线-地面求交时最稳定。

---

### `getRaycaster`

返回 `Raycaster` 的引用（const 和非 const 两个重载）。外部代码（如 `pose_node`）通过它加载 3D Mesh。

---

# 第二部分：`posesolver.cpp`

## 一、构造函数

```cpp
PoseSolver::PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat)
{
    this->K = camMat.clone();
    this->D = disMat.clone();
}
```

---

### `.clone()`

深拷贝。`camMat` 和 `disMat` 是外部传入的，`clone()` 确保 `PoseSolver` 有自己的独立副本，不受外部矩阵生命周期影响。

---

## 二、`calibrate`：PnP 标定

```cpp
void PoseSolver::calibrate(const std::vector<cv::Point3f>& objectPoints,
                           const std::vector<cv::Point2f>& imagePoints)
{
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
```

---

### `cv::solvePnP`

OpenCV 的 PnP 解算函数。输入：

* `objectPoints`：N 个 3D 点在世界坐标系中的坐标
* `imagePoints`：N 个对应的 2D 点在图像上的像素坐标
* `K`：相机内参矩阵
* `D`：畸变系数

输出：

* `rvec`：旋转向量（3×1，Rodrigues 形式）
* `tvec`：平移向量（3×1）

数学原理：最小化重投影误差

```text
argmin Σ || project(R·P + T) - p ||²
```

其中 `P` 是 3D 点，`p` 是 2D 点，`project` 是相机投影函数。

---

### 旋转向量转旋转矩阵

```cpp
    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat);
```

`solvePnP` 输出的是**旋转向量**（3 个参数），但后续坐标变换需要**旋转矩阵**（3×3）。

`cv::Rodrigues` 做两者的相互转换：

```text
旋转向量 v = (vx, vy, vz)
旋转角度 θ = ||v||
旋转轴 n = v / ||v||
旋转矩阵 R = I·cosθ + (1-cosθ)·nn^T + sinθ·[n]×
```

---

### 外参的存储形式

```cpp
    this->R = R_mat.t();
    this->T = -this->R * tvec;
    this->isPoseEstimated = true;
}
```

---

#### 为什么 `R = R_mat.t()`？

`solvePnP` 输出的 `R_mat` 是**相机坐标系 → 世界坐标系**的旋转（把世界点转到相机系）。

但 `Raycaster::pixelToWorld` 需要的是**世界坐标系 → 相机坐标系**的逆旋转，也就是转置（因为旋转矩阵是正交矩阵，`R⁻¹ = R^T`）。

所以代码存的是 `R_mat.t()`。

---

#### 为什么 `T = -R * tvec`？

`solvePnP` 输出的 `tvec` 是**世界原点在相机坐标系中的位置**。

而我们想要的是**相机原点在世界坐标系中的位置**。

由 `P_camera = R·P_world + tvec`，令 `P_camera = 0`（相机原点）：

```text
0 = R·P_world + tvec
P_world = -R^T · tvec = -R · tvec  （因为 R = R_mat^T）
```

所以 `T = -R * tvec` 就是相机光心在世界坐标系中的坐标。

---

## 三、`setExtrinsic` / `getExtrinsic`

```cpp
void PoseSolver::setExtrinsic(const cv::Mat& R_in, const cv::Mat& T_in)
{
    this->R = R_in.clone();
    this->T = T_in.clone();
    this->isPoseEstimated = true;
}

void PoseSolver::getExtrinsic(cv::Mat& R_out, cv::Mat& T_out) const
{
    R_out = this->R.clone();
    T_out = this->T.clone();
}
```

直接从外部传入/传出 `R` 和 `T`。用于标定结果的保存和加载。

`clone()` 再次确保数据独立性。

---

## 四、`middletoworld`：像素 → 世界

```cpp
cv::Point2f PoseSolver::middletoworld(const cv::Rect& box)
{
    if (!isPoseEstimated) return cv::Point2f(0, 0);
```

如果没有标定，直接返回 `(0, 0)`。这是防御性检查。

---

### 取底部中心点

```cpp
    std::vector<cv::Point2f> middle = {
        cv::Point2f(box.x + box.width / 2.0f, box.y + box.height)
    };
```

计算检测框的**底部中心点**：

* x = 框中心 x
* y = 框底部 y

为什么用底部中心而不是几何中心？

> 因为车辆底部最接近地面，而射线-地面求交需要地面上的点。用几何中心（可能在车身上半部分）会导致射线与地面交点偏离车辆实际位置。

---

### 调用射线碰撞

```cpp
    cv::Point3f worldPoint = raycaster_.pixelToWorld(middle[0], K, D, R, T);
    return {worldPoint.x, worldPoint.z};
}
```

把像素坐标交给 `Raycaster` 做完整的**像素 → 世界**转换，然后返回 `(x, z)`。

注意返回值是 `cv::Point2f`，但取的是 `worldPoint.x` 和 `worldPoint.z`，跳过了 `y`（高度）。这是因为在 RoboMaster 场地中，机器人通常在地面上运动，高度信息暂时不需要。

---

# 五、从 PoseSolver 层学到的设计要点

## 1. 相机模型的四层参数

```text
内参 K, D  →  描述相机怎么看（出厂固定）
外参 R, T  →  描述相机在哪里（每次部署可能变化）
```

内参和外参分离，是计算机视觉的**标准做法**。

## 2. PnP 的两步转换

```text
solvePnP 输出：rvec, tvec（相机系下的位姿）
    ↓
Rodrigues(rvec) → R_mat
    ↓
R = R_mat^T, T = -R * tvec（世界系下的相机位置）
```

这个转换容易搞混方向，是 PnP 使用的**经典易错点**。

## 3. 射线碰撞的抽象

`PoseSolver` 只负责"取底部中心点"和"调用射线碰撞"，具体的射线计算、地面求交、Mesh 碰撞全部交给 `Raycaster`。

这体现了**单一职责原则**：`PoseSolver` 管相机几何，`Raycaster` 管空间碰撞。
