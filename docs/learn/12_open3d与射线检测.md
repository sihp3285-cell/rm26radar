# 12 Open3D 与射线检测

> Open3D 是一个开源的 3D 数据处理库，本项目使用它的 `RaycastingScene` 模块实现**像素坐标到世界坐标的转换**。当你从相机画面中检测到一个目标的像素位置后，需要知道它在真实 3D 空间中的位置——这就是 Raycaster 要解决的问题。

---

## 12.1 Open3D 简介

### 12.1.1 什么是 Open3D

Open3D 提供了 3D 数据结构（点云、三角网格）和算法（配准、重建、射线检测等）。本项目中只用到了它的一个子功能：**射线-三角形碰撞检测**。

```bash
# 安装（本项目通过源码编译）
# 通常在 ROS2 环境中已包含，或通过 apt 安装
sudo apt install libopen3d-dev
```

### 12.1.2 Open3D 的两套 API

Open3D 有两套 Mesh 系统：

| API | 命名空间 | 特点 |
|:---|:---|:---|
| **Legacy**（传统） | `open3d::geometry` | CPU 为主，文件 I/O 完善 |
| **Tensor**（新版） | `open3d::t::geometry` | 支持 GPU 加速，适合射线碰撞 |

`RaycastingScene` 属于 Tensor API，但文件读取 API 仍返回 Legacy Mesh，因此需要**格式转换**。

---

## 12.2 射线检测的基本原理

### 12.2.1 问题描述

相机拍摄到图像上的一个像素 `(u, v)`，它对应现实世界中的哪个 3D 点？

```
相机光心 O
    \
     \  射线方向 D
      \
       \__________  ← 3D 场地 Mesh（地面、高台、障碍物）
        * 交点 P_hit = O + t_hit · D
```

**核心思路**：从相机光心出发，沿该像素对应的光线前进，直到与场地 Mesh 相交，交点即为世界坐标。

### 12.2.2 射线方程

```text
P(t) = O + t · D,   t ≥ 0

其中：
O = (ox, oy, oz)   // 射线起点（相机光心在世界坐标系中的位置）
D = (dx, dy, dz)   // 射线方向（世界坐标系中）
```

射线碰撞检测就是求解 `t_hit`——射线与三角形交点的距离参数。

### 12.2.3 为什么需要 3D Mesh 而不只是平面假设

最简单的方案是假设目标都在 `y = 0` 平面上，但对于 RoboMaster 比赛场地：

- 存在**高台、坡道、障碍块**等 3D 结构
- 同一像素投射到平面和投射到高台，世界坐标差异巨大
- 使用真实场地 Mesh 可以准确处理不同高度的目标

---

## 12.3 Raycaster 类设计

### 12.3.1 头文件概览

```cpp
// raycaster.hpp
#ifndef RCASTER_HPP
#define RCASTER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

// 前向声明——避免在头文件中引入 Open3D 重量级头文件
namespace open3d {
    namespace t {
        namespace geometry {
            class RaycastingScene;
        }
    }
}

class Raycaster {
public:
    Raycaster();
    ~Raycaster();

    bool loadingMesh(const std::string& mesh_path);

    cv::Point3f pixelToWorld(const cv::Point2f& pixel,
                             const cv::Mat& K, const cv::Mat& D,
                             const cv::Mat& R_inv, const cv::Mat& T) const;
private:
    std::unique_ptr<open3d::t::geometry::RaycastingScene> scene_;
};

#endif
```

### 12.3.2 设计要点

| 设计决策 | 原因 |
|:---|:---|
| **前向声明** Open3D 类 | Open3D 头文件很大（数千行），前向声明可大幅减少编译时间 |
| **`unique_ptr`** 管理 `scene_` | 前向声明的类只能用于指针/引用；`unique_ptr` 确保独占所有权和自动释放 |
| **构造/析构在 `.cpp`** 中定义 | 因为 `unique_ptr` 的默认删除器需要完整类型定义，所以析构函数必须在 `.cpp` 中实现 |

这种 **Pimpl（Pointer to Implementation）** 模式在 C++ 项目中非常常见，用于隔离编译依赖。

---

## 12.4 加载 3D 场地 Mesh

### 12.4.1 实现流程

```cpp
bool Raycaster::loadingMesh(const std::string& mesh_path) {
    // 1. 用 Legacy API 读取文件
    open3d::geometry::TriangleMesh legacy_mesh;
    bool success = open3d::io::ReadTriangleMesh(mesh_path, legacy_mesh);

    if (!success || legacy_mesh.vertices_.empty()) {
        scene_.reset();
        return false;
    }

    // 2. 转换为 Tensor API
    open3d::t::geometry::TriangleMesh tensor_mesh =
        open3d::t::geometry::TriangleMesh::FromLegacy(legacy_mesh);

    // 3. 创建射线场景并添加网格
    scene_ = std::make_unique<open3d::t::geometry::RaycastingScene>();
    scene_->AddTriangles(tensor_mesh);
    return true;
}
```

### 12.4.2 步骤解析

```
.ply 文件
    │  open3d::io::ReadTriangleMesh()
    ▼
Legacy Mesh（CPU，open3d::geometry::TriangleMesh）
    │  FromLegacy()
    ▼
Tensor Mesh（GPU-friendly，open3d::t::geometry::TriangleMesh）
    │  scene_->AddTriangles()
    ▼
RaycastingScene（内部构建 BVH 加速结构）
```

`AddTriangles` 会在内部构建 **BVH（Bounding Volume Hierarchy）** 加速结构，使得射线碰撞的复杂度从 O(n)（暴力遍历所有三角形）降低到 O(log n)。

### 12.4.3 Mesh 文件格式

本项目使用的场地 Mesh 文件：

| 文件 | 说明 |
|:---|:---|
| `configs/RB2026_rmuc.ply` | 2026 赛季场地 Mesh |
| `configs/RMUC2025_National.PLY` | 2025 赛季全国赛场地 Mesh |

`.ply`（Polygon File Format）是一种通用的 3D 网格格式，存储顶点坐标和三角面索引。

---

## 12.5 像素到世界坐标：pixelToWorld

这是整个项目**最数学化**的函数。下面分步骤讲解。

### 12.5.1 函数签名

```cpp
cv::Point3f Raycaster::pixelToWorld(
    const cv::Point2f& pixel,   // 图像上的像素坐标 (u, v)
    const cv::Mat& K,           // 相机内参矩阵 3×3
    const cv::Mat& D,           // 畸变系数
    const cv::Mat& R_inv,       // 旋转矩阵的逆（即 R^T）
    const cv::Mat& T            // 相机光心在世界坐标系中的位置
) const;
```

### 12.5.2 完整流程图

```
像素 (u, v)
    │  cv::undistortPoints()
    ▼
归一化平面坐标 (x', y')
    │  组成向量 P_c = (x', y', 1)^T
    ▼
相机坐标系方向向量
    │  R_inv * P_c
    ▼
世界坐标系方向向量 D
    │
    ├─ 有 Mesh？──是──► Open3D CastRays(D) ──命中──► P_hit = O + t_hit·D
    │                                    │
    │                                    └─未命中──► fallback
    │
    └─ 无 Mesh ──────────────────────────────────► fallback
                                                        │
                                                        ▼
                                                  平地假设 (y=0)
                                                  P = O + (-oy/dy)·D
```

### 12.5.3 步骤 1：去畸变

```cpp
std::vector<cv::Point2f> src_pts = {pixel}, dst_pts;
cv::undistortPoints(src_pts, dst_pts, K, D);
```

`cv::undistortPoints` 的作用：

| 输入 | 输出 |
|:---|:---|
| 像素坐标 `(u, v)` | 归一化平面坐标 `(x', y')` |

归一化平面是 `z = 1` 的假想平面，坐标计算为：

```text
x' = (u - cx) / fx
y' = (v - cy) / fy
```

其中 `fx, fy` 是焦距，`cx, cy` 是主点。去畸变过程同时消除了镜头畸变（桶形、枕形等）的影响。

### 12.5.4 步骤 2：构建方向向量

```cpp
cv::Mat P_c = (cv::Mat_<double>(3, 1) << dst_pts[0].x, dst_pts[0].y, 1.0);
```

`(x', y', 1)^T` 就是该像素在**相机坐标系**中的方向向量（未归一化）。

### 12.5.5 步骤 3：坐标系转换

```cpp
cv::Mat Ray_world = R_inv * P_c;  // 方向向量：相机坐标系 → 世界坐标系
cv::Mat Cam_world = T;            // 射线起点：相机光心在世界坐标系中的位置
```

为什么要用 `R_inv`？

- `R` 是世界坐标系 → 相机坐标系的旋转
- `R_inv = R^T`（正交矩阵的逆等于转置）是相机 → 世界的旋转
- 方向向量需要从相机系转到世界系，所以乘 `R_inv`

### 12.5.6 步骤 4：射线碰撞

```cpp
// 构造 Open3D 射线 Tensor：{ox, oy, oz, dx, dy, dz}
std::vector<float> ray_data = {ox, oy, oz, dx, dy, dz};
open3d::core::Tensor ray(ray_data, {1, 6}, open3d::core::Float32);

// 执行射线碰撞
auto result = scene_->CastRays(ray);
float t_hit = result["t_hit"].Item<float>();
```

`CastRays` 的返回值含义：

| `t_hit` 值 | 含义 |
|:---|:---|
| `t_hit > 0` | 射线在距离 `t_hit` 处命中 Mesh |
| `t_hit = inf` | 射线未命中任何三角形 |
| `t_hit < 0` | 命中点在射线起点后方（通常不出现） |

### 12.5.7 步骤 5：Fallback 机制

```cpp
// 回退方案：假设目标在 y=0 平面上
auto fallback_to_flat_ground = [&]() -> cv::Point3f {
    if (std::abs(dy) < 1e-6) return cv::Point3f(0, 0, 0);  // 射线几乎水平
    double t_fb = -oy / dy;
    return cv::Point3f(ox + t_fb * dx, 0.0f, oz + t_fb * dz);
};

// 优先 Mesh 碰撞，失败则回退
if (!scene_) return fallback_to_flat_ground();
if (std::isinf(t_hit)) return fallback_to_flat_ground();

// 命中 Mesh
return cv::Point3f(ox + t_hit * dx, oy + t_hit * dy, oz + t_hit * dz);
```

**两层 fallback 保证**：

```
有 Mesh？→ CastRays 命中 → 返回精确 3D 坐标
    ↓ 没有 Mesh 或未命中
平地假设（y = 0）→ 返回近似 3D 坐标
    ↓ dy ≈ 0（射线水平）
返回 (0, 0, 0)（兜底，避免崩溃）
```

---

## 12.6 数学推导：平地假设

当没有 Mesh 或射线未命中时，假设目标在 `y = 0` 平面上：

```text
射线方程：P(t) = O + t · D

y 分量：oy + t · dy = 0

解得：t = -oy / dy

代入求交点：
  x = ox + t · dx
  y = 0
  z = oz + t · dz
```

这本质上就是**射线与平面求交**的特例。对于大部分水平场地上的目标，这个近似已经足够好。

---

## 12.7 在项目中的使用

### 12.7.1 PoseSolver 中的集成

```cpp
class PoseSolver {
private:
    cv::Mat K, D;           // 相机内参、畸变
    cv::Mat R, T;           // 外参
    Raycaster raycaster_;   // 持有一个 Raycaster 实例
public:
    PoseSolver(const cv::Mat& camMat, const cv::Mat& disMat);
    // ...
    Raycaster& getRaycaster() { return raycaster_; }
};
```

`PoseSolver` 持有 `Raycaster`，在需要将检测框中心像素转为世界坐标时调用 `pixelToWorld`。

### 12.7.2 典型调用链

```
检测框中心像素 (u, v)
    │
    ▼ PoseSolver::middletoworld()
    │
    ├─ 去畸变 + 坐标转换
    │
    └─ Raycaster::pixelToWorld()
        │
        ├─ Open3D CastRays（精确 3D 坐标）
        │
        └─ 平地 fallback（近似坐标）
```

---

## 12.8 前向声明与 Pimpl 模式

### 12.8.1 为什么头文件不直接 include Open3D

```cpp
// ❌ 不推荐：头文件直接 include
#include <open3d/t/geometry/RaycastingScene.h>  // 这个头文件非常大

// ✅ 推荐：前向声明
namespace open3d::t::geometry {
    class RaycastingScene;  // 只声明，不定义
}
```

| 方式 | 编译时间 | 头文件耦合 | 可用操作 |
|:---|:---|:---|:---|
| 直接 include | 慢 | 高（所有包含者都依赖 Open3D） | 可以实例化、调用方法 |
| 前向声明 | **快** | **低** | 只能用于指针/引用 |

### 12.8.2 unique_ptr 与不完整类型

```cpp
// 头文件中：前向声明 + unique_ptr
std::unique_ptr<open3d::t::geometry::RaycastingScene> scene_;

// 源文件中：include 完整头文件 + 实现构造/析构
#include <open3d/t/geometry/RaycastingScene.h>

Raycaster::~Raycaster() = default;  // 必须在 .cpp 中，因为需要完整类型
Raycaster::Raycaster() = default;
```

`unique_ptr` 的默认删除器在析构时需要知道类型的完整定义（调用析构函数），所以**析构函数必须在 `.cpp` 文件中定义**，此时 Open3D 头文件已被包含。

---

## 12.9 性能考量

### 12.9.1 BVH 加速

`scene_->AddTriangles()` 内部构建 BVH（Bounding Volume Hierarchy）树：

```
整个场景 AABB
├── 左子树 AABB
│   ├── 三角形 1-100
│   └── ...
└── 右子树 AABB
    ├── 三角形 101-200
    └── ...
```

射线碰撞时，先检查 AABB 包围盒，不命中则跳过整棵子树。复杂度从 O(n) 降到 O(log n)。

### 12.9.2 单条 vs 批量射线

```cpp
// 本项目：单条射线（每次检测框调用一次）
open3d::core::Tensor ray(ray_data, {1, 6}, open3d::core::Float32);

// Open3D 支持批量：{N, 6}，一次投射 N 条射线
// 批量处理可以充分利用 GPU 并行性
```

对于实时系统，每帧可能有多个检测框需要转换。如果性能成为瓶颈，可以考虑将多条射线打包成一个 `{N, 6}` 的 Tensor 批量处理。

---

## 12.10 本章小结

| 技术点 | 说明 | 关键代码 |
|:---|:---|:---|
| Open3D RaycastingScene | GPU 加速的射线-三角形碰撞 | `scene_->CastRays(ray)` |
| Legacy → Tensor 转换 | 文件读取用 Legacy，碰撞用 Tensor | `FromLegacy()` |
| 前向声明 | 减少编译时间和头文件耦合 | `namespace open3d::t::geometry { class RaycastingScene; }` |
| Pimpl 模式 | `unique_ptr` + 前向声明 | 构造/析构在 `.cpp` 中定义 |
| 去畸变 + 归一化平面 | 像素坐标 → 相机系方向向量 | `cv::undistortPoints()` |
| 坐标系旋转 | 相机系方向 → 世界系方向 | `R_inv * P_c` |
| 射线方程 | `P(t) = O + t · D` | `ox + t_hit * dx, ...` |
| 平地 fallback | 无 Mesh 时的降级方案 | `t = -oy / dy` |
| BVH 加速 | O(log n) 碰撞检测 | `AddTriangles()` 内部构建 |
