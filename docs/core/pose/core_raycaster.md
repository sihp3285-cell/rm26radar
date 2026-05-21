# `raycaster.hpp` + `raycaster.cpp` 逐行讲解

这两份文件是项目的**3D 射线碰撞核心**，负责从相机像素坐标反推世界坐标。它结合了计算机视觉（相机几何）和计算几何（射线-三角形求交），是整个系统中数学含量最高的模块之一。

---

# 第一部分：`raycaster.hpp`

## 一、头文件保护

```cpp
#ifndef RCASTER_HPP
#define RCASTER_HPP
```

标准 include guard。

---

## 二、引入的头文件

```cpp
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
```

OpenCV 用于 `cv::Point2f`、`cv::Point3f`、`cv::Mat`；`memory` 用于 `std::unique_ptr`。

---

## 三、Open3D 前向声明

```cpp
namespace open3d {
    namespace t {
        namespace geometry {
            class RaycastingScene;
        }
    }
}
```

---

### 为什么用前向声明？

`open3d::t::geometry::RaycastingScene` 是 Open3D 的类型。如果在头文件中直接 `#include <open3d/...>`，会：

1. 大幅增加编译时间（Open3D 头文件很大）
2. 增加头文件耦合（所有包含 `raycaster.hpp` 的文件都间接依赖 Open3D）

用**前向声明（Forward Declaration）**只告诉编译器"有这么一个类存在"，不需要知道它的完整定义。这加快了编译，减少了依赖。

限制：前向声明的类只能用于**指针或引用**（如 `std::unique_ptr<RaycastingScene>`），不能用于值类型（如 `RaycastingScene scene`，因为编译器不知道它的大小）。

---

## 四、`Raycaster` 类

```cpp
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
```

---

### `scene_`

`std::unique_ptr<open3d::t::geometry::RaycastingScene>`。

`RaycastingScene` 是 Open3D 的**射线碰撞场景**。它内部存储了三角网格（Triangle Mesh），并提供了高效的射线-网格求交功能（通常基于 BVH 加速结构）。

用 `unique_ptr` 管理，因为：

1. `RaycastingScene` 的完整定义在 `raycaster.cpp` 中才可见（前向声明不能实例化）
2. `unique_ptr` 确保每个 `Raycaster` 独占一个场景
3. 析构时自动释放 Open3D 资源

---

### `pixelToWorld`

核心接口。输入像素坐标和相机参数，输出世界坐标（3D）。

参数：

| 参数 | 含义 |
|------|------|
| `pixel` | 图像上的像素坐标 `(u, v)` |
| `K` | 相机内参矩阵 3×3 |
| `D` | 畸变系数 |
| `R_inv` | 旋转矩阵的逆（即 `R^T`） |
| `T` | 相机光心在世界坐标系中的位置 |

---

# 第二部分：`raycaster.cpp`

## 一、构造函数和析构函数

```cpp
Raycaster::~Raycaster() = default;
Raycaster::Raycaster() = default;
```

显式声明默认构造和默认析构。

`scene_` 是 `unique_ptr`，默认构造时为空（`nullptr`），默认析构时会调用 `RaycastingScene` 的析构函数。

---

## 二、`loadingMesh`

```cpp
bool Raycaster::loadingMesh(const std::string& mesh_path) {
    std::cout << "尝试加载 3D 网格文件: " << mesh_path << std::endl;

    open3d::geometry::TriangleMesh legacy_mesh;
    bool success = open3d::io::ReadTriangleMesh(mesh_path, legacy_mesh);
```

---

### Open3D 两种 Mesh API

Open3D 有两套 Mesh 系统：

1. **Legacy**（传统）：`open3d::geometry::TriangleMesh`
2. **Tensor**（新版）：`open3d::t::geometry::TriangleMesh`

`RaycastingScene` 需要 Tensor 版本的 Mesh，但文件读取 API 返回 Legacy 版本，所以需要转换。

---

### 读取失败处理

```cpp
    if (!success) {
        std::cerr << "错误：Open3D 无法读取网格文件: " << mesh_path << std::endl;
        scene_.reset();
        return false;
    }

    if (legacy_mesh.vertices_.empty()) {
        std::cerr << "错误：网格文件为空或格式不正确" << std::endl;
        scene_.reset();
        return false;
    }
```

如果文件读取失败或为空，`reset()` 释放已有场景（如果有），返回 `false`。

---

### 打印 Mesh 信息

```cpp
    std::cout << "成功加载网格，顶点数: " << legacy_mesh.vertices_.size()
              << ", 三角面数: " << legacy_mesh.triangles_.size() << std::endl;
```

顶点数和面数是衡量 Mesh 复杂度的重要指标。场地 Mesh 可能有数千到数万个面。

---

### Legacy → Tensor 转换

```cpp
    open3d::t::geometry::TriangleMesh tensor_mesh =
        open3d::t::geometry::TriangleMesh::FromLegacy(legacy_mesh);
```

把 Legacy Mesh 转成 Tensor Mesh。Tensor Mesh 的数据存储在 GPU-friendly 的 `open3d::core::Tensor` 中，更适合射线碰撞等并行计算。

---

### 创建场景并添加 Mesh

```cpp
    scene_ = std::make_unique<open3d::t::geometry::RaycastingScene>();
    scene_->AddTriangles(tensor_mesh);
    return true;
}
```

1. 创建 `RaycastingScene`
2. 把三角网格添加到场景中
3. Open3D 内部会构建加速结构（如 BVH），让后续的射线求交达到 O(log n) 复杂度

---

## 三、`pixelToWorld`：像素 → 世界

```cpp
cv::Point3f Raycaster::pixelToWorld(const cv::Point2f& pixel,
                                    const cv::Mat& K, const cv::Mat& D,
                                    const cv::Mat& R_inv, const cv::Mat& T) const
```

这是整个项目**最数学化**的函数。它的目标：

> 给定图像上的一个像素点，找到它在 3D 世界中的对应位置。

核心思路：**从相机光心出发，沿该像素对应的光线前进，直到与地面/Mesh 相交，交点即为世界坐标。**

---

### 步骤 1：去畸变

```cpp
    std::vector<cv::Point2f> src_pts = {pixel}, dst_pts;
    cv::undistortPoints(src_pts, dst_pts, K, D);
```

---

#### `cv::undistortPoints`

输入像素坐标 `(u, v)`，去除镜头畸变后，输出**归一化平面上的理想坐标** `(x', y')`。

归一化平面的含义：

```text
x' = (u - cx) / fx
y' = (v - cy) / fy
```

这是一个假想的 `z = 1` 平面上的坐标。去畸变后，`(x', y', 1)` 就是该像素在**相机坐标系**中的方向向量（未归一化）。

---

### 步骤 2：构建相机坐标系下的方向向量

```cpp
    cv::Mat P_c = (cv::Mat_<double>(3, 1) << dst_pts[0].x, dst_pts[0].y, 1.0);
```

把去畸变后的 `(x', y')` 和 `z = 1` 组成 3D 向量 `P_c = (x', y', 1)^T`。

这个向量表示：从相机光心出发，指向该像素的三维方向。

---

### 步骤 3：转到世界坐标系

```cpp
    cv::Mat Ray_world = R_inv * P_c;
    cv::Mat Cam_world = T;
```

---

#### `R_inv * P_c`

`R_inv` 是**世界 → 相机**旋转的逆，即**相机 → 世界**的旋转。

`R_inv * P_c` 把相机坐标系下的方向向量，转成世界坐标系下的方向向量。

---

#### `Cam_world = T`

`T` 是相机光心在世界坐标系中的位置，也就是射线的**起点**。

---

### 步骤 4：提取射线参数

```cpp
    float ox = Cam_world.at<double>(0), oy = Cam_world.at<double>(1), oz = Cam_world.at<double>(2);
    float dx = Ray_world.at<double>(0), dy = Ray_world.at<double>(1), dz = Ray_world.at<double>(2);
```

射线的参数方程：

```text
P(t) = O + t·D,  t ≥ 0

其中：
O = (ox, oy, oz)   // 起点（相机光心）
D = (dx, dy, dz)   // 方向向量
```

---

### 步骤 5：Fallback 到平地

```cpp
    auto fallback_to_flat_ground = [&]() -> cv::Point3f {
        if (std::abs(dy) < 1e-6) return cv::Point3f(0, 0, 0);
        double t_fb = -oy / dy;
        return cv::Point3f(ox + t_fb * dx, 0.0f, oz + t_fb * dz);
    };
```

这是一个 **Lambda 函数**，作为 fallback 方案。

---

#### 平地假设的数学

假设地面是 `y = 0` 平面。求射线与 `y = 0` 的交点：

```text
oy + t·dy = 0
t = -oy / dy
```

如果 `dy` 接近 0（射线几乎水平），则没有交点或交点极远，返回 `(0, 0, 0)`。

否则，代入 `t` 求交点坐标：

```text
x = ox + t·dx
y = 0
z = oz + t·dz
```

---

### 步骤 6：优先尝试 Mesh 碰撞

```cpp
    if (!scene_) return fallback_to_flat_ground();
```

如果没有加载 3D Mesh（`scene_` 为空），直接回退到平地假设。

---

### 步骤 7：Open3D 射线碰撞

```cpp
    std::vector<float> ray_data = {ox, oy, oz, dx, dy, dz};
    open3d::core::Tensor ray(ray_data, {1, 6}, open3d::core::Float32);

    auto result = scene_->CastRays(ray);
    float t_hit = result["t_hit"].Item<float>();
```

---

#### 构造 Open3D Tensor

`ray_data` 按 Open3D 的格式排列：`{ox, oy, oz, dx, dy, dz}`（起点 + 方向）。

Shape 为 `{1, 6}`，表示 1 条射线，6 个参数。

---

#### `CastRays`

Open3D 的批量射线碰撞函数。内部使用 GPU 加速的 BVH 遍历，效率很高。

返回值 `result` 是一个字典，`"t_hit"` 键对应每条射线的**命中距离**。

---

#### `t_hit` 的含义

* `t_hit > 0`：射线在 `t = t_hit` 处与 Mesh 相交
* `t_hit = inf`：射线没有命中任何三角形（飞向无穷远）
* `t_hit < 0`：命中点在射线起点后方（通常不会发生）

---

### 步骤 8：处理碰撞结果

```cpp
    if (std::isinf(t_hit)) return fallback_to_flat_ground();

    return cv::Point3f(ox + t_hit * dx, oy + t_hit * dy, oz + t_hit * dz);
}
```

---

#### 未命中 Mesh

如果 `t_hit` 是无穷大，说明射线没有碰到任何场地结构（比如看向天空）。回退到平地假设。

---

#### 命中 Mesh

返回交点坐标：

```text
P_hit = O + t_hit · D
      = (ox + t_hit·dx, oy + t_hit·dy, oz + t_hit·dz)
```

这就是像素对应的真实 3D 世界坐标。

---

# 四、从 Raycaster 层学到的设计要点

## 1. 射线方程

```text
P(t) = O + t·D
```

所有射线碰撞的基础。`O` 是起点，`D` 是方向，`t` 是距离参数。

## 2. 两层 fallback

```text
有 Mesh？→ Mesh 碰撞
    ↓ 没有/未命中
平地假设（y = 0）
    ↓ dy ≈ 0
返回 (0, 0, 0)
```

保证任何情况下都有合理输出，不会崩溃。

## 3. 前向声明减少编译依赖

头文件中只前向声明 Open3D 类，cpp 文件中才 include Open3D 头文件。这大大减少了编译时间和头文件耦合。

## 4. 归一化平面的概念

```text
像素(u,v) → 去畸变 → 归一化平面(x', y', 1)
```

归一化平面是连接像素坐标和 3D 射线的关键桥梁。

## 5. Open3D Tensor API

Open3D 的新版 Tensor API 支持 GPU 加速。`CastRays` 内部在 CUDA 上并行执行，比 CPU 串行遍历快几个数量级。
