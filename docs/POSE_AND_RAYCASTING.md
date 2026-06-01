# 姿态估计与射线投射 (Pose Estimation & Raycasting)

## 1. 概述

姿态解算子系统完成从 **2D 像素坐标 → 3D 世界坐标** 的映射，由两层组成：
1. **PnP 外参标定**：求解相机在世界坐标系中的位姿 (R, T)
2. **Raycasting 投影**：从相机光心发射射线，与 3D 场地网格求交，获取地面目标的世界坐标

**核心文件**：

| 文件 | 职责 |
|---|---|
| [posesolver.hpp/cpp](../src/tensorrt_detect/include/tensorrt_detect/core/posesolver.hpp) | PnP 标定 + 世界坐标解算入口 |
| [raycaster.hpp/cpp](../src/tensorrt_detect/include/tensorrt_detect/core/raycaster.hpp) | Open3D Raycasting 射线投射 |
| [calibrate_node.cpp](../src/tensorrt_detect/src/nodes/calibrate_node.cpp) | 交互式标定节点 |
| [camera.yaml](../../configs/camera.yaml) | 相机内参 + 世界标定点配置 |

---

## 2. 相机模型

### 2.1 内参矩阵 (Intrinsics)

```yaml
# configs/camera.yaml
camera_matrix:
  - [5052.4576, 0.0, 2688.0]      # fx, 0, cx
  - [0.0, 5052.4576, 1880.0]      # 0, fy, cy
  - [0.0, 0.0, 1.0]

distortion_coefficients: [0.185, 0.0378, -0.0227, 0.0072, 0.0]
# k1, k2, p1, p2, k3 (OpenCV radial + tangential 模型)
```

- **焦距 fx=fy=5052**：长焦镜头，主要用于远距离观察赛场
- **主点 (2688, 1880)**：传感器中心偏下
- **5 参数畸变模型**：k1,k2 径向畸变 + p1,p2 切向畸变

### 2.2 外参模型

外参 (R, T) 通过 PnP 求解，存储格式与 OpenCV `solvePnP` 输出一致：

```
相机坐标系 → 世界坐标系的变换:
  P_world = R^(-1) * (P_cam - tvec) = R^T * P_cam + T

其中:
  R_mat = Rodrigues(rvec)^T     (3×3 旋转矩阵)
  T = -R_mat * tvec             (3×1 平移向量)
```

**为什么这样存储**：
- OpenCV `solvePnP` 解出 `rvec` (Rodrigues) 和 `tvec` (相机坐标系原点在世界坐标系中的位置)
- `R_mat^T` 将相机坐标旋转到世界坐标（旋转变换反转）
- `-R_mat * tvec` 计算世界坐标系中相机的位置
- 最终：世界点 = `R * 相机点 + T`，其中 R 和 T 直接存储在 `calib_result.yaml`

---

## 3. PnP 标定流程

### 3.1 标定原理

使用 **至少 6 个点对**（世界 3D 坐标 ↔ 图像 2D 坐标）通过 `cv::solvePnP` 求解相机外参：

```cpp
// posesolver.cpp
void PoseSolver::calibrate(
    const std::vector<cv::Point3f>& objectPoints,  // 世界 3D 点
    const std::vector<cv::Point2f>& imagePoints)   // 图像 2D 点
{
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
    // K=相机内参(3×3), D=畸变系数

    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat);  // Rodrigues 向量 → 3×3 旋转矩阵

    // 变换为 "相机点 * R + T = 世界点" 的形式
    R = R_mat.t();          // R = R_mat^T (旋转矩阵的逆=转置)
    T = -R * tvec;          // 相机在世界坐标系中的位置

    isPoseEstimated = true;
}
```

### 3.2 世界标定点

```yaml
# configs/camera.yaml — 6 个场地标定点 (世界坐标，以场地中心为原点)
world_points:
  - [-12.8, 0, -7.0]    # 场地左下角?
  - [12.8, 0, -7.0]     # 场地右下角?
  - [12.8, 0, 7.0]      # 场地右上角?
  - [-12.8, 0, 7.0]     # 场地左上角?
  - [0, 3.0, -7.0]      # 中间...
  - [0, 3.0, 7.0]
```

**坐标系约定**：
- **X**：场地长轴方向，范围约 [-14, 14] 米
- **Y**：垂直高度（地面 = 0）
- **Z**：场地短轴方向，范围约 [-7.5, 7.5] 米

### 3.3 calibrate_node 交互式标定

[calibrate_node.cpp](../src/tensorrt_detect/src/nodes/calibrate_node.cpp)

```
用户操作:
  1. 从视频流捕获一帧
  2. 用鼠标依次点击与世界标定点对应的图像位置
  3. 至少点击 6 个点（与 world_points 一一对应）
  4. 运行 solvePnP
  5. 检查重投影误差 < 阈值
  6. 保存到 calib_result.yaml
  7. 触发 pose_node 服务重载
```

**标定结果格式**：

```yaml
# calib_result.yaml
r: [0.998, 0.012, -0.056,          # 3×3 旋转矩阵，行优先展平
    -0.015, 0.999, -0.034,
    0.055, 0.035, 0.998]
t: [0.5, 2.3, -12.0]               # 3×1 平移向量 (相机在世界坐标中的位置)
```

---

## 4. Open3D Raycasting 射线投射

[raycaster.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/raycaster.hpp) / [raycaster.cpp](../src/tensorrt_detect/src/core/raycaster.cpp)

### 4.1 原理

给定图像中的像素点，从相机光心发射一条射线穿过该像素，求射线与场地 3D mesh 的交点。

```
                    Camera Center (ox, oy, oz)
                        │
                        │ 射线方向 (dx, dy, dz)
                        ▼
        ┌───────────────┼────────────────── 图像平面
        │               │                   │
        │           像素点 (u,v)             │
        │               │                   │
        │               ▼                   │
        │  ─ ─ ─ ─ ─ ─ ┼ ─ ─ ─ ─ ─ ─ ─ ─ │
        │         射线与场地交点              │
        │    (世界坐标: ox+t*dx, oy+t*dy,     │
        │              oz+t*dz)              │
        └───────────────────────────────────┘
                    3D 场地 Mesh (PLY)
```

### 4.2 Mesh 加载

```cpp
// raycaster.cpp:15-36
bool Raycaster::loadingMesh(const std::string& mesh_path) {
    // 1. 通过 Open3D Legacy API 读取 PLY 文件
    open3d::geometry::TriangleMesh legacy_mesh;
    open3d::io::ReadTriangleMesh(mesh_path, legacy_mesh);

    // 2. 转换为 Tensor Mesh (GPU 加速格式)
    open3d::t::geometry::TriangleMesh tensor_mesh =
        open3d::t::geometry::TriangleMesh::FromLegacy(legacy_mesh);

    // 3. 创建光线投射场景
    scene_ = std::make_unique<open3d::t::geometry::RaycastingScene>();
    scene_->AddTriangles(tensor_mesh);  // 添加三角形到场景
    // Open3D 内部构建 BVH 加速结构
}
```

### 4.3 单点投射

```cpp
// raycaster.cpp:38-77
cv::Point3f Raycaster::pixelToWorld(
    const cv::Point2f& pixel,     // 图像像素坐标
    const cv::Mat& K,             // 相机内参 3×3
    const cv::Mat& D,             // 畸变系数
    const cv::Mat& R_inv,         // R 的逆 (OpenCV solvePnP 的 R_mat)
    const cv::Mat& T) const       // T (相机在世界坐标中的位置)
{
    // Step 1: 畸变校正
    std::vector<cv::Point2f> src_pts = {pixel}, dst_pts;
    cv::undistortPoints(src_pts, dst_pts, K, D);
    // dst_pts: 归一化相机坐标 (x', y')

    // Step 2: 构建相机坐标系射线
    cv::Mat P_c = (cv::Mat_<double>(3,1) << dst_pts[0].x, dst_pts[0].y, 1.0);

    // Step 3: 变换到世界坐标系
    cv::Mat Ray_world = R_inv * P_c;  // 射线方向在世界坐标系
    // 相机位置在世界坐标系 = T
    float ox = T.at<double>(0), oy = T.at<double>(1), oz = T.at<double>(2);
    float dx = Ray_world.at<double>(0);
    float dy = Ray_world.at<double>(1);
    float dz = Ray_world.at<double>(2);

    // Step 4: Fallback — 若没有 mesh 则求与 y=0 平面的交点
    auto fallback_to_flat_ground = [&]() -> cv::Point3f {
        if (std::abs(dy) < 1e-6) return cv::Point3f(0,0,0);
        double t_fb = -oy / dy;  // 令 y=0 求解 t
        return cv::Point3f(ox + t_fb*dx, 0.0f, oz + t_fb*dz);
    };

    if (!scene_) return fallback_to_flat_ground();

    // Step 5: Cast Ray (CUDA 互斥保护)
    open3d::core::Tensor ray(ray_data, {1,6}, open3d::core::Float32);
    {
        std::lock_guard<std::mutex> cudaLock(cuda_guard::getCudaMutex());
        auto result = scene_->CastRays(ray);
        t_hit = result["t_hit"].Item<float>();  // 射线参数 t (交点距离)
    }

    // Step 6: 交点计算或 fallback
    if (std::isinf(t_hit) || std::isnan(t_hit)) return fallback_to_flat_ground();
    return cv::Point3f(ox + t_hit*dx, oy + t_hit*dy, oz + t_hit*dz);
}
```

### 4.4 批量投射 (优化版)

```cpp
// raycaster.cpp:79-161
std::vector<cv::Point3f> Raycaster::pixelToWorldBatch(
    const std::vector<cv::Point2f>& pixels, ...)
{
    // 1. 批量畸变校正
    std::vector<cv::Point2f> dst_pts;
    cv::undistortPoints(pixels, dst_pts, K, D);

    // 2. 批量构建射线 (CPU 端，向量化)
    std::vector<float> ray_data;  // [N×6]
    ray_data.reserve(N * 6);
    for (size_t i = 0; i < N; ++i) {
        cv::Mat P_c = (cv::Mat_<double>(3,1) << dst_pts[i].x, dst_pts[i].y, 1.0);
        cv::Mat Ray_world = R_inv * P_c;
        // 存入 ray_data: ox, oy, oz, dx, dy, dz
    }

    // 3. 单次 CastRays 批量调用 (一次 CUDA 锁)
    open3d::core::Tensor ray(ray_data, {N, 6}, open3d::core::Float32);
    {
        std::lock_guard<std::mutex> cudaLock(cuda_guard::getCudaMutex());
        auto result = scene_->CastRays(ray);  // 一次 GPU 调用处理全部
        std::vector<float> t_hit_vec = result["t_hit"].ToFlatVector<float>();
    }

    // 4. 逐结果处理 (含 fallback)
    for (size_t i = 0; i < N; ++i) {
        if (t_hit is valid) → 交点 = (ox + t*dx, oy + t*dy, oz + t*dz)
        else → fallback_to_flat_ground()
    }
}
```

**批量 vs 单点的性能优势**：
- 单次 CUDA 锁开销分摊到 N 个点
- Open3D `CastRays` 批量版本内部并行处理多条射线
- `cv::undistortPoints` 也受益于向量化

---

## 5. PoseSolver 世界坐标解算

[posesolver.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/posesolver.hpp) / [posesolver.cpp](../src/tensorrt_detect/src/core/posesolver.cpp)

### 5.1 middletoworld — 框底中心 → 世界坐标

```cpp
// posesolver.cpp — middletoworld() 单点版本
cv::Point2f PoseSolver::middletoworld(const cv::Rect& box) {
    // 计算边界框底边中心 (图像像素坐标)
    // OpenCV 坐标系: x 向右, y 向下
    float cx = box.x + box.width  / 2.0f;   // 水平中心
    float cy = box.y + box.height;           // 底边 (最接近地面)
    cv::Point2f pixel(cx, cy);

    // 通过 Raycaster 求交点
    cv::Point3f world3d = raycaster_.pixelToWorld(
        pixel, K, D,
        R.inv() * cv::Mat::eye(3,3,CV_64F),  // R^(-1) 用于射线方向变换
        T                                       // 相机在世界坐标中的位置
    );

    return cv::Point2f(world3d.x, world3d.z);  // 返回 (x, z)，y=0 省略
}
```

### 5.2 middletoworldBatch — 批量世界坐标解算

```cpp
// posesolver.cpp — 批量版本
std::vector<cv::Point2f> PoseSolver::middletoworldBatch(
    const std::vector<cv::Rect>& boxes)
{
    // 1. 计算所有框的底边中心
    std::vector<cv::Point2f> pixels;
    for (const auto& box : boxes) {
        pixels.push_back(cv::Point2f(
            box.x + box.width / 2.0f,
            box.y + box.height
        ));
    }

    // 2. 批量投射
    auto world3d = raycaster_.pixelToWorldBatch(pixels, ...);

    // 3. 转换为 2D 世界坐标 (x, z)
    std::vector<cv::Point2f> result;
    for (const auto& p : world3d) {
        result.push_back(cv::Point2f(p.x, p.z));
    }
    return result;
}
```

### 5.3 PoseNode 中的世界坐标批量预计算

```cpp
// pose_node.cpp:194-352 — armor_callback()
// 在送入 Tracker 之前，批量预计算所有检测的世界坐标
std::vector<cv::Rect> boxes_for_raycast;
for (const auto& det : msg->detections) {
    // 优先使用 car_box（车辆整体框的底边中心）
    // 如果 car_box 无效，fallback 到 armor_box
    cv::Rect car_box(det.car_x, det.car_y, det.car_width, det.car_height);
    if (car_box.width > 0 && car_box.height > 0) {
        boxes_for_raycast.push_back(car_box);
    } else {
        boxes_for_raycast.push_back(cv::Rect(det.x, det.y, det.width, det.height));
    }
}
// 一次批量调用，返回所有世界坐标
world_positions = pose_solver_->middletoworldBatch(boxes_for_raycast);

// 然后将世界坐标分配给对应的 WorldMeasurement
for (size_t i = 0; i < msg->detections.size(); ++i) {
    cv::Point2f world_pos = world_positions[i];
    WorldMeasurement m;
    m.world = world_pos;  // x=world_x, y=world_z
    // ...
}
```

---

## 6. 坐标系统

### 6.1 坐标系定义

```
世界坐标系 (RoboMaster 场地):
  X ──→ 场地长轴 (28m)  范围 [-14, 14]
  Y ──→ 垂直高度          0 = 地面
  Z ──→ 场地短轴 (15m)  范围 [-7.5, 7.5]

相机坐标系:
  X ──→ 传感器水平方向
  Y ──→ 传感器垂直方向 (向下)
  Z ──→ 光轴方向 (向前)

图像坐标系 (OpenCV):
  col (x) ──→ 从左到右
  row (y) ──→ 从上到下
  原点 = (0, 0) 左上角
```

### 6.2 变换链

```
图像像素 (u, v)
    │  cv::undistortPoints()
    ▼
归一化相机坐标 (x', y')  [去畸变]
    │  射线方向 = R_inv * [x', y', 1]^T
    ▼
世界坐标系射线: 原点 = T, 方向 = (dx,dy,dz)
    │  CastRays() 与场地 mesh 求交
    ▼
世界坐标 (x_world, y_world, z_world)
    │  取 (x, z) 投影到地面平面
    ▼
世界 2D 坐标 (x, z)
```

---

## 7. Mesh 文件与 Fallback

### 7.1 场地网格

```
RB2026_rmuc.ply  — 2026 赛季 RMUC 场地
RMUC2025_National.PLY — 2025 赛季国赛场地
```

这些 PLY 文件包含场地的高精度 3D 模型（地面、护坡、岛区等），Open3D RaycastingScene 建立 BVH (Bounding Volume Hierarchy) 加速结构，支持高效的射线-三角形求交。

### 7.2 Flat Ground Fallback

当以下情况发生时，系统回退到平面地面假设：
- Mesh 文件不存在或加载失败
- 射线与 mesh 无交点（`t_hit = inf`）
- 射线方向接近水平（`dy ≈ 0`）

```cpp
// Flat ground: y = 0 平面
double t_fb = -oy / dy;      // 解 y=oy+t*dy=0
world_point = (ox + t_fb*dx, 0.0, oz + t_fb*dz)
```

**平面假设的局限性**：场地中的斜坡、高台、岛区等地形特征被忽略，世界坐标精度取决于地面是否平坦。

---

## 8. CUDA 互斥保护

```cpp
// raycaster.cpp:68-71
{
    std::lock_guard<std::mutex> cudaLock(cuda_guard::getCudaMutex());
    auto result = scene_->CastRays(ray);
    t_hit = result["t_hit"].Item<float>();
}
```

**为什么 Raycaster 需要 CUDA 互斥锁**：
- Open3D 的 `RaycastingScene::CastRays()` 内部调用 CUDA kernel
- 由于核心流水线运行在**单线程容器**中，TensorRT 推理和 Raycasting **串行执行**
- 这个互斥锁是**防御性**的：保护未来可能的多线程场景，以及单线程中异常的执行顺序

---

## 9. 标定工作流

```
启动时:
  ┌─────────────────┐
  │ PoseNode 初始化   │
  │ 1. 检查 calib_result.yaml │
  │ 2. 若有效 → setExtrinsic() │
  │ 3. 加载 mesh → loadingMesh()│
  │ 4. 若未标定 → 警告等待   │
  └─────────────────┘

手动标定:
  ┌─────────────────────────────┐
  │ calibrate_node (独立进程)    │
  │ 1. 从 video_node 获取一帧   │
  │ 2. 暂停视频发布             │
  │ 3. 用户点击 6+ 个世界对应点 │
  │ 4. solvePnP() 求解          │
  │ 5. 检查重投影误差           │
  │ 6. 保存 calib_result.yaml   │
  │ 7. 调用 /pose_node/reload   │
  │ 8. 恢复视频                 │
  └─────────────────────────────┘

运行时:
  ┌─────────────────────────────┐
  │ 每帧检测结果到达             │
  │ 1. 若未标定 → 跳过          │
  │ 2. 批量计算世界坐标         │
  │ 3. 送入 Tracker             │
  └─────────────────────────────┘
```
