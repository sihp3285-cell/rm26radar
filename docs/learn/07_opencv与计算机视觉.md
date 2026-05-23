# 07 OpenCV 与计算机视觉

> OpenCV 是本项目使用最广泛的第三方库，覆盖图像采集、预处理、相机标定、位姿解算、可视化等全流程。本章结合源码深入讲解核心 API 和视觉算法原理。

---

## 7.1 OpenCV 基础操作

### 7.1.1 视频捕获

```cpp
// video_node 中：ROS2 节点内的视频源
cv::VideoCapture cap;
cap.open(video_path_);
if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_path_.c_str());
    rclcpp::shutdown();
    return;
}

// 读取一帧
cv::Mat frame;
if (!cap.read(frame)) {
    // 视频结束，循环播放
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
}
```

### 7.1.2 图像基本操作

```cpp
// 克隆（深拷贝）
cv::Mat clone = frame.clone();

// 浅拷贝（共享数据）
cv::Mat ref = frame;

// ROI 裁剪
cv::Mat roi = frame(cv::Rect(x, y, w, h));

// Resize
cv::Mat resized;
cv::resize(frame, resized, cv::Size(new_w, new_h));

// 颜色空间转换
cv::Mat gray, hsv;
cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
```

### 7.1.3 绘图

```cpp
// 绘制检测框
cv::rectangle(frame, box, color, thickness);

// 绘制文字
cv::putText(frame, label, cv::Point(x, y),
            cv::FONT_HERSHEY_SIMPLEX, fontScale, color, thickness);

// 绘制圆
cv::circle(frame, center, radius, color, thickness, cv::LINE_AA);

// 绘制线段
cv::line(frame, pt1, pt2, color, thickness, cv::LINE_AA);
```

本项目在 `draw.cpp` 中封装了完整的检测结果可视化：

```cpp
void drawDetect(cv::Mat &frame, const std::vector<Result>& results, 
                const std::vector<std::string> &classNames) {
    for (const auto& res : results) {
        // 画框
        cv::rectangle(frame, res.box, COLORS[res.armorColor], 2);
        // 画标签背景
        cv::rectangle(frame, ...);
        // 画文字
        cv::putText(frame, label, ...);
    }
}
```

---

## 7.2 相机模型与标定

### 7.2.1 针孔相机模型

```
世界坐标 (X, Y, Z)
    ↓ [R|t] 外参
相机坐标 (Xc, Yc, Zc)
    ↓ 投影
图像平面 (x, y)
    ↓ K 内参
像素坐标 (u, v)
```

投影公式：

```
s * [u]   [fx  0  cx]   [Xc]
s * [v] = [0  fy  cy] * [Yc]
s * [1]   [0   0   1]   [Zc]
```

### 7.2.2 相机内参矩阵 K

```yaml
# configs/camera.yaml
cameraMatrix: [5033.78, 0, 2829.23,
               0, 5036.14, 1929.49,
               0, 0, 1]
```

```cpp
cv::Mat K = (cv::Mat_<double>(3, 3) <<
    5033.780199, 0.000000, 2829.234535,
    0.000000, 5036.139955, 1929.489557,
    0.000000, 0.000000, 1.000000);
```

- `fx`, `fy`：焦距（像素单位）
- `cx`, `cy`：主点（图像中心）

### 7.2.3 畸变系数 D

```yaml
distCoeffs: [-0.061883, 0.104794, 0.000434, -0.000036, 0.000000]
```

- `k1, k2`：径向畸变系数
- `p1, p2`：切向畸变系数
- `k3`：高阶径向畸变

### 7.2.4 solvePnP —— 从 3D-2D 对应解算位姿

```cpp
// posesolver.cpp
void PoseSolver::calibrate(
    const std::vector<cv::Point3f>& objectPoints,  // 世界坐标系下的 3D 点
    const std::vector<cv::Point2f>& imagePoints)   // 图像上的 2D 点
{
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
    
    // 旋转向量 → 旋转矩阵
    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat);
    
    // 转换为相机在世界坐标系下的位姿
    this->R = R_mat.t();            // R_world_to_camera^T = R_camera_to_world
    this->T = -this->R * tvec;      // 相机在世界坐标系下的位置
    this->isPoseEstimated = true;
}
```

**PnP 方法选择**：

| 方法 | 特点 | 适用场景 |
|:---|:---|:---|
| `SOLVEPNP_ITERATIVE` | Levenberg-Marquardt 优化 | 点数 ≥ 4，精度要求高 |
| `SOLVEPNP_P3P` | 解析解，快速 | 恰好 3 个点 |
| `SOLVEPNP_AP3P` | 改进的 P3P | 3-4 个点 |
| `SOLVEPNP_EPNP` | 高效，适合大量点 | 点数 ≥ 4 |

本项目使用 `ITERATIVE`，因为标定点数（6个）较多，且精度要求高于速度。

### 7.2.5 projectPoints —— 重投影误差计算

```cpp
// calibrate_node.cpp：计算标定精度
std::vector<cv::Point2f> projected;
cv::projectPoints(world_points_, rvec, tvec,
                  camera_matrix_, dist_coeffs_, projected);

double total_err = 0.0;
for (size_t i = 0; i < image_points.size(); ++i) {
    double dx = image_points[i].x - projected[i].x;
    double dy = image_points[i].y - projected[i].y;
    total_err += std::sqrt(dx * dx + dy * dy);
}
double mean_error = total_err / image_points.size();
```

**重投影误差**是衡量标定质量的核心指标，本项目阈值设为 **10 像素**。

---

## 7.3 位姿解算与世界坐标转换

### 7.3.1 像素到地面的投影

```cpp
// posesolver.cpp
cv::Point2f PoseSolver::middletoworld(const cv::Rect& box) {
    if (!isPoseEstimated) return cv::Point2f(0, 0);
    
    // 取框底边中点作为接地点
    cv::Point2f middle(box.x + box.width / 2.0f, box.y + box.height);
    
    // 通过 Raycasting 计算与地面的交点
    cv::Point3f worldPoint = raycaster_.pixelToWorld(middle, K, D, R, T);
    
    // 返回 (world_x, world_z) —— world_y 始终为 0（地面）
    return {worldPoint.x, worldPoint.z};
}
```

### 7.3.2 Raycasting 原理

从像素点发射一条射线，求与 3D 场景（地面或 mesh）的交点：

```
1. 像素 (u, v) 反投影为归一化平面坐标 (x, y, 1)
2. 去畸变：undistortPoints
3. 转换为相机坐标系射线方向：dir = R_cam^T * (x, y, 1)
4. 射线方程：P = camera_center + t * dir
5. 求射线与 mesh/地面的交点
```

---

## 7.4 交互式标定工具

### 7.4.1 MouseCallback

```cpp
// mouseback.hpp
class MouseBack {
public:
    std::vector<cv::Point2f> getPoints(const cv::Mat& image) {
        cv::namedWindow(windowName_, cv::WINDOW_NORMAL);
        cv::setMouseCallback(windowName_, onMouse, this);
        
        while (true) {
            cv::imshow(windowName_, displayImg);
            int key = cv::waitKey(30);
            if (key == 'q' || key == 27) break;      // 退出
            if (key == ' ' && !points_.empty()) {     // 空格撤销
                points_.pop_back();
            }
        }
        cv::destroyWindow(windowName_);
        return points_;
    }
    
private:
    static void onMouse(int event, int x, int y, int flags, void* userdata) {
        auto* self = static_cast<MouseBack*>(userdata);
        if (event == cv::EVENT_LBUTTONDOWN) {
            self->points_.emplace_back(x, y);
        }
    }
};
```

### 7.4.2 标定流程

```
1. 用户按 'S' 截取当前帧
2. 弹出 OpenCV 窗口，依次点击 N 个标定点
3. solvePnP 计算 R, t
4. projectPoints 计算重投影误差
5. 误差 > 阈值 → 提示重新标定
6. 误差 ≤ 阈值 → 保存到 calib_result.yaml
```

---

## 7.5 cv_bridge 图像转换

详细内容已在 [03 ROS2 消息与接口](03_ros2消息与接口.md) 中讲解，本节补充 OpenCV 侧的要点：

```cpp
// 浅拷贝（共享内存）—— 仅 intra-process 安全
auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
cv::Mat frame = cv_ptr->image;

// 深拷贝（独立内存）—— 通用安全
auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
cv::Mat frame = cv_ptr->image.clone();

// Mat → ROS Image（发布）
cv_bridge::CvImage(header, "bgr8", debug_output_frame_).toImageMsg(*out_msg);
```

---

## 7.6 本章小结

| 技术点 | 应用场景 | 关键 API |
|:---|:---|:---|
| VideoCapture | 视频源读取 | `cap.open()`, `cap.read()` |
| 相机内参 | 像素 ↔ 相机坐标 | `cv::Mat K` |
| 畸变系数 | 去畸变 | `cv::undistortPoints()` |
| solvePnP | 相机位姿标定 | `cv::solvePnP()` |
| projectPoints | 重投影误差 | `cv::projectPoints()` |
| Rodrigues | 旋转向量 ↔ 矩阵 | `cv::Rodrigues()` |
| 鼠标回调 | 交互式标定 | `cv::setMouseCallback()` |
| cv_bridge | ROS 图像 ↔ OpenCV | `toCvShare()`, `toCvCopy()` |
