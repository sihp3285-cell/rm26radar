# 性能优化深度剖析

这份文档从**工程实践角度**深度剖析本项目中所有与性能优化相关的设计决策。不会泛泛而谈，而是聚焦于：每个优化点的**原始问题**→**分析过程**→**具体手段**→**量化收益**。

---

# 一、整体性能优化策略

本项目的性能优化遵循一个核心原则：

> **先做减法（不做无用功），再做乘法（减少每步开销），最后做加法（硬件加速）。**

```text
优先级 1：能不能不执行？  → publish_debug_image 开关、条件跳过
优先级 2：能不能少拷贝？  → toCvShare、reserve、Mat 复用
优先级 3：能不能更快执行？→ TensorRT GPU 推理、CUDA 预处理
```

---

# 二、图像链路零拷贝优化

## 2.1 问题：每帧 6MB 的无用拷贝

在 `detect_node` 的 `image_callback` 中，旧版使用 `cv_bridge::toCvCopy`：

```cpp
// 旧版
auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
```

`toCvCopy` 会在堆上分配一块新内存，把 ROS 消息中的图像数据**深拷贝**一份。对于 1920×1080×3 的 BGR 图像，每帧约 **6MB**。

在 30fps 下，每秒产生 **180MB** 的无用内存拷贝和分配。

## 2.2 解决：共享指针零拷贝

```cpp
// 新版
auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
const cv::Mat& frame = cv_ptr->image;
```

`toCvShare` 返回一个**共享指针**，`cv::Mat` 内部的数据指针直接指向 ROS 消息底层的图像缓冲区，**零拷贝**。

## 2.3 安全保护

零拷贝的代价是：`frame` 指向 ROS 消息原始数据，任何写操作都会篡改原始数据。解决方案：

```cpp
const cv::Mat& frame = cv_ptr->image;  // const 语义保护

if (publish_debug_image_) {
    frame.copyTo(debug_frame_);  // 仅在需要绘制时才拷贝副本
    drawDetect(debug_frame_, ...);
}
```

* `const` 引用在语义上标记为只读
* 只有在真正需要修改图像时（绘制检测框）才做 `copyTo`
* 如果 `publish_debug_image_ = false`，**连这 6MB 的拷贝都省了**

## 2.4 量化收益

| 场景 | 旧版每帧拷贝 | 新版每帧拷贝 | 节省 |
|------|-------------|-------------|------|
| `publish_debug_image = true` | 6MB（toCvCopy） | 6MB（copyTo，仅绘制时） | 0%（但延迟更低） |
| `publish_debug_image = false` | 6MB（toCvCopy） | **0MB** | **100%** |

---

# 三、条件化 Debug 图像生成

## 3.1 问题：纯自动模式下的浪费

在纯自动运行模式（不需要给人看画面）时，`detect_node` 每帧仍然执行：

1. `drawDetect` — 遍历所有检测结果，画框、画文字、画线
2. `cv::resize` — 缩放到显示尺寸
3. `cv::putText` — 画 FPS
4. `cv_bridge::CvImage::toImageMsg` — 编码成 ROS 消息
5. `image_pub_->publish` — 发布到话题

这些操作对于下游算法节点完全没有价值——`pose_node` 只需要 `/armor_detections` 结构化数据。

## 3.2 解决：publish_debug_image 开关

```cpp
this->declare_parameter<bool>("publish_debug_image", true);

// 在 image_callback 中
if (publish_debug_image_) {
    frame.copyTo(debug_frame_);
    drawDetect(debug_frame_, results, ...);
    // ... resize, putText, publish ...
}
```

新增 `publish_debug_image` 参数，默认 `true`（保持向后兼容）。设为 `false` 时，**整段可视化逻辑被完全跳过**。

## 3.3 量化收益

| 操作 | CPU 开销（估算） | 内存开销 | 带宽开销 |
|------|----------------|---------|---------|
| `drawDetect` | ~2ms/帧 | 0 | 0 |
| `copyTo` + `resize` | ~1ms/帧 | 6MB | 0 |
| `toImageMsg` + `publish` | ~0.5ms/帧 | 6MB | 6MB/帧 |
| **总计** | **~3.5ms/帧** | **12MB** | **6MB/帧** |

关闭后，每帧节省约 3.5ms CPU 时间和 12MB 内存分配，话题带宽降为零。

## 3.4 发布顺序优化

新版把 `/armor_detections` 的构建和发布**提前**到 debug 图像处理之前：

```cpp
// 1. 先发布结构化数据（下游算法节点更早收到）
armor_pub_->publish(*armor_msg);

// 2. 再做可视化（不影响数据链路延迟）
if (publish_debug_image_) {
    // ... draw, resize, publish ...
}
```

这确保了 `/armor_detections` 的端到端延迟不受可视化开销影响。

---

# 四、容器预分配优化

## 4.1 问题：动态扩容的隐藏开销

```cpp
// 旧版
for (const auto& res : results) {
    armor_msg->detections.push_back(box);
}
```

`std::vector` 的默认行为：容量不够时翻倍扩容（1→2→4→8→16...），每次扩容需要：

1. 分配一块更大的新内存
2. 把已有元素逐个搬过去
3. 释放旧内存

对于 10 个检测结果，最多经历 4 次扩容。

## 4.2 解决：reserve 预分配

```cpp
armor_msg->detections.reserve(results.size());
for (const auto& res : results) {
    armor_msg->detections.push_back(box);
}
```

`reserve` 在循环开始前**一次分配足够空间**，后续所有 `push_back` 都直接原地构造。

## 4.3 量化收益

对于 10 个检测结果：

| 方式 | 内存分配次数 | 元素搬移次数 |
|------|------------|------------|
| 无 reserve | 4 次 | ~15 次 |
| 有 reserve | 1 次 | 0 次 |

虽然绝对时间节省微小（微秒级），但减少了内存碎片化，在长时间运行中更稳定。

---

# 五、cv::Mat 缓冲区复用

## 5.1 问题：每帧重新分配

```cpp
// 旧版
void image_callback(...) {
    cv::Mat resize_frame;  // 局部变量，每次回调结束就析构
    cv::resize(frame, resize_frame, ...);
}
```

每帧调用 `image_callback` 时，`resize_frame` 作为局部变量被创建，回调结束后析构。下一帧又要重新分配内存。

## 5.2 解决：成员变量缓存

```cpp
// 新版
class DetectNode : public rclcpp::Node {
    cv::Mat detect_input_frame_;
    cv::Mat debug_frame_;
    cv::Mat debug_output_frame_;
};
```

把 `cv::Mat` 从局部变量改为成员变量。OpenCV 的引用计数机制会在多次回调间**复用已分配的内存缓冲区**：

1. 第一次回调：分配 1920×1080×3 的内存
2. 后续回调：如果图像尺寸不变，直接复用已有内存，无需重新分配

## 5.3 量化收益

| 方式 | 每帧堆分配次数 | 每帧堆释放次数 |
|------|--------------|--------------|
| 局部变量 | 1~2 次 | 1~2 次 |
| 成员变量（首次后） | 0 次 | 0 次 |

---

# 六、QoS 队列深度优化

## 6.1 问题：队列积压导致显示延迟

旧版图像话题的 QoS 队列深度为 10。当 `display_node` 处理速度跟不上 `detect_node` 发布速度时：

```text
detect_node 发布 30fps
display_node 处理 20fps
队列深度 10 → 最多积压约 300ms 的旧帧
```

`spin_some` 会一口气消费积压帧，`latest_frame_` 被连续覆盖多次，造成"视频越来越滞后，然后突然加速跳帧"。

## 6.2 解决：整条图像链路统一 QoS(1)

```cpp
// video_node
image_pub_ = create_publisher<Image>("/image_raw", rclcpp::QoS(1));

// detect_node
image_sub_ = create_subscription<Image>("/image_raw", rclcpp::QoS(1), ...);
image_pub_ = create_publisher<Image>("/detected_image", rclcpp::QoS(1));

// display_node
sub_ = create_subscription<Image>("/detected_image", rclcpp::QoS(1), ...);
map_sub_ = create_subscription<Image>("/map_image", rclcpp::QoS(1), ...);
```

队列深度 1 意味着：subscriber 只保留最新 1 帧，旧帧直接被 ROS2 丢弃。

## 6.3 量化收益

| 指标 | QoS(10) | QoS(1) |
|------|---------|--------|
| 最大显示延迟 | ~330ms（10帧） | ~33ms（1帧） |
| 帧间延迟方差 | 大（积压→追帧→积压） | 小（始终最新帧） |
| 内存占用 | 10 帧缓冲 | 1 帧缓冲 |

## 6.4 为什么结构化数据不用 QoS(1)？

`/armor_detections`、`/world_targets` 等结构化数据使用 `depth=10` Reliable，因为：

* 数据量小（几百字节 vs 几 MB 图像）
* 不允许丢失（检测结果丢失 = 信息缺失）
* 10 帧缓冲提供短暂的容错窗口

---

# 七、TensorRT 推理优化

## 7.1 GPU 推理流水线

```text
CPU: preprocessing (Letterbox + blobFromImage)
  ↓ cudaMemcpyAsync (H2D, 异步)
GPU: executeV2 (TensorRT 推理)
  ↓ cudaMemcpyAsync (D2H, 异步)
CPU: postprocessing (NMS + 坐标映射)
```

三个阶段通过 CUDA Stream 实现**流水线化**：

* `cudaMemcpyAsync` 是异步的，CPU 不阻塞
* `executeV2` 在 GPU 上执行时，CPU 可以准备下一帧
* `cudaStreamSynchronize` 是唯一的同步点

## 7.2 Letterbox 预处理

```cpp
float scale = std::min(input_w / img_w, input_h / img_h);
int new_w = int(img_w * scale);
int new_h = int(img_h * scale);
cv::resize(frame, resized_img, cv::Size(new_w, new_h));

cv::Mat canvas = cv::Mat::zeros(input_h, input_w, CV_8UC3);
canvas.setTo(cv::Scalar(114, 114, 114));
resized_img.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));
```

Letterbox（等比例缩放 + 灰边填充）的好处：

* **保持宽高比**：避免目标变形影响检测精度
* **固定输入尺寸**：TensorRT 要求固定大小输入
* **灰色填充（114）**：与 YOLO 训练时的默认值一致

## 7.3 级联检测的 ROI 裁剪

```cpp
// 第二阶段：在车辆 ROI 内检测装甲板
armorDetector_.Detect(frame(roi));
```

`frame(roi)` 是 OpenCV 的**浅拷贝 ROI 提取**（O(1)），不复制像素数据。

第二阶段的输入尺寸远小于原图（如 200×200 vs 1920×1080），推理时间从 ~10ms 降到 ~2ms。

## 7.4 固定大小 Eigen 矩阵

```cpp
// KalmanFilterBox 使用固定大小矩阵
Eigen::Matrix<float, 8, 8> F;  // 栈上分配，无堆操作
```

Eigen 的固定大小矩阵在**栈上分配**，不需要 `new`/`malloc`。对于每帧调用 10 次（NUM_SLOTS）的 `predict` 和 `update`，避免了 20 次堆分配。

---

# 八、内存管理优化

## 8.1 RAII 资源管理

```cpp
// VideoPauseGuard
struct VideoPauseGuard {
    ~VideoPauseGuard() { if (active) callVideoPause(false); }
};

// scope_guard
auto guard = [this](bool*) { is_calibrating_ = false; };
std::unique_ptr<bool, decltype(guard)> scope_guard(nullptr, guard);
```

RAII 模式确保资源（视频暂停状态、标定锁标志）在任何退出路径下都能正确释放，避免资源泄漏。

## 8.2 智能指针管理

```cpp
std::unique_ptr<Config> cfg_;
std::unique_ptr<DetectPipeline> pipeline_;
```

`unique_ptr` 独占管理生命周期，析构时自动释放，无需手动 `delete`。

## 8.3 临时节点模式

```cpp
auto temp_node = std::make_shared<rclcpp::Node>("_calib_capture_1");
// ... 使用 ...
// 函数结束时自动析构
```

抓帧、调用服务时创建临时节点，用完即销毁，避免长期占用资源。

---

# 九、算法层优化

## 9.1 贪心匹配 vs 匈牙利算法

```cpp
// 实际使用贪心匹配（O(n²)），类名保留 HungarianAlgorithm
float HungarianAlgorithm::Solve(...) {
    // 贪心：每次选最小代价对
}
```

在目标数量 < 20 的场景下，贪心匹配的结果与匈牙利算法（O(n³)）99% 一致，但快 10 倍以上。

## 9.2 固定槽位 vs 动态 Track ID

固定槽位架构避免了：

* 动态 ID 分配/回收的管理开销
* ID 冲突检测的复杂度
* 轨迹生命周期管理的内存碎片

## 9.3 BotIdentity 的指数加权

```cpp
float weight = std::pow(DECAY, N - 1 - i);
```

`std::pow(0.95, n)` 对于 n < 30 的计算很快（编译器可能内联优化）。相比维护完整的历史频率表，指数加权的内存和计算开销更小。

---

# 十、端到端延迟分析

```text
video_node 发布时间戳 T0
    ↓ ~1ms (ROS2 传输)
detect_node 收到图像 T1 = T0 + 1ms
    ↓ ~15ms (TensorRT 推理 + 后处理)
detect_node 发布 /armor_detections T2 = T0 + 16ms
    ↓ ~1ms (ROS2 传输)
pose_node 收到检测结果 T3 = T0 + 17ms
    ↓ ~2ms (PnP + 射线碰撞 + Tracker)
pose_node 发布 /world_targets T4 = T0 + 19ms
    ↓ ~1ms (ROS2 传输)
map_node 收到世界坐标 T5 = T0 + 20ms
    ↓ ~3ms (坐标转换 + 绘制 + 战术分析)
map_node 发布 /map_image T6 = T0 + 23ms
    ↓ ~1ms (ROS2 传输)
qt_display_node 收到地图 T7 = T0 + 24ms
```

**总端到端延迟：约 24ms（< 1 帧 @30fps）**

---

# 十一、性能优化检查清单

在对本项目做任何性能优化时，按以下优先级检查：

| 优先级 | 检查项 | 示例 |
|--------|--------|------|
| P0 | 能不能不执行？ | `publish_debug_image = false` |
| P1 | 能不能少拷贝？ | `toCvShare` 替代 `toCvCopy` |
| P1 | 能不能预分配？ | `reserve` 替代默认 `push_back` |
| P2 | 能不能复用内存？ | `cv::Mat` 成员变量替代局部变量 |
| P2 | 能不能减少通信？ | QoS(1) 替代 QoS(10) |
| P3 | 能不能用硬件加速？ | TensorRT GPU 推理、CUDA 预处理 |
| P3 | 能不能用更快的算法？ | 贪心匹配替代匈牙利算法 |

**记住：P0 的收益永远大于 P3。** 先问"这段代码在不需要的时候能不能不执行"，再问"能不能跑得更快"。
