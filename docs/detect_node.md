# `detect_node.cpp` 逐行讲解

这份文件是**检测节点**，在你当前 ROS2 视觉链路里的位置是：

```text
video_node
   ↓  /image_raw
detect_node  ← 你在这里
   ↓  /detected_image（可视化图像）
   ↓  /armor_detections（结构化检测结果）
display_node / pose_node
```

`detect_node` 是整个系统的**核心算法节点**。它负责：

1. 订阅原始图像
2. 调用 TensorRT 检测流水线
3. 发布两路输出：带框的可视化图像 + 结构化检测数据

相比旧版，新版最大的变化是**新增了 `/armor_detections` 话题**，用自定义消息 `DetectionArray` 把检测结果以结构化方式发给下游节点（如 `pose_node`），而不只是发一张画好框的图。

这体现了 ROS2 节点设计的一个重要原则：

> **视觉图像供人看，结构化数据供机器下游节点用。**

此外，新版经过一轮**性能优化**，核心思路是：

> **减少图像拷贝、避免不必要的图像处理、预分配内存、零拷贝访问。**

---

# 一、头文件部分

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "pipeline.hpp"
#include "draw.hpp"
```

---

### `#include <rclcpp/rclcpp.hpp>`

ROS2 C++ 开发的核心头文件。`rclcpp::Node`、publisher/subscriber 接口、日志宏 `RCLCPP_INFO` 等都来自这里。

没有它，这份文件就不是 ROS2 节点。

---

### `#include <sensor_msgs/msg/image.hpp>`

标准 ROS2 图像消息类型。这个节点需要：

* **订阅** `sensor_msgs::msg::Image`（输入原图）
* **发布** `sensor_msgs::msg::Image`（输出可视化图）

---

### `#include <cv_bridge/cv_bridge.hpp>`

ROS 图像消息和 OpenCV `cv::Mat` 之间的桥梁。

这个节点同时涉及两个方向的转换：

```text
订阅时：sensor_msgs::msg::Image → cv::Mat（给检测算法用）
发布时：cv::Mat → sensor_msgs::msg::Image（给下游节点看）
```

---

### `#include <std_msgs/msg/header.hpp>`

图像消息里的 `header` 包含时间戳 `stamp` 和坐标系标识 `frame_id`。

这个节点的一个重要职责是**header 的继承与改写**：输入图像的 header 会被复制到输出消息中，保证时间链的连续性。

---

### `#include <opencv2/opencv.hpp>`

节点内部继续做 OpenCV 处理：

* `cv::Mat` 图像操作
* 图像缩放（新版新增）
* 画 FPS 文字

---

### `#include "tensorrt_detect_msgs/msg/detection_array.hpp"`

这是**自定义消息类型**，定义在 `tensorrt_detect_msgs` 包里。

`DetectionArray` 是一个数组，里面每个元素是一个 `DetectionBox`。这个节点要把检测流水线的结果打包成这种结构化消息发出去。

使用自定义消息是 ROS2 多包协作的典型做法：

> 算法包定义消息 → 消息包自动生成代码 → 所有节点包都能用

---

### `#include "tensorrt_detect_msgs/msg/detection_box.hpp"`

`DetectionBox` 是 `DetectionArray` 的数组元素类型，包含检测框坐标、置信度、类别 ID、车辆框、世界坐标等字段。

---

### `#include "ConfigManager.hpp"`

项目自己的配置管理类。节点通过它读取：

* 模型路径（TensorRT engine 文件）
* 相机参数
* 地图参数
* 类别名称表

---

### `#include "pipeline.hpp"`

检测流水线封装。节点本身不写检测细节，只调用 `pipeline_->process(frame)`。

这体现了**分层设计**：

> ROS2 节点 = 外壳（负责通信、参数、生命周期）
> Pipeline = 内核（负责模型推理、NMS、后处理）

---

### `#include "draw.hpp"`

项目自己的可视化函数。在图像上绘制检测框、类别标签等。

---

# 二、定义节点类

```cpp
class DetectNode : public rclcpp::Node
```

所有 ROS2 C++ 节点的固定起手式：继承 `rclcpp::Node`。

这样 `DetectNode` 就自动拥有了 ROS2 节点的一切能力：声明参数、创建 pub/sub、打日志、获取时钟等。

---

# 三、构造函数

```cpp
public:
    DetectNode() : Node("detect_node")
```

节点在 ROS2 系统中的注册名是 `"detect_node"`。以后在日志、ros2 topic list、ros2 node list 里看到这个名字，就是这里决定的。

---

# 四、声明参数

```cpp
this->declare_parameter<std::string>("config_dir",
    "/home/delphine/rm/tensorrt10_detect/configs");
this->declare_parameter<std::string>("input_topic", "/image_raw");
this->declare_parameter<std::string>("output_topic", "/detected_image");
this->declare_parameter<bool>("publish_debug_image", true);
this->declare_parameter<int>("debug_output_max_width", 1280);
```

这五行向 ROS2 参数服务器注册参数。

---

### `config_dir`

配置目录路径。节点启动时会从这里读取 `model.yaml`、`camera.yaml`、`map.yaml`、`runtime.yaml` 等配置文件。

用参数而不是硬编码，是为了让节点在不同机器、不同部署环境下都能灵活配置。

---

### `input_topic`

默认 `/image_raw`。说明这个节点等待外部节点（如 `video_node`）把图像发给它。

这是 ROS2 **话题解耦**思想的典型体现：`detect_node` 不关心图像到底来自视频文件、USB 摄像头还是网络串流，只要有人往 `/image_raw` 上发图，它就开始工作。

---

### `output_topic`

默认 `/detected_image`。处理完后，带框的可视化图像发到这个话题。

---

### `publish_debug_image`

**（性能优化新增）** 控制是否发布可视化调试图像，默认 `true`。

在纯自动运行模式（不需要给人看画面）时，可以设为 `false`，**彻底跳过** `drawDetect`、`resize`、`putText` 和图像发布，显著降低 CPU 和话题带宽占用。这是本次性能优化中收益最大的开关。

---

### `debug_output_max_width`

**（性能优化新增）** 控制调试输出图像的最大宽度，默认 `1280`。

旧版硬编码为 1280，新版改为参数，可根据实际带宽和显示需求动态调整。

---

# 五、读取参数

```cpp
std::string config_dir = this->get_parameter("config_dir").as_string();
input_topic_ = this->get_parameter("input_topic").as_string();
output_topic_ = this->get_parameter("output_topic").as_string();
publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
const int64_t debug_output_max_width_param =
    this->get_parameter("debug_output_max_width").as_int();
debug_output_max_width_ = static_cast<int>(std::max<int64_t>(1, debug_output_max_width_param));
```

`declare_parameter` 只是注册，`get_parameter` 才是真正取值。

`config_dir` 是局部变量，因为只在构造函数里初始化 `Config` 时用一次；`input_topic_` 和 `output_topic_` 是成员变量，因为后面创建 pub/sub 时还要用。

`std::max<int64_t>(1, ...)` 保证即使参数传入非法值（如 0 或负数），宽度也不会变成 0，避免后续 `resize` 出现除零或空图。

---

# 六、打印初始化日志

```cpp
RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());
```

这三行是 ROS2 的 INFO 级别日志。节点启动时打印关键配置，是排查参数是否生效的第一道防线。

---

# 七、创建业务对象

```cpp
cfg_ = std::make_unique<Config>(config_dir);
pipeline_ = std::make_unique<DetectPipeline>(*cfg_);
```

---

### `std::make_unique<Config>(config_dir)`

创建配置对象，读取指定目录下的所有 YAML 配置。

用 `std::unique_ptr`（独占指针）管理，因为这个配置对象只属于这个节点，不需要共享。

---

### `std::make_unique<DetectPipeline>(*cfg_)`

用配置对象初始化检测流水线。

这里传的是 `*cfg_`（解引用），因为 `DetectPipeline` 的构造函数接收 `Config&` 引用。

这行代码背后可能涉及大量初始化工作：

* 加载 TensorRT engine 文件
* 分配 GPU 显存
* 初始化 CUDA 流
* 构建预处理/后处理管线

所以 `pipeline_` 必须在构造函数里**只初始化一次**，然后反复复用。每帧重新创建的开销是不可接受的。

---

# 八、创建两个 Publisher

```cpp
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::QoS(1));
armor_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::DetectionArray>("/armor_detections", 10);
```

这是新版相比旧版最重要的变化：**从 1 个 publisher 变成 2 个 publisher**。

---

### `image_pub_`

* 类型：`sensor_msgs::msg::Image`
* 话题：`/detected_image`
* QoS：`rclcpp::QoS(1)`（Keep Last 1）

发布带检测框、FPS 文字的可视化图像，供 `display_node` 显示给人类看。

**（性能优化相关）** 如果 `publish_debug_image_` 为 `false`，这个 publisher 实际上不会发布任何消息，避免了带宽浪费。

---

### `armor_pub_`

* 类型：`tensorrt_detect_msgs::msg::DetectionArray`
* 话题：`/armor_detections`
* QoS：`depth=10` Reliable

发布**结构化检测结果**，供 `pose_node` 做位姿解算、世界坐标转换。

这里体现了 ROS2 中一个非常重要的设计模式：

> **一个节点可以发布多个话题，把同一批数据用不同形式发出去。**
>
> * 图像 → 给人看
> > * 结构化消息 → 给机器下游节点用

---

# 九、创建 Subscriber

```cpp
image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_, rclcpp::QoS(1),
    std::bind(&DetectNode::image_callback, this, std::placeholders::_1));
```

这是事件驱动的核心：

> 当 `/image_raw` 上有新图像时，自动调用 `image_callback`。

`std::bind` 把成员函数绑定为回调，这是现代 C++ 配合 ROS2 的标准写法。

---

# 十、构造函数完成日志

```cpp
RCLCPP_INFO(this->get_logger(), "DetectNode 初始化完成，等待图像输入...");
```

所有准备工作做完：参数读取、配置加载、模型初始化、publisher/subscriber 创建。节点进入待机状态，等待图像消息触发回调。

---

# 十一、核心回调：`image_callback`

```cpp
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
```

这是节点真正的"干活函数"。每收到一帧图像，ROS2 的 Executor 就会自动调用它。

参数 `msg` 是 `SharedPtr`（`std::shared_ptr`），ROS2 用智能指针管理消息生命周期，避免拷贝开销。

---

# 十二、`try` 块与图像转换

```cpp
try {
    double input_delay_ms = (this->now() - msg->header.stamp).seconds() * 1000.0;

    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    const cv::Mat& frame = cv_ptr->image;
```

---

### `input_delay_ms`

**（链路时延监控）**

计算从图像产生（`msg->header.stamp`，由 `video_node` 在发布时设置）到进入检测回调的**传输+调度延迟**。

```cpp
double input_delay_ms = (this->now() - msg->header.stamp).seconds() * 1000.0;
```

这个指标可以帮你定位延迟瓶颈：

* `input_delay_ms ≈ 0-5ms`：正常，图像几乎无排队直接到达
* `input_delay_ms > 30ms`：subscriber 队列积压，或 CPU 调度延迟
* `input_delay_ms` 持续增大：下游消费速度跟不上上游发布速度

它被打印在节流日志中，每 10 秒输出一次，方便长期观察链路健康状况。

----

### `cv_bridge::toCvShare(msg, "bgr8")`

**（性能优化关键改动）**

把 ROS2 图像消息按 `bgr8` 编码转成 OpenCV 可用的 `cv::Mat`。

`"bgr8"` 是 OpenCV 最常用的彩色图像格式：蓝-绿-红三通道，每通道 8 位。

旧版使用的是 `toCvCopy`，会在堆上分配一块新内存，把整帧图像**深拷贝**一份。对于 1920×1080×3 的图像，每帧约 **6MB** 的拷贝开销。

新版改用 `toCvShare`，它只返回一个**共享指针**，`cv::Mat` 内部的数据指针直接指向 ROS 消息底层的图像缓冲区，**零拷贝**。这是本次优化中最基础、也最重要的一步。

---

### `const cv::Mat& frame = cv_ptr->image`

从 `cv_ptr` 中取出真正的 `cv::Mat`，并用 `const` 引用绑定。

`const` 在这里很重要：因为 `toCvShare` 让 `frame` 直接指向 ROS 消息的原始数据，任何对 `frame` 的写操作都会**篡改原始消息**，可能影响到其他订阅者。用 `const` 在语义上明确这是只读访问。

---

# 十三、调用检测流水线

```cpp
std::vector<Result> results = pipeline_->process(frame);
```

这行是整个节点的**算法核心入口**。

* 输入：当前帧 `frame`
* 调用：TensorRT 检测流水线
* 输出：`Result` 数组，每个元素包含检测框、类别、置信度、车辆框、世界坐标等

节点本身不实现任何检测算法，完全复用原有的 `DetectPipeline`。

---

# 十四、FPS 计算

```cpp
auto now = std::chrono::steady_clock::now();
double dt = std::chrono::duration<double>(now - last_time_).count();
last_time_ = now;

double instant_fps = 1.0 / std::max(dt, 1e-6);
fps_ = 0.9 * fps_ + 0.1 * instant_fps;
```

---

### 时间测量

`std::chrono::steady_clock` 是 C++11 提供的单调时钟，不受系统时间调整影响，适合测时间间隔。

`dt` 是两次回调之间的时间差。

---

### 指数平滑

```cpp
fps_ = 0.9 * fps_ + 0.1 * instant_fps;
```

90% 保留旧值，10% 引入新值。这样显示出来的 FPS 不会剧烈抖动。

`std::max(dt, 1e-6)` 防止除零。

---

# 十五、构建并发布结构化检测消息（新版核心）

```cpp
auto armor_msg = std::make_shared<tensorrt_detect_msgs::msg::DetectionArray>();
armor_msg->header = msg->header;   // 复用图像时间戳，方便下游同步
armor_msg->header.frame_id = "detection";
armor_msg->detections.reserve(results.size());
```

**（性能优化新增 `reserve`）**

---

### 为什么要复用 `msg->header`？

这是 ROS2 **消息链时间同步**的关键技巧。

`pose_node` 收到 `DetectionArray` 后，可以通过 `header.stamp` 知道这批检测对应的是哪一帧图像。如果以后要做多传感器时间对齐（比如和雷达、IMU 融合），这个共同的时间戳就是纽带。

---

### `reserve(results.size())` —— 预分配内存

**（性能优化关键改动）**

旧版直接 `push_back`，如果检测结果较多（比如一帧检测到 10+ 个目标），`std::vector` 会经历多次翻倍扩容（1→2→4→8→16...），每次扩容都要：

1. 分配一块更大的新内存
2. 把已有元素逐个搬过去
3. 释放旧内存

`reserve(results.size())` 在循环开始前**一次分配足够空间**，后续所有 `push_back` 都直接原地构造，**消除所有动态扩容开销**。

---

### 填充 DetectionBox 数组

```cpp
for (const auto& res : results) {
    if (res.idx == robot_id::CAR) {
        continue;  // 只发布装甲板结果，车辆检测不发给下游
    }

    tensorrt_detect_msgs::msg::DetectionBox box;
    box.idx         = res.idx;
    box.confidence  = res.confidence;
    box.x           = res.box.x;
    box.y           = res.box.y;
    box.width       = res.box.width;
    box.height      = res.box.height;
    box.armor_color = res.armorColor;
    box.is_dead     = res.isDead;
    box.car_x       = res.car_box.x;
    box.car_y       = res.car_box.y;
    box.car_width   = res.car_box.width;
    box.car_height  = res.car_box.height;
    box.world_x     = res.worldPoint.x;
    box.world_y     = res.worldPoint.y;
    box.fps         = static_cast<float>(fps_);

    armor_msg->detections.push_back(box);
}
```

这段代码把 C++ 结构体 `Result` 翻译成 ROS2 消息 `DetectionBox`。

---

### CAR 过滤

```cpp
if (res.idx == robot_id::CAR) {
    continue;
}
```

**只发布装甲板检测结果**，第一层车辆检测（`CAR`, `idx=0`）不发给下游节点。

原因：`pose_node` 和 `map_node` 只关心装甲板（和死亡装甲板），车辆检测框对它们没有意义。debug 图像仍然会画车辆框（供人看），但结构化数据只发装甲板。

---

### `DetectionBox` 包含的字段解读

| 字段 | 含义 |
|------|------|
| `idx` | 类别 ID |
| `confidence` | 置信度 |
| `x/y/width/height` | 装甲板检测框 |
| `armor_color` | 装甲颜色/队伍 ID |
| `is_dead` | **死亡装甲板标志**（`true` = 死亡） |
| `car_x/car_y/car_width/car_height` | 车辆整体检测框 |
| `world_x/world_y` | 世界坐标（如果 pipeline 已计算） |
| `fps` | 当前帧率 |

字段设计非常完整，下游节点（`pose_node`）可以按需取用。新增的 `is_dead` 字段让死亡状态独立传递，不再依赖 `armor_color == 0` 的隐式判断。

---

### 发布

```cpp
armor_pub_->publish(*armor_msg);
```

把结构化检测数据发到 `/armor_detections`。`pose_node` 订阅这个话题后，就能拿到每个目标的像素坐标，进而解算出世界坐标。

**（性能优化相关）** 新版把 `/armor_detections` 的构建和发布**提前**到了 debug 图像处理之前。这意味着：

- 下游节点能**更早收到**检测结果
- 如果 `publish_debug_image=false`，结构化数据发布的延迟进一步降低

---

# 十六、条件化发布可视化图像（性能优化核心）

```cpp
if (publish_debug_image_) {
    frame.copyTo(debug_frame_);
    drawDetect(debug_frame_, results, cfg_->model.classNames);

    if (debug_frame_.cols > debug_output_max_width_) {
        const double debug_scale = static_cast<double>(debug_output_max_width_) /
                                   static_cast<double>(debug_frame_.cols);
        const int target_height = static_cast<int>(debug_frame_.rows * debug_scale);
        cv::resize(debug_frame_, debug_output_frame_, cv::Size(debug_output_max_width_, target_height));
    } else {
        debug_output_frame_ = debug_frame_;
    }

    cv::putText(debug_output_frame_, cv::format("FPS: %.1f", fps_),
                cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);

    std_msgs::msg::Header header = msg->header;
    header.frame_id = "detected_frame";
    auto out_msg = cv_bridge::CvImage(header, "bgr8", debug_output_frame_).toImageMsg();
    image_pub_->publish(*out_msg);
}
```

---

### 为什么要条件化？

旧版**每帧都**执行：

1. `drawDetect` —— 遍历所有检测结果，画框、画文字、画线
2. `cv::resize` —— 如果图大，还要做图像缩放
3. `cv::putText` —— 画 FPS
4. `toImageMsg` + `publish` —— 编码并发布

这些操作对于**纯自动运行模式**（不需要给人看画面，只需要 `/armor_detections` 给下游算法用）完全是浪费。

新版通过 `publish_debug_image_` 参数，可以**彻底跳过**这段逻辑。当设为 `false` 时：

- 省掉 `drawDetect` 的 CPU 开销
- 省掉 `resize` 的 CPU 开销
- 省掉 `toImageMsg` 的内存分配和编码开销
- 省掉 `/detected_image` 话题的全部带宽占用

这是本次性能优化中**收益最大**的改动。

---

### `copyTo` 保护原始数据

```cpp
frame.copyTo(debug_frame_);
drawDetect(debug_frame_, results, cfg_->model.classNames);
```

因为前面用了 `toCvShare`，`frame` 直接指向 ROS 消息原始缓冲区。如果直接 `drawDetect(frame, ...)`，会**篡改原始图像数据**，可能导致同一消息的其他订阅者看到被画过框的图。

`copyTo` 保证只修改副本，逻辑更安全。

---

### 成员变量缓存 `cv::Mat`

```cpp
cv::Mat debug_frame_;
cv::Mat debug_output_frame_;
```

**（性能优化新增）**

旧版 `resize_frame` 是局部变量，每次回调结束就析构，下次来新帧再重新分配内存。改成成员变量后，OpenCV 的引用计数机制可以在多次回调间**复用已分配的内存缓冲区**，减少频繁的堆分配/释放。

---

### 图像缩放（参数化版）

```cpp
if (debug_frame_.cols > debug_output_max_width_) {
    const double debug_scale = static_cast<double>(debug_output_max_width_) /
                               static_cast<double>(debug_frame_.cols);
    const int target_height = static_cast<int>(debug_frame_.rows * debug_scale);
    cv::resize(debug_frame_, debug_output_frame_, cv::Size(debug_output_max_width_, target_height));
} else {
    debug_output_frame_ = debug_frame_;
}
```

宽度超过 `debug_output_max_width_` 时，等比例缩放到指定宽度。

为什么要缩放？原始视频可能是 1920×1080 甚至 4K。如果直接发布原图：

* 话题带宽占用大
* `display_node` 处理慢
* 网络传输延迟高

所以这里做了一个**上限控制**。新版把硬编码的 1280 改成参数 `debug_output_max_width_`，部署更灵活。

---

### 把 FPS 画到图像上

```cpp
cv::putText(debug_output_frame_, cv::format("FPS: %.1f", fps_),
            cv::Point(20, 40),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0, cv::Scalar(0, 255, 0), 2);
```

注意这里是画在 `debug_output_frame_` 上，因为最终发布的是缩放后的图。

---

### Header 继承与改写

```cpp
std_msgs::msg::Header header = msg->header;
header.frame_id = "detected_frame";
```

* **继承**：原图的时间戳 `stamp` 被保留下来。这对于下游节点做时间同步非常重要。
* **改写**：`frame_id` 改成 `"detected_frame"`，语义上表明这是"检测后输出的图像帧"，不再是原始视频帧。

---

### 转回 ROS 消息并发布

```cpp
auto out_msg = cv_bridge::CvImage(header, "bgr8", debug_output_frame_).toImageMsg();
image_pub_->publish(*out_msg);
```

`CvImage` 构造时传入 header、编码、图像数据，然后 `toImageMsg()` 生成 ROS2 图像消息，最后通过 publisher 发出去。

---

# 十七、节流日志

```cpp
RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    10000,
    "检测到 %zu 个目标，fps: %.1f，input_delay: %.2f ms",
    results.size(), fps_, input_delay_ms);
```

`RCLCPP_INFO_THROTTLE` 是**节流日志**，最多每 10000 毫秒（10 秒）打印一次。

日志内容包含：

* 检测到的目标数量
* 当前平滑 FPS
* 图像输入延迟 `input_delay_ms`

---

# 十八、异常处理

```cpp
catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
}
catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "检测回调异常: %s", e.what());
}
```

两层异常捕获：

1. `cv_bridge::Exception`：图像编码转换失败
2. `std::exception`：其他所有标准异常

这样做的好处是：**某一帧处理失败不会导致整个节点崩溃**，下一帧来了还能继续处理。

---

# 十九、成员变量

```cpp
std::unique_ptr<Config> cfg_;
std::unique_ptr<DetectPipeline> pipeline_;

std::string input_topic_;
std::string output_topic_;
bool publish_debug_image_ = true;
int detect_input_width_ = 0;
int detect_input_height_ = 0;
int debug_output_max_width_ = 1280;

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::Publisher<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_pub_;
std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
double fps_ = 0.0;
cv::Mat detect_input_frame_;
cv::Mat debug_frame_;
cv::Mat debug_output_frame_;
```

---

### `cfg_` / `pipeline_`

用 `unique_ptr` 独占管理的业务对象。节点生命周期内只创建一次，反复复用。

---

### `image_sub_` / `image_pub_` / `armor_pub_`

Pub/Sub 对象必须作为成员变量保存。如果只在构造函数里用局部变量，函数结束后它们会被析构，节点就失去了通信能力。

---

### `armor_pub_`（新版新增）

这是新版最重要的成员变量。没有它，结构化检测结果就发不出去，`pose_node` 也就收不到数据。

---

### `publish_debug_image_` / `debug_output_max_width_`

**（性能优化新增）** 控制可视化输出的开关和尺寸上限。

---

### `debug_frame_` / `debug_output_frame_`

**（性能优化新增）** 缓存的 OpenCV 图像缓冲区，避免每帧重复分配内存。

---

# 二十、`reloadROI`：前哨站 ROI 动态重载服务

```cpp
void reloadROI(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
               std_srvs::srv::Trigger::Response::SharedPtr response)
{
    try {
        std::string config_dir = this->get_parameter("config_dir").as_string();
        std::filesystem::path dir(config_dir);
        std::string outpost_yaml = (dir / "outpost_roi.yaml").string();

        YAML::Node cfg = YAML::LoadFile(outpost_yaml);
        cfg_->model.outpostEnabled = cfg["outpost_enabled"]
                                        ? cfg["outpost_enabled"].as<bool>()
                                        : false;
        if (cfg["outpost_roi"]) {
            cfg_->model.outpostRoi = cfg["outpost_roi"].as<std::vector<int>>();
        }
        cfg_->model.outpostScoreThreshold = cfg["outpost_score_threshold"]
                                                ? cfg["outpost_score_threshold"].as<float>()
                                                : 0.0f;

        response->success = true;
        response->message = "outpost ROI 配置已重载";
        RCLCPP_INFO(this->get_logger(), "outpost ROI 配置已重载: enabled=%s, roi=[%d,%d,%d,%d]",
                    cfg_->model.outpostEnabled ? "true" : "false",
                    cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[0] : -1,
                    cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[1] : -1,
                    cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[2] : -1,
                    cfg_->model.outpostRoi.size() >= 4 ? cfg_->model.outpostRoi[3] : -1);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("重载失败: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "outpost ROI 重载失败: %s", e.what());
    }
}
```

---

### 为什么读取 `outpost_roi.yaml` 而不是 `model.yaml`？

`roi_set_node` 标定后把前哨站 ROI 保存到 `outpost_roi.yaml`。如果 `reloadROI` 去读 `model.yaml`，永远读不到更新后的 ROI，重载就失去了意义。

### 服务接口

* 话题：`/detect_node/reload_roi`
* 类型：`std_srvs/srv/Trigger`
* 触发方式：`roi_set_node` 标定完成后自动调用，或手动 `ros2 service call /detect_node/reload_roi std_srvs/srv/Trigger`

---

# 二十一、`main` 函数

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

这是 ROS2 C++ 节点的经典四步法：

| 步骤 | 函数 | 作用 |
|------|------|------|
| 1 | `rclcpp::init` | 初始化 ROS2 通信框架 |
| 2 | `make_shared<DetectNode>()` | 创建节点对象，执行构造函数 |
| 3 | `rclcpp::spin` | 进入事件循环，等待并处理回调 |
| 4 | `rclcpp::shutdown` | 清理资源，优雅退出 |

---

# 二十一、完整数据流回顾

```text
video_node ──/image_raw──→ detect_node
                                      │
                                      ├── /detected_image ──→ display_node
                                      │
                                      └── /armor_detections ──→ pose_node
```

`detect_node` 是系统的**分水岭**：

* 上游：只关心图像输入
* 下游：分成两条独立链路
  * 可视化链路：`display_node` 给人看
  * 数据链路：`pose_node` → `map_node` 给算法下游用

两条链路互不干扰，可以独立启动、停止、替换。这就是 ROS2 **话题解耦**带来的灵活性。

---

# 二十二、性能优化总结

本次 `detect_node` 的性能优化围绕一个核心原则：**不做无用功**。具体手段包括：

## 1. 零拷贝图像访问

```cpp
auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
const cv::Mat& frame = cv_ptr->image;
```

用 `toCvShare` 替代 `toCvCopy`，每帧省下一次整图深拷贝（约 6MB 对于 1080p）。

## 2. 条件化 Debug 图像生成

```cpp
if (publish_debug_image_) { /* draw + resize + publish */ }
```

增加 `publish_debug_image` 参数。纯自动模式时关闭可视化，彻底跳过 `drawDetect`、`resize`、`putText` 和图像发布，CPU 和带宽双省。

## 3. 预分配容器内存

```cpp
armor_msg->detections.reserve(results.size());
```

提前为 `DetectionArray` 分配足够空间，消除 `push_back` 过程中的多次动态扩容。

## 4. 复用 Mat 缓冲区

```cpp
cv::Mat debug_frame_;
cv::Mat debug_output_frame_;
```

把临时 `cv::Mat` 从局部变量改为成员变量，OpenCV 在多次回调间复用已分配的内存，减少堆操作。

## 5. 保护原始数据

```cpp
frame.copyTo(debug_frame_);
drawDetect(debug_frame_, ...);
```

因为使用了 `toCvShare`，`frame` 指向 ROS 消息原始缓冲区。`copyTo` 保证绘制操作不篡改原始数据，避免影响其他订阅者。

## 6. 参数化缩放上限

```cpp
this->declare_parameter<int>("debug_output_max_width", 1280);
```

把硬编码的 1280 改为参数，方便在不同带宽环境下调整。

## 7. 先发布结构化数据，再做可视化

把 `/armor_detections` 的发布提前到 debug 图像处理之前，下游节点能更早拿到结果，端到端延迟更低。

---

# 二十三、从这份代码里学到的设计要点

## 1. 一输入、多输出

一个节点完全可以根据同一批原始数据，生成多种形式的输出：

* 图像 → 可视化
* 结构化消息 → 机器处理

## 2. Header 时间戳链

```cpp
output_msg->header = input_msg->header;
```

把输入消息的时间戳传递到输出消息，是 ROS2 多节点时间同步的基础。

## 3. 精度与带宽分离

检测用原图保证精度，可视化图缩放后发布节省带宽。两个路径独立，互不影响。

## 4. 异常隔离

用 `try/catch` 包裹回调，单帧失败不崩节点，保证系统鲁棒性。

## 5. 性能优化要抓主要矛盾

不要一上来就抠微优化。先问自己：**这段代码在不需要的时候能不能不执行？**

`publish_debug_image` 开关就是一个典型例子——关掉它，一整段高开销逻辑直接消失，收益远大于在某几个循环里省几个时钟周期。
