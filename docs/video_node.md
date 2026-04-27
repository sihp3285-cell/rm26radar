# `video_node.cpp` 逐行讲解

这份文件是**视频源节点**，在你当前 ROS2 视觉链路里的位置是：

```text
video_node  ← 你在这里
   ↓  /image_raw
detect_node
   ↓  /detected_image, /armor_detections
pose_node
   ↓  /world_targets
map_node
   ↓  /map_image, /radar_map
display_node
```

`video_node` 是整个系统的**数据源头**。它不负责任何算法，只负责一件事：

> **把本地视频文件一帧一帧读出来，包装成 ROS2 图像消息，发到话题上。**

在 ROS2 架构中，这种节点叫做 **Sensor Node（传感器节点）** 或 **Source Node（源节点）**。它的核心职责是：

* 把非 ROS 格式的数据（本地视频文件）转换成 ROS 消息
* 按固定节奏发布，模拟真实传感器的时序行为
* 让下游算法节点**无需关心数据来源**

---

# 一、头文件部分

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
```

---

### `#include <rclcpp/rclcpp.hpp>`

ROS2 C++ 节点的基础头文件。你用它来定义节点类、创建 publisher、创建 timer、打日志、调用 `spin` 等。

---

### `#include <sensor_msgs/msg/image.hpp>`

标准 ROS2 图像消息类型。`video_node` 的输出就是这个类型。

```text
cv::Mat（OpenCV 世界）→ sensor_msgs::msg::Image（ROS2 世界）
```

---

### `#include <cv_bridge/cv_bridge.hpp>`

虽然这个节点没有"订阅"图像，但它依然需要 `cv_bridge`。

原因是它要把 OpenCV 的 `cv::Mat` 转成 ROS2 的 `sensor_msgs::msg::Image`。`cv_bridge` 提供了这个方向的转换能力。

---

### `#include <std_msgs/msg/header.hpp>`

发布图像时需要填充 `header`，包含：

* `stamp`：时间戳，告诉下游这帧图像是在什么时刻产生的
* `frame_id`：坐标系标识，告诉下游这帧图像属于哪个坐标系

---

### `#include <opencv2/opencv.hpp>`

因为本地视频读取完全依赖 OpenCV：

* `cv::VideoCapture`：打开视频文件、读取帧
* `cv::Mat`：存储每一帧图像

---

# 二、定义节点类

```cpp
class VideoNode : public rclcpp::Node
```

固定起手式：所有 ROS2 节点类都继承自 `rclcpp::Node`。

这样 `VideoNode` 自动获得 ROS2 节点的一切基础设施：参数系统、日志系统、时钟、Pub/Sub 接口等。

---

# 三、构造函数

```cpp
public:
    VideoNode() : Node("video_node")
```

节点在 ROS2 系统中的注册名是 `"video_node"`。

在 `ros2 node list` 里能看到它，日志里也会显示 `[video_node]`。

---

# 四、声明参数

```cpp
this->declare_parameter<std::string>("video_path", "/home/delphine/rm/car_project/test/005.mp4");
this->declare_parameter<std::string>("topic_name", "/image_raw");
this->declare_parameter<int>("fps", 30);
```

这三行向 ROS2 参数服务器注册参数。

---

### `video_path`

字符串参数，默认值指向一个本地 MP4 文件。

参数化而不是硬编码，意味着你以后可以通过 launch 文件或命令行换视频源，而不需要重新编译：

```bash
ros2 run tensorrt_detect video_node --ros-args -p video_path:=/path/to/another.mp4
```

---

### `topic_name`

默认 `/image_raw`。

这是 ROS2 视觉领域的一个**约定俗成的话题名**。很多视觉工具（如 `image_view`、`rviz2`）默认就订阅 `/image_raw`。

把视频发到这个名字上，下游节点不需要改任何配置就能对接。

---

### `fps`

整数参数，默认 30。

这个参数的意义非常深：

> 它不是在"限制视频播放速度"，而是在**模拟真实传感器的采样频率**。

真实的 USB 摄像头、工业相机、GigE 相机，都是以固定帧率采集图像的。`video_node` 用 `fps` 参数来模拟这种行为，让下游算法节点感受到的时序特性和真实部署时一致。

---

# 五、读取参数

```cpp
video_path_ = this->get_parameter("video_path").as_string();
topic_name_ = this->get_parameter("topic_name").as_string();
fps_setting_ = this->get_parameter("fps").as_int();
```

把参数值取出来，保存到成员变量中。

这三个值会在整个节点生命周期里反复使用：`video_path_` 和 `topic_name_` 不需要再变，但 `fps_setting_` 参与了定时器周期的计算。

---

# 六、打印初始化信息

```cpp
RCLCPP_INFO(this->get_logger(), "视频路径: %s", video_path_.c_str());
RCLCPP_INFO(this->get_logger(), "发布话题: %s", topic_name_.c_str());
RCLCPP_INFO(this->get_logger(), "设定帧率: %d", fps_setting_);
```

节点启动时打印关键配置，这是排查参数是否生效的最佳实践。

比如如果你通过 launch 文件改了 `fps`，但日志里还是显示 30，说明参数没传进来。

---

# 七、打开视频

```cpp
cap_.open(video_path_);
if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_path_.c_str());
    rclcpp::shutdown();
    return;
}
```

---

### `cap_.open(video_path_)`

`cap_` 是 `cv::VideoCapture` 对象。这行让它尝试打开指定路径的视频文件。

---

### 错误处理

如果打开失败：

1. `RCLCPP_ERROR` 打印错误日志
2. `rclcpp::shutdown()` 请求关闭 ROS2 系统
3. `return` 结束构造函数

这体现了**快速失败（Fail Fast）**原则：输入源都打不开，节点没有继续运行的意义，尽早退出比带着错误跑更好。

---

# 八、创建 Publisher

```cpp
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, rclcpp::QoS(1));
```

---

### `create_publisher<sensor_msgs::msg::Image>`

说明这个 publisher 发送的是图像消息。

---

### `topic_name_`

话题名由参数控制，默认 `/image_raw`。

---

### `rclcpp::QoS(1)`

图像话题的 QoS 配置：**Keep Last 1**。

在 ROS2 中，publisher 和 subscriber 之间是**异步解耦**的。如果 subscriber 处理得慢，消息会在 publisher 的发送队列里缓冲。`QoS(1)` 表示只保留最新 1 条消息，旧消息直接丢弃。

对于视频流这种连续数据：

* 用户更关心最新的画面，旧帧无意义
* 深度 1 可以防止队列积压导致整条图像链路延迟
* 与下游 `detect_node`（`QoS(1)` 订阅）和 `display_node`（`QoS(1)` 订阅）保持一致，形成统一的低延迟图像传输策略

---

# 九、根据 FPS 计算定时器周期

```cpp
int interval_ms = 1000 / std::max(fps_setting_, 1);
```

---

### 计算逻辑

如果 `fps_setting_ = 30`，那每帧间隔就是 `1000 / 30 ≈ 33ms`。

这个变量表示：定时器每隔多少毫秒触发一次。

---

### `std::max(fps_setting_, 1)`

防御式编程。防止用户把 fps 设成 0 或负数，导致除零错误。

---

# 十、创建定时器

```cpp
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(interval_ms),
    std::bind(&VideoNode::timer_callback, this));
```

这是 `video_node` 的**运行驱动核心**。

---

### `create_wall_timer`

创建基于**系统挂钟时间（Wall Clock）**的定时器。

在 ROS2 中，定时器有两种概念：

| 定时器类型 | 依据 | 适用场景 |
|-----------|------|---------|
| Wall Timer | 系统真实时间 | 模拟固定帧率、UI 刷新 |
| ROS Timer | ROS 仿真时间（`/clock`） | 与 Gazebo 等仿真器同步 |

这里用 `create_wall_timer` 而不是 `create_timer`，是因为视频播放要跟着真实时间走，而不是仿真时间。

---

### 定时器周期

```cpp
std::chrono::milliseconds(interval_ms)
```

用 C++11 的 `std::chrono` 库表示时间长度，类型安全、可读性好。

---

### 回调绑定

```cpp
std::bind(&VideoNode::timer_callback, this)
```

意思是：每次定时器触发，就调用当前对象的 `timer_callback()` 函数。

这建立了一个循环机制：

```text
每隔 interval_ms 毫秒
→ 调 timer_callback
→ 读一帧视频
→ 转成 ROS 图像消息
→ 发布到 /image_raw
```

---

# 十一、初始化完成日志

```cpp
RCLCPP_INFO(this->get_logger(), "VideoNode 初始化完成，开始发布视频帧");
```

构造函数执行完毕：参数读取、视频打开、publisher 创建、timer 创建。节点进入持续发帧的运行状态。

---

# 十二、定时器回调：`timer_callback`

```cpp
private:
    void timer_callback()
```

这是整个 `video_node` 的真正执行体。每次定时器触发，它就执行一次。

可以把它理解成"视频节点版本的主循环"。

---

# 十三、读取一帧视频

```cpp
cv::Mat frame;
if (!cap_.read(frame)) {
    RCLCPP_WARN(this->get_logger(), "视频播放结束，重新回到开头");
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    return;
}
```

---

### `cap_.read(frame)`

从视频里读取一帧到 `frame` 中。如果成功，`frame` 里就有图像内容。

---

### 循环播放

如果读取失败（通常是播到结尾了）：

1. `RCLCPP_WARN` 打印警告
2. `cap_.set(cv::CAP_PROP_POS_FRAMES, 0)` 把读头跳回第 0 帧
3. `return` 结束本次回调

所以你的视频节点不是播完就停，而是会**从头循环播放**。这对于长时间测试和调试非常方便。

---

# 十四、转成 ROS2 图像消息

```cpp
auto msg = cv_bridge::CvImage(
    std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
```

这行非常关键。

* 用一份新的 `std_msgs::msg::Header()`
* 指定编码 `"bgr8"`
* 把 `frame` 这张 `cv::Mat`
* 包装成 ROS2 的图像消息

也就是：

```text
cv::Mat → sensor_msgs::msg::Image
```

---

# 十五、填充 Header

```cpp
msg->header.stamp = this->now();
msg->header.frame_id = "video_frame";
```

---

### `msg->header.stamp = this->now()`

给这帧图像打上**当前 ROS2 时间戳**。

`this->now()` 返回的是 `rclcpp::Time`，它基于 ROS2 时钟（默认是系统时钟）。

这个时间戳非常重要：

* 下游节点可以用它做**消息时序对齐**
* 如果以后做多摄像头同步，所有源节点都用 `this->now()` 打戳，就能对齐
* `ros2 bag record` 录制时，时间戳是回放的关键

---

### `msg->header.frame_id = "video_frame"`

给图像消息一个**坐标系/来源标识**。

这里写 `"video_frame"`，语义上表示：

> 这是来自视频源节点的一帧图像。

后面 `detect_node` 收到它时，会继承这个 header 再改写为 `"detected_frame"`。这就形成了一条清晰的**消息血缘链**：

```text
video_frame → detected_frame
```

---

# 十六、发布图像

```cpp
image_pub_->publish(*msg);
```

这是 `video_node` 的最终输出动作。

一旦发布成功，`detect_node` 就能在 `/image_raw` 上收到它，整个视觉链就流动起来了。

注意这里传的是 `*msg`（解引用），因为 `publish` 接收的是消息对象的引用，而 `msg` 是 `std::shared_ptr`。

---

# 十七、成员变量

```cpp
cv::VideoCapture cap_;
std::string video_path_;
std::string topic_name_;
int fps_setting_;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::TimerBase::SharedPtr timer_;
```

---

### `cv::VideoCapture cap_`

视频读取器对象。必须作为成员变量保存，因为每次回调都要继续从同一个视频流中读下一帧。

---

### `image_pub_`

Publisher 对象。必须长期存活，否则节点就不能持续发消息。

---

### `timer_`

定时器对象。这个也必须是成员变量，不然构造函数一结束定时器就没了，回调根本不会触发。

这是一个 ROS2 新手常踩的坑：

> **如果把 timer 写成局部变量，它会随构造函数结束而销毁，节点看似启动了，实际上什么都不做。**

---

# 十八、`main` 函数

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

经典四步法：

| 步骤 | 函数 | 作用 |
|------|------|------|
| 1 | `rclcpp::init(argc, argv)` | 初始化 ROS2 通信框架，解析命令行参数 |
| 2 | `make_shared<VideoNode>()` | 创建节点，执行构造函数（读参数、开视频、建 pub、建 timer） |
| 3 | `rclcpp::spin(node)` | 进入事件循环，让 timer 一次次触发 |
| 4 | `rclcpp::shutdown()` | 清理资源 |

---

### 为什么 `spin` 能让 timer 工作？

`rclcpp::spin` 内部是一个循环，不断检查节点上的事件：

* 有定时器到期了？→ 执行 timer_callback
* 有订阅消息到了？→ 执行 subscription callback（这个节点没有订阅）
* 有服务请求到了？→ 执行 service callback（这个节点没有服务）

所以虽然你代码里没有显式写 `while` 循环，但 `spin` 就是它的幕后推手。

---

# 十九、把整份 `video_node.cpp` 总结成一句话

> **把本地视频文件包装成一个按固定帧率发布图像的 ROS2 话题源。**

也就是把你原本"自己在程序里读视频"的做法，拆成了一个**独立、可替换、可复用的节点**。

以后你想换视频源，不需要改 `detect_node` 的任何代码，只需要：

```bash
ros2 run tensorrt_detect video_node --ros-args -p video_path:=/new/video.mp4
```

或者直接在 launch 文件里改参数。

这就是 ROS2 **话题解耦**的威力。

---

# 二十、从这份代码里学到的"源节点模板"

以后你写任何"从某处取数据，转成 ROS 消息发出去"的节点，都可以套这个模板：

## 源节点五步法

1. **声明输入参数**：文件路径、设备 ID、topic 名、帧率等
2. **初始化输入源**：打开文件、初始化设备、连接数据库
3. **创建 Publisher**：把数据要发到哪里去声明好
4. **用 Timer 或线程持续取数据**：模拟固定频率，或阻塞读取
5. **转换并发布**：把原始数据转成 ROS 消息，填好 header，发出去

以后你写 `camera_node` 时，结构会很像，只不过：

* `cv::VideoCapture` → 工业相机 SDK（如 Hikrobot、Daheng）
* `fps` 参数 → 相机的真实采集帧率
* `this->now()` 打戳 → 尽可能用相机硬件触发的时间戳（更精确）

但外壳几乎一样。

---

# 二十一、ROS2 时间系统补充

`video_node` 里涉及两种时间概念，值得深入理解：

## 1. Wall Time（挂钟时间）

就是你手腕上手表的时间，真实世界流逝的时间。

`create_wall_timer` 依据的就是它。无论 ROS2 仿真时间怎么变，wall timer 都按真实节奏走。

## 2. ROS Time（ROS 时间）

由 ROS2 时钟系统维护的时间。默认情况下它和 Wall Time 一致，但在仿真环境中（如 Gazebo），它可能被 `/clock` 话题控制，可以暂停、加速、减速。

`this->now()` 默认返回的是 ROS Time。

## 3. 时间戳的重要性

```cpp
msg->header.stamp = this->now();
```

这行代码看似简单，但它是 ROS2 **分布式时间同步**的基石。

假设以后你的系统扩展成多摄像头：`video_node_1` 和 `video_node_2` 同时发布到各自的 topic。下游做图像融合时，就可以根据 `header.stamp` 找到同一时刻的两帧图像，实现时间对齐。

没有这个时间戳，分布式系统中的数据就只是一堆无序的像素块。
