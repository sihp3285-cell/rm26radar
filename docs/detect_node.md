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
```

这三行向 ROS2 参数服务器注册参数。

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

# 五、读取参数

```cpp
std::string config_dir = this->get_parameter("config_dir").as_string();
input_topic_ = this->get_parameter("input_topic").as_string();
output_topic_ = this->get_parameter("output_topic").as_string();
```

`declare_parameter` 只是注册，`get_parameter` 才是真正取值。

`config_dir` 是局部变量，因为只在构造函数里初始化 `Config` 时用一次；`input_topic_` 和 `output_topic_` 是成员变量，因为后面创建 pub/sub 时还要用。

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
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
armor_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::DetectionArray>("/armor_detections", 10);
```

这是新版相比旧版最重要的变化：**从 1 个 publisher 变成 2 个 publisher**。

---

### `image_pub_`

* 类型：`sensor_msgs::msg::Image`
* 话题：`/detected_image`
* 队列深度：10

发布带检测框、FPS 文字的可视化图像，供 `display_node` 显示给人类看。

---

### `armor_pub_`

* 类型：`tensorrt_detect_msgs::msg::DetectionArray`
* 话题：`/armor_detections`
* 队列深度：10

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
    input_topic_, 10,
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
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;
```

---

### `cv_bridge::toCvCopy(msg, "bgr8")`

把 ROS2 图像消息按 `bgr8` 编码转成 OpenCV 可用的 `cv::Mat`。

`"bgr8"` 是 OpenCV 最常用的彩色图像格式：蓝-绿-红三通道，每通道 8 位。

---

### `cv::Mat frame = cv_ptr->image`

从 `cv_ptr` 中取出真正的 `cv::Mat`。从这行开始，你就回到了熟悉的纯 OpenCV 世界。

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

# 十四、绘制检测结果

```cpp
drawDetect(frame, results, cfg_->model.classNames);
```

在原始图像上画框、画类别标签。`cfg_->model.classNames` 把数字类别 ID 映射成人类可读的字符串（如 "R1", "R2", "S" 等）。

注意这里画的是 `frame`，也就是原始分辨率图像。

---

# 十五、图像缩放（新版新增）

```cpp
cv::Mat resize_frame = frame;
int target_width = 1280;
if(frame.cols > target_width) {
     double scale = static_cast<double>(target_width) / frame.cols;
     int target_height = static_cast<int>(frame.rows * scale);
     cv::resize(frame, resize_frame, cv::Size(target_width, target_height));
}
```

这是新版增加的重要优化。

---

### 为什么要缩放？

原始视频可能是 1920×1080 甚至 4K。如果直接发布原图：

* 话题带宽占用大
* `display_node` 处理慢
* 网络传输延迟高

所以这里做了一个**上限控制**：宽度超过 1280 时，等比例缩放到 1280 宽。

---

### 为什么只在发布可视化图时缩，而不在检测时缩？

仔细看代码：`drawDetect` 是在缩放**之前**调用的，检测用的也是原始 `frame`。

```text
检测算法 ← 原始 frame（保证精度）
可视化图 ← resize_frame（保证传输效率）
```

这体现了**精度与带宽的权衡**：检测需要原图保证精度，但给人看的图不需要那么高的分辨率。

---

# 十六、FPS 计算

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

# 十七、把 FPS 画到图像上

```cpp
cv::putText(resize_frame, cv::format("FPS: %.1f", fps_),
            cv::Point(20, 40),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0, cv::Scalar(0, 255, 0), 2);
```

注意这里是画在 `resize_frame` 上，因为最终发布的是缩放后的图。

---

# 十八、发布可视化图像

```cpp
std_msgs::msg::Header header = msg->header;
header.frame_id = "detected_frame";

auto out_msg = cv_bridge::CvImage(header, "bgr8", resize_frame).toImageMsg();
image_pub_->publish(*out_msg);
```

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
auto out_msg = cv_bridge::CvImage(header, "bgr8", resize_frame).toImageMsg();
image_pub_->publish(*out_msg);
```

`CvImage` 构造时传入 header、编码、图像数据，然后 `toImageMsg()` 生成 ROS2 图像消息，最后通过 publisher 发出去。

---

# 十九、构建并发布结构化检测消息（新版核心）

```cpp
auto armor_msg = std::make_shared<tensorrt_detect_msgs::msg::DetectionArray>();
armor_msg->header = msg->header;   // 复用图像时间戳，方便下游同步
armor_msg->header.frame_id = "detection";
```

---

### 为什么要复用 `msg->header`？

这是 ROS2 **消息链时间同步**的关键技巧。

`pose_node` 收到 `DetectionArray` 后，可以通过 `header.stamp` 知道这批检测对应的是哪一帧图像。如果以后要做多传感器时间对齐（比如和雷达、IMU 融合），这个共同的时间戳就是纽带。

---

### 填充 DetectionBox 数组

```cpp
for (const auto& res : results) {
    tensorrt_detect_msgs::msg::DetectionBox box;
    box.idx         = res.idx;
    box.confidence  = res.confidence;
    box.x           = res.box.x;
    box.y           = res.box.y;
    box.width       = res.box.width;
    box.height      = res.box.height;
    box.armor_color = res.armorColor;
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

### `DetectionBox` 包含的字段解读

| 字段 | 含义 |
|------|------|
| `idx` | 类别 ID |
| `confidence` | 置信度 |
| `x/y/width/height` | 装甲板检测框 |
| `armor_color` | 装甲颜色/队伍 ID |
| `car_x/car_y/car_width/car_height` | 车辆整体检测框 |
| `world_x/world_y` | 世界坐标（如果 pipeline 已计算） |
| `fps` | 当前帧率 |

字段设计非常完整，下游节点（`pose_node`）可以按需取用。

---

### 发布

```cpp
armor_pub_->publish(*armor_msg);
```

把结构化检测数据发到 `/armor_detections`。`pose_node` 订阅这个话题后，就能拿到每个目标的像素坐标，进而解算出世界坐标。

---

# 二十、节流日志

```cpp
RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    10000,
    "检测到 %zu 个目标，fps: %.1f，发布了 DetectionArray 消息",
    results.size(), fps_);
```

`RCLCPP_INFO_THROTTLE` 是**节流日志**，最多每 10000 毫秒（10 秒）打印一次。

日志内容包含：

* 检测到的目标数量
* 当前平滑 FPS
* 提示已发布 DetectionArray

---

# 二十一、异常处理

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

# 二十二、成员变量

```cpp
std::unique_ptr<Config> cfg_;
std::unique_ptr<DetectPipeline> pipeline_;

std::string input_topic_;
std::string output_topic_;

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::Publisher<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_pub_;
std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
double fps_ = 0.0;
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

# 二十三、`main` 函数

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

# 二十四、完整数据流回顾

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

# 二十五、从这份代码里学到的设计要点

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
