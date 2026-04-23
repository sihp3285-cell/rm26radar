我们接着把 **`video_node.cpp` 也逐行讲透**。

这份文件在你当前三节点链路里的位置是：

```text id="s76gvf"
video_node
   ↓  /image_raw
detect_node
   ↓  /detected_image
display_node
```

也就是说，它不是算法节点，而是**输入源节点**。它负责把本地视频文件一帧一帧读出来，包装成 ROS2 图像消息，再发到 `/image_raw` 上。

---

# 一、头文件部分

```cpp id="se1pq2"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
```

这几行和 `detect_node` 很像，但职责不完全一样。

---

### `#include <rclcpp/rclcpp.hpp>`

这是 ROS2 节点开发基础头文件。
你后面要用它来：

* 定义节点类
* 打日志
* 创建 publisher
* 创建 timer
* `spin`

没有它，就不能写 ROS2 节点。

---

### `#include <sensor_msgs/msg/image.hpp>`

这表明你这个节点要发布的是图像消息。

也就是说，`video_node` 的输出不是普通变量，而是 ROS2 里的：

```text id="td4jgv"
sensor_msgs::msg::Image
```

---

### `#include <cv_bridge/cv_bridge.hpp>`

虽然这个节点没有订阅图像，但它依然需要 `cv_bridge`。
原因是：

* OpenCV 读出来的是 `cv::Mat`
* ROS2 话题上传输的是 `sensor_msgs::msg::Image`

所以这里要做的是：

```text id="vq7gaf"
cv::Mat → ROS2 Image
```

---

### `#include <std_msgs/msg/header.hpp>`

因为你发布图像时，也会给消息加 `header`，里面有：

* 时间戳
* `frame_id`

所以这个头文件也需要。

---

### `#include <opencv2/opencv.hpp>`

因为本地视频读取靠的是 OpenCV 的：

* `cv::VideoCapture`
* `cv::Mat`

这行当然必需。

---

# 二、定义节点类

```cpp id="hbimw7"
class VideoNode : public rclcpp::Node
{
```

这一行说明：

> 你定义了一个叫 `VideoNode` 的 ROS2 节点类。

以后你写任何 ROS2 节点，最常见的起手式都差不多是这样。

---

# 三、构造函数开始

```cpp id="fmsn3m"
public:
    VideoNode() : Node("video_node")
```

这行的意思是：

* 构造函数名叫 `VideoNode()`
* 同时初始化父类 `Node`
* 这个节点在 ROS2 系统里的名字叫 `"video_node"`

所以以后你在日志里会看到 `[video_node]`。

---

# 四、声明参数

```cpp id="9oa0ct"
this->declare_parameter<std::string>("video_path", "/home/delphine/rm/car_project/test/005.mp4");
this->declare_parameter<std::string>("topic_name", "/image_raw");
this->declare_parameter<int>("fps", 30);
```

这三行是参数注册。

---

### 第一行：`video_path`

告诉 ROS2：

> 这个节点有一个字符串参数，叫 `video_path`，默认值是视频文件路径。

也就是说，视频源不是写死在逻辑里的，而是参数化的。

---

### 第二行：`topic_name`

告诉 ROS2：

> 这个节点发布图像的话题默认叫 `/image_raw`。

这和后面 `detect_node` 的输入话题默认值正好对上。

---

### 第三行：`fps`

告诉 ROS2：

> 这个节点还有一个整数参数，表示你希望按多少帧率发布视频。

默认值是 30。

这里很有意义，因为它说明这个节点不只是“尽力读帧”，而是试图按固定节奏发图像。

---

# 五、读取参数

```cpp id="41f9kf"
video_path_ = this->get_parameter("video_path").as_string();
topic_name_ = this->get_parameter("topic_name").as_string();
fps_setting_ = this->get_parameter("fps").as_int();
```

前面是注册参数，这里才是真正把参数值取出来。

---

### 第一行

把 `video_path` 的值读出来，保存到成员变量 `video_path_`。

---

### 第二行

把 `topic_name` 的值读出来，保存到成员变量 `topic_name_`。

---

### 第三行

把 `fps` 的值读出来，保存到成员变量 `fps_setting_`。

这里为什么都存成成员变量？
因为这些值会在整个节点生命周期里反复被用到，不只是构造函数里临时使用。

---

# 六、打印初始化信息

```cpp id="ll0kbl"
RCLCPP_INFO(this->get_logger(), "视频路径: %s", video_path_.c_str());
RCLCPP_INFO(this->get_logger(), "发布话题: %s", topic_name_.c_str());
RCLCPP_INFO(this->get_logger(), "设定帧率: %d", fps_setting_);
```

这三行是初始化日志。

它们会告诉你：

* 视频路径是什么
* 要发到哪个话题
* 目标帧率是多少

这非常适合排查 launch 文件、参数文件、命令行传参是否生效。

---

# 七、打开视频

```cpp id="fceli0"
cap_.open(video_path_);
if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_path_.c_str());
    rclcpp::shutdown();
    return;
}
```

这是 `video_node` 的核心初始化之一：准备输入源。

---

### `cap_.open(video_path_);`

这里 `cap_` 是一个 `cv::VideoCapture` 对象。
这行让它尝试打开视频文件。

---

### `if (!cap_.isOpened())`

检查打开是否成功。

如果失败，通常可能是：

* 路径写错
* 文件不存在
* 编码不支持

---

### `RCLCPP_ERROR(...)`

打印错误日志。

---

### `rclcpp::shutdown();`

因为输入源都打不开，节点就没必要继续运行了，所以直接请求关闭 ROS2。

---

### `return;`

结束构造函数，避免后面继续执行。

这体现了一个很好的工程习惯：

> 初始化失败时尽早退出，而不是带着错误继续跑。

---

# 八、创建 publisher

```cpp id="0p0g07"
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
```

这行创建图像发布者。

它的含义是：

* 类型：`sensor_msgs::msg::Image`
* 话题：`topic_name_`
* 队列深度：10

所以这行本质上是在说：

> `video_node` 具备了把视频帧发到 ROS2 网络中的能力。

---

# 九、根据 FPS 计算定时器周期

```cpp id="zax2zq"
int interval_ms = 1000 / std::max(fps_setting_, 1);
```

这行是一个很典型的小计算。

逻辑是：

* 如果你想 30fps
* 那每帧间隔就是大约 `1000 / 30 ≈ 33ms`

所以这个变量 `interval_ms` 表示：

> 定时器每隔多少毫秒触发一次

为什么用了 `std::max(fps_setting_, 1)`？
因为防止用户把 fps 设成 0，导致除零错误。

---

# 十、创建定时器

```cpp id="mev5za"
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(interval_ms),
    std::bind(&VideoNode::timer_callback, this));
```

这是 `video_node` 的运行驱动核心。

---

### 第一行

调用 `create_wall_timer(...)` 创建一个基于系统时间的定时器。

---

### 第二行

```cpp id="u3y8lw"
std::chrono::milliseconds(interval_ms)
```

表示这个定时器的触发周期。

比如如果 `interval_ms = 33`，那就大约每 33ms 触发一次。

---

### 第三行

```cpp id="s5mjlwm"
std::bind(&VideoNode::timer_callback, this)
```

意思是：

> 每次定时器到时间，就调用当前这个 `VideoNode` 对象的 `timer_callback()` 函数。

所以这行建立了一个循环机制：

```text id="vokc4m"
每隔 interval_ms 毫秒
→ 调一次 timer_callback
→ 读一帧视频
→ 发布一帧图像
```

这就是 `video_node` 的“主循环”。

---

# 十一、初始化完成日志

```cpp id="u1zcb8"
RCLCPP_INFO(this->get_logger(), "VideoNode 初始化完成，开始发布视频帧");
```

这行说明构造函数已经完成了所有准备工作：

* 参数读完了
* 视频打开成功了
* publisher 建好了
* timer 建好了

从这里开始，节点就会进入“持续发帧”的运行状态。

---

# 十二、进入 `private`，看真正干活的定时器回调

```cpp id="7wifyu"
private:
    void timer_callback()
```

这个函数就是整个 `video_node` 的真正执行体。

每次定时器一触发，它就执行一次。
你可以把它理解成“视频节点版本的主循环”。

---

# 十三、先定义当前帧变量

```cpp id="jxcsj8"
cv::Mat frame;
```

这行创建一个 OpenCV 图像对象，用来接收当前这一帧视频画面。

---

# 十四、读取一帧视频

```cpp id="15wa5s"
if (!cap_.read(frame)) {
    RCLCPP_WARN(this->get_logger(), "视频播放结束，重新回到开头");
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    return;
}
```

这几行是视频循环播放逻辑。

---

### `cap_.read(frame)`

从视频里读取一帧到 `frame` 中。

如果成功，`frame` 里就有图像内容。

---

### `if (!cap_.read(frame))`

如果返回失败，通常说明视频已经读到结尾，或者读取出错。

---

### `RCLCPP_WARN(...)`

打印警告日志，告诉你视频播完了。

---

### `cap_.set(cv::CAP_PROP_POS_FRAMES, 0);`

把当前视频位置跳回第 0 帧。

所以你的视频节点不是播完就停，而是会从头循环播放。

---

### `return;`

这次回调先结束，等下一次定时器再重新开始读。

---

# 十五、把 OpenCV 图像转成 ROS2 图像消息

```cpp id="hr2l9l"
auto msg = cv_bridge::CvImage(
    std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
```

这行非常关键。

意思是：

* 用一份新的 `Header`
* 指定编码格式 `"bgr8"`
* 把 `frame` 这张 `cv::Mat`
* 包装成 ROS2 的图像消息

这就是：

```text id="gwj8a9"
cv::Mat → sensor_msgs::msg::Image
```

---

# 十六、给消息补 header 信息

```cpp id="0di82o"
msg->header.stamp = this->now();
msg->header.frame_id = "video_frame";
```

这两行是在给图像消息加身份信息。

---

### `msg->header.stamp = this->now();`

给这帧图像打上当前 ROS2 时间戳。

这表示：

> 这张图是在这个时刻发布出去的。

以后如果你做时间同步、多节点对齐，这个很重要。

---

### `msg->header.frame_id = "video_frame";`

给图像消息一个坐标系/来源标识。

这里写 `"video_frame"`，语义上表示：

> 这是来自视频源节点的一帧图像。

后面 `detect_node` 收到它时，会继承这个 header 再加工。

---

# 十七、真正发布图像

```cpp id="khdb98"
image_pub_->publish(*msg);
```

这一行就是 `video_node` 的最终输出动作。

它的意义很简单：

> 把这一帧图像送到 ROS2 话题网络中去。

一旦发布成功：

* `detect_node` 就能在 `/image_raw` 上收到它
* 整个视觉链就流动起来了

---

# 十八、成员变量部分

```cpp id="vvmbj7"
cv::VideoCapture cap_;
std::string video_path_;
std::string topic_name_;
int fps_setting_;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::TimerBase::SharedPtr timer_;
```

这些都是 `video_node` 在整个生命周期里要长期保存的状态。

---

### `cv::VideoCapture cap_`

视频读取器对象。
必须作为成员变量保存，因为每次回调都要继续从同一个视频流中读下一帧。

---

### `std::string video_path_`

保存视频路径。

---

### `std::string topic_name_`

保存发布话题名。

---

### `int fps_setting_`

保存设定帧率。

---

### `image_pub_`

保存 publisher 对象。
如果不保存，publisher 会被析构，节点就不能持续发消息了。

---

### `timer_`

保存定时器对象。
这个也必须是成员变量，不然构造函数一结束定时器就没了，回调根本不会触发。

---

# 十九、最后看 `main`

```cpp id="c00jq5"
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

这部分和 `detect_node` 的主函数结构完全一样。

---

### `rclcpp::init(argc, argv);`

初始化 ROS2 运行环境。

---

### `auto node = std::make_shared<VideoNode>();`

创建一个 `VideoNode` 节点对象。

这里会自动执行它的构造函数，也就是：

* 读参数
* 打开视频
* 创建 publisher
* 创建 timer

---

### `rclcpp::spin(node);`

进入 ROS2 事件循环。

在这个文件里，`spin` 主要负责让 timer 一次次触发 `timer_callback()`。

也就是说：

> `spin` 让这个视频节点真正“活起来”。

---

### `rclcpp::shutdown();`

退出时关闭 ROS2。

---

### `return 0;`

程序正常结束。

---

# 二十、把整份 `video_node.cpp` 总结成一句话

这份代码做的事情就是：

> **把本地视频文件包装成一个 ROS2 图像话题源。**

也就是把你原本“自己在程序里读视频”的做法，拆成了一个独立节点。
这样后面的算法节点就不用关心视频文件在哪里，只要订阅 `/image_raw` 就行。

---

# 二十一、你现在应该从这份代码里学到什么

这份代码最重要的不是“怎么读视频”，而是学会下面这个模板：

## 输入源节点模板

1. 声明输入相关参数
2. 初始化输入设备/输入文件
3. 创建 publisher
4. 用 timer 或线程持续取数据
5. 把数据转换成 ROS 消息
6. 发布出去

以后你写 `camera_node` 时，结构会很像，只不过这里的：

* `cv::VideoCapture`

会换成：

* 工业相机 SDK
* 或者你的相机封装类

但外壳几乎一样。

---
