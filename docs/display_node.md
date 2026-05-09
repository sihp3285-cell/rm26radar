# `display_node.cpp` 逐行讲解

这份文件是**可视化显示节点**，在你当前 ROS2 视觉链路里的位置是：

```text
video_node
   ↓  /image_raw
detect_node
   ↓  /detected_image, /armor_detections
pose_node
   ↓  /world_targets
map_node
   ↓  /map_image, /radar_map
display_node  ← 订阅 /detected_image 和 /map_image
```

也就是说，它是整个流水线的**末端消费者**，负责把算法结果以可视化的方式呈现给操作员。

但它远不止"收图显示"这么简单。这份代码里藏着一个 ROS2 开发中非常关键的课题：

> **当 ROS2 回调线程和 GUI 主线程冲突时，怎么优雅地处理？**

---

# 一、头文件部分

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
```

---

### `#include <rclcpp/rclcpp.hpp>`

ROS2 C++ 节点的基础头文件。你用它来定义节点类、创建订阅者、打日志等。

---

### `#include <sensor_msgs/msg/image.hpp>`

说明这个节点要处理图像消息。它订阅的不是普通文本或数字，而是 `sensor_msgs::msg::Image` 类型的话题。

---

### `#include <cv_bridge/cv_bridge.hpp>`

这个节点收到的是 ROS2 图像消息，但最终要在 OpenCV 窗口里显示，所以必须做转换：

```text
sensor_msgs::msg::Image → cv::Mat
```

`cv_bridge` 就是干这件事的桥。

---

### `#include <opencv2/opencv.hpp>`

因为节点内部要用 OpenCV 做：

* 图像缩放 (`cv::resize`)
* 图像拼接 (`cv::hconcat`)
* 窗口创建与显示 (`cv::namedWindow`, `cv::imshow`, `cv::waitKey`)

---

### `#include <mutex>`

这行非常关键，值得单独讲。

在 ROS2 中，**订阅者的回调函数默认可能在独立的线程中被调用**（特别是当使用多线程 Executor 时）。而你的显示逻辑 `show_latest()` 是在主循环线程里运行的。

这意味着：

> 同一个 `latest_frame_` 和 `latest_map_frame_` 变量，可能同时被**回调线程写**和**主线程读**。

如果不加保护，这就是经典的**数据竞争（Data Race）**，会导致程序崩溃或图像撕裂。

`std::mutex` 就是用来解决这个问题的同步原语。

---

# 二、定义节点类

```cpp
class DisplayNode : public rclcpp::Node
{
```

和 `detect_node`、`video_node` 一样，DisplayNode 也继承自 `rclcpp::Node`。

这再次印证了 ROS2 节点的固定起手式：

> 所有节点类都继承自 `rclcpp::Node`。

---

# 三、构造函数开始

```cpp
public:
    DisplayNode() : Node("display_node")
```

这行说明：

* 构造函数名为 `DisplayNode()`
* 初始化父类时把节点名注册为 `"display_node"`

以后你在日志里看到 `[display_node]`，就是这行决定的。

---

# 四、声明参数

```cpp
this->declare_parameter<std::string>("topic", "/detected_image");
this->declare_parameter<std::string>("window_name", "Video & Radar");
this->declare_parameter<int>("window_width", 1920);
this->declare_parameter<int>("window_height", 720);

this->declare_parameter<std::string>("map_topic", "/map_image");
```

---

### 第一行：`topic`

默认订阅 `/detected_image`，也就是 `detect_node` 发布的带检测框的图像。

---

### 第二行：`window_name`

OpenCV 窗口的标题名，默认 `"Video & Radar"`。

---

### 第三、四行：`window_width` / `window_height`

窗口初始尺寸。默认 1920×720，这是一个很宽的比例，因为后面要水平拼接两张图。

---

### 第五行：`map_topic`

默认订阅 `/map_image`，也就是 `map_node` 发布的小地图图像。

所以这五个参数合起来，已经把这个节点的职责说清楚了：

```text
收检测图 + 收地图图 → 拼接 → 在一个宽窗口里显示
```

---

# 五、读取参数

```cpp
topic_ = this->get_parameter("topic").as_string();
window_name_ = this->get_parameter("window_name").as_string();
int win_w = this->get_parameter("window_width").as_int();
int win_h = this->get_parameter("window_height").as_int();

map_topic_ = this->get_parameter("map_topic").as_string();
```

`declare_parameter` 只是注册，`get_parameter` 才是真正取值。

注意 `win_w` 和 `win_h` 是局部变量，因为只用在构造函数里初始化窗口；而 `topic_` 和 `map_topic_` 是成员变量，因为后面创建订阅者时还要用。

---

# 六、创建 OpenCV 窗口

```cpp
cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
cv::resizeWindow(window_name_, win_w, win_h);
```

---

### `cv::namedWindow(...)`

创建一个可缩放（`WINDOW_NORMAL`）的 OpenCV 窗口。

注意这个窗口是在**构造函数里**创建的。也就是说，节点一启动，窗口就已经准备好了，哪怕还没有收到任何图像。

---

### `cv::resizeWindow(...)`

把窗口尺寸设成参数指定的大小。

---

# 七、创建两个订阅者

```cpp
sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    topic_, rclcpp::QoS(1),
    std::bind(&DisplayNode::image_callback, this, std::placeholders::_1));

map_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    map_topic_, rclcpp::QoS(1),
    std::bind(&DisplayNode::map_callback, this, std::placeholders::_1));
```

这个节点同时订阅两个话题，这是 ROS2 多输入节点的典型写法。

---

### 第一个订阅者 `sub_`

* 类型：`sensor_msgs::msg::Image`
* 话题：`topic_`（默认 `/detected_image`）
* QoS：`rclcpp::QoS(1)` —— 只保留最新 1 帧，旧帧直接丢弃
* 回调：`image_callback`

负责接收检测后的图像。

---

### 第二个订阅者 `map_sub_`

* 类型：`sensor_msgs::msg::Image`
* 话题：`map_topic_`（默认 `/map_image`）
* QoS：`rclcpp::QoS(1)` —— 只保留最新 1 帧，旧帧直接丢弃
* 回调：`map_callback`

负责接收小地图图像。

---

### 为什么用 `rclcpp::QoS(1)`？

**（性能优化关键改动）**

旧版队列深度是 10。对于实时图像显示来说，这是**反作用**的：

> 显示节点只需要看**最新一帧**，积压的旧帧毫无意义。

如果 `detect_node` 发布 30fps，但 `display_node` 因为渲染慢只能处理 20fps，队列深度 10 意味着最多可以积压约 300ms 的旧帧。`spin_some` 会一口气消费这些积压帧，`latest_frame_` 被连续覆盖多次，前几次的拷贝全是**无用功**。最终显示的是延迟了好几帧的旧图像，造成"视频越来越滞后，然后突然加速跳帧"的现象。

改成 `rclcpp::QoS(1)` 后：

* **深度 1**：subscriber 队列只保留最新 1 帧，旧帧直接被 ROS2 丢弃
* 配合整条图像链路（`video_node` → `detect_node` → `display_node`）统一使用 KeepLast(1)，从根源上消除了图像链路的队列积压

---

### 为什么有两个独立的回调？

因为这两个话题的发布频率、 publishers 可能完全不同。

在 ROS2 中，**每个订阅者独立维护自己的回调队列**。如果只用一个大消息把两张图打包在一起，反而会增加耦合。现在这样分开订阅，是更灵活、更符合 ROS2 "话题即接口" 哲学的做法。

---

# 八、初始化完成日志

```cpp
RCLCPP_INFO(this->get_logger(),
    "显示节点启动 | 订阅主图像: %s | 订阅地图: %s | 按 Q/ESC 退出",
    topic_.c_str(), map_topic_.c_str());
```

打印节点状态，方便调试时确认参数是否生效。

---

# 九、析构函数

```cpp
~DisplayNode()
{
    cv::destroyWindow(window_name_);
}
```

这行非常重要，体现了 RAII（资源获取即初始化）原则。

当节点被销毁时（比如 `Ctrl+C` 退出），OpenCV 窗口也会被自动关闭。如果不写这个析构函数，窗口可能会残留在桌面上，变成"幽灵窗口"。

---

# 十、`show_latest()`：供主循环调用的显示函数

```cpp
bool show_latest()
```

这个函数不是回调，而是**主动轮询式**的。它由主循环反复调用，负责：

1. 拿到最新的两帧图像
2. 拼接
3. 显示
4. 检测键盘事件
5. 返回是否继续运行

---

## 10.1 定义局部图像变量

```cpp
cv::Mat frame;
cv::Mat map_frame;
```

这里用局部变量而不是直接操作成员变量，是为了**缩小临界区**。

---

## 10.2 加锁读取最新帧

```cpp
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_frame_.empty()) {
        frame = latest_frame_.clone();
    }
    if (!latest_map_frame_.empty()) {
        map_frame = latest_map_frame_.clone();
    }
}
```

这是这份代码里最值得学习的一段。

---

### `std::lock_guard<std::mutex> lock(mutex_);`

这是一种**RAII 风格的锁**。它的语义是：

> 构造时加锁，析构时自动解锁。

因为 `lock_guard` 是一个局部对象，当代码块 `{}` 结束时，它会自动销毁，锁也就释放了。

这比手动 `mutex_.lock()` / `mutex_.unlock()` 更安全，不会因为异常或提前 return 而漏解锁。

---

### 为什么用 `.clone()`？

```cpp
frame = latest_frame_.clone();
```

如果不 `clone()`，直接 `frame = latest_frame_`，那 `frame` 和 `latest_frame_` 会共享同一块内存。

问题在于：`latest_frame_` 在锁释放后，随时可能被回调线程覆盖。如果主线程正在用 `frame` 做 `cv::resize` 或 `hconcat`，回调线程同时往 `latest_frame_` 里写新数据，就会造成数据竞争。

`.clone()` 做了**深拷贝**，把数据复制到 `frame` 自己的内存里。这样锁一释放，`frame` 就独立了，回调线程怎么更新 `latest_frame_` 都不影响主线程。

> 这是一个非常重要的多线程图像处理技巧：**在锁里只做拷贝，在锁外做处理。**

---

## 10.3 等待两路图像都就绪

```cpp
if (frame.empty() && map_frame.empty()) {
    cv::waitKey(1);
    return true;
}
```

如果两路图像都还没收到，就调用 `cv::waitKey(1)` 让 OpenCV 处理一下窗口事件（比如响应鼠标、重绘），然后返回 `true` 表示继续运行。

注意这里返回 `true` 而不是退出，是因为 ROS2 节点启动有先后顺序，`display_node` 可能先于 `detect_node` 或 `map_node` 启动，需要耐心等待。

---

## 10.4 图像拼接逻辑

```cpp
cv::Mat combined;
if (!frame.empty() && !map_frame.empty()) {
    int targetHeight = frame.rows;
    int targetWidth = static_cast<int>(map_frame.cols *
                                       (static_cast<float>(targetHeight) / map_frame.rows));
    cv::Mat resized_map;
    cv::resize(map_frame, resized_map, cv::Size(targetWidth, targetHeight));
    cv::hconcat(std::vector<cv::Mat>{frame, resized_map}, combined);
} else if (!frame.empty()) {
    combined = frame.clone();
} else {
    combined = map_frame.clone();
}
```

---

### 两帧都有时

把小地图缩放到和主图像相同的高度，保持宽高比，然后水平拼接。

这对应了窗口名 `"Video & Radar"` 的设计意图：左边是视频，右边是雷达小地图。

---

### 只有一帧时

退化成只显示有数据的那一路。这提高了节点的**鲁棒性**：即使 `map_node` 没启动，`display_node` 仍然可以显示检测图。

---

## 10.5 显示与键盘检测

```cpp
cv::imshow(window_name_, combined);
int key = cv::waitKey(1);

if (key == 'q' || key == 'Q' || key == 27) {
    RCLCPP_INFO(this->get_logger(), "收到退出键，关闭显示...");
    return false;
}
return true;
```

---

### `cv::imshow(...)`

把拼接后的图像显示到窗口上。

---

### `cv::waitKey(1)`

等待 1 毫秒，检测是否有键盘按下。

**这是 OpenCV 窗口能够正常响应的命脉。** 如果不调用 `waitKey`，窗口会卡住、不刷新、甚至显示"未响应"。

注意这里只等待 1 毫秒，而不是阻塞等待。因为主循环还需要频繁地去 `spin_some` 处理 ROS2 回调。

---

### 退出检测

如果用户按了 `q`、`Q` 或 `ESC`（ASCII 27），返回 `false`，告诉主循环该退出了。

---

# 十一、`image_callback`：主图像回调

```cpp
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
```

每次 `detect_node` 发布新图像时，这个函数被调用。

---

## 11.1 图像转换

```cpp
try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
```

把 ROS2 图像消息转成 OpenCV 格式。用 `try` 包起来是因为编码可能不对。

---

## 11.2 加锁保存图像

```cpp
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_ = cv_ptr->image.clone();
    }
```

`cv_bridge::toCvCopy(msg, "bgr8")` 已经在内部做了**一次深拷贝**，把 ROS 消息的数据复制到了 `cv_ptr->image` 的新内存里。但这里仍然调用 `.clone()`，确保 `latest_frame_` 拥有完全独立的内存副本，避免后续 OpenCV 操作可能对底层缓冲区的意外修改。

`mutex_` 仍然是必须的：回调线程在写 `latest_frame_`，主线程在读 `latest_frame_`，需要保证同一时刻只有一个线程访问。

---

## 11.3 节流日志

```cpp
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        10000,
        "收到图像: %d x %d",
        latest_frame_.cols,
        latest_frame_.rows
    );
```

最多每 10 秒打印一次收到的图像尺寸。这对于调试图像分辨率是否正确非常有用，同时又不会刷屏。

---

## 11.4 异常处理

```cpp
catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
}
```

如果图像编码不对，记录错误但不崩节点。

---

# 十二、`map_callback`：地图图像回调

```cpp
void map_callback(const sensor_msgs::msg::Image::SharedPtr msg)
```

逻辑和 `image_callback` 几乎完全一样，只是操作的是 `latest_map_frame_`。

这再次体现了**两个订阅者独立工作**的设计：它们各自维护自己的数据，互不干扰，只在 `show_latest()` 里做最终融合。

---

# 十三、成员变量

```cpp
std::string topic_;
std::string window_name_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
cv::Mat latest_frame_;

std::string map_topic_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr map_sub_;
cv::Mat latest_map_frame_;

std::mutex mutex_;
```

---

### `latest_frame_` / `latest_map_frame_`

这两张图是**回调线程写、主线程读**的共享数据，所以必须用 `mutex_` 保护。

---

### `mutex_`

`std::mutex` 是 C++11 标准库提供的互斥锁。

在 ROS2 中，如果你的节点同时满足以下两个条件，就必须考虑加锁：

1. 有回调函数在写某个数据
2. 有另一个线程（或另一个回调）在读同一个数据

`display_node` 正是这种情况的典型代表。

---

# 十四、`main` 函数：这里和别的节点不一样

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(10));
        if (!node->show_latest()) break;
    }

    rclcpp::shutdown();
    return 0;
}
```

这段 `main` 和 `detect_node`、`video_node` 的 `main` **完全不同**，是整个文件最精华的部分。

---

### 为什么不用 `rclcpp::spin(node)`？

如果写 `rclcpp::spin(node)`，ROS2 会进入一个内部循环，不断处理回调。但问题是：

> `cv::imshow` 和 `cv::waitKey` 必须在**主线程**中定期调用，否则窗口会卡死。

如果 `spin` 占住了主线程，你就没有机会去调 `show_latest()` 了。

---

### `SingleThreadedExecutor`

这是 ROS2 提供的一种**单线程执行器**。它的特点是：

* 所有回调都在调用 `spin_some` 的那个线程里执行
* 不会偷偷开新线程跑回调

这在这个场景下是有好处的，因为：

> 虽然只有一个回调线程，但 `spin_some` 和 `show_latest` 是交替执行的，我们仍然需要用 `mutex` 保护数据。不过至少回调不会从"不知道哪个线程"突然冒出来。

---

### `executor.add_node(node)`

把节点注册到执行器上，这样执行器才知道要处理这个节点的回调。

---

### `while (rclcpp::ok())`

主循环条件。`rclcpp::ok()` 会在以下情况返回 `false`：

* 收到 `SIGINT`（`Ctrl+C`）
* 调用了 `rclcpp::shutdown()`

---

### `executor.spin_some(std::chrono::milliseconds(10))`

这是整个设计的关键。

`spin_some` 的意思是：

> 在最多 10 毫秒的时间里，处理当前队列里已有的回调，然后立刻返回。

它和 `spin` 的区别：

| 函数 | 行为 |
|------|------|
| `spin` | 阻塞，永远循环处理回调，不返回 |
| `spin_some` | 非阻塞，处理一批已有的回调，然后返回 |

正因为 `spin_some` 会**返回**，主循环才能继续执行下一行：

```cpp
if (!node->show_latest()) break;
```

---

### 这个循环的本质

```text
处理 ROS2 回调（收图）→ 显示图像 → 处理 ROS2 回调 → 显示图像 → ...
```

两个任务交替进行，谁都不会饿死。

* 如果图像来得快，`spin_some` 每次都能处理到新回调
* 如果用户按了退出键，`show_latest()` 返回 `false`，循环结束

---

# 十五、把整份 `display_node.cpp` 总结成一句话

> **用多线程安全的方式，把两路 ROS2 图像话题拼接显示到一个 OpenCV 窗口中。**

---

# 十六、你真正该从这份代码里学到的

## 1. ROS2 + GUI 的经典模式

当节点需要同时做两件事：

* 响应 ROS2 回调（事件驱动）
* 维护 GUI（需要主线程定期刷新）

标准解法就是：

```text
SingleThreadedExecutor + spin_some 循环 + 主循环里做 GUI
```

## 2. 多线程图像共享的固定套路

```cpp
// 写端（回调）
{
    std::lock_guard<std::mutex> lock(mutex_);
    buffer = new_frame.clone();
}

// 读端（主循环）
{
    std::lock_guard<std::mutex> lock(mutex_);
    local_copy = buffer.clone();
}
// 在锁外处理 local_copy
```

## 3. 节点作为末端消费者的定位

`display_node` 不发布任何话题，只订阅。这种"纯消费者"节点在 ROS2 中非常常见，比如：

* 可视化工具（RViz、你的 display_node）
* 数据记录器（rosbag）
* 监控面板

它们的存在不会破坏上游节点的运行，随时可以启动或停止，这正是 ROS2 分布式架构的优雅之处。

---

# 十七、完整数据流回顾

```text
video_node ──/image_raw──→ detect_node ──/detected_image──→ display_node
                                │
                                └──/armor_detections──→ pose_node ──/world_targets──→ map_node ──/map_image──→ display_node
```

`display_node` 就像整个系统的"仪表盘"，把所有分散处理的结果汇集到人类肉眼可见的地方。
