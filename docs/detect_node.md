
---

# 一、文件最上面的头文件

```cpp id="zimq3j"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

#include "ConfigManager.hpp"
#include "utils/include/pipeline.hpp"
#include "utils/include/draw.hpp"
```

我们一行一行看。

### `#include <rclcpp/rclcpp.hpp>`

这是 ROS2 C++ 开发最核心的头文件。
你后面要用到的这些东西几乎都来自它：

* `rclcpp::Node`
* `rclcpp::init`
* `rclcpp::spin`
* `RCLCPP_INFO`
* publisher / subscriber 的创建接口

少了它，这份文件就不是 ROS2 节点了。

---

### `#include <sensor_msgs/msg/image.hpp>`

这行说明你的节点要处理 ROS2 里的“图像消息”。

也就是说，你的话题不是发普通数字，而是发：

```text id="kup3ro"
sensor_msgs::msg::Image
```

这和你现在项目里“图像是主要数据流”完全对应。

---

### `#include <cv_bridge/cv_bridge.hpp>`

这是 ROS 图像消息和 OpenCV 图像之间的桥。
因为你原本算法里处理的是 `cv::Mat`，但话题里传的是 `sensor_msgs::msg::Image`，所以必须有这个桥。它负责：

* 订阅后：`Image -> cv::Mat`
* 发布前：`cv::Mat -> Image`

这行非常关键。没有它，你的原工程几乎没法平滑接进 ROS2。

---

### `#include <std_msgs/msg/header.hpp>`

图像消息里会带头信息 `header`，常见内容有：

* 时间戳 `stamp`
* 坐标系标识 `frame_id`

你后面确实会从输入图像里取 header，再修改 `frame_id`。

---

### `#include <opencv2/opencv.hpp>`

因为你节点内部还要继续做 OpenCV 处理，比如：

* 处理 `cv::Mat`
* 在图上画 FPS
* 调用你原来基于 OpenCV 的绘图函数

所以这行当然需要。

---

### `#include "ConfigManager.hpp"`

这是你项目自己的配置类。
这说明节点不会把模型路径、配置目录这些东西写死在逻辑里，而是交给原有配置系统。

---

### `#include "utils/include/pipeline.hpp"`

这是你的检测流水线封装。
节点本身不写检测细节，它只调用 `pipeline_->process(frame)`。
这就是我们之前一直强调的：

> ROS2 节点负责组织流程，业务逻辑仍然放在你原来的 C++ 模块里。 

---

### `#include "utils/include/draw.hpp"`

这是你原工程里的绘图函数。
说明节点不仅输出检测结果，还会顺手生成可视化调试图像。

---

# 二、定义节点类

```cpp id="cih7fh"
class DetectNode : public rclcpp::Node
{
```

这一行的意思很直接：

> 我定义了一个类，叫 `DetectNode`，它继承自 ROS2 的 `Node`。

以后你写别的节点，几乎都长这样：

* `class VideoNode : public rclcpp::Node`
* `class PoseNode : public rclcpp::Node`
* `class TrackerNode : public rclcpp::Node`

所以这行是“写 ROS2 节点”的固定起点。

---

# 三、进入 `public`，看构造函数

```cpp id="dsglc6"
public:
    DetectNode() : Node("detect_node")
```

这一段可以拆成两部分理解。

### `DetectNode()`

这是构造函数。
只要你创建了这个类的对象，它就会自动执行。

### `: Node("detect_node")`

这是初始化父类 `rclcpp::Node`。
意思是：这个节点在 ROS2 系统中的名字叫：

```text id="s8n8uy"
detect_node
```

所以你以后在日志里看到 `[detect_node]`，就是这行决定的。

---

# 四、构造函数里第一步：声明参数

```cpp id="hf5xvi"
this->declare_parameter<std::string>("config_dir",
    "/home/delphine/rm/tensorrt10_detect/configs");
this->declare_parameter<std::string>("input_topic", "/image_raw");
this->declare_parameter<std::string>("output_topic", "/detected_image");
```

这三行很重要。它们不是在“赋值”，而是在**向 ROS2 注册参数**。

---

### 第一行：声明 `config_dir`

参数名是 `config_dir`，类型是 `std::string`，默认值是你的配置目录路径。

它的作用是：

> 告诉节点：配置文件应该从哪里读。

---

### 第二行：声明 `input_topic`

默认值是 `/image_raw`。

这说明：

> 这个检测节点准备从 `/image_raw` 上接收原始图像。

也就是说，它已经不再自己读视频，而是等待外部节点把图像发给它。

---

### 第三行：声明 `output_topic`

默认值是 `/detected_image`。

这说明：

> 这个检测节点处理完后，会把带框图像发到 `/detected_image`。

所以这三行合起来，已经把这个节点的职责完全说清楚了：

```text id="oqnplg"
读配置
订阅原图
发布结果图
```

---

# 五、读取参数值

```cpp id="yzzd5m"
std::string config_dir = this->get_parameter("config_dir").as_string();
input_topic_ = this->get_parameter("input_topic").as_string();
output_topic_ = this->get_parameter("output_topic").as_string();
```

前面 `declare_parameter` 只是注册参数，
这里才是真正把参数值取出来。

---

### 第一行

```cpp id="r1rqwj"
std::string config_dir = this->get_parameter("config_dir").as_string();
```

从节点参数表中读取 `config_dir`，转成字符串，保存到局部变量 `config_dir`。

注意这里是局部变量，因为后面只是在构造函数里初始化配置对象时用一次。

---

### 第二行

```cpp id="gtgda0"
input_topic_ = this->get_parameter("input_topic").as_string();
```

把输入话题名读出来，保存到成员变量 `input_topic_`。

为什么是成员变量？
因为它属于整个节点的长期状态，不只是构造阶段临时用一下。

---

### 第三行

```cpp id="mwaj9d"
output_topic_ = this->get_parameter("output_topic").as_string();
```

同理，把输出话题名保存到成员变量 `output_topic_`。

---

# 六、打印初始化日志

```cpp id="i1hbi0"
RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());
```

这三行都是 ROS2 日志输出。

---

### 第一行

打印当前配置目录。

### 第二行

打印当前订阅的话题名。

### 第三行

打印当前发布的话题名。

这些日志的作用不是“必须”，但特别利于调试。
你以后一旦 launch 文件写错参数，第一时间就能从这里看出来。

---

# 七、创建原有业务对象

```cpp id="dfw4en"
cfg_ = std::make_unique<Config>(config_dir);
pipeline_ = std::make_unique<DetectPipeline>(*cfg_);
```

这两行是 ROS2 外壳和你原工程之间真正接上的地方。

---

### 第一行

```cpp id="fi2ms5"
cfg_ = std::make_unique<Config>(config_dir);
```

这里做了三件事：

1. 创建一个 `Config` 对象
2. 用 `config_dir` 初始化它
3. 用 `unique_ptr` 让节点独占管理这个对象

为什么用 `unique_ptr`？
因为这个配置对象只属于这个节点，不需要共享。

---

### 第二行

```cpp id="u0gyg0"
pipeline_ = std::make_unique<DetectPipeline>(*cfg_);
```

这表示：

* 用配置对象初始化检测流水线
* 让检测节点长期持有它

这也体现了一个工程原则：

> pipeline 不应该每帧重新创建，而应该初始化一次、反复复用。

否则 TensorRT 这类资源开销会很大。

---

# 八、创建 publisher

```cpp id="wloybo"
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
```

这一行创建图像发布者。

---

### `create_publisher<sensor_msgs::msg::Image>`

说明这个发布者发送的是图像消息。

### `output_topic_`

说明发送到哪个话题，由参数控制。

### `10`

表示队列深度。
如果短时间内消息来得太快、订阅者来不及处理，就先缓冲最多 10 条。

所以这行的本质是：

> `detect_node` 以后可以把处理后的图像发出去给别的节点看。

---

# 九、创建 subscriber

```cpp id="0rjwb1"
image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_, 10,
    std::bind(&DetectNode::image_callback, this, std::placeholders::_1));
```

这是新版 `detect_node` 的核心变化。

---

### `create_subscription<sensor_msgs::msg::Image>(...)`

说明你在创建一个图像订阅者。

### `input_topic_`

说明订阅的话题来自参数，比如 `/image_raw`。

### `10`

仍然是队列深度。

### `std::bind(...)`

这部分非常值得你理解。

```cpp id="j7w5jl"
std::bind(&DetectNode::image_callback, this, std::placeholders::_1)
```

意思是：

> 当订阅到一条图像消息时，调用当前这个 `DetectNode` 对象的 `image_callback(msg)` 成员函数。

这里：

* `&DetectNode::image_callback`：表示成员函数地址
* `this`：表示当前对象
* `std::placeholders::_1`：表示“未来传进来的第一个参数”，也就是消息本身

这就是 ROS2 回调和你学过的现代 C++ 绑定机制结合的典型例子。

---

# 十、构造函数结尾日志

```cpp id="db1r87"
RCLCPP_INFO(this->get_logger(), "DetectNode 初始化完成，等待图像输入...");
```

这表示构造函数里所有准备工作都做完了：

* 参数注册和读取完成
* 配置初始化完成
* pipeline 初始化完成
* publisher 创建完成
* subscriber 创建完成

接下来节点就进入：

```text id="05j9ra"
等待外部图像消息到来
```

的状态。

---

# 十一、进入 `private`，开始看核心处理函数

```cpp id="l1pq6e"
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
```

这就是这个节点真正的“干活函数”。

它的意思是：

> 每收到一张 ROS2 图像消息，就自动执行这个函数。

参数 `msg` 是一个智能指针，指向收到的图像消息。

这和以前你自己写循环读帧已经不一样了。
现在的触发方式是事件驱动：

```text id="jlwmhb"
有图像到来 → 自动回调
```

---

# 十二、`try` 块开始

```cpp id="gfjlnh"
try {
```

这里表示：

> 下面的图像转换和检测逻辑，可能抛异常，所以先用 `try` 包起来。

这是好习惯。
因为图像编码不对、cv_bridge 转换失败、某些内部函数报错时，都不至于直接把整个节点崩掉。

---

# 十三、先把 ROS 图像消息转回 OpenCV 图像

```cpp id="ghdzig"
auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
cv::Mat frame = cv_ptr->image;
```

这两行极其关键。

---

### 第一行

```cpp id="8cx7zs"
auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
```

意思是：

* 把 ROS 图像消息 `msg`
* 按照 `"bgr8"` 的编码方式
* 转成一个 OpenCV 可用的对象

为什么是 `"bgr8"`？
因为 OpenCV 常用的彩色图像格式就是 BGR 三通道 8 位。

---

### 第二行

```cpp id="cnlydd"
cv::Mat frame = cv_ptr->image;
```

从 `cv_ptr` 里取出真正的 OpenCV 图像 `cv::Mat`。

从这一行开始，你就回到了自己熟悉的纯 OpenCV 世界。
后面检测、画框、写字，全部都可以像以前那样写。

---

# 十四、调用检测 pipeline

```cpp id="d8b0lf"
std::vector<Result> results = pipeline_->process(frame);
```

这行就是你的检测主逻辑入口。

含义是：

* 输入：当前这一帧图像 `frame`
* 调用：你的检测流水线
* 输出：检测结果 `results`

这里非常能体现“ROS2 外壳 + 原业务模块”的设计思想。
ROS2 节点没有重新实现检测算法，而是直接复用你原来的 pipeline。

---

# 十五、绘制检测结果

```cpp id="f3zyj6"
drawDetect(frame, results, cfg_->model.classNames);
```

这行表示：

* 把检测结果画到图像上
* 类别名字从配置里取

为什么要传 `cfg_->model.classNames`？
因为 `results` 里往往只有类别 id，而绘图时需要把 id 变成人能看懂的标签名。

所以这一行本质是在生成一张“调试可视化图”。

---

# 十六、开始计算 FPS

```cpp id="vdem13"
auto now = std::chrono::steady_clock::now();
double dt = std::chrono::duration<double>(now - last_time_).count();
last_time_ = now;
```

这三行是时间测量。

---

### 第一行

获取当前时刻。

### 第二行

计算“本次回调”和“上次回调”之间相差多少秒。

### 第三行

把当前时间保存成下一次的“上次时间”。

这个 `dt` 其实就是：

> 两次处理之间的时间间隔

---

# 十七、用时间间隔算瞬时 FPS，并做平滑

```cpp id="0xjuvk"
double instant_fps = 1.0 / std::max(dt, 1e-6);
fps_ = 0.9 * fps_ + 0.1 * instant_fps;
```

这两行一起看。

---

### 第一行

`instant_fps = 1 / dt`

这就是最基本的 FPS 计算方式。
但这里用了：

```cpp id="gwnvta"
std::max(dt, 1e-6)
```

是为了防止 `dt` 极小导致除零或数值爆炸。

---

### 第二行

```cpp id="yxz7ng"
fps_ = 0.9 * fps_ + 0.1 * instant_fps;
```

这是一个简单的指数平滑。

意思是：

* 90% 保留旧值
* 10% 引入新值

好处是：
显示出来的 FPS 不会每帧剧烈抖动。

---

# 十八、把 FPS 画到图像上

```cpp id="vm8yxx"
cv::putText(frame, cv::format("FPS: %.1f", fps_),
            cv::Point(20, 40),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0, cv::Scalar(0, 255, 0), 2);
```

这是标准 OpenCV 绘字。

它做的事情是：

* 把字符串 `"FPS: xx.x"` 格式化出来
* 画到图像左上角附近
* 绿色字体
* 线宽 2

这一步是为了方便调试和观察实时性能。

---

# 十九、拷贝输入消息的 header

```cpp id="k3z2v5"
std_msgs::msg::Header header = msg->header;
header.frame_id = "detected_frame";
```

这两行很值得理解。

---

### 第一行

把输入消息的头信息复制一份出来。

这意味着：

* 原图的时间戳会保留下来
* 其他 header 信息也继承下来

---

### 第二行

把 `frame_id` 改成 `"detected_frame"`。

这表示：

> 这已经不是“原始视频帧”了，而是“经过检测后输出的图像帧”。

这是一种很好的语义区分。

---

# 二十、把 OpenCV 图像重新转回 ROS2 消息

```cpp id="d1xil9"
auto out_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
```

这一步是前面 `toCvCopy` 的反向过程。

意思是：

* 用处理后的 `frame`
* 配上刚才准备好的 `header`
* 指定编码 `"bgr8"`
* 生成一条新的 ROS 图像消息

也就是说：

```text id="i88t7u"
OpenCV 图像 → ROS2 图像消息
```

---

# 二十一、发布处理后的图像

```cpp id="a2a7la"
image_pub_->publish(*out_msg);
```

这是整个检测节点输出结果的关键一行。

它的含义很直接：

> 把当前这张已经画好框、写好 FPS 的图，发到输出话题上。

这样你的 `display_node` 就能订阅到它并显示出来。

---

# 二十二、打印节流日志

```cpp id="a1qgkt"
RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "检测到 %zu 个目标，fps: %.1f",
    results.size(), fps_);
```

这几行是“节流日志”。

和普通 `RCLCPP_INFO` 不同，它不会每次回调都打日志。
这里 `1000` 表示：

> 最多每 1000 毫秒打印一次

这样终端不会被刷爆。

日志内容是：

* 当前检测到的目标数量
* 当前平滑后的 FPS

这很适合实时系统。

---

# 二十三、异常处理：`cv_bridge` 失败

```cpp id="9li7uh"
catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
}
```

这表示：

> 如果消息转 OpenCV 图像失败，就记录错误，但不要让节点直接崩掉。

常见原因可能是：

* 图像编码不是 `bgr8`
* 消息内容不合法

这里单独把 `cv_bridge::Exception` 拿出来处理，是比较专业的写法。

---

# 二十四、异常处理：其他标准异常

```cpp id="jlwmr1"
catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "检测回调异常: %s", e.what());
}
```

这表示：

> 只要回调里有其他标准异常，也统一拦截并打印日志。

这样做的好处是：
某一帧处理出问题时，不至于整个节点当场退出。

---

# 二十五、成员变量部分

```cpp id="thgaes"
std::unique_ptr<Config> cfg_;
std::unique_ptr<DetectPipeline> pipeline_;

std::string input_topic_;
std::string output_topic_;

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
double fps_ = 0.0;
```

这些是节点长期保存的状态。

---

### `cfg_`

保存配置对象。

### `pipeline_`

保存检测流水线对象。

这两个都用 `unique_ptr`，因为节点独占它们，不需要共享。

---

### `input_topic_` / `output_topic_`

保存输入和输出话题名。

为什么不只用局部变量？
因为这是节点自己的固定属性。

---

### `image_sub_`

保存订阅者对象。

这很重要。
如果你把订阅者只写成构造函数里的局部变量，构造结束后它就会被销毁，节点就收不到消息了。

---

### `image_pub_`

保存发布者对象。

同理，publisher 也必须活着，节点才能继续发消息。

---

### `last_time_`

保存上一帧的时间点，用于下一帧算 `dt`。

---

### `fps_`

保存平滑后的 FPS 数值。

---

# 二十六、最后看 `main`

```cpp id="bwhm3n"
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

这是 ROS2 C++ 节点的经典模板。

---

### `rclcpp::init(argc, argv);`

初始化 ROS2 系统。

没有这一步，ROS2 通信框架根本没准备好。

---

### `auto node = std::make_shared<DetectNode>();`

创建一个 `DetectNode` 对象。

为什么用 `make_shared`？
因为 ROS2 节点通常用 `shared_ptr` 生命周期管理。

---

### `rclcpp::spin(node);`

进入 ROS2 事件循环。

这行的作用是：

> 不断等待并处理这个节点的回调。

在你这份代码里，最重要的回调就是：

```text id="0xllgc"
image_callback(...)
```

只要 `/image_raw` 上有新图，spin 就会让这个回调执行。

---

### `rclcpp::shutdown();`

当节点退出时，关闭 ROS2 系统并清理资源。

---

### `return 0;`

程序正常结束。

---

# 二十七、把整份代码压缩成一个“流程图”

你这份新版 `detect_node.cpp` 的实际工作流程就是：

```text id="0m7ywr"
启动节点
→ 声明参数
→ 读取参数
→ 初始化 Config 和 DetectPipeline
→ 创建订阅者和发布者
→ 等待 /image_raw 图像
→ 收到图像后转成 cv::Mat
→ 跑检测
→ 画框和 FPS
→ 转回 ROS 图像消息
→ 发布到 /detected_image
```

这就是你以后写大多数算法节点的模板。

---

# 二十八、它和 `video_node`、`display_node` 是怎么配合的

你现在三者的关系已经很清楚了：

## `video_node`

负责：

* 打开视频
* 定时发 `/image_raw` 

## `detect_node`

负责：

* 收 `/image_raw`
* 跑检测
* 发 `/detected_image` 

## `display_node`

负责：

* 收 `/detected_image`
* 显示窗口 

也就是说，你现在已经搭出了一个非常标准的小型 ROS2 视觉链。

---

# 二十九、你从这份代码里真正该学到的“写节点流程”

以后你写别的节点，可以照这个顺序思考：

## 1. 这个节点叫什么

比如 `PoseNode`

## 2. 它要读哪些参数

比如模型路径、topic 名、配置文件路径

## 3. 它内部要持有哪些业务对象

比如 `PoseSolver`、`Raycaster`

## 4. 它要订阅什么

比如检测结果、图像、坐标

## 5. 它要发布什么

比如世界坐标、地图点、Marker

## 6. 它的核心 callback 里做什么

比如：

* 收消息
* 转格式
* 调业务函数
* 生成新消息
* 发布

---

