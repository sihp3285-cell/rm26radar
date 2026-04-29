# `map_node.cpp` 逐行讲解

这份文件是**小地图节点**，在你当前 ROS2 视觉链路里的位置是：

```text
video_node
   ↓  /image_raw
detect_node
   ↓  /armor_detections
pose_node
   ↓  /world_targets  ← map_node 订阅这里
map_node  ← 你在这里
   ↓  /map_image       ← 地图可视化图像
   ↓  /radar_map       ← 结构化雷达数据
display_node
```

`map_node` 是系统的**可视化汇聚层**和**数据汇总层**。它负责：

1. 订阅 `pose_node` 发布的 `WorldTargetArray`（世界坐标目标）
2. 把世界坐标转换成小地图上的像素坐标
3. 绘制小地图图像，发布到 `/map_image`
4. 同时把各机器人位置打包成结构化 `RadarMap` 消息，发布到 `/radar_map`

在 ROS2 架构中，这种节点叫做 **Visualization / Aggregation Node**：

> 它接收上游的抽象数据（世界坐标），生成两种形式的输出：
> * 给人看的图像（`map_image`）
> * 给机器下游用的结构化数据（`radar_map`，比如供决策节点或通信节点发给下位机）

---

# 一、头文件部分

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/radar_map.hpp"
#include "ConfigManager.hpp"
#include "radarmap.hpp"
#include "robot_id.hpp"
```

---

### `#include <rclcpp/rclcpp.hpp>`

ROS2 C++ 节点基础。

---

### `#include <sensor_msgs/msg/image.hpp>` / `#include <cv_bridge/cv_bridge.hpp>`

`map_node` 要发布地图图像，所以需要把 OpenCV 的 `cv::Mat` 转成 ROS2 图像消息。

注意这个节点**只发图，不收图**，所以 `cv_bridge` 只做 `cv::Mat → Image` 这一个方向。

---

### 自定义消息

```cpp
#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/radar_map.hpp"
```

这个节点同时涉及三种自定义消息：

| 消息 | 方向 | 用途 |
|------|------|------|
| `WorldTargetArray` / `WorldTarget` | 输入 | 接收世界坐标目标 |
| `RadarMap` | 输出 | 发布按队伍/编号整理后的地图坐标 |

---

### `#include "ConfigManager.hpp"`

读取 `map.yaml`（地图图片路径、场地尺寸、地图像素尺寸、是否翻转）。

---

### `#include "radarmap.hpp"`

`RadarMap` 类封装了：

* 加载地图底图
* 世界坐标 ↔ 地图像素坐标的标定转换
* 在地图上绘制目标点、标签

---

### `#include "robot_id.hpp"`

定义了队伍 ID（RED/BLUE）和机器人类别（R1~R4, S, CAR）的枚举和工具函数。

---

# 二、定义节点类

```cpp
class MapNode : public rclcpp::Node
```

固定起手式。

---

# 三、构造函数

```cpp
public:
    MapNode() : Node("map_node")
```

节点注册名 `"map_node"`。

---

# 四、声明参数

```cpp
this->declare_parameter<std::string>("config_dir",
    "/home/delphine/rm/tensorrt10_detect/configs");
this->declare_parameter<std::string>("input_topic", "/world_targets");
this->declare_parameter<std::string>("output_image_topic", "/map_image");
this->declare_parameter<std::string>("output_map_topic", "/radar_map");
```

---

### `config_dir`

配置目录。MapNode 需要读取 `map.yaml`，里面包含地图路径、场地尺寸等。

---

### `input_topic`

默认 `/world_targets`。订阅 `pose_node` 发布的解算结果。

---

### `output_image_topic`

默认 `/map_image`。发布绘制好的小地图图像，供 `display_node` 显示。

---

### `output_map_topic`

默认 `/radar_map`。发布结构化雷达地图数据，供决策系统或通信模块使用。

---

# 五、读取参数并打印

```cpp
std::string config_dir = this->get_parameter("config_dir").as_string();
input_topic_ = this->get_parameter("input_topic").as_string();
output_image_topic_ = this->get_parameter("output_image_topic").as_string();
output_map_topic_ = this->get_parameter("output_map_topic").as_string();

RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
RCLCPP_INFO(this->get_logger(), "发布图像话题: %s", output_image_topic_.c_str());
RCLCPP_INFO(this->get_logger(), "发布地图话题: %s", output_map_topic_.c_str());
```

打印四个关键配置，确认节点初始化状态。

---

# 六、初始化 Config 和 RadarMap

```cpp
cfg_ = std::make_unique<Config>(config_dir);
radar_map_ = std::make_unique<RadarMap>(cfg_->map.mapPath, cfg_->map.isFlip);
radar_map_->calibrate2(
    cfg_->map.race_size[0],
    cfg_->map.race_size[1],
    cfg_->map.map_size[0],
    cfg_->map.map_size[1]);
```

---

### `RadarMap`

加载地图底图图片（如 `map.png`）。`isFlip` 控制是否需要垂直翻转地图。

---

### `calibrate2`

做**世界坐标到地图像素坐标的标定**。

参数含义：

* `race_size[0], race_size[1]`：场地的物理尺寸（长、宽，单位：米）
* `map_size[0], map_size[1]`：地图图片的像素尺寸（宽、高）

`calibrate2` 内部会计算缩放比例和偏移量，以后调用 `worldtomap()` 时就能快速转换。

---

### 标定结果检查

```cpp
if (!radar_map_->m_isCalibrated) {
    RCLCPP_ERROR(this->get_logger(), "RadarMap 校准失败，请检查 map.yaml 配置");
} else {
    RCLCPP_INFO(this->get_logger(), "RadarMap 校准完成");
}
```

如果 `map.yaml` 里的尺寸参数写错了（比如场地尺寸和地图尺寸不匹配），校准会失败。这里打印错误但不退出，因为节点可能仍然能绘制地图，只是坐标不对。

---

# 七、创建两个 Publisher

```cpp
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, rclcpp::QoS(1));
radar_map_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::RadarMap>(output_map_topic_, 10);
```

和 `detect_node` 一样，`map_node` 也采用**一输入、双输出**模式：

| Publisher | 类型 | 话题 | QoS | 给谁用 |
|-----------|------|------|-----|--------|
| `image_pub_` | `sensor_msgs::msg::Image` | `/map_image` | `QoS(1)` | `display_node`（给人看） |
| `radar_map_pub_` | `RadarMap` | `/radar_map` | `depth=10` | 决策系统/通信模块（给机器用） |

---

# 八、创建 Subscriber

```cpp
target_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::WorldTargetArray>(
    input_topic_, 10,
    std::bind(&MapNode::target_callback, this, std::placeholders::_1));
```

订阅 `pose_node` 发布的 `WorldTargetArray`。

---

# 九、初始化完成日志

```cpp
RCLCPP_INFO(this->get_logger(), "MapNode 初始化完成，等待世界坐标输入...");
```

---

# 十、核心回调：`target_callback`

```cpp
private:
    void target_callback(const tensorrt_detect_msgs::msg::WorldTargetArray::SharedPtr msg)
```

这是 `map_node` 的干活函数。每收到一批世界坐标目标，就执行一次。

---

# 十一、准备本地数据结构和输出消息

```cpp
std::vector<Mappoint> mappoints;
auto radar_msg = std::make_shared<tensorrt_detect_msgs::msg::RadarMap>();

// 初始化数组为 0
for (int i = 0; i < 6; ++i) {
    radar_msg->blue_x[i] = 0.0f;
    radar_msg->blue_y[i] = 0.0f;
    radar_msg->red_x[i] = 0.0f;
    radar_msg->red_y[i] = 0.0f;
}
```

---

### `mappoints`

本地缓存，存储每个目标在地图上的像素坐标、类别、队伍等信息。稍后传给 `radar_map_->drawMap()` 做可视化。

---

### `RadarMap` 消息清零

`RadarMap` 消息里有 4 个长度为 6 的浮点数组：

```cpp
float32[6] blue_x, blue_y   // 蓝方 1~5 号 + 哨兵
float32[6] red_x, red_y     // 红方 1~5 号 + 哨兵
```

初始化为 0，表示"还没看到这个机器人"。如果某个机器人在当前帧没被检测到，对应的数组元素就保持 0。

这是 ROS2 消息设计中常见的**稀疏数组**模式：用固定长度数组 + 有效值/零值来表示动态数量的目标。

---

# 十二、遍历每个目标并处理

```cpp
for (const auto& target : msg->targets) {
    if (!target.valid) {
        continue;
    }

    // 过滤掉车辆检测（CAR, class_id == 0）
    if (target.class_id == robot_id::CAR) {
        continue;
    }
```

---

### `valid` 检查

如果 `pose_node` 标记了某个目标无效，直接跳过。

---

### 过滤 CAR

`class_id == 0` 对应 `robot_id::CAR`，即未分类的车辆整体检测框。

在小地图上，我们只关心具体的机器人（R1、R2、R3、R4、哨兵），不关心模糊的"这是一辆车"的检测结果。所以过滤掉 CAR，保证地图上只显示有明确身份的目标。

> 注意：`detect_node` 已经在发布 `/armor_detections` 时过滤掉了 CAR，所以这里理论上不会收到 CAR。这个过滤是**防御性检查**，防止未来代码变更导致意外数据进入地图绘制。

---

# 十三、世界坐标 → 地图坐标

```cpp
// pose_node 中 world_x 为场地 X（宽），world_z 为场地 Z（长）
cv::Point2f world_pt(target.world_x, target.world_z);
cv::Point2f map_pt = radar_map_->worldtomap(world_pt);
```

---

### 坐标约定

`pose_node` 输出的约定：

* `world_x` → 场地宽度方向（X 轴）
* `world_z` → 场地长度方向（Z 轴）

`map_node` 消费时遵循同一套约定，把 `(world_x, world_z)` 当成二维平面上的点，传给 `worldtomap()`。

---

### `worldtomap`

`RadarMap` 内部根据之前在 `calibrate2` 里计算的缩放比例和偏移量，把世界坐标（米）转换成地图像素坐标：

```text
世界坐标（米） → 地图像素坐标
```

---

# 十四、填充本地可视化结构

```cpp
Mappoint mp;
mp.map_point = map_pt;
mp.label = "";
mp.classIdx = target.class_id;
mp.armorColor = target.team_id;
mp.isDead = target.is_dead;
mappoints.push_back(mp);
```

`label` 传空字符串，因为 `drawMap` 内部会根据 `classIdx` 和 `isDead` 自己计算标签（如 `"dead"`、`"1"`、`"s"`）。

这和 `standalone_main.cpp` 里的做法保持一致。

---

### `armorColor` 和 `isDead`

* `armorColor = target.team_id`：颜色/队伍信息（红/蓝/未知）
* `isDead = target.is_dead`：死亡状态标志

两者分离传递，确保 `drawMap` 能正确区分死亡装甲板（画黑色 `"dead"`）和未分类存活装甲板。

---

# 十五、填充 RadarMap 结构化消息

```cpp
// class_id 映射: R1=2, R2=3, R3=4, R4=5, S=6
// RadarMap 数组: [1号, 2号, 3号, 4号, 5号, 哨兵]
int idx = -1;
if (target.class_id >= robot_id::R1 && target.class_id <= robot_id::R4) {
    idx = target.class_id - robot_id::R1; // 2->0, 3->1, 4->2, 5->3
} else if (target.class_id == robot_id::S) {
    idx = 5; // 哨兵放在索引 5
}

if (!target.is_dead && idx >= 0 && idx < 6) {
    if (target.team_id == robot_id::BLUE) {
        radar_msg->blue_x[idx] = map_pt.x;
        radar_msg->blue_y[idx] = map_pt.y;
    } else if (target.team_id == robot_id::RED) {
        radar_msg->red_x[idx] = map_pt.x;
        radar_msg->red_y[idx] = map_pt.y;
    }
}
```

---

### 索引映射

`class_id` 和 `RadarMap` 数组索引的对应关系：

| class_id | 含义 | 数组索引 |
|---------|------|---------|
| 2 (R1) | 1号机器人 | 0 |
| 3 (R2) | 2号机器人 | 1 |
| 4 (R3) | 3号机器人 | 2 |
| 5 (R4) | 4号机器人 | 3 |
| 6 (S) | 哨兵 | 5 |

注意索引 4（5号机器人）在这个项目里没有使用。

---

### 死亡装甲板不填入 RadarMap

```cpp
if (!target.is_dead && idx >= 0 && idx < 6) {
```

**死亡装甲板不填入 `RadarMap` 的红蓝数组**。

原因：`RadarMap` 消息格式只有 `blue_x/blue_y` 和 `red_x/red_y` 两个数组，没有专门存放死亡车辆的位置。死亡装甲板只在 `/map_image` 上显示为黑色圆点，不进入结构化雷达数据。

这是消息格式限制导致的合理取舍：如果后续需要把死亡车辆位置发给下位机，可以扩展 `RadarMap` 消息定义（比如加一个 `dead_x/dead_y` 数组）。

---

### 按队伍填充

根据 `team_id`（BLUE=2, RED=1）分别填入 `blue_x/blue_y` 或 `red_x/red_y`。

这样下游节点收到 `RadarMap` 后，可以直接用数组下标访问任意机器人的位置，不需要再遍历搜索。

---

# 十六、发布 RadarMap 消息

```cpp
radar_msg->header = msg->header;
radar_map_pub_->publish(*radar_msg);
```

---

### Header 继承

把输入消息的时间戳和坐标系信息原样传递下去。

---

### 发布

结构化雷达数据发出去了。通信节点可以订阅 `/radar_map`，把蓝红双方各机器人的位置通过串口/网络发给下位机或队友。

---

# 十七、绘制并发布地图图像

```cpp
cv::Mat map_frame = radar_map_->drawMap(mappoints, cfg_->model.classNames);

std_msgs::msg::Header header = msg->header;
header.frame_id = "radar_map";
auto out_msg = cv_bridge::CvImage(header, "bgr8", map_frame).toImageMsg();
image_pub_->publish(*out_msg);
```

---

### `drawMap`

`RadarMap` 在地图底图上绘制所有目标点：

* 用 `isDead` 和 `armorColor` 决定颜色（死亡=黑，红=红，蓝=蓝，未知=青）
* 用 `isDead` 和 `classIdx` 决定标签（死亡=`"dead"`，存活=类别名）
* 在 `map_point` 位置画圆点和文字

---

### 发布地图图像

和 `detect_node` 发布可视化图的逻辑一样：

1. 继承输入消息的 header（保留时间戳）
2. 改写 `frame_id` 为 `"radar_map"`（语义标识）
3. 用 `cv_bridge` 转回 ROS 图像消息
4. `publish`

---

# 十八、节流日志

```cpp
RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    10000,
    "接收到 %zu 个世界坐标目标，发布了 RadarMap 和地图图像",
    msg->targets.size());
```

每 10 秒打印一次处理统计。

---

# 十九、异常处理

```cpp
catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "地图回调异常: %s", e.what());
}
```

拦截所有标准异常，保证节点不崩。

---

# 二十、成员变量

```cpp
std::unique_ptr<Config> cfg_;
std::unique_ptr<RadarMap> radar_map_;

std::string input_topic_;
std::string output_image_topic_;
std::string output_map_topic_;

rclcpp::Subscription<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr target_sub_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::Publisher<tensorrt_detect_msgs::msg::RadarMap>::SharedPtr radar_map_pub_;
```

---

### `cfg_`

配置对象，读取地图参数和类别名称。

---

### `radar_map_`

地图绘制器。持有地图底图和坐标转换参数。

---

### `target_sub_` / `image_pub_` / `radar_map_pub_`

一个订阅者 + 两个发布者。Pub/Sub 对象都必须作为成员变量长期保存。

---

# 二十一、`main` 函数

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

经典四步法。

---

# 二十二、完整数据流回顾

```text
pose_node ──/world_targets──→ map_node
                                      │
                                      ├── /map_image ──→ display_node
                                      │
                                      └── /radar_map ──→ (决策/通信节点)
```

`map_node` 是系统的**末端汇聚点之一**：

* 接收抽象的世界坐标
* 输出两种形式：图像（人看）+ 结构化数组（机器用）
* 对原始数据做了**语义整理**：按队伍和编号分类，填入固定长度数组

---

# 二十三、从这份代码里学到的设计要点

## 1. 聚合节点的价值

`map_node` 不只做可视化，还做了**数据重组**。

`pose_node` 输出的是"一个目标列表"，每个目标带有 `class_id` 和 `team_id`。`map_node` 把它重组成了"按队伍和编号索引的数组"。

这种重组让下游消费者（如通信节点）的代码大大简化：

```cpp
// 收到 RadarMap 后，直接访问：
float red3_x = radar_map_msg.red_x[2];   // 红3的X坐标
float blueS_y = radar_map_msg.blue_y[5]; // 蓝哨兵的Y坐标
```

不需要遍历、不需要匹配、不需要查表。

## 2. 固定长度数组表示动态目标

`RadarMap` 用 `float32[6]` 表示每方最多 6 个机器人（1~5号 + 哨兵）。如果某个索引值为 0，表示"这个机器人当前未检测到"。

这是在 ROS2 消息带宽受限场景下的常用技巧：

> 用固定长度数组替代变长数组，减少序列化/反序列化开销，同时让下游访问更简单。

## 3. 语义过滤

```cpp
if (target.class_id == robot_id::CAR) continue;
```

在节点边界处做过滤，保证输出数据的语义纯净。`map_node` 只关心"有明确身份的机器人"，模糊的 CAR 检测结果在这里被拦截，不进入地图和雷达消息。

## 4. 坐标系 layered 转换

整个链路经历了三层坐标变换：

```text
像素坐标（图像）
    ↓  detect_node / pose_node（PnP + Raycasting）
世界坐标（米，场地坐标系）
    ↓  map_node（worldtomap）
地图像素坐标（小地图上的点）
```

每一层节点只负责自己擅长的变换，层与层之间用清晰的消息接口衔接。

## 5. 双输出模式

和 `detect_node` 一样，`map_node` 同时发图像和结构化消息。这再次印证了 ROS2 的一个设计模式：

> **同一批数据，用不同形式服务不同消费者。**
