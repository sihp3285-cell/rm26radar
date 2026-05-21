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
   ↓  /map_tactics     ← 战术态势数据（新增）
display_node / 决策节点
```

`map_node` 是系统的**可视化汇聚层**、**数据汇总层**和**战术分析层**。它负责：

1. 订阅 `pose_node` 发布的 `WorldTargetArray`（固定槽位 + Outpost + 死亡装甲板）
2. 把世界坐标转换成小地图上的像素坐标
3. 绘制小地图图像，发布到 `/map_image`
4. 把各机器人位置打包成结构化 `RadarMap` 消息，发布到 `/radar_map`
5. **通过 `MapAnalyzer` 做战术态势分析，发布 `/map_tactics`（新增）**
6. **在地图上叠加绘制前哨站状态（存活/摧毁）（新增）**

---

# 一、头文件部分

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>

#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/radar_map.hpp"
#include "tensorrt_detect_msgs/msg/map_tactics.hpp"  // ← 新增：战术态势消息
#include "ConfigManager.hpp"
#include "radarmap.hpp"
#include "robot_id.hpp"
#include "map_analyzer.hpp"                             // ← 新增：战术分析器
```

---

### 新增依赖

| 头文件 | 用途 |
|--------|------|
| `map_tactics.hpp` | 定义 `MapTactics` 消息类型，包含 4 个战术标志 |
| `map_analyzer.hpp` | `MapAnalyzer` 类，分析工程上岛、攻防态势、堡垒威胁 |

---

# 二、构造函数

```cpp
MapNode() : Node("map_node")
```

---

## 2.1 参数声明（新增参数）

```cpp
this->declare_parameter<std::string>("config_dir",
    "/home/delphine/rm/tensorrt10_detect/configs");
this->declare_parameter<std::string>("input_topic", "/world_targets");
this->declare_parameter<std::string>("output_image_topic", "/map_image");
this->declare_parameter<std::string>("output_map_topic", "/radar_map");
this->declare_parameter<std::string>("output_tactics_topic", "/map_tactics");  // ← 新增
this->declare_parameter<int>("out_team_id", robot_id::RED);                     // ← 新增
this->declare_parameter<bool>("flip_team", false);
this->declare_parameter<bool>("field_x_flip", false);                           // ← 新增
```

---

### 新增参数说明

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `output_tactics_topic` | `/map_tactics` | 战术态势消息的话题名 |
| `out_team_id` | `robot_id::RED` | 我方实际阵营（不影响显示翻转，只影响战术分析） |
| `field_x_flip` | `false` | 场地 X 轴是否翻转（适配不同标定方向） |

---

## 2.2 初始化 RadarMap（与旧版相同）

```cpp
cfg_ = std::make_unique<Config>(config_dir);
radar_map_ = std::make_unique<RadarMap>(cfg_->map.mapPath, cfg_->map.isFlip);
radar_map_->calibrate2(
    cfg_->map.race_size[0], cfg_->map.race_size[1],
    cfg_->map.map_size[0], cfg_->map.map_size[1]);
radar_map_->setFlipTeam(flip_team_);
```

---

## 2.3 初始化 MapAnalyzer（新增）

```cpp
analyzer_ = std::make_unique<MapAnalyzer>(out_team_id_);
analyzer_->setTeamByFlip(flip_team_);
analyzer_->setFieldXFlip(!flip_team_);
RCLCPP_INFO(this->get_logger(), "初始阵营: %s", flip_team_ ? "红方" : "蓝方");
```

---

### `MapAnalyzer` 初始化链

1. `MapAnalyzer(out_team_id_)`：设置我方实际阵营
2. `setTeamByFlip(flip_team_)`：根据当前视角设置"我方/对方"映射
3. `setFieldXFlip(!flip_team_)`：设置场地坐标翻转（注意取反）

为什么 `field_x_flip` 和 `flip_team` 取反？因为 `flip_team_` 控制的是**地图显示翻转**，而 `field_x_flip_` 控制的是**世界坐标到场地坐标的转换方向**。两者在逻辑上是相反的。

---

## 2.4 创建 Publisher（新增 `tactics_pub_`）

```cpp
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, rclcpp::QoS(1));
radar_map_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::RadarMap>(output_map_topic_, 10);
tactics_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::MapTactics>(output_tactics_topic_, 10);  // ← 新增
```

---

## 2.5 `/flip_team` 订阅（新增 Analyzer 同步）

```cpp
flip_team_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/flip_team", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
        flip_team_ = msg->data;
        if (radar_map_) {
            radar_map_->setFlipTeam(flip_team_);
        }
        if (analyzer_) {                              // ← 新增
            analyzer_->setTeamByFlip(flip_team_);
            analyzer_->setFieldXFlip(!flip_team_);
        }
        RCLCPP_INFO(this->get_logger(), "阵营视角已切换为: %s", flip_team_ ? "红方" : "蓝方");
    });
```

阵营切换时，同时更新 `RadarMap`（地图显示）和 `MapAnalyzer`（战术分析），保持两者阵营视角一致。

---

# 三、核心回调：`target_callback`（重大更新）

---

## 3.1 初始化输出消息（与旧版相同）

```cpp
auto radar_msg = std::make_shared<tensorrt_detect_msgs::msg::RadarMap>();
for (int i = 0; i < 6; ++i) {
    radar_msg->blue_x[i] = 0.0f;
    radar_msg->blue_y[i] = 0.0f;
    radar_msg->red_x[i] = 0.0f;
    radar_msg->red_y[i] = 0.0f;
}
```

---

## 3.2 读取前哨站状态（更新：从固定索引读取）

```cpp
bool has_outpost = false;
bool outpost_alive = false;
if (msg->targets.size() > 10) {
    const auto& outpost_target = msg->targets[10];
    has_outpost = outpost_target.valid;
    outpost_alive = has_outpost && !outpost_target.is_dead;
}
```

---

### 与旧版的区别

旧版遍历所有 `targets` 查找 `class_id == OUTPOST`。新版直接用**固定索引 10** 读取（`pose_node` 约定索引 10 是前哨站），更高效。

---

## 3.3 遍历目标（更新：处理固定槽位输入）

```cpp
for (size_t i = 0; i < msg->targets.size(); ++i) {
    const auto& t = msg->targets[i];
    if (!t.valid) continue;

    cv::Point2f raw_pt = radar_map_->worldtomap(cv::Point2f(t.world_x, t.world_z));

    // 前哨站单独处理
    if (t.class_id == robot_id::OUTPOST) continue;

    // 动态死亡装甲板直接绘制
    if (t.class_id == robot_id::ARMOR && t.is_dead) {
        Mappoint mp;
        mp.map_point = raw_pt;
        mp.classIdx = robot_id::ARMOR;
        mp.armorColor = robot_id::UNKNOWN;
        mp.isDead = true;
        mp.label = "dead";
        mappoints.push_back(mp);
        continue;
    }

    // 固定槽位（R1~S）
    Mappoint mp;
    mp.map_point = raw_pt;
    // 优先使用 BotIdentity 稳定身份（指数加权历史），无效时回落到单帧 class_id
    int display_class = (t.stable_class_id >= 0 && t.stable_class_conf > 0.0f)
                            ? t.stable_class_id : t.class_id;
    mp.classIdx = display_class;
    mp.armorColor = t.team_id;
    mp.isDead = t.is_dead;
    if (display_class >= 0 && display_class < static_cast<int>(cfg_->model.classNames.size())) {
        mp.label = cfg_->model.classNames[display_class];
    }
    mappoints.push_back(mp);
```

---

### 与旧版的关键区别

旧版只有一个简单的 `for` 循环，过滤 CAR 和 OUTPOST。新版需要处理**三种来源不同的目标**：

| 来源 | 索引 | 处理方式 |
|------|------|---------|
| Tracker 固定槽位 | 0~9 | 正常绘制 + 填充 RadarMap |
| Outpost 透传 | 10 | 跳过（后面单独叠加绘制） |
| 动态死亡装甲板 | 11+ | 绘制黑色 "dead"，不填 RadarMap |

---

## 3.4 填充 RadarMap 消息（与旧版相同逻辑）

```cpp
int idx = -1;
if (t.class_id >= robot_id::R1 && t.class_id <= robot_id::R4) {
    idx = t.class_id - robot_id::R1;
} else if (t.class_id == robot_id::S) {
    idx = 5;
}
if (!t.is_dead && idx >= 0 && idx < 6) {
    if (t.team_id == robot_id::BLUE) {
        radar_msg->blue_x[idx] = raw_pt.x;
        radar_msg->blue_y[idx] = raw_pt.y;
    } else if (t.team_id == robot_id::RED) {
        radar_msg->red_x[idx] = raw_pt.x;
        radar_msg->red_y[idx] = raw_pt.y;
    }
}
```

只把固定槽位中**非死亡**的 R1~S 目标填入 RadarMap。

---

## 3.5 发布 RadarMap（与旧版相同）

```cpp
radar_msg->header = msg->header;
radar_map_pub_->publish(*radar_msg);
```

---

## 3.6 战术态势分析（新增）

```cpp
analyzer_->evaluate(msg->targets);

auto tactics_msg = std::make_shared<tensorrt_detect_msgs::msg::MapTactics>();
tactics_msg->header = msg->header;
tactics_msg->engineer_on_island = analyzer_->engineer_on_island();
tactics_msg->opponent_attack = analyzer_->opponent_attack();
tactics_msg->our_attack = analyzer_->our_attack();
tactics_msg->opponent_near_fortress = analyzer_->opponent_near_fortress();

tactics_pub_->publish(*tactics_msg);
```

---

### `MapTactics` 消息字段

| 字段 | 类型 | 含义 |
|------|------|------|
| `engineer_on_island` | int32 | 0=无, 1=我方工程上岛, 2=对方工程上岛 |
| `opponent_attack` | int32 | 0=否, 1=对方发起大攻 |
| `our_attack` | int32 | 0=否, 1=我方发起大攻 |
| `opponent_near_fortress` | int32 | 0=否, 1=对方接近我方堡垒 |

详见 `docs/core_map_analyzer.md`。

---

### 战术日志

```cpp
if (analyzer_->opponent_attack()) {
    RCLCPP_WARN_THROTTLE(..., "⚠️ 敌方大攻!");
}
if (analyzer_->our_attack()) {
    RCLCPP_INFO_THROTTLE(..., "✅ 我方大攻!");
}
if (analyzer_->engineer_on_island() == 1) {
    RCLCPP_WARN_THROTTLE(..., "⚠️ 我方工程上岛!");
}
if (analyzer_->engineer_on_island() == 2) {
    RCLCPP_WARN_THROTTLE(..., "⚠️ 敌方工程上岛!");
}
if (analyzer_->opponent_near_fortress() == 1) {
    RCLCPP_WARN_THROTTLE(..., "⚠️ 敌方接近堡垒!");
}
```

每 10 秒打印一次战术警报，帮助操作员关注关键态势变化。

---

## 3.7 绘制地图图像（与旧版相同）

```cpp
cv::Mat map_frame = radar_map_->drawMap(mappoints, cfg_->model.classNames);
```

---

## 3.8 前哨站叠加绘制（更新：基于 has_outpost 检查）

```cpp
const auto& outpostPts = cfg_->map.getOutpostMapPoints(flip_team_);
if (outpostPts.size() >= 2 && has_outpost) {
    // ... 坐标变换 ...
    if (outpost_alive) {
        cv::circle(map_frame, pt, 8, cv::Scalar(0, 215, 255), -1, cv::LINE_AA);
        cv::circle(map_frame, pt, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        cv::putText(map_frame, "ALIVE", ...);
    } else {
        cv::line(map_frame, pt + ..., dead_color, 3, cv::LINE_AA);  // 叉号
        cv::putText(map_frame, "DEAD", ...);
    }
}
```

---

### 与旧版的区别

旧版只要 `outpostPts.size() >= 2` 就绘制。新版额外检查 `has_outpost`，只有当前帧确实检测到前哨站（或前哨站曾经被检测到且 valid）时才绘制。

---

## 3.9 发布地图图像（与旧版相同）

```cpp
std_msgs::msg::Header header = msg->header;
header.frame_id = "radar_map";
auto out_msg = cv_bridge::CvImage(header, "bgr8", map_frame).toImageMsg();
image_pub_->publish(*out_msg);
```

---

# 四、成员变量（新增）

```cpp
bool flip_team_ = false;
std::unique_ptr<Config> cfg_;
std::unique_ptr<RadarMap> radar_map_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flip_team_sub_;

int out_team_id_ = robot_id::BLUE;                                               // ← 新增
std::string input_topic_;
std::string output_image_topic_;
std::string output_map_topic_;
std::string output_tactics_topic_;                                                // ← 新增

rclcpp::Subscription<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr target_sub_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::Publisher<tensorrt_detect_msgs::msg::RadarMap>::SharedPtr radar_map_pub_;
rclcpp::Publisher<tensorrt_detect_msgs::msg::MapTactics>::SharedPtr tactics_pub_;  // ← 新增
std::unique_ptr<MapAnalyzer> analyzer_;                                             // ← 新增
```

---

# 五、完整数据流回顾

```text
pose_node ──/world_targets──→ map_node
                                      │
                                      ├── MapAnalyzer（战术态势分析）
                                      │     └── /map_tactics ──→ 决策节点
                                      │
                                      ├── RadarMap + drawMap
                                      │     ├── /radar_map ──→ 通信/决策节点
                                      │     └── /map_image ──→ display_node
                                      │
                                      └── 前哨站叠加绘制
```

输出的三个话题分别服务不同消费者：

| 话题 | 消费者 | 用途 |
|------|--------|------|
| `/map_image` | display_node / qt_display_node | 人类操作员看地图 |
| `/radar_map` | 通信节点 / 决策节点 | 机器读取敌方位置 |
| `/map_tactics` | 决策节点 | 机器读取战术态势 |

---

# 六、从这份代码里学到的设计要点

## 1. 固定索引输入的适配

`pose_node` 使用固定槽位输出后，`map_node` 的输入格式从"动态列表"变成了"固定索引数组"。`map_node` 通过索引直接访问前哨站（索引 10），而不是遍历查找。

## 2. 战术分析与渲染分离

`MapAnalyzer` 只做数值计算，`map_node` 负责日志和消息发布。这种分离让 `MapAnalyzer` 可以被独立测试。

## 3. 三路输出的信息层次

* `/map_image`：最丰富的信息（位置 + 颜色 + 标签 + 前哨站状态），但人类才能理解
* `/radar_map`：结构化位置数据，机器可直接读取
* `/map_tactics`：最高层的语义浓缩（4 个整数覆盖全场态势）

从下到上，信息从具体到抽象，数据量从大到小。

## 4. 阵营感知的同步

`RadarMap`（地图翻转）和 `MapAnalyzer`（阵营归属）共享同一个 `flip_team_` 状态。当用户切换视角时，两者同步更新，避免地图翻转了但战术判断还在用旧阵营的错误。
