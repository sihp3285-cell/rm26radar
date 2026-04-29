# `pose_node.cpp` 逐行讲解

这份文件是**位姿解算节点**，在你当前 ROS2 视觉链路里的位置是：

```text
video_node
   ↓  /image_raw
detect_node
   ↓  /armor_detections  ← pose_node 订阅这里
pose_node  ← 你在这里
   ↓  /world_targets      ← pose_node 发布这里
map_node
   ↓  /map_image, /radar_map
display_node
```

`pose_node` 是系统的**几何转换层**。它负责把像素世界（图像坐标系）翻译成物理世界（场地坐标系）。

具体职责：

1. 订阅 `detect_node` 输出的结构化检测结果 `/armor_detections`
2. 用相机内参、外参、畸变系数，把每个目标的像素坐标解算成世界坐标
3. 优先使用车辆底部中心（更稳定），回退到装甲板中心
4. 发布 `WorldTargetArray` 给 `map_node` 绘制小地图

在 ROS2 理论中，这种节点属于 **Processing Node（处理节点）**：

> 它不直接和硬件交互，而是订阅上游的算法输出，经过数学变换后，产生新的语义数据供下游消费。

---

# 一、头文件部分

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "tensorrt_detect_msgs/msg/world_target_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_array.hpp"
#include "tensorrt_detect_msgs/msg/detection_box.hpp"
#include "ConfigManager.hpp"
#include "posesolver.hpp"
```

---

### `#include <rclcpp/rclcpp.hpp>`

ROS2 C++ 节点基础头文件。

---

### `#include <sensor_msgs/msg/image.hpp>` / `#include <std_msgs/msg/header.hpp>`

这个节点本身不传输完整图像，但 `header` 会被继承。`std_msgs::msg::Header` 是 ROS2 所有消息的时间戳和坐标系载体。

---

### `#include <opencv2/opencv.hpp>`

位姿解算底层依赖 OpenCV 的相机模型和几何变换：

* `cv::Mat`：存储相机内参矩阵 `K`、畸变系数 `D`、旋转矩阵 `R`、平移向量 `T`
* `cv::Rect`：检测框数据结构

---

### `#include <yaml-cpp/yaml.h>` / `#include <filesystem>`

用于加载标定结果文件 `calib_result.yaml`。当 `Config` 里没有有效标定数据时，节点会尝试从磁盘文件加载外参。

这体现了**配置系统的降级策略**：优先用内存配置，fallback 到文件系统。

---

### `#include "tensorrt_detect_msgs/msg/..."`

这个节点同时涉及三种自定义消息：

| 消息类型 | 方向 | 用途 |
|---------|------|------|
| `DetectionArray` / `DetectionBox` | 输入 | 接收检测框像素坐标 |
| `WorldTargetArray` / `WorldTarget` | 输出 | 发布世界坐标目标 |

---

### `#include "ConfigManager.hpp"`

读取相机参数（内参 `K`、畸变 `D`、世界坐标点 `worldPoints`）和标定结果。

---

### `#include "posesolver.hpp"`

核心算法类 `PoseSolver`。它封装了：

* PnP（Perspective-n-Point）解算
* 射线与地面/3D Mesh 的碰撞检测
* 像素坐标 → 世界坐标的映射

---

# 二、定义节点类

```cpp
class PoseNode : public rclcpp::Node
```

固定起手式。

---

# 三、构造函数

```cpp
public:
    PoseNode() : Node("pose_node")
```

节点注册名 `"pose_node"`。

---

# 四、声明参数

```cpp
this->declare_parameter<std::string>("config_dir",
    "/home/delphine/rm/tensorrt10_detect/configs");
this->declare_parameter<std::string>("input_topic", "/armor_detections");
this->declare_parameter<std::string>("output_topic", "/world_targets");
```

---

### `config_dir`

配置目录。PoseNode 需要读取 `camera.yaml`（内参、畸变、3D 标定点）和可能的 `calib_result.yaml`（外参 `R` 和 `T`）。

---

### `input_topic`

默认 `/armor_detections`。订阅 `detect_node` 发布的结构化检测结果。

---

### `output_topic`

默认 `/world_targets`。发布解算后的世界坐标目标数组。

---

# 五、读取参数并打印

```cpp
std::string config_dir = this->get_parameter("config_dir").as_string();
input_topic_ = this->get_parameter("input_topic").as_string();
output_topic_ = this->get_parameter("output_topic").as_string();

RCLCPP_INFO(this->get_logger(), "配置目录: %s", config_dir.c_str());
RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());
```

日志打印三个关键配置，方便排查问题。

---

# 六、初始化 Config 和 PoseSolver

```cpp
cfg_ = std::make_unique<Config>(config_dir);
pose_solver_ = std::make_unique<PoseSolver>(cfg_->camera.cameraMatrix, cfg_->camera.distCoeffs);
```

---

### `Config`

加载所有配置文件。其中 `camera.yaml` 里包含了：

* `cameraMatrix`：3×3 相机内参矩阵 `K`
* `distCoeffs`：畸变系数向量 `D`
* `worldPoints`：用于 PnP 的 3D 世界坐标点
* `meshPath`：场地 3D 网格模型路径（用于射线碰撞）

---

### `PoseSolver`

用内参和畸变初始化。此时它只知道"相机怎么看世界"，但还不知道"相机在世界中的位姿"（外参）。

外参需要在下一步加载。

---

# 七、加载外参（Extrinsic Parameters）

```cpp
if (cfg_->calib.valid) {
    pose_solver_->setExtrinsic(cfg_->calib.R, cfg_->calib.T);
    RCLCPP_INFO(this->get_logger(), "成功从 Config 加载校准结果，已设置外参");
} else {
    // ... 尝试从 calib_result.yaml 加载
}
```

这段代码展示了一个非常实用的**配置降级（Fallback）策略**。

---

## 7.1 第一优先级：Config 内存中的标定结果

如果 `camera.yaml` 里已经包含了有效的 `R` 和 `T`，直接用它。

---

## 7.2 第二优先级：calib_result.yaml 文件

```cpp
std::filesystem::path configDir = std::filesystem::path(config_dir);
std::string calibPath = (configDir / "calib_result.yaml").string();
if (std::filesystem::exists(calibPath)) {
    try {
        YAML::Node node = YAML::LoadFile(calibPath);
        if (node["r"].IsSequence() && node["t"].IsSequence()) {
            std::vector<double> r_data = node["r"].as<std::vector<double>>();
            std::vector<double> t_data = node["t"].as<std::vector<double>>();
            if (r_data.size() == 9 && t_data.size() == 3) {
                cv::Mat R(3, 3, CV_64F);
                cv::Mat T(3, 1, CV_64F);
                for (int i = 0; i < 9; ++i) {
                    R.at<double>(i / 3, i % 3) = r_data[i];
                }
                for (int i = 0; i < 3; ++i) {
                    T.at<double>(i, 0) = t_data[i];
                }
                pose_solver_->setExtrinsic(R, T);
                RCLCPP_INFO(...);
            }
        }
    } catch (...) { ... }
}
```

---

### 为什么要做 fallback？

因为在实际工程中，标定数据可能来自两个渠道：

1. **预置配置**：部署前已经标定好，直接写在 `camera.yaml` 里
2. **现场标定**：到了比赛场地重新标定，结果存到 `calib_result.yaml`

现场标定的结果通常比预置的更准，所以优先尝试内存配置，如果没有再读文件。

---

### 数据验证

代码里有多层验证：

* 文件是否存在
* YAML 节点是否是序列
* `r` 是否有 9 个元素（3×3 旋转矩阵）
* `t` 是否有 3 个元素（3×1 平移向量）

这体现了**防御式编程**：外参错了，后面所有世界坐标都会错，所以加载时必须严格校验。

---

# 八、加载 3D Mesh（用于射线碰撞）

```cpp
if (!cfg_->camera.meshPath.empty()) {
    bool mesh_ok = pose_solver_->getRaycaster().loadingMesh(cfg_->camera.meshPath);
    if (mesh_ok) {
        RCLCPP_INFO(this->get_logger(), "成功加载 3D 网格: %s", ...);
    } else {
        RCLCPP_WARN(this->get_logger(), "加载 3D 网格失败...将使用平地 fallback");
    }
} else {
    RCLCPP_WARN(this->get_logger(), "未配置 meshPath，将使用平地 fallback");
}
```

---

### 射线碰撞（Ray Casting）是什么？

从相机光心出发，穿过图像上的像素点，形成一条射线。这条射线会和场地地面（或 3D 模型）相交，交点就是目标在世界坐标系中的位置。

如果加载了场地 3D Mesh（比如场地起伏、障碍物、坡道），射线可以和真实地形碰撞，得到更准确的高度和位置。

如果没有 Mesh，就回退到**平地假设**（`z = 0`）。

---

### 为什么用 fallback？

3D Mesh 文件可能很大，加载耗时，或者文件路径配置错了。节点不能因为 Mesh 加载失败就退出，因为平地假设在很多场景下已经足够用了。

---

# 九、创建 Publisher 和 Subscriber

```cpp
world_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::WorldTargetArray>(output_topic_, 10);

armor_sub_ = this->create_subscription<tensorrt_detect_msgs::msg::DetectionArray>(
    input_topic_, 10,
    std::bind(&PoseNode::armor_callback, this, std::placeholders::_1));
```

---

### Publisher

* 类型：`WorldTargetArray`
* 话题：`/world_targets`
* 队列深度：10

发布解算后的世界坐标目标。

---

### Subscriber

* 类型：`DetectionArray`
* 话题：`/armor_detections`
* 回调：`armor_callback`

订阅结构化检测结果，每收到一帧检测数据，就触发一次解算。

---

# 十、构造函数完成日志

```cpp
RCLCPP_INFO(this->get_logger(), "PoseNode 初始化完成，等待检测结果输入...");
```

外参加载、Mesh 加载、Pub/Sub 创建都完成后，节点进入待机状态。

---

# 十一、核心回调：`armor_callback`

```cpp
private:
    void armor_callback(const tensorrt_detect_msgs::msg::DetectionArray::SharedPtr msg)
```

这是 `pose_node` 的干活函数。每收到一帧 `DetectionArray`，就执行一次。

---

# 十二、构建输出消息

```cpp
auto world_msg = std::make_shared<tensorrt_detect_msgs::msg::WorldTargetArray>();
world_msg->header = msg->header;
```

---

### Header 继承

直接把输入消息的 `header` 复制到输出消息。

这非常重要：

* 时间戳 `stamp` 被保留 → 下游 `map_node` 知道这批世界坐标对应的是哪一帧图像
* `frame_id` 被保留 → 坐标系信息不丢失

这就是 ROS2 的 **消息血缘传递**。

---

# 十三、遍历每个检测目标

```cpp
for (const auto& det : msg->detections) {
    tensorrt_detect_msgs::msg::WorldTarget target;
    target.idx = det.idx;
    target.class_id = det.idx;
    target.is_dead = det.is_dead;
    target.score = det.confidence;
    target.valid = true;
    target.bbox_x = det.x;
    target.bbox_y = det.y;
    target.bbox_w = det.width;
    target.bbox_h = det.height;

    if (det.is_dead) {
        target.team_id = robot_id::UNKNOWN;
    } else {
        target.team_id = det.armor_color;
    }
```

先把 `DetectionBox` 里的基础字段复制到 `WorldTarget`。这些字段不经过任何变换，原样传递。

---

### `is_dead` 的处理

```cpp
    target.is_dead = det.is_dead;
```

死亡状态从上游 `detect_node` 原样传递，**不经过任何变换**。

---

### `team_id` 的赋值逻辑

```cpp
    if (det.is_dead) {
        target.team_id = robot_id::UNKNOWN;
    } else {
        target.team_id = det.armor_color;
    }
```

* **死亡装甲板**（`is_dead == true`）：`team_id` 显式设为 `UNKNOWN`（0）。死亡车辆没有队伍归属。
* **存活装甲板**（`is_dead == false`）：`team_id` 从 `armor_color` 复制，表示红方（1）或蓝方（2）。

> 原始代码直接写 `target.team_id = det.armor_color`，导致死亡装甲板（`armor_color` 被设为 0）的 `team_id` 也是 0。虽然结果一样，但语义不清晰。现在通过显式的 `if/else` 区分，下游节点一看就知道 `UNKNOWN` 可能是因为死亡，也可能是因为颜色未识别。

---

# 十四、像素坐标 → 世界坐标（核心算法）

```cpp
// 优先使用 car_box 底部中心进行世界坐标解算
cv::Rect car_box(det.car_x, det.car_y, det.car_width, det.car_height);
if (car_box.width > 0 && car_box.height > 0) {
    cv::Point2f world_pos = pose_solver_->middletoworld(car_box);
    target.world_x = world_pos.x;
    target.world_y = 0.0f;
    target.world_z = world_pos.y;
} else {
    // 若 car_box 无效，回退到 armor box
    cv::Rect armor_box(det.x, det.y, det.width, det.height);
    cv::Point2f world_pos = pose_solver_->middletoworld(armor_box);
    target.world_x = world_pos.x;
    target.world_y = 0.0f;
    target.world_z = world_pos.y;
}
```

这是整个节点**最有算法含量**的一段。

---

### 为什么优先用 `car_box`？

在 RoboMaster 场景中，一辆车可能有多个装甲板。用装甲板中心做世界坐标解算，会因为装甲板位置不同（车头、车身、车尾）导致同一辆车的坐标跳动。

而 `car_box` 是整辆车的检测框，用它的**底部中心**做解算，能更好地代表车辆在地面上的真实位置，结果更稳定。

---

### `middletoworld` 内部在做什么？

虽然代码没有展开，但它的数学过程大致是：

1. **去畸变**：用畸变系数 `D` 把像素坐标矫正成理想 pinhole 坐标
2. **反向投影**：用内参矩阵 `K` 的逆，把像素坐标转成归一化平面上的方向向量
3. **坐标系变换**：用外参 `R` 和 `T`，把相机坐标系下的方向向量转到世界坐标系
4. **射线碰撞**：从相机光心沿方向向量发射射线，与地面（或 3D Mesh）求交点
5. **返回交点**：这就是目标在世界坐标系中的 `(x, z)` 位置

---

### `world_y = 0.0f`

`PoseSolver` 返回的是 `(x, z)` 二维坐标（场地平面坐标）。`world_y` 被硬编码为 `0.0f`，表示目标在地面上。

如果以后加载了真实 3D Mesh，`middletoworld` 内部可能会根据地形起伏返回不同高度，这里可以扩展为接收真实 `y` 值。

---

### 坐标系约定

注意这里的坐标映射：

```cpp
target.world_x = world_pos.x;   // 场地宽度方向（X轴）
target.world_z = world_pos.y;   // 场地长度方向（Z轴）
```

`PoseSolver` 返回的 `cv::Point2f` 中，`x` 对应场地宽，`y` 对应场地长。这是项目内部的坐标约定，`map_node` 在消费这个数据时也要遵循同一套约定。

---

# 十五、发布结果

```cpp
world_pub_->publish(*world_msg);
```

把填充好的 `WorldTargetArray` 发到 `/world_targets`。

`map_node` 订阅这个话题后，就能拿到每个目标在世界坐标系中的位置，进而绘制到小地图上。

---

# 十六、节流日志

```cpp
RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    10000,
    "接收到 %zu 个检测，发布了 %zu 个世界坐标目标",
    msg->detections.size(), world_msg->targets.size());
```

每 10 秒打印一次处理统计：

* 输入多少检测框
* 输出多少世界坐标目标

如果这两个数字不一致，说明有些检测框因为 `car_box` 无效等原因被丢弃了。

---

# 十七、异常处理

```cpp
catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "姿态解算回调异常: %s", e.what());
}
```

回调里的任何标准异常都被拦截，节点不崩。

---

# 十八、成员变量

```cpp
std::unique_ptr<Config> cfg_;
std::unique_ptr<PoseSolver> pose_solver_;

std::string input_topic_;
std::string output_topic_;

rclcpp::Subscription<tensorrt_detect_msgs::msg::DetectionArray>::SharedPtr armor_sub_;
rclcpp::Publisher<tensorrt_detect_msgs::msg::WorldTargetArray>::SharedPtr world_pub_;
```

---

### `cfg_`

保存配置对象。包含相机参数、标定结果、模型配置等。

---

### `pose_solver_`

位姿解算器。持有相机模型、外参、射线碰撞器。是节点的核心算法对象。

---

# 十九、`main` 函数

```cpp
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

经典四步法。`spin` 负责让 `armor_callback` 在每次 `/armor_detections` 有新消息时被调用。

---

# 二十、完整数据流回顾

```text
detect_node ──/armor_detections──→ pose_node ──/world_targets──→ map_node
```

`pose_node` 是**像素世界到物理世界的桥梁**：

* 输入：`DetectionArray`（像素坐标、检测框、置信度）
* 处理：PnP 解算 + 射线碰撞
* 输出：`WorldTargetArray`（世界坐标、队伍、类别）

下游 `map_node` 不需要知道任何相机参数，也不需要懂 PnP，它只接收纯粹的几何坐标，这正是**节点分工**的价值。

---

# 二十一、从这份代码里学到的设计要点

## 1. 处理节点的定位

`pose_node` 是经典的**纯处理节点**：

* 不碰硬件
* 不显示图像
* 只订阅结构化数据 → 做数学变换 → 发布新的结构化数据

这种节点非常容易单元测试：你甚至可以手写一个 `DetectionArray`，喂给 `PoseNode`，验证输出的 `WorldTargetArray` 坐标是否正确。

## 2. 配置降级策略

```cpp
if (cfg_->calib.valid) {
    // 用内存配置
} else if (文件存在) {
    // 用文件配置
} else {
    // 报错，但继续运行（如果可能的话）
}
```

多层级 fallback 让节点在不同部署环境下都能尽量工作。

## 3. 算法回退策略

```cpp
if (car_box 有效) {
    // 用更稳定的车辆底部中心
} else {
    // 回退到装甲板中心
}
```

优先用高质量数据，降级时也不放弃，用次优数据顶上。

## 4. Header 血缘链

```cpp
world_msg->header = msg->header;
```

输入消息的时间戳原样传递给输出消息。在分布式系统中，这是**数据溯源**和**时间同步**的基础。

## 5. 坐标系约定的文档化

```cpp
target.world_x = world_pos.x;   // 场地 X（宽）
target.world_z = world_pos.y;   // 场地 Z（长）
```

这种约定必须在整个团队内达成一致。`pose_node` 生产 `world_x`/`world_z`，`map_node` 消费 `world_x`/`world_z`，如果双方理解不一致，地图上的点就会错位。

在大型 ROS2 项目中，通常会用 **TF2（Transform Frame）** 来形式化坐标系关系，避免这种"约定靠口头"的问题。
