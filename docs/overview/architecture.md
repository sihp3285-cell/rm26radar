# 系统架构与数据流总览

这份文档从**全局视角**讲解你当前 ROS2 视觉检测系统的架构设计。不会逐行抠代码，而是侧重：

* 为什么拆成多个节点？
* 话题之间怎么配合？
* 自定义消息为什么这样设计？
* Launch 文件怎么把节点串起来？
* 这套架构体现了哪些 ROS2 核心思想？

---

# 一、完整节点拓扑图

```text
┌─────────────┐     /image_raw      ┌─────────────┐
│  video_node │ ──────────────────→ │ detect_node │
│  (传感器层)  │                     │  (算法核心层) │
└─────────────┘                     └──────┬──────┘
    ↑      ↑                               │
    │      │      /video_node/set_pause     │
    │      │           (服务)               │
    │   ┌──┴─────────┐                     │
    │   │calibrate_node│                   │
    │   │  (标定控制层) │                   │
    │   └─────────────┘                   │
    │      ↑                               │
    │      │      /calibration/start       │
    │      │           (服务)               │
    │   ┌──┴─────────┐                     │
    └──→│  roi_set_node│ ←─────────────────┤
        │ (ROI 控制层) │   /detect_node/     │
        └─────────────┘    reload_roi       │
                              (服务)        │
                          ┌────────────────┼────────────────┐
                          │                │                │
                          ↓                ↓                ↓
                   /detected_image   /armor_detections      │
                          │                │                │
                          ↓                ↓                │
                   ┌─────────────┐   ┌─────────────┐       │
                   │ qt_display  │   │  pose_node  │       │
                   │   _node     │   │(几何变换+跟踪)│       │
                   └─────────────┘   │ PoseSolver  │       │
                          ↑          │ Tracker     │       │
                          │          └──────┬──────┘       │
                   /flip_team               │              │
                          │                 ↓              │
                          │          /world_targets        │
                          │                 │              │
                          │                 ↓              │
                          │          ┌─────────────┐       │
                          │          │   map_node  │       │
                          │          │(汇聚+战术分析)│       │
                          │          │ RadarMap    │       │
                          │          │ MapAnalyzer │       │
                          │          └──────┬──────┘       │
                          │                 │              │
                          └─────────────────┼──────────────┘
                                            │
                     ┌──────────────────────┼──────────────────────┐
                     │                      │                      │
                     ↓                      ↓                      ↓
              /map_image              /radar_map            /map_tactics
                     │                      │                      │
                     ↓                      ↓                      ↓
              ┌─────────────┐       ┌─────────────┐       ┌─────────────┐
              │ qt_display  │       │ (决策节点)   │       │ (决策节点)   │
              │   _node     │       │             │       │             │
              └─────────────┘       └─────────────┘       └─────────────┘
```

> 注：`display_node`（OpenCV 版本）同样订阅 `/detected_image` 与 `/map_image`，提供水平拼接的显示窗口。Launch 文件中默认不启动，可手动运行。

---

# 二、分层架构设计思想

你的系统可以自然地分成五层：

## 2.1 传感器层（Sensor Layer）

**节点**：`video_node`

**职责**：产生原始数据。

* 打开视频文件 / 相机设备
* 按固定帧率发布图像
* 给每帧图像打上时间戳和 `frame_id`

在 ROS2 中，传感器节点的特点是：

> **只有 Publisher，没有 Subscriber。** 它是整个数据流的起点。

以后你把 `video_node` 换成真正的 `camera_node`（接工业相机），这一层以下的所有节点**完全不需要改动**。这就是 ROS2 **话题解耦**的价值。

---

## 2.2 算法核心层（Algorithm Core Layer）

**节点**：`detect_node`

**职责**：接收原始图像，输出算法结果。

* 跑 TensorRT 检测模型
* 同时输出两路数据：
  * `/detected_image`：带框的可视化图像（供人看）
  * `/armor_detections`：结构化检测数据（供机器下游用）

这是系统的**分水岭节点**。上游只关心图像输入， downstream 分成两条独立链路，互不干扰。

---

## 2.3 几何变换 + 跟踪层（Geometric Transform + Tracking Layer）

**节点**：`pose_node`

**职责**：像素坐标 → 世界坐标 + 多目标跟踪 + Kalman 平滑。

* 接收 `DetectionArray`（像素坐标 + 检测框）
* 用相机内参、外参、PnP 解算、射线碰撞，得到世界坐标
* **通过固定槽位 Tracker 做多目标跟踪和 Kalman 平滑**
* **三路分流处理**：
  * 前哨站（Outpost）→ 直接透传（索引 10）
  * 死亡装甲板 → 动态追加（索引 11+）
  * 正常装甲板 → 进入 Tracker 固定槽位（索引 0~9）
* 发布 `WorldTargetArray`（固定槽位 + Outpost + 死亡装甲板）

`pose_node` 内部包含 `PoseSolver`（几何变换）和 `Tracker`（跟踪平滑）两个核心模块。Tracker 使用 `KalmanFilterBox`（像素框平滑）、`KalmanFilter2d`（世界坐标平滑）和 `HungarianAlgorithm`（贪心数据关联），为每个兵种维护一个固定槽位，消除 ID 跳变和坐标抖动。

> 你可以手写一个 `DetectionArray`，喂给 `pose_node`，验证输出的世界坐标和跟踪结果是否等于预期值。

---

## 2.4 汇聚输出 + 战术分析层（Aggregation + Tactical Analysis Layer）

**节点**：`map_node`

**职责**：接收世界坐标，生成最终输出，分析战术态势。

* 把世界坐标转成地图像素坐标
* 绘制小地图图像 → `/map_image`（含前哨站存活/摧毁状态叠加）
* 整理成结构化 `RadarMap` → `/radar_map`
* **通过 `MapAnalyzer` 分析战术态势** → `/map_tactics`
  * 工程上岛检测
  * 攻防态势判断（大攻）
  * 堡垒威胁检测

这个节点有三个消费者：

* `display_node`：看 `/map_image`
* 决策/通信节点：读 `/radar_map`，把敌方位置发给下位机或队友
* 决策节点：读 `/map_tactics`，获取战术层面的浓缩信息

---

## 2.5 标定控制层（Calibration Control Layer）

**节点**：`calibrate_node`

**职责**：相机外参的手动标定与自动标定。

* 启动时自动检测 `calib_result.yaml` 是否有效
* 无效时自动进入手动标定流程（临时订阅 `/image_raw` 获取一帧，弹出 OpenCV 交互窗口收集标定点）
* 计算 PnP 外参和重投影误差，误差过大时强制重新标定
* 标定结果保存到 `calib_result.yaml` 后，通过 `/pose_node/reload_calibration` 服务通知 `pose_node` 重新加载
* 标定过程中通过 `/video_node/set_pause` 服务暂停/恢复视频，避免标定窗口弹出时视频继续播放
* 同时提供 `/calibration/start` 服务，支持用户手动触发重新标定

---

## 2.6 ROI 控制层（ROI Control Layer）

**节点**：`roi_set_node`

**职责**：前哨站 ROI 的自动检测与手动框定。

* 启动时自动检测 `outpost_roi.yaml` 是否有效
* 无效时自动进入 ROI 框定流程（临时订阅 `/image_raw` 获取一帧，弹出 OpenCV 交互窗口两点框选）
* 若相机标定无效，先自动触发 `/calibration/start` 完成标定
* 框定过程中通过 `/video_node/set_pause` 服务暂停/恢复视频
* 保存到 `outpost_roi.yaml`（保留原有其他字段，仅更新 `outpost_roi`）
* 通过 `/detect_node/reload_roi` 服务通知 `detect_node` 重载配置
* 提供 `/roi_set/start` 服务，支持用户手动触发重新框定

---

## 2.7 可视化末端层（Visualization Layer）

**节点**：`qt_display_node`

**职责**：把图像数据呈现给人类，并提供交互控制。

| 节点 | 技术栈 | 订阅话题 | 特点 |
|------|--------|---------|------|
| `qt_display_node` | Qt5 + ROS2 | `/detected_image`、`/map_image` | 界面美观、支持 Qt 样式表、顶部状态栏、阵营切换按钮 |
| `display_node` | OpenCV + ROS2 | `/detected_image`、`/map_image` | 轻量、水平拼接视频与小地图，按 Q/ESC 退出 |

`qt_display_node` 不仅是消费者，还发布 `/flip_team`（`std_msgs/Bool`）话题，用于控制 `map_node` 的红蓝方阵营视角翻转。这是系统中**唯一的人机交互入口**。

---

# 三、话题设计详解

## 3.1 话题一览表

| 话题/服务名 | 类型 | 发布者/服务器 | 订阅者/客户端 | 作用 |
|------------|------|--------------|--------------|------|
| `/image_raw` | `sensor_msgs/Image` | video_node | detect_node / calibrate_node | 原始图像 |
| `/detected_image` | `sensor_msgs/Image` | detect_node | qt_display_node、display_node | 带检测框的可视化图 |
| `/armor_detections` | `DetectionArray` | detect_node | pose_node | 结构化检测结果 |
| `/world_targets` | `WorldTargetArray` | pose_node | map_node | 世界坐标目标 |
| `/map_image` | `sensor_msgs/Image` | map_node | qt_display_node、display_node | 小地图图像 |
| `/radar_map` | `RadarMap` | map_node | (决策节点) | 结构化雷达数据 |
| `/map_tactics` | `MapTactics` | map_node | (决策节点) | 战术态势分析数据 |
| `/pipeline_timing` | `PipelineTiming` | detect_node | (监控工具) | 流水线各阶段耗时分解 |
| `/flip_team` | `std_msgs/Bool` | qt_display_node | map_node | 红蓝方阵营视角切换 |
| `/calibration/start` | `std_srvs/Trigger` (Service) | calibrate_node | (用户/roi_set_node) | 手动触发标定 |
| `/roi_set/start` | `std_srvs/Trigger` (Service) | roi_set_node | (用户/脚本) | 手动触发 ROI 框定 |
| `/detect_node/reload_roi` | `std_srvs/Trigger` (Service) | detect_node | roi_set_node | ROI 更新后通知重载 |
| `/pose_node/reload_calibration` | `std_srvs/Trigger` (Service) | pose_node | calibrate_node | 标定完成后通知重载 |
| `/video_node/set_pause` | `std_srvs/SetBool` (Service) | video_node | calibrate_node / roi_set_node | 暂停/恢复视频播放 |

---

## 3.2 为什么用两个话题发检测结果？

`detect_node` 同时发 `/detected_image` 和 `/armor_detections`。

这体现了 ROS2 中一个经典设计原则：

> **同一批原始数据，用不同消息类型服务不同消费者。**

| 消费者 | 需要的数据形式 | 订阅的话题 |
|--------|--------------|-----------|
| 人类操作员 | 带框的彩色图像 | `/detected_image` |
| pose_node（机器） | 像素坐标、类别、置信度 | `/armor_detections` |

如果只用图像，下游机器要重新做 OCR 或图像解析才能拿到坐标，荒谬且低效。

如果只用结构化消息，人类就无法直观看到检测效果。

所以**双管齐下**是最优解。

---

## 3.3 为什么不用一个超大消息包？

有人可能会想：为什么不把检测框、世界坐标、地图坐标全部打包成一个大消息，从一个节点直接发到另一个节点？

这样设计的坏处：

1. **耦合严重**：改一个字段，所有节点都要重新编译
2. **带宽浪费**：`display_node` 根本不关心世界坐标，却被迫接收
3. **节点无法独立替换**：如果想换一个检测模型，必须同时换后面的解算逻辑
4. **不利于调试**：无法单独测试某个环节

而你的**多节点 + 多话题**架构的好处：

```text
video_node 可以换成 camera_node，detect_node 不用改
detect_node 可以换检测模型，pose_node 不用改
pose_node 可以换标定参数，map_node 不用改
```

这就是 ROS2 **节点即插件（Node as Plugin）** 的思想。

---

# 四、自定义消息设计

你的自定义消息定义在 `tensorrt_detect_msgs` 包里，共 7 种：

## 4.1 `DetectionBox`（单条检测）

```yaml
int32 idx           # 类别 ID
float32 confidence  # 置信度
int32 x, y, width, height      # 装甲板检测框
int32 armor_color   # 装甲颜色 / 队伍 ID
bool is_dead        # 是否被标记为死亡/摧毁
int32 car_x, car_y, car_width, car_height  # 车辆整体检测框
float32 world_x, world_y  # 世界坐标（如果 pipeline 已算）
float32 fps         # 当前帧率
```

**设计要点**：字段足够完整，让下游节点按需取用。

`pose_node` 主要用 `car_x/car_y/car_width/car_height` 做位姿解算，`display_node` 如果以后想显示结构化信息，可以直接读 `confidence` 和 `idx`。

---

## 4.2 `DetectionArray`（检测数组）

```yaml
std_msgs/Header header
DetectionBox[] detections
```

**设计要点**：

* 带 `header`，时间戳可以沿消息链传递
* 变长数组 `[]`，适应每帧检测数量不同的情况

---

## 4.3 `WorldTarget`（单个世界坐标目标）

```yaml
int32 idx, class_id, team_id
float32 score
bool is_dead
bool valid
float32 world_x, world_y, world_z
int32 bbox_x, bbox_y, bbox_w, bbox_h
int32 stable_class_id      # BotIdentity 稳定类别 ID
float32 stable_class_conf  # BotIdentity 归一化置信度
```

**设计要点**：

* `valid` 字段允许 `pose_node` 标记某个目标"解算失败"
* 保留了原始检测框 `bbox_*`，方便回溯
* `world_y` 通常表示高度，目前为 0（平地假设）
* `stable_class_id` / `stable_class_conf` 来自 `BotIdentity` 的跨帧指数加权投票，用于消除分类模型的逐帧抖动。详见 `docs/core_bot_identity.md`

---

## 4.4 `WorldTargetArray`（世界坐标目标数组）

```yaml
std_msgs/Header header
WorldTarget[] targets
```

和 `DetectionArray` 结构对称，形成**输入-输出对**。

---

## 4.5 `RadarMap`（结构化雷达地图）

```yaml
std_msgs/Header header
float32[6] blue_x, blue_y
float32[6] red_x, red_y
```

**设计要点**：

* 用**固定长度数组**而非变长数组
* 索引约定：0~3 对应 R1~R4，5 对应哨兵
* 0 值表示"未检测到"

这种设计的好处是**下游消费极简单**：

```cpp
// 直接数组索引，O(1) 访问
float red3_x = msg.red_x[2];
```

不需要遍历、不需要查表、不需要字符串匹配。

---

## 4.6 `MapTactics`（战术态势）

```yaml
std_msgs/Header header
bool engineer_on_island   # 工程上岛：false=无, true=有
bool opponent_attack      # 对方大攻
bool our_attack           # 我方大攻
bool opponent_near_fortress  # 对方接近堡垒
```

**设计要点**：

* 用 4 个布尔值浓缩全场战术态势
* `map_node` 中的 `MapAnalyzer` 基于规则分析后填充
* 下游决策节点无需自己解析世界坐标，直接读取战术标志
* 信息密度极高：4 个字段覆盖了工程上岛、攻防态势、堡垒威胁

---

## 4.7 `WorldTargetArray` 中的固定索引约定

`pose_node` 发布的 `WorldTargetArray` 使用固定索引：

| 索引 | 内容 | 来源 |
|------|------|------|
| 0~4 | Red R1~R4, Red S | Tracker 固定槽位（Kalman 平滑） |
| 5~9 | Blue R1~R4, Blue S | Tracker 固定槽位（Kalman 平滑） |
| 10 | Outpost（前哨站） | 直接透传（无平滑） |
| 11+ | 死亡装甲板 | 动态追加（无平滑） |

这种固定索引设计让下游 `map_node` 可以直接用下标访问，不需要遍历查找。

---

# 五、Launch 文件详解

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 使用 FindPackageShare 自动定位参数文件，不依赖绝对路径
    params_file = PathJoinSubstitution([
        FindPackageShare('tensorrt_detect'),
        'config',
        'ros2_params.yaml',
    ])

    return LaunchDescription([
        Node(
            package='tensorrt_detect',
            executable='video_node',
            name='video_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='tensorrt_detect',
            executable='detect_node',
            name='detect_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='tensorrt_detect',
            executable='pose_node',
            name='pose_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='tensorrt_detect',
            executable='map_node',
            name='map_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='tensorrt_detect',
            executable='calibrate_node',
            name='calibrate_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='tensorrt_detect',
            executable='qt_display_node',
            name='qt_display_node',
            output='screen',
            parameters=[params_file],
        ),

        # ROI 设置节点：自动检测 outpost_roi 是否为空，为空则自动进入框定
        Node(
            package='tensorrt_detect',
            executable='roi_set_node',
            name='roi_set_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
```

---

## 5.1 Launch 文件的价值

没有 launch 文件时，你需要开 7 个终端，分别运行 7 个节点（若再启动 `display_node`，则需要 8 个）。每个节点还要手动 source ROS2 环境。

有了 launch 文件，一行命令启动整个系统：

```bash
ros2 launch tensorrt_detect detect_pipeline.launch.py
```

---

## 5.2 参数集中配置

launch 文件里，通过 `parameters=[params_file]` 把参数集中到外部 YAML 文件管理，实现参数与代码的完全分离：

```yaml
# ros2_params.yaml
detect_node:
  ros__parameters:
    config_dir: "/home/delphine/rm/tensorrt10_detect/configs"
    input_topic: "/image_raw"
    output_topic: "/detected_image"
    publish_debug_image: true
    debug_output_max_width: 1280
```

这意味着：

* 换视频源 → 改 `ros2_params.yaml` 里的 `video_path`，不用改代码
* 换话题名 → 改 `ros2_params.yaml` 里的 `topic` 参数，不用改代码
* 换配置目录 → 改 `ros2_params.yaml` 里的 `config_dir`，不用改代码
* 改完参数后重新 `colcon build` 即可生效

此外，`FindPackageShare('tensorrt_detect')` 会自动从 `ament` 包索引中找到包的安装位置，无论项目被克隆到哪个目录都能正确加载参数文件，避免了硬编码绝对路径的维护负担。

---

## 5.3 话题名的契约

launch 文件里隐含了一套**话题契约**：

```text
video_node 的 topic_name     = "/image_raw"
                              ↓
detect_node 的 input_topic   = "/image_raw"
detect_node 的 armor topic   = "/armor_detections"
                              ↓
pose_node 的 input_topic     = "/armor_detections"
pose_node 的输出_topic       = "/world_targets"
                              ↓
map_node 的 input_topic      = "/world_targets"
```

只要这套契约保持一致，节点之间就能正确对接。这就是 ROS2 **话题即接口（Topic as API）** 的理念。

---

# 六、ROS2 核心思想在本项目中的体现

## 6.1 节点即进程（Node as Process）

ROS2 中，每个节点通常是一个**独立的操作系统进程**（你的 `CMakeLists.txt` 里每个节点都是独立的 `add_executable`）。

好处：

* 一个节点崩溃，不会影响其他节点
* 每个节点可以独立重启
* 可以分布在不同机器上（比如 `video_node` 在边缘计算盒，`detect_node` 在服务器）

## 6.2 话题解耦（Topic Decoupling）

Publisher 和 Subscriber 之间是**匿名**的：

* `video_node` 不知道谁在订阅 `/image_raw`
* `detect_node` 不知道 `/detected_image` 有没有人在看
* `map_node` 不知道 `/radar_map` 的消费者是否存在

这种匿名性让系统极其灵活。你可以随时启动或停止 `display_node`，上游节点毫无感知。

## 6.3 参数系统（Parameter System）

每个节点都通过 `declare_parameter` / `get_parameter` 管理配置。

在 ROS2 中，参数可以在运行时通过命令行修改：

```bash
ros2 param set /detect_node output_topic /my_custom_topic
```

不过你的节点在构造函数里就读取了参数值，运行时修改不会生效。如果要做动态调参，需要用 `add_on_set_parameters_callback` 注册回调。

## 6.4 消息血缘（Message Lineage）

注意观察这个模式在多个节点中重复出现：

```cpp
output_msg->header = input_msg->header;
```

* `detect_node` 继承 `video_node` 的时间戳
* `pose_node` 继承 `detect_node` 的时间戳
* `map_node` 继承 `pose_node` 的时间戳

这就形成了一条**时间戳血缘链**。如果以后引入多传感器融合（比如雷达 + 视觉），可以用 `message_filters::TimeSynchronizer` 根据 `header.stamp` 对齐不同来源的数据。

## 6.5 QoS（Quality of Service）

当前代码对不同类别的话题采用**分层 QoS 策略**：

### 图像话题（视频流）

所有图像 Pub/Sub 统一使用 `rclcpp::QoS(1)`，即 **Keep Last 1 + Reliable**：

```cpp
// video_node.cpp
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, rclcpp::QoS(1));

// detect_node.cpp
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::QoS(1));
image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(input_topic_, rclcpp::QoS(1), ...);

// display_node.cpp
sub_ = this->create_subscription<sensor_msgs::msg::Image>(topic_, rclcpp::QoS(1), ...);
map_sub_ = this->create_subscription<sensor_msgs::msg::Image>(map_topic_, rclcpp::QoS(1), ...);

// map_node.cpp
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, rclcpp::QoS(1));
```

| 策略 | 配置 | 原因 |
|------|------|------|
| History | Keep Last 1 | 视频流只需要最新帧，旧帧无意义 |
| Depth | 1 | 防止队列积压导致显示延迟 |
| Reliability | Reliable（默认） | 先保留可靠传输，后续如需适应高频丢包网络再改为 Best Effort |

### 结构化数据话题

检测/位姿/地图的结构化消息保持 `depth=10` Reliable：

```cpp
armor_pub_ = this->create_publisher<DetectionArray>("/armor_detections", 10);
world_pub_ = this->create_publisher<WorldTargetArray>(output_topic_, 10);
radar_map_pub_ = this->create_publisher<RadarMap>(output_map_topic_, 10);
```

| 策略 | 配置 | 原因 |
|------|------|------|
| History | Keep Last 10 | 结构化数据量小，允许短暂缓冲 |
| Reliability | Reliable | 检测结果、世界坐标、雷达图数据不允许丢失 |

### 为什么图像用 KeepLast(1)？

对于实时图像显示，队列深度大于 1 是**反作用**的：

> 显示节点只需要看**最新一帧**，积压的旧帧毫无意义。

如果 `detect_node` 发布 30fps，但 `display_node` 因为渲染慢只能处理 20fps，队列深度 10 意味着最多积压约 300ms 的旧帧。Subscriber 会一口气消费这些积压帧，`latest_frame_` 被连续覆盖多次，最终显示的是延迟了好几帧的旧图像，造成"视频越来越滞后，然后突然加速跳帧"的现象。

改成 `QoS(1)` 后，队列只保留最新 1 帧，旧帧直接被 ROS2 丢弃，display_node 永远只处理最新帧，从根源上消除了图像链路的队列积压。

---

# 七、新增模块：BotIdentity 身份稳定化

当前版本新增了 `BotIdentity` 模块，用于解决第三阶段分类模型的**分类抖动**问题。

## 7.1 问题

分类模型对每一帧独立推理，输出的类别 ID 可能逐帧跳变（R3→R4→R3），导致地图上的机器人标签闪烁。

## 7.2 解决方案

`BotIdentity` 为每个 Tracker 槽位维护一个**跨帧身份轨迹池**：

* 保存最近 30 帧的分类历史（`class_id` + `class_conf`）
* 使用指数加权投票（衰减因子 0.95）计算稳定类别
* 自动清除机制：连续丢失 30 帧后清空历史

## 7.3 输出

`Tracker::SlotOutput` 新增两个字段：

| 字段 | 含义 |
|------|------|
| `stable_class_id` | BotIdentity 投票出的稳定类别 ID |
| `stable_class_conf` | 归一化后的稳定置信度 |

详见 `docs/core_bot_identity.md`。

---

# 八、四状态跟踪状态机

Tracker 的状态机从三状态扩展为四状态：

```text
ACTIVE → PREDICTED → LOST → DEAD
```

| 状态 | 含义 | 对外输出 |
|------|------|---------|
| ACTIVE | 正常跟踪 | ✅ valid=true |
| PREDICTED | 短暂丢失，卡尔曼外推 | ✅ valid=true |
| LOST | 丢失较久 | ❌ valid=false |
| DEAD | 未激活/已超时 | ❌ valid=false |

新增的 `PREDICTED` 状态让目标在短暂遮挡（≤2 帧）时仍然显示卡尔曼外推位置，避免地图闪烁。详见 `docs/core_tracker_types.md` 和 `docs/core_tracker.md`。

---

# 九、新增类别：AIRPLANE（无人机）

`robot_id.hpp` 新增 `AIRPLANE = 8`，为 RMUC 2026 赛季的无人机兵种准备。BotIdentity 的 `NUM_CLASSES = 9` 也对应了这个扩展。详见 `docs/core_robot_id.md`。

---

# 十、从 Standalone 到 ROS2 的演进

你的项目里同时保留了两个入口：

* `standard`（`apps/standalone_main.cpp`）：单进程 monolithic 程序
* 八个 ROS2 节点（`src/nodes/*.cpp`）：分布式节点程序

对比两者，可以清晰看到 ROS2 带来的变化：

| 维度 | Standalone | ROS2 节点化 |
|------|-----------|------------|
| 架构 | 单进程，函数调用 | 多进程，话题通信 |
| 耦合 | 高，改一处可能影响全局 | 低，节点间通过话题解耦 |
| 可替换性 | 差，换相机要改 main | 好，换 video_node 即可 |
| 可调试性 | 差，只能整体调试 | 好，可单独测试每个节点 |
| 可视化 | 内置 OpenCV 窗口 | display_node 独立运行 |
| 部署 | 固定配置 | launch 文件灵活配置 |

---

# 十一、性能优化设计

本项目在多个层面进行了系统性的性能优化：

| 层面 | 优化手段 | 收益 |
|------|---------|------|
| 预处理 | GPU CUDA kernel（`preprocess.cu`） | 预处理从 ~3.3ms 降到 ~0.5ms |
| 图像链路 | `toCvShare` 零拷贝 | 每帧省 6MB 拷贝 |
| 可视化 | `publish_debug_image` 开关 | 纯自动模式省 ~3.5ms/帧 |
| 容器 | `reserve` 预分配 | 消除动态扩容 |
| 缓冲区 | `cv::Mat` 成员变量复用 | 消除每帧堆分配 |
| 通信 | QoS(1) 统一图像链路 | 最大延迟从 330ms 降到 33ms |
| 推理 | TensorRT GPU + `enqueueV3` API | 支持动态 batch + 异步执行 |
| 批量推理 | `predictClassBatch` | N 帧分类从 N 次推理降到 1 次 |
| D2H 拷贝 | Pinned Memory（`cudaMallocHost`） | `cudaMemcpyAsync` 真正异步 |
| 算法 | 贪心匹配替代匈牙利 | O(n²) vs O(n³) |
| 状态机 | PREDICTED 状态 | 减少不必要的状态跳变 |
| 多线程安全 | `cuda_guard` 全局互斥锁 | 避免 component_container_mt 崩溃 |
| 性能监控 | `/pipeline_timing` 话题 | 实时分解各阶段耗时，便于调优 |

详见 `docs/performance_optimization.md`。

---

# 十二、卡尔曼滤波器架构

本项目使用两个独立的卡尔曼滤波器 + BotIdentity 构成三维平滑体系：

| 组件 | 平滑对象 | 坐标空间 | 作用 |
|------|---------|---------|------|
| KalmanFilterBox | 检测框 | 像素空间 | 消除框抖动 |
| KalmanFilter2d | 世界坐标 | 世界空间 | 消除坐标抖动 |
| BotIdentity | 类别 ID | 语义空间 | 消除分类跳变 |

详见 `docs/kalman_architecture.md`。

---

# 十三、扩展方向

基于你当前的架构，未来可以很方便地扩展：

## 13.1 Tracker 已内置（固定槽位多目标跟踪）

> **注意：Tracker 已在当前版本中实现，内置于 `pose_node` 内部。**

`pose_node` 内部集成了固定槽位 Tracker，包含：

* `KalmanFilterBox`：像素框 8 维卡尔曼平滑（位置 + 速度）
* `KalmanFilter2d`：世界坐标 4 维卡尔曼平滑
* `HungarianAlgorithm`：贪心数据关联（10×N 代价矩阵）

每个兵种（Red R1~R4, S, Blue R1~R4, S）预分配一个永久槽位，`team_id` 和 `class_id` 在构造时"焊死"，从根源上消除 ID 跳变。

如果未来需要更高级的跟踪算法（如 DeepSORT 的外观特征匹配），可以在 Tracker 框架内替换 `HungarianAlgorithm` 的 `Solve` 实现，接口不变。

详见 `docs/core_tracker.md`、`docs/core_kalman.md`、`docs/core_hungarian.md`。

## 13.2 加入决策节点

```text
map_node ──/radar_map──→ decision_node ──/cmd_vel──→ 下位机
```

`decision_node` 订阅 `/radar_map`，根据敌方位置计算最优移动策略，发布控制指令。

## 13.3 多相机融合

```text
camera_node_1 ──/image_raw_1──→ detect_node_1 ──┐
                                                  ├── fusion_node ── ...
camera_node_2 ──/image_raw_2──→ detect_node_2 ──┘
```

多个相机从不同角度拍摄，`fusion_node` 做数据融合，消除遮挡盲区。

## 13.4 录制与回放

```bash
# 录制所有话题
ros2 bag record -a

# 回放时，不需要 video_node，bag 直接发布 /image_raw
ros2 bag play my_bag
```

因为数据流完全通过话题，`ros2 bag` 可以无缝录制和回放整个系统，方便离线调试算法。

## 13.5 标定流程的自动化扩展

当前 `calibrate_node` 已支持：

* 启动时自动检测标定文件有效性
* 手动 `/calibration/start` 触发标定
* 重投影误差自动校验（阈值 10px）
* 标定完成后自动 reload `pose_node`
* 标定期间自动暂停/恢复视频

未来可扩展：

* 自动标定点检测：用角点检测或 ArUco 标定板替代手动点击
* 多相机联合标定：同时标定多个相机的相对位姿
* 在线重标定：检测到重投影误差漂移时自动触发重新标定

---

# 十四、总结

你的系统架构遵循了 ROS2 的最佳实践：

1. **单一职责**：每个节点只做一件事
2. **话题解耦**：节点通过匿名话题通信，互不依赖
3. **多路输出**：同一批数据用不同形式服务不同消费者
4. **Header 传递**：时间戳沿消息链继承，支持时序对齐
5. **参数化配置**：launch 文件集中管理，无需重新编译
6. **异常隔离**：try/catch 保护回调，单帧失败不崩节点
7. **固定槽位跟踪**：每个兵种一个永久槽位，消除 ID 管理复杂度
8. **战术态势浓缩**：从原始世界坐标到 4 个战术标志的语义蒸馏
9. **四状态跟踪状态机**：ACTIVE/PREDICTED/LOST/DEAD 精细控制输出时机
10. **BotIdentity 身份稳定化**：跨帧指数加权投票消除分类抖动
11. **零拷贝图像链路**：`toCvShare` + `publish_debug_image` 开关大幅降低延迟
12. **三维平滑体系**：像素空间 + 世界空间 + 语义空间的全面平滑

这套架构不是过度设计，而是为未来的扩展、替换、调试、部署打下了坚实基础。
