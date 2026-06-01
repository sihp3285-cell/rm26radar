# ROS2 节点系统 (ROS2 Node Architecture)

## 1. 节点架构概览

本项目使用 **7 个 ROS2 节点**，分为两个部署层级：

### 第一层：核心流水线 (ComposableNodeContainer 单进程内)

```
┌───────────────────────────────────────────┐
│    component_container (单线程)             │
│                                            │
│    VideoNode ──→ DetectNode ──→ PoseNode   │
│       │               │              │       │
│       │               │              └──→ MapNode
│       │               │                      │
│       ▼               ▼                      ▼
│  /image_raw    /armor_detections    /world_targets
│               /detected_image      /map_image
│               /pipeline_timing     /radar_map
│                                    /map_tactics
└───────────────────────────────────────────┘
```

### 第二层：独立进程 (各自有事件循环)

```
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│calibrate_node│  │qt_display_nod│  │ roi_set_node │
│  (OpenCV     │  │  (Qt main    │  │  (OpenCV     │
│   highgui)   │  │   loop +     │  │   highgui)   │
│              │  │   OpenGL)    │  │              │
└──────────────┘  └──────────────┘  └──────────────┘
```

**关键设计决策**：

1. **单线程容器 (component_container)** 非多线程容器：避免 TensorRT enqueueV3、Open3D CastRays、CUDA kernel launch 等多 CUDA 操作并发导致的 SIGSEGV
2. **Intra-process 零拷贝通信**：4 个核心节点使用 `use_intra_process_comms: True`，大帧数据（图像）通过共享指针传递，无需序列化/反序列化
3. **独立进程** 用于有独立事件循环的节点（Qt/OpenCV），保证 UI 响应不阻塞推理流水线

---

## 2. 节点详解

### 2.1 VideoNode — 视频帧读取

[video_node.cpp](../src/tensorrt_detect/src/nodes/video_node.cpp)

```
功能:
  · 从视频文件读取帧
  · 帧率控制 (可配置)
  · 视频循环播放
  · 暂停/恢复服务

发布话题:
  /image_raw → sensor_msgs/Image (bgr8)

服务:
  /video_node/set_pause → std_srvs/SetBool

实现细节:
  · 独立捕获线程: 使用 std::thread + CV 同步
  · 帧间隔补偿: 如果读取/编码延迟，自动跳帧保持目标 FPS
  · 循环: 到达视频末尾后自动 seek 到开头
```

**帧率控制**：

```cpp
// 独立捕获线程策略
while (running) {
    if (!paused) {
        cap >> frame;
        // 计算实际间隔与目标间隔的差值
        auto elapsed = now - last_frame_time;
        auto target = 1000ms / fps;
        if (elapsed < target) {
            sleep(target - elapsed);  // 补足到目标帧率
        } else if (elapsed > target * 2) {
            // 延迟过大 → 跳过中间帧
            for (int skip = 0; skip < elapsed/target - 1; ++skip)
                cap.grab();  // 仅 grab，不解码
            cap.retrieve(frame);
        }
        publish(frame);
    }
}
```

### 2.2 DetectNode — 4 阶段检测

[detect_node.cpp](../src/tensorrt_detect/src/nodes/detect_node.cpp)

```
功能:
  · 接收原始图像
  · 运行 DetectPipeline → 4 阶段检测
  · 发布检测结果 + 耗时统计
  · 发布调试标注图像 (可选)

订阅话题:
  /image_raw → sensor_msgs/Image

发布话题:
  /detected_image → sensor_msgs/Image (调试图像，带 BBox 标注)
  /armor_detections → DetectionArray (检测结果)
  /pipeline_timing → PipelineTiming (各阶段耗时)

服务:
  /detect_node/reload_roi → std_srvs/Trigger (重新加载前哨站 ROI)
```

**初始化流程**：

```cpp
DetectNode::DetectNode() {
    // 1. CUDA primary context 初始化 (防御性)
    cudaFree(0);

    // 2. 加载配置
    cfg_ = std::make_unique<Config>(config_dir);
    // 配置文件中加载 model paths, thresholds, class names, ROI 等

    // 3. 创建推理流水线
    // 构造函数内部加载 4 个 TensorRT engine
    // 并启动异步无人机检测线程
    pipeline_ = std::make_unique<DetectPipeline>(*cfg_);

    // 4. 创建 ROS2 通信接口
    image_sub_ = create_subscription(..., &image_callback);
    armor_pub_ = create_publisher<DetectionArray>(...);
    timing_pub_ = create_publisher<PipelineTiming>(...);
}
```

**回调主循环**：

```cpp
void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 1. cv_bridge 转换 ROS → OpenCV
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    // 2. 运行 4 阶段流水线 (GPU 推理)
    std::vector<Result> results = pipeline_->process(frame);

    // 3. 构建 DetectionArray 消息 (包括前哨站心跳)
    auto armor_msg = std::make_unique<DetectionArray>();
    for (const auto& res : results) {
        DetectionBox box;
        box.idx = res.idx;         // class_id
        box.confidence = res.confidence;
        box.x/y/width/height = res.box;  // 像素框
        box.armor_color = res.armorColor;
        box.is_dead = res.isDead;
        box.car_x/y/width/height = res.car_box;  // 关联的车辆框
        armor_msg->detections.push_back(box);
    }
    // 前哨站未检测到时追加状态消息
    if (outpostEnabled && !hasOutpost) {
        DetectionBox statusBox;
        statusBox.idx = OUTPOST;
        statusBox.is_dead = !pipeline_->isOutpostAlive();
        armor_msg->detections.push_back(statusBox);
    }
    armor_pub_->publish(std::move(armor_msg));

    // 4. 发布耗时统计 (含端到端延迟 = now - image_timestamp)
    auto timing_msg = std::make_unique<PipelineTiming>();
    timing_msg->car_ms = ...;  // 从 pipeline_ 获取
    timing_msg->end_to_end_ms = (this->now() - msg->header.stamp).seconds() * 1000.0;
    timing_pub_->publish(std::move(timing_msg));

    // 5. 可选: 发布调试标注图像
    if (publish_debug_image_) {
        drawDetect(debug_frame, results, classNames);
        image_pub_->publish(debug_frame);
    }
}
```

**前哨站心跳**：即使前哨站未被检测到（被遮挡或已死亡），也会发布空框的状态消息，确保下游节点持续收到前哨站的存活/死亡状态。

### 2.3 PoseNode — 姿态解算 + 跟踪

[pose_node.cpp](../src/tensorrt_detect/src/nodes/pose_node.cpp)

```
功能:
  · 解算检测结果的世界坐标 (批量 raycaster)
  · 运行 Tracker (Kalman + Hungarian + BotIdentity)
  · 发布跟踪后的目标数组 (10 个 official slots + Outpost + 动态死亡装甲板)

订阅话题:
  /armor_detections → DetectionArray

发布话题:
  /world_targets → WorldTargetArray

服务:
  /pose_node/reload_calibration → std_srvs/Trigger (重载标定文件)
```

**回调主循环**：

```cpp
void armor_callback(const DetectionArray::ConstSharedPtr msg) {
    if (!is_calibrated_) return;  // 未标定 → 跳过

    // 0. 批量预计算世界坐标
    std::vector<cv::Rect> boxes_for_raycast;
    for (const auto& det : msg->detections) {
        // 优先使用 car_box, fallback 到 armor_box
        boxes_for_raycast.push_back(car_box_valid ? car_box : armor_box);
    }
    world_positions = pose_solver_->middletoworldBatch(boxes_for_raycast);

    // 1. 构建 WorldMeasurement 输入
    for (each detection) {
        if (det.idx == OUTPOST) → 直接透传到 slot[10]
        if (det.idx == ARMOR && is_dead) → 动态追加到 slot[11+]
        else → WorldMeasurement { class_id, team_id, box, world }
    }

    // 2. Tracker 更新
    tracker_.update(measurements);

    // 3. 输出映射 + 发布
    auto slot_outputs = tracker_.get_outputs();  // 10 official slots
    // 构造 WorldTargetArray: slots[0-9] + slot[10](Outpost) + dead_targets[]
    world_pub_->publish(world_msg);
}
```

### 2.4 MapNode — 雷达地图 + 战术分析

[map_node.cpp](../src/tensorrt_detect/src/nodes/map_node.cpp)

```
功能:
  · 将世界坐标映射到地图像素坐标
  · 绘制雷达地图 (OpenCV)
  · 前哨站状态叠加绘制
  · MapAnalyzer 战术分析
  · 阵营视角切换

订阅话题:
  /world_targets → WorldTargetArray
  /flip_team → std_msgs/Bool

发布话题:
  /map_image → sensor_msgs/Image (雷达地图图像)
  /radar_map → RadarMap (地图序列化数据)
  /map_tactics → MapTactics (战术分析)
```

**世界→地图坐标变换**：

```cpp
// Linear mapping: world (meters) → map (pixels)
// 基于 calibrate2() 预先计算的 scale_x, scale_y, offset_x, offset_y

cv::Point2f RadarMap::worldtomap(const cv::Point2f& world_point) {
    return {
        offset_x + world_point.x * scale_x,   // x → 图像 col
        offset_y + world_point.y * scale_y    // z → 图像 row
    };
}
```

**drawMap 可视化**：

```
Robot markers:
  ╔═══════════════╗
  ║      ◎        ║  ← 实心圆 (team color), 白色描边
  ║    蓝3        ║  ← team+class 文字标签
  ╚═══════════════╝

Outpost markers:
  · ALIVE: 橙色实心圆 + "ALIVE" 文字
  · DEAD:  黑色 X 标记 + "DEAD" 文字

Dead armor:
  · 灰色圆, "dead" 标签
```

### 2.5 QtDisplayNode — Qt GUI 显示

[qt_display_node.cpp](../src/tensorrt_detect/src/nodes/qt_display_node.cpp)

详见 [QT_VISUALIZATION.md](QT_VISUALIZATION.md)

### 2.6 calibrate_node — 交互式相机标定

```
功能:
  · 从视频流捕获帧
  · 用户交互点选世界对应点
  · solvePnP 求解外参
  · 保存 calib_result.yaml
  · 触发 pose_node 重载

服务提供:
  /calibration/start → std_srvs/Trigger

服务调用:
  /pose_node/reload_calibration → 触发重载
  /video_node/set_pause → 暂停/恢复视频
```

**标定流程**：

```
1. 收到 /calibration/start 请求
2. 等待下一帧到达 (从 /image_raw)
3. 暂停视频 (调用 /video_node/set_pause)
4. 显示帧到 OpenCV 窗口
5. 设置 mouseback 回调 (用户点击世界对应点)
6. 用户点击 6+ 个点后运行 solvePnP
7. 检查重投影误差
8. 若通过 → 保存 calib_result.yaml → 触发 reload
9. 恢复视频
```

### 2.7 roi_set_node — 前哨站 ROI 设置

```
功能:
  · 从视频流捕获帧
  · 用户拖框选择前哨站区域
  · 保存 outpost_roi.yaml
  · 触发 detect_node 重载

服务提供:
  /roi_set/start → std_srvs/Trigger

服务调用:
  /detect_node/reload_roi → 触发重载
  /calibration/start → 若未标定则先标定
```

---

## 3. 通信机制

### 3.1 话题 QoS 配置

| 话题 | QOS Profile | 理由 |
|---|---|---|
| `/image_raw` | reliable, queue=1 | 最新帧优先，旧帧丢弃 |
| `/detected_image` | reliable, queue=1 | 调试用，只需最新帧 |
| `/armor_detections` | best_effort, queue=10 | 允许丢帧，保证实时性 |
| `/pipeline_timing` | reliable, queue=1 | 只需最新耗时数据 |
| `/world_targets` | best_effort, queue=10 | 允许丢帧，保证实时性 |
| `/map_image` | reliable, queue=1 | 只需最新地图 |
| `/radar_map` | reliable, queue=10 | 需要每帧数据 |
| `/map_tactics` | reliable, queue=10 | 战术变化需要及时通知 |
| `/flip_team` | reliable, queue=1 | 切换指令，不能丢失 |

### 3.2 服务定义

| 服务名 | 类型 | 调用者 | 提供者 |
|---|---|---|---|
| `/calibration/start` | Trigger | qt_display_node | calibrate_node |
| `/roi_set/start` | Trigger | qt_display_node | roi_set_node |
| `/pose_node/reload_calibration` | Trigger | calibrate_node | pose_node |
| `/detect_node/reload_roi` | Trigger | roi_set_node | detect_node |
| `/video_node/set_pause` | SetBool | calibrate_node, qt_display_node | video_node |

**服务调用特点**：
- Qt 节点中所有服务调用在**后台线程**执行（`std::thread(...).detach()`），不阻塞 Qt 事件循环
- 标定/ROI 操作有 **5 分钟超时**，因为用户交互点选可能需要较长时间
- `QMetaObject::invokeMethod` 用于从后台线程安全更新 Qt UI

### 3.3 Intra-process 零拷贝通信

```python
# launch 文件中的配置
ComposableNode(
    package='tensorrt_detect',
    plugin='DetectNode',
    name='detect_node',
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

**效果**：
- 同一 container 内的节点间 Publisher/Subscription 直接使用 `rclcpp::IntraProcessManager`
- 消息通过 `unique_ptr` 传递，不经过 DDS/网络序列化
- 对于大帧图像 (`1920×1080×3 bytes ≈ 6MB`)，节省 **~2-5ms** 序列化开销

**限制**：
- 仅适用同一 container 内的节点
- 必须使用完全相同的话题名和 QOS
- 不支持跨机器通信

---

## 4. 配置管理

所有节点通过 ROS2 parameter 机制接收 `config_dir` 路径：

```yaml
# src/tensorrt_detect/config/ros2_params.yaml
video_node:
  ros__parameters:
    video_path: "/path/to/video.mp4"
    target_fps: 30

detect_node:
  ros__parameters:
    config_dir: "/home/delphine/rm/tensorrt10_detect/configs"
    input_topic: "/image_raw"
    publish_debug_image: true

pose_node:
  ros__parameters:
    config_dir: "/home/delphine/rm/tensorrt10_detect/configs"
    input_topic: "/armor_detections"

map_node:
  ros__parameters:
    config_dir: "/home/delphine/rm/tensorrt10_detect/configs"
    input_topic: "/world_targets"
    flip_team: false
```

每个节点通过 `Config` 类统一加载 `configs/` 目录下的 YAML 文件：

```cpp
// 所有核心节点的通用初始化模式
this->declare_parameter<std::string>("config_dir", "...");
std::string config_dir = this->get_parameter("config_dir").as_string();
cfg_ = std::make_unique<Config>(config_dir);
// Config 构造时自动加载 model.yaml, camera.yaml, map.yaml,
// runtime.yaml, tracker.yaml, calib_result.yaml, outpost_roi.yaml
```

详见 [ConfigManager.hpp](../src/tensorrt_detect/include/tensorrt_detect/ConfigManager.hpp)

---

## 5. 节点生命周期

### 5.1 启动顺序

```
1. component_container 启动
   ├── VideoNode:  等待视频文件可用 → 开始发布 /image_raw
   ├── DetectNode: 加载 4 个 TensorRT engine (~1-3 秒)
   │               订阅 /image_raw → 开始推理
   ├── PoseNode:   加载 camera.yaml + calib_result.yaml
   │               加载 3D mesh → 等待标定
   │               订阅 /armor_detections → 开始解算
   └── MapNode:    加载 map.yaml + map.png
                   订阅 /world_targets → 开始绘制

2. calibrate_node (独立进程)
   └── 提供 /calibration/start 服务

3. qt_display_node (独立进程)
   └── 订阅 /detected_image, /map_image, /armor_detections,
       /map_tactics, /pipeline_timing

4. roi_set_node (独立进程)
   └── 提供 /roi_set/start 服务
```

### 5.2 故障恢复

- **TensorRT engine 加载失败**：`Model` 构造函数抛出异常 → 节点启动失败
- **视频文件不可用**：VideoNode 等待，定期重试
- **标定未就绪**：PoseNode 跳过处理，定期 (10s) 打印警告
- **Mesh 加载失败**：Raycaster 使用 flat ground fallback
- **服务超时**：Qt 节点中 `wait_for_service(5s)` 超时 → 显示错误提示

---

## 6. 独立运行模式 (Standalone)

[standalone_main.cpp](../src/tensorrt_detect/apps/standalone_main.cpp) 提供非 ROS2 运行模式：

```cpp
// 不依赖 ROS2 通信，直接创建核心对象
Config cfg(config_dir);
DetectPipeline pipeline(cfg);
PoseSolver posesolver(cfg.camera.cameraMatrix, cfg.camera.distCoeffs);
Tracker tracker(tracker_params);
RadarMap radarmap(...);

// 直接调用
auto results = pipeline.process(frame);
// ...
```

用于快速验证模型性能和算法逻辑，无需启动 ROS2 环境。

---

## 7. 全链路延时检测

最近提交 `8fbf68e` 加入了全链路延时检测：

```
端到端延时 = detect_node 当前时间 - 原始图像的 header.stamp
  = 视频帧时间戳 → detect_node 接收 → GPU 推理 → 后处理 → 发布 timing 消息的时间差

time 日志中的含义:
  car_ms     = Stage 1 车辆检测耗时
  armor_ms   = Stage 2a 装甲板检测耗时 (内部计时)
  cls_ms     = Stage 3 分类耗时
  output_ms  = Stage 2b 前哨站检测耗时 (outpost 显示为 output)
  airplane_ms= Stage 4 异步无人机耗时
  total_ms   = process() 总耗时 (不含异步无人机)
  e2e_ms     = 端到端延迟 (从图像时间戳到 timing 消息发布时间)
  fps        = 检测帧率 (指数移动平均)
```
