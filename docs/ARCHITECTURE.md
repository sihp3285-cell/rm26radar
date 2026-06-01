# 系统架构总览 (System Architecture Overview)

## 1. 项目简介

本项目是一个基于 **ROS2 Humble** + **TensorRT 10** + **CUDA 12.5** 的实时目标检测与多目标跟踪系统，专为 **RoboMaster 机甲大师赛** 设计。系统通过单目相机实时检测赛场上的机器人车辆、装甲板、前哨站和无人机，求解它们的 3D 世界坐标，进行多目标跟踪（Kalman 滤波 + 匈牙利匹配），并在雷达地图和 Qt GUI 上可视化结果。

### 核心技术栈

| 技术层 | 选型 |
|---|---|
| 构建系统 | CMake 3.22+, C++17 + CUDA (arch 86) |
| 推理引擎 | TensorRT 10.2 (NVIDIA) |
| GPU 加速 | CUDA 12.5, OpenGL (Qt), Open3D Raycasting |
| 图像处理 | OpenCV 4.11 |
| 3D 运算 | Open3D 0.19, Eigen |
| 通信框架 | ROS2 Humble (rclcpp, rclcpp_components) |
| 可视化 | Qt5 (QOpenGLWidget + 自定义 GLSL shader) |
| 配置管理 | yaml-cpp |

---

## 2. 目录结构

```
tensorrt10_detect/
├── configs/                    # YAML 配置文件
│   ├── model.yaml              # 模型路径、输入尺寸、阈值
│   ├── camera.yaml             # 相机内参、畸变系数、世界标定点、3D 网格路径
│   ├── map.yaml                # 雷达地图图像、场地物理尺寸
│   ├── tracker.yaml            # 跟踪器参数（Kalman 门限、BotIdentity 衰减等）
│   ├── runtime.yaml            # 运行时开关
│   ├── outpost_roi.yaml        # 前哨站感兴趣区域
│   ├── calib_result.yaml       # 外参标定结果（PnP 输出）
│   ├── RB2026_rmuc.ply          # 2026 RMUC 赛场 3D 网格
│   └── map.png                 # 雷达地图底图
├── models/                     # 预导出 TensorRT .engine 文件
│   ├── robot_only.engine       # 车辆检测模型 (1280×1280)
│   ├── newarmor.engine         # 装甲板检测模型 (192×192)
│   ├── classify_hku.engine     # 装甲板分类模型 (64×64)
│   └── airplane640.engine      # 无人机检测模型 (640×640)
├── src/
│   ├── tensorrt_detect/        # 主 ROS2 包
│   │   ├── CMakeLists.txt       # 构建规则
│   │   ├── package.xml          # 包描述
│   │   ├── config/ros2_params.yaml
│   │   ├── launch/detect_pipeline.launch.py
│   │   ├── apps/standalone_main.cpp
│   │   ├── include/tensorrt_detect/
│   │   │   ├── ConfigManager.hpp
│   │   │   ├── core/           # 核心算法头文件 (12 个)
│   │   │   └── visualization/  # 可视化头文件 (2 个)
│   │   └── src/
│   │       ├── config/ConfigManager.cpp
│   │       ├── core/           # 核心算法实现 (10 .cpp + 1 .cu)
│   │       ├── nodes/          # ROS2 节点实现 (7 个节点)
│   │       └── visualization/  # 可视化实现 (2 个)
│   └── tensorrt_detect_msgs/   # 自定义消息定义
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── msg/                # 7 个 .msg 定义
```

---

## 3. 系统架构总览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          Qt Display Node (独立进程)                       │
│  ┌──────────────────────┐   ┌────────────┐   ┌─────────────────────────┐ │
│  │  GLVideoWidget (GPU) │   │ Radar Map  │   │ Status Bar (Timing/战术) │ │
│  │  BGR→RGB shader 交换  │   │ QLabel     │   │ car/armor/cls/airplane  │ │
│  │  glTexSubImage2D DMA  │   │            │   │ outpost alive/dead      │ │
│  └──────────────────────┘   └────────────┘   └─────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
         ↑ /detected_image        ↑ /map_image    ↑ /pipeline_timing
         │                         │               │ /map_tactics
         │     ┌───────────────────┼───────────────┼──────────────┐
         │     │  ComposableNodeContainer (单进程，intra-process)    │
         │     │                                                   │
         │     │  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌─────────┐│
  Video  │     │  │VideoNode │→→→│DetectNode│→→→│ PoseNode │→→→│ MapNode ││
  File ──┘     │  │(帧读取)  │   │(4-stage  │   │(PnP+Kal- │   │(雷达地图)││
               │  │          │   │pipeline) │   │man+track)│   │         ││
               │  └──────────┘   └──────────┘   └──────────┘   └─────────┘│
               │        /image_raw   /armor_detections  /world_targets     │
               └───────────────────────────────────────────────────────────┘
```

**关键设计决策**：
- 核心流水线（Video → Detect → Pose → Map）运行在 **单个 ComposableNodeContainer** 内，使用 ROS2 进程内零拷贝通信（`use_intra_process_comms: true`），避免大帧数据的序列化/反序列化开销
- 使用**单线程容器** `component_container`（非 `component_container_mt`），避免 TensorRT 推理与 Open3D RaycastingScene 在多线程中并发执行 CUDA 操作导致 SIGSEGV
- Qt Display 和 calibrate_node 运行在**独立进程**中，因为它们各自需要事件循环（Qt main loop / OpenCV highgui）

---

## 4. 核心数据处理流水线

```
输入帧 (BGR 8UC3)
    │
    ▼
┌──────────────────┐
│ Stage 1: 车辆检测 │  robot_only.engine (1280×1280)
│   runDetect()    │  检测所有车辆边界框，按最小 ROI 尺寸过滤
└──────┬───────────┘
       │ cars[]
       ▼
┌──────────────────────┐
│ Stage 2a: 装甲板检测  │  newarmor.engine (192×192)
│   runArmorDetect()   │  对每个车辆 ROI 内部检测装甲板（取最高置信度）
│                      │  raw_id=0 → 死亡装甲板, raw_id>0 → 装甲板颜色
│ Stage 2b: 前哨站检测  │  对配置的 ROI 区域独立运行 armorDetector_
│   detectOutpost()    │  连续 N 帧未检测到则标记 DEAD
└──────┬───────────────┘
       │ armors[] + outposts[]
       ▼
┌──────────────────┐
│ Stage 3: 装甲分类 │  classify_hku.engine (64×64)
│   runClassify()  │  raw_id 0-3 → R2-R4, raw_id 4 → Sentry
│                  │  (死亡装甲板跳过分类，保持 ARMOR 类型)
└──────┬───────────┘
       │ classified armors[]
       ▼
┌──────────────────────┐
│ Stage 4: 无人机检测   │  airplane640.engine (640×640)
│   runAirplaneDetect() │  **异步线程**，在图像右半区域运行
│                      │  每 airplaneIntervalMs (默认33ms) 执行一次
│                      │  结果缓存到 cachedAirplaneResults_
└──────┬───────────────┘
       │ all results[]
       ▼
    发布 /armor_detections
```

---

## 5. 跟踪与姿态解算流水线 (PoseNode)

```
/armor_detections
    │
    ▼
┌─────────────────────────┐
│ 0. 批量 Raycast          │  middletoworldBatch()
│    所有 box 的世界坐标    │  输入: car_box 中心点像素
│                          │  输出: 3D 世界坐标 (x, z)  // y=0 地面
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────────┐
│ 1. 分类处理                  │
│    · Outpost → 直接透传到 slot[10]
│    · 死亡装甲板 → 动态追加 slot[11+]
│    · 正常装甲板 → 构建 WorldMeasurement 送入 Tracker
└──────┬──────────────────────┘
       │
       ▼
┌─────────────────────────────┐
│ 2. Tracker.update()         │
│    · Kalman predict (box + world)
│    · 匈牙利匹配 (team 过滤 + class 惩罚)
│    · Kalman update / 新 track 创建
│    · 状态机推进 (ACTIVE/PREDICTED/LOST/DEAD)
│    · BotIdentity 身份更新
└──────┬──────────────────────┘
       │
       ▼
┌─────────────────────────────┐
│ 3. 输出映射                   │
│    · 10 个官方槽位 (Red R1-R4,S | Blue R1-R4,S)
│    · track→slot 映射 (team+stable_class)
│    · 同队同兵种仲裁 (保留最高置信度)
│    · 发布 /world_targets
└─────────────────────────────┘
```

---

## 6. 可视化流水线 (MapNode + QtDisplayNode)

```
/world_targets
    │
    ▼
┌──────────────────────┐
│ MapNode              │
│ · worldtomap() 坐标映射│
│ · drawMap() 雷达图    │
│ · 前哨站叠加绘制       │
│ · MapAnalyzer 战术分析 │
└──────┬───────────────┘
       │ 发布 /map_image, /radar_map, /map_tactics
       ▼
┌──────────────────────────────────────┐
│ QtDisplayNode (独立进程)              │
│ ┌──────────────────────────────────┐ │
│ │ GLVideoWidget (QOpenGLWidget)    │ │
│ │ · glTexSubImage2D 纹理上传 (DMA) │ │
│ │ · BGR→RGB 在 fragment shader 完成│ │
│ │ · GPU 双线性缩放保持宽高比        │ │
│ └──────────────────────────────────┘ │
│ ┌──────────────────────────────────┐ │
│ │ DisplayWindow (QMainWindow)      │ │
│ │ · 状态栏: 各阶段耗时 + 前哨站状态  │ │
│ │ · 战术栏: 攻防态势 + 工程上岛      │ │
│ │ · 操作按钮: 重标定/重框ROI/视角切换│ │
│ └──────────────────────────────────┘ │
└──────────────────────────────────────┘
```

---

## 7. 自定义消息类型 (ROS2 Messages)

| 消息 | 用途 |
|---|---|
| `DetectionBox` | 单个检测结果：idx, confidence, bbox, armor_color, is_dead, car_bbox, world_x/y |
| `DetectionArray` | 检测数组：Header + DetectionBox[] |
| `WorldTarget` | 跟踪目标：idx, class_id, team_id, score, world_xyz, bbox_xywh, stable_class |
| `WorldTargetArray` | 目标数组：Header + WorldTarget[10固定槽位 + Outpost透传 + 死亡装甲板] |
| `RadarMap` | 雷达地图：blue_x/y[6], red_x/y[6] |
| `MapTactics` | 战术分析：engineer_on_island, opponent_attack, our_attack, opponent_near_fortress |
| `PipelineTiming` | 流水线耗时：car_ms, armor_ms, cls_ms, outpost_ms, airplane_ms, total_ms, end_to_end_ms, fps |

---

## 8. 关键算法一览

| 模块 | 算法 | 位置 |
|---|---|---|
| 预处理 | GPU 纹理双线性插值 + Letterbox + BGR→RGB + Normalization | [preprocess.cu](../src/tensorrt_detect/src/core/preprocess.cu) |
| 车辆检测 | TensorRT YOLO-style + NMS (1280×1280) | [model.cpp](../src/tensorrt_detect/src/core/model.cpp) |
| 装甲检测 | 级联检测 (每车 ROI 内独立检测) | [pipeline.cpp](../src/tensorrt_detect/src/core/pipeline.cpp) |
| 装甲分类 | TensorRT CNN Classifier (64×64) | [model.cpp](../src/tensorrt_detect/src/core/model.cpp#L445) |
| 异步无人机 | 独立线程 + 低频控制 | [pipeline.cpp](../src/tensorrt_detect/src/core/pipeline.cpp#L304) |
| 姿态估计 | PnP + Rodrigues + Raycasting | [posesolver.cpp](../src/tensorrt_detect/src/core/posesolver.cpp) + [raycaster.cpp](../src/tensorrt_detect/src/core/raycaster.cpp) |
| 多目标跟踪 | 匈牙利匹配 + Kalman (8D Box + 4D World) + BotIdentity 指数衰减 | [tracker.cpp](../src/tensorrt_detect/src/core/tracker.cpp) + [kalman.cpp](../src/tensorrt_detect/src/core/kalman.cpp) + [bot_identity.cpp](../src/tensorrt_detect/src/core/bot_identity.cpp) |
| 雷达可视化 | 线性坐标映射 + OpenCV 绘图 | [radarmap.cpp](../src/tensorrt_detect/src/core/radarmap.cpp) |
| GPU 视频渲染 | OpenGL GLTexture2D + BGR→RGB shader swizzle | [qt_display_node.cpp:41-203](../src/tensorrt_detect/src/nodes/qt_display_node.cpp) |

---

## 9. 性能优化策略

1. **GPU 全链路**: 从预处理 (CUDA kernel) → 推理 (TensorRT) → 渲染 (OpenGL)，所有图像数据保持在 GPU，避免 CPU-GPU 往返
2. **Pinned Memory + Async**: `cudaMallocHost` 分配页锁定内存，`cudaMemcpyAsync` 异步 H2D，配合 CUDA stream 实现异步预处理
3. **Texture Memory 预处理**: 使用 CUDA 纹理对象 (`cudaTextureObject_t`) 的硬件双线性插值替代软件实现，性能优于原始指针版本
4. **GPU Buffer 增长按需分配**: `gpuInputBuffer8U_` / `hInputBuffer8U_` 在容量不足时才重新分配，避免每帧动分配
5. **DMA 纹理更新**: 帧尺寸不变时使用 `glTexSubImage2D` 替代 `glTexImage2D`，仅更新数据不重分配
6. **Intra-process 零拷贝**: 核心节点在 ComposableNodeContainer 内通过共享指针传递，避免序列化
7. **异步无人机检测**: 独立线程低频运行（默认 33ms 间隔），不阻塞主流水线
8. **批量 Raycast**: `middletoworldBatch()` 一次 CUDA 锁内批量处理所有像素投射，减少互斥锁开销
9. **CUDA 全局互斥锁**: `cuda_guard::getCudaMutex()` 序列化所有 CUDA 操作，防止多线程并发导��� SIGSEGV

---

## 10. 构建与启动

```bash
# 构建
cd ~/rm/tensorrt10_detect
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 启动完整流水线
source install/setup.bash
ros2 launch tensorrt_detect detect_pipeline.launch.py

# 或单独启动某个节点
ros2 run tensorrt_detect qt_display_node
```

**启动后系统中的节点和话题**:

```
节点列表:
  detect_pipeline_container  # 单进程内含 4 个 component 节点
  calibrate_node
  qt_display_node
  roi_set_node

话题列表:
  /image_raw           → (sensor_msgs/Image)           # 原始视频帧
  /detected_image      → (sensor_msgs/Image)           # 带标注的调试图像
  /armor_detections    → (DetectionArray)              # 检测结果
  /pipeline_timing     → (PipelineTiming)              # 各阶段耗时
  /world_targets       → (WorldTargetArray)            # 跟踪后的世界目标
  /map_image           → (sensor_msgs/Image)           # 雷达地图图像
  /radar_map           → (RadarMap)                    # 雷达地图数据
  /map_tactics         → (MapTactics)                  # 战术分析
  /flip_team           → (std_msgs/Bool)               # 阵营视角切换

服务列表:
  /pose_node/reload_calibration   → 重载相机外参
  /detect_node/reload_roi         → 重载前哨站 ROI
  /calibration/start              → 启动标定流程
  /roi_set/start                  → 启动 ROI 框定
  /video_node/set_pause           → 暂停/恢复视频
```
