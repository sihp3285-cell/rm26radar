# RViz2 可视化使用方法

本文说明 `tensorrt_detect` 项目的 RViz2 调试画面里**每一类图形对应哪一份数据**，以及如何读懂它们。
调试链路是**纯旁路**：Pose/Tracker/Position Prior/Map 都不订阅这些 topic，关掉调试对主流程零影响。

---

## 1. 怎么启动

先编译并 source 环境：

```bash
colcon build --packages-select tensorrt_detect tensorrt_detect_msgs position_prior
source install/setup.bash
```

### 方式 A：launch 时一起拉起（推荐）

```bash
# 视频回放模式
ros2 launch tensorrt_detect detect_pipeline.launch.py \
  mode:=video \
  rviz_debug_enabled:=true \
  enable_rviz:=true

# 工业相机模式
ros2 launch tensorrt_detect detect_pipeline.launch.py \
  mode:=camera \
  rviz_debug_enabled:=true \
  enable_rviz:=true
```

- `rviz_debug_enabled:=true`：加载 `RvizDebugNode`，并让 `PoseNode` 创建 Marker/TF 发布器。
- `enable_rviz:=true`：自动打开 rviz2 并加载预置配置 `config/radar_debug.rviz`。

### 方式 B：主链已开调试，手动补开 RViz

```bash
rviz2 -d "$(ros2 pkg prefix tensorrt_detect)/share/tensorrt_detect/config/radar_debug.rviz"
```

静态场景（Mesh / 相机 / FOV / BlindZone / NavGrid）用 **Reliable + Transient Local** 发布，晚启动的 RViz 也能收到；
动态图层（射线 / Tracker / Guesser）用 **BestEffort + depth 小队列**，允许丢帧换低延迟。

---

## 2. 坐标系约定（先看这里）

- 固定坐标系 **Fixed Frame = world**。
- 地面是 **X-Z 平面，Y 是高度**，单位米。项目**不交换坐标轴**。
- 场地参考网格范围：`x ∈ [-7.5, 7.5]`、`z ∈ [-14, 14]`，对应 **15×28 m** 场地。
- `TF` 显示里只有一条 `world → camera_link`：由 PoseSolver 当前外参 R/T 发布，静态 TF。
- 场地地图类数据（BlindZone / NavGrid）内部是“红方 canonical 坐标”，运行时按当前敌方阵营
  （订阅 `/flip_team`）转换到 world；切换阵营会整帧重发静态场景。

---

## 3. 画面元素速查表

| 显示分组 | 画面里的东西 | 对应的数据 / 来源 |
|---|---|---|
| **TF** | `world → camera_link` | PoseNode 当前相机外参（R/T），静态 TF |
| **Static Scene** | 半透明蓝灰三角面 | `configs/RB2026_rmuc.ply` 场地 Mesh（与 Raycaster 同一份） |
| | 灰色 1m 网格 | 项目自绘场地参考网格（替代 rviz 自带 Grid，防不渲染） |
| | 橙色小方块 + 黄色箭头 | 相机本体（位置=相机原点）与光轴（相机系 z+） |
| | 橙色四棱锥线框 | 由内参 fx/fy/cx/cy 反投影的 FOV，锥体画到 3 m 处 |
| | 红色多边形（面/边/字） | `generated/home.yaml`、`gully.yaml` 里的盲区（红方视角） |
| | 品红多边形 | `generated/engineer.yaml` 仅工程机器人可走的盲区，标签带 `[engineer]` |
| | 绿/红/灰薄片 | NavGrid 格子：绿色=可达、红色=不可达、灰色=无效高度（障碍） |
| **Pose Debug** | 黄色射线 | 相机原点 → 每个检测框被选中的投影射线终点 |
| | 品红散点 | **所有**射线终点（含未选中的备选），检查遮挡/筛选 |
| | 黄/灰/队色圆点 + 文字 | 本帧实际选中的世界落点（详见 §4） |
| **Tracker Debug** | 绿/青/灰/深灰球体 | Tracker 状态 ACTIVE/PREDICTED/LOST/DEAD（`/world_targets`） |
| | 队色小球（radar/robots） | 未进 Tracker 的直通测量（前哨站等，track_id<0） |
| | 队色折线 | 目标历史轨迹（track_id 稳定，最多 50 点） |
| | 青色箭头 | 速度矢量（放大 `velocity_scale_seconds` 秒） |
| | 品红椭圆 | Kalman 滤波协方差 P 的 xz 平面 2σ 椭圆 |
| | 白色椭圆 + 白点 | 最近一次被 Kalman 实际消费的测量噪声 R（在原始测量位置） |
| **Guesser Debug** | 蓝色球（大小∝概率） | 先验候选 Top-K，`fused_probability` 越大球越大 |
| | 红球 / 灰球 | 候选被障碍遮挡（BLOCKED）/ 不可达（UNREACHABLE） |
| | 品红大球 + 连线 + 文字 | 主猜点 MAIN GUESS、Tracker 位置→主猜点连线、置信度 |
| | 候选文字 | `#排名 兵种`、`Pfused=..`、`prior=..`、`d=距离`，可带 `blind`/`stay` 标记 |

---

## 4. 每个 Marker 命名空间 → 数据字段

四个发布 topic：

| Topic | 发布者 | QoS | 内容 |
|---|---|---|---|
| `/radar/rviz/static` | PoseNode + RvizDebugNode | Reliable + Transient Local | 场景静态要素 |
| `/radar/rviz/pose` | PoseNode | BestEffort | 投影射线 / 落点 |
| `/radar/rviz/tracker` | RvizDebugNode | BestEffort | `/world_targets` 直出 |
| `/radar/rviz/guesser` | RvizDebugNode | BestEffort | `/prior_predictions` 直出 |

### 4.1 Static Scene（/radar/rviz/static）

| Namespace | 图形 | 数据 |
|---|---|---|
| `radar/mesh` | TRIANGLE_LIST 半透明面 | PLY 顶点/三角面，与 Raycaster 同一文件 |
| `radar/field_grid` | 1m 网格线 | 程序生成，非数据 |
| `radar/camera` (id0/1) | 相机立方体 + 光轴箭头 | PoseSolver 当前 R/T |
| `radar/fov` | FOV 四棱锥线框 | 内参矩阵；分辨率取 `rviz_debug_image_width/height`（只画图，不影响算法） |
| `radar/blind_zones/area|boundary|text` | 盲区面/边界/标签 | 盲区 YAML 的 `blind_zones[].polygon`；文件名含 `engineer` 标品红 |
| `radar/nav_grid/reachable|unreachable|obstacle` | 薄片立方体 | NavGrid JSON 的 `surface.height_mm` + 对应兵种 `profiles[role].walkable`；格子 y = 高度(mm)×0.001 + 0.025，无效高度画为障碍 |

### 4.2 Pose Debug（/radar/rviz/pose）

| Namespace | 图形 | 数据 |
|---|---|---|
| `radar/rays` | 黄色 LINE_LIST | 相机原点 → `PoseDebugSample.ray_endpoint`（每帧全部检测） |
| `radar/ray_hits` | 品红 POINTS | 所有 `ray_endpoint`（含未选中项） |
| `radar/measurements` | 圆点 | `measurement`（被选中的 world 坐标） |
| `radar/measurement_text` | 文字 | 标签 + `raw (x, y, z) c=置信度`；死亡装甲板追加 `NEGATIVE` |

落点颜色语义：

- **黄色**：该落点会作为测量进入 Tracker（正常装甲板）。
- **灰色**：负样本（死亡装甲板），确认“此处无目标”。
- **队色（红/蓝）**：直通项（前哨站等，不走 Tracker）。

### 4.3 Tracker Debug（/radar/rviz/tracker）

数据源是 `tensorrt_detect_msgs/msg/WorldTarget`（`/world_targets`）。

| Namespace | 图形 | 字段 |
|---|---|---|
| `radar/tracks` | 球体（0.36） | `world_x/y/z` + `tracking_state` 颜色 + `track_id`（跨帧稳定） |
| `radar/robots` | 球体（0.30，队色） | `track_id<0 && valid` 的直通目标（前哨站/死亡装甲板） |
| `radar/track_text` | 文字 | 兵种标签、`Track=`、状态、`obs=yes/no`、`lost=..s`、`q=`（tracking_confidence）、`stable=`（stable_class_id + 置信度）、坐标 |
| `radar/velocity` | 青色 ARROW | `velocity_x/y/z` × `velocity_scale_seconds`（默认 1.0 s） |
| `radar/trajectory` | 队色 LINE_STRIP | 每 track 最近 ≤50 个滤波位置（track 消失即清除） |
| `radar/covariance` | 品红椭圆 | `state_covariance[0],[1],[4],[5]`（[x,z,vx,vz] 的 xz 2×2 块）→ 2σ 椭圆 |
| `radar/measurement_R` | 白色椭圆 + 白点 | `measurement_x/z`（原始测量位置）+`measurement_covariance`（像素误差经 5 射线 Jacobian 传播出的 R）；本帧未观测时半透明 |

> **读图技巧**：白色 R 椭圆画在“原始测量处”，品红 P 椭圆画在“滤波后状态处”。
> 两者并排能直接看出一次 Kalman 更新把不确定度收敛了多少。

状态颜色：ACTIVE 绿 / PREDICTED 青 / LOST 灰 / DEAD 深灰。

### 4.4 Guesser Debug（/radar/rviz/guesser）

数据源是 `tensorrt_detect_msgs/msg/PriorPredictionArray`（`/prior_predictions`）。

| Namespace | 图形 | 字段 |
|---|---|---|
| `radar/guess_candidates/accepted` | 蓝球 | `PriorCandidate.world_x/z`，半径 = clamp(0.13 + Pfused×0.65, 0.13, 0.65) |
| `radar/guess_candidates/rejected` | 红球/灰球 | `blocked=true` 红、`reachable=false` 灰 |
| `radar/guess_candidates/text` | 文字 | `#排名 兵种`、`Pfused=`、`prior=`、`d=`（distance_from_last_m）、`blind`/`stay`/`BLOCKED`/`UNREACHABLE` 标记 |
| `radar/main_guess` | 品红大球 | `prior_world_x/z`（主猜点） |
| `radar/main_guess/link` | ARROW | `tracker_world_x/z` → `prior_world_x/z` |
| `radar/main_guess/text` | 文字 | `MAIN GUESS 兵种`、`confidence=`（prior_confidence）、`lost=..s`、`reachable_mass=` |

Marker id 约定：每个 slot 占 100 号段，候选 = `slot_idx*100 + rank`，主猜点 = `slot_idx*100 + 90`，互不冲突。

---

## 5. 常用开关

launch 参数：

| 参数 | 作用 |
|---|---|
| `rviz_debug_enabled` | 总开关：加载 RvizDebugNode + PoseNode debug hook |
| `enable_rviz` | 是否自动打开 rviz2 |

`config/ros2_params.yaml` 里的子开关（可单独关掉减轻渲染压力）：

- `rviz_debug_node`：`mesh`、`field_grid`、`tracks`、`trajectories`、`velocity`、`covariance`、`measurement_covariance`、`guess_candidates`、`blind_zones`、`nav_grid`、`trajectory_length`、`velocity_scale_seconds`。
- `pose_node`：`rviz_debug_camera`、`rviz_debug_fov`、`rviz_debug_rays`、`rviz_debug_ray_hits`、`rviz_debug_measurements`、`rviz_debug_image_width/height`。

在 RViz 左侧 Displays 面板里也可以直接勾掉某个 Namespace 单独对比观察。

---

## 6. 已知边界 / 排查提示

- **射线命中来源不区分**：Raycaster 在 PLY 未命中时退化为 `world y=0` 平面，`ray_hits` 忠实显示算法实际采用的终点，但不标注是 PLY 命中还是平面回退。
- **没有关联线**：Hungarian 逐关联边未暴露，Measurement 与 filtered track 分别显示，肉眼比较位置差即可。
- **动态图层每帧先 DELETEALL 再重建**；轨迹以 `track_id` 为稳定 id，目标消失后折线会被清理。
- **FOV 只是内参反投影的画图**，分辨率参数改错不影响检测/投影精度。
- **静态场景看不到**：确认主链用了 `rviz_debug_enabled:=true` 启动；手动开的 RViz 依赖 transient-local，先开 RViz 再启主链也能收到。
- **阵营不对**：BlindZone/NavGrid 按 `/flip_team` 重发，红方视角时敌方为蓝方，盲区会按 x=14 轴左右翻转；Mesh 和动态 world 点不做像素显示旋转。
