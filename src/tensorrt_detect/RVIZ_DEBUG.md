# RViz2 调试系统

RViz2 是纯旁路观察层。Pose、Tracker、Position Prior、Map 和比赛输出不订阅
RViz topic；关闭调试时不会加载 `RvizDebugNode`，PoseNode 也不会创建 Marker/TF
publisher 或复制逐帧调试数据。

## 启动

```bash
source install/setup.bash
ros2 launch tensorrt_detect detect_pipeline.launch.py \
  mode:=video \
  rviz_debug_enabled:=true \
  enable_rviz:=true
```

工业相机使用 `mode:=camera`。只打开已配置的 RViz：

```bash
rviz2 -d "$(ros2 pkg prefix tensorrt_detect)/share/tensorrt_detect/config/radar_debug.rviz"
```

如果主链已用 `rviz_debug_enabled:=true` 启动，后打开的 RViz 仍能通过
transient-local 收到 Mesh、相机、FOV、BlindZone 和 NavGrid 静态场景。

## 预置显示

- `TF`：`world -> camera_link`，直接使用 PoseSolver 当前 R/T。
- `Static Scene`：Raycaster 使用的同一 PLY、相机/FOV、BlindZone、NavGrid。
- `Pose Debug`：当前选中投影的射线、三维终点和 Tracker 原始 measurement。
- `Tracker Debug`：滤波位置、ACTIVE/PREDICTED/LOST/DEAD、速度、有限轨迹、
  2σ 协方差、stable class 和置信度。
- `Guesser Debug`：Top-K、prior probability、Pfused、可达/拒绝状态和主猜点。

BlindZone 和 NavGrid 固定采用红方 canonical 数据。`RvizDebugNode` 订阅
`/flip_team`，按当前敌方阵营执行与 Position Prior 相同的 canonical→field→world
转换；敌方为蓝方时 BlindZone 多边形会先按 x=14 轴左右翻转（物理盲区随相机换
半场），阵营切换后会重新发布静态 Guesser 场景，不对 Mesh 或动态 world 点应用
MapNode 的像素显示旋转。

项目世界地面是 X-Z 平面，Y 是高度。配置已把 Grid 设为 XZ；可视化不会交换
已有世界坐标轴。动态 topic 使用 BestEffort、depth=1；静态 topic 使用 Reliable +
Transient Local。动态 Marker 每帧先 `DELETEALL`，轨迹以 `track_id` 为稳定 ID，
最多保留 50 点，并在 track 不再出现时删除。

## 完全关闭

```bash
ros2 launch tensorrt_detect detect_pipeline.launch.py \
  enable_rviz:=false \
  rviz_debug_enabled:=false
```

子开关位于 `config/ros2_params.yaml`：`mesh`、`tracks`、`trajectories`、
`velocity`、`covariance`、`guess_candidates`、`blind_zones`、`nav_grid`，以及
PoseNode 的 `rviz_debug_camera/fov/rays/ray_hits/measurements`。

## 当前只读边界

Raycaster 在 PLY 未命中时会由原核心代码退化到 `world y=0` 平面，且只返回最终
投影点，没有暴露“PLY hit / flat fallback”标志。因此 `ray_hits` 忠实显示算法
实际采用的射线终点，但不额外区分来源。Hungarian 的逐关联边也没有现成输出；
为避免读取私有状态或重做匹配，本调试层没有构造伪关联线。Measurement 与 filtered
track 已分别显示，可直接观察位置差异。
