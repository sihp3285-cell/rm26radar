# Radar27 Web Dashboard

独立只读 ROS2 进程：Topics → VisionAdapter → DashboardState → HTTP/SSE → 原生 JS/SVG。
不修改主链算法/消息/launch，不提供主链启停或参数修改接口，不订阅 Image，不存历史。
界面参考 SHARK 的分层地图、位置来源对照与 SSE 思路，代码根据 Radar27 消息重新实现：
[shark-radar-system](https://github.com/JNU-SHARK/shark-radar-system)、
[shark-radar-radio](https://github.com/JNU-SHARK/shark-radar-radio)。

## 构建与启动

```bash
cd /home/delphine/rm/radar27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select radar27_dashboard
source install/setup.bash
ros2 launch radar27_dashboard dashboard.launch.py
```

浏览器打开 <http://127.0.0.1:8765>。主链在另一个终端启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/delphine/rm/radar27/install/setup.bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=video video_path:=/home/delphine/rm/car_project/test/08.mp4 \
  model_dir:=/home/delphine/rm/radar27/models \
  playback_mode:=realtime enable_recording:=false \
  enable_qt_display:=false enable_tools:=false enable_rviz:=false
```

首次干净构建可用 `colcon build --packages-up-to radar27_dashboard`（仅拉起消息依赖，
不会要求编译 GPU/主链包）。默认从已安装的 `radar27_bringup/config/default/map.yaml`
读取 Qt 的 `mapPath`、`race_size`、`isflip`，底图通过 `/map.png` 单次传输。
未安装 bringup 时显式指定地图配置；配置缺失时页面明确提示，不生成替代底图。

```bash
ros2 launch radar27_dashboard dashboard.launch.py \
  map_config:=/home/delphine/rm/radar27/src/radar27_bringup/config/default/map.yaml
# 或直接运行，支持标准 ROS remapping：
ros2 run radar27_dashboard dashboard_node --ros-args \
  -p port:=8766 -r /world_targets:=/my/world_targets
```

独立关闭 Dashboard 使用其终端 Ctrl+C；主链不依赖此进程。网页隐藏时关闭 SSE，
重新可见自动连接，断线原生 EventSource 重连。默认仅绑定 localhost；同一受信局域网
访问可显式使用 `host:=0.0.0.0`，第一版无登录功能。

## 已核对的输入

| Topic | radar27_interfaces 类型 | 用途 |
|---|---|---|
| `/armor_detections` | DetectionArray | Detection 新鲜度、检测数 |
| `/pipeline_timing` | PipelineTiming | 所有性能指标、Camera 输入活跃推断 |
| `/world_measurements` | WorldMeasurementArray | 单帧 Measurement 图层 |
| `/world_targets` | WorldTargetArray | Tracking 与目标真实测量、速度、生命周期 |
| `/prior_predictions` | PriorPredictionArray | Prior、候选、门控与拒绝诊断 |
| `/fused_targets` | FusedTargetArray | 最终坐标、真实 source 与 source_stamp |
| `/map_tactics` | MapTactics | Decision 新鲜度与战术状态 |

全部订阅 `BEST_EFFORT / KEEP_LAST(1) / VOLATILE`，兼容主链已有可靠/尽力发布者，
不会让慢 HTTP 消费者回压主链。当前 Camera 无独立健康消息，不能区分相机硬件与视频
回放；不通过订阅高分辨率图像判断健康。LiDAR / Radio / Referee 显示 `NOT IMPLEMENTED`。
`LIVE` 仅代表收到新鲜消息，不代表算法正确、有目标或标定准确。

## 地图与解释语义

- 使用 **Qt 同一原始 PNG**，不订阅 Qt `/map_image` 渲染帧。
- 与 `RadarMap::worldtomap()` 一致：底图默认顺时针 90°，旋转后宽 W、高 H，
  `u = world_x * W / field_width + W/2`，`v = world_z * H / field_length + H/2`。
  默认 `W=388,H=722,field_width=15,field_length=28`。180° 后坐标为
  `(W-1-u,H-1-v)`；视角按钮只影响网页，不发布 `/display_flip` 或改变阵营。
  此 Qt 世界像素映射不再额外执行 canonical/field 变换。
- 无效位置、NaN、Infinity 输出 JSON `null`；前端不把 null 强转零，超出底图的点也不画。
  合法世界 `(0,0)` 位于底图中心，保留显示。
- `WorldMeasurement` 没有 track_id，显示为独立测量点，绝不按距离猜匹配。
  点击目标后读取 `WorldTarget.measurement_x/z`（measurement_covariance_valid 有效）作为
  最后一次原始测量，`observed=false` 时明确仍是历史测量。
- Prior `last_world_x/z` 是最后可靠观测时的 **跟踪输出锚点**，不是未滤波原始测量。
  页面分别标注 + 最后原始测量、× Prior 锚点、□ Kalman、◇ Prior 主猜点、小圆候选。
  Prior 早期拒绝未填充 tracker_world 时不显示默认 `(0,0)`。
- 目标详情按 `(slot_idx, track_id, team_id, class_id)` 关联；身份不一致独立展示，
  不把旧轨迹 Prior 接到新轨迹。原始测量字段来自轨迹消息本身。
- 各话题保存独立最新消息，**不是同步帧快照**；保留各 source 时间戳/年龄/标定版本。
  Prior/Measurement 的标定版本与当前 Tracking 不同则不叠加。
  FusedTargetArray 没有 calibration_version，仅在其 header 与新鲜 Tracking 完全同帧时
  叠加/关联；否则显示“等待同帧”，可能短暂闪烁，绝不重新计算融合。
- 默认 2 秒未收到的源变为 STALE、隐藏对应图层。无消息为 WAITING，Prior 模型关闭
  为 DISABLED。浏览器断开流会清空实时点，避免把旧位置伪装成当前结果。

## 接口与开销

| 接口 | 内容 |
|---|---|
| `GET /api/state` | schema_version、field、modules、sources 完整快照 |
| `GET /api/radar-stream` | SSE `event: radar`，默认最多 5 Hz |
| `GET /api/performance-stream` | SSE `event: performance`，默认最多 1 Hz |

每个 source 包含 topic、status、age_s、stamp_ns、calibration_version、data。
大整数时间戳和标定版本使用字符串，避免 JS 精度损失。耗时 ms，位置 m，速度 m/s。
ROS 回调仅在短锁内替换消息引用；转换/JSON 在 HTTP 线程按需执行，多个客户端共用
短期 JSON 缓存。最多 16 个 HTTP 活跃连接、socket 超时 3 秒，无无限发送队列/历史。
无浏览器时只有轻量 ROS 接收和 HTTP accept，不运行状态序列化定时器。

参数：`host`、`port`、`map_config`、`radar_hz`(0–20]、`performance_hz`(0–5]、
`stale_after_s`。`field_length/field_width` 为无地图配置时的元数据默认值；有地图时
采用 Qt YAML 的尺寸。`world_z_toward_blue` 仅保留 field 元数据，不改变 Qt 像素映射。

## 扩展 Adapter

后续 Adapter 定义自己的 topics/converters 和 `snapshot(samples, now, stale_after)`，
注册到 `DashboardState.adapters`，在节点创建轻量订阅，将源健康度加入 modules。
结果进入现有 sources 和 HTTP/SSE，不需要替换状态容器、服务器或主链。
如果新增定位层，再给前端添加图层与详情字段。Dashboard 始终保持只读旁路。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select radar27_dashboard --event-handlers console_direct+
colcon test-result --test-result-base build/radar27_dashboard
```

单元测试使用实际生成的 ROS 消息，覆盖无效/NaN 坐标、Prior 早期拒绝、时间戳精度、
来源/身份保留、过期状态与 HTTP/SSE 断连释放。实际回放必须额外验证 GPU 性能与进程隔离。

本机 2026-09-29 验证：构建成功，7 项 pytest 全部通过；使用上述 `08.mp4` 的实际
TensorRT/Localization/Tracking/Prior/Fusion/Decision 主链。各阶段采样 10 秒：

| 阶段 | PipelineTiming 平均 FPS | 新增丢帧 |
|---|---:|---:|
| 无 Dashboard | 20.23 | 0 |
| Dashboard、无浏览器 | 20.21 | 0 |
| 双 SSE 流连接 | 20.12 | 0 |
| 浏览器断开 | 20.07 | 0 |
| Dashboard SIGINT 后 | 20.14 | 0 |
| Dashboard SIGKILL 后 | 20.00 | 0 |

SIGINT / SIGKILL 后分别继续收到 197 / 196 帧 Tracking 消息。Dashboard 无浏览器时
单核 CPU 约 6.6%、RSS 70.8 MiB；双流时约 7.4%、71.2 MiB，断开后 HTTP 线程回落。
这是不同视频片段的短时采样，不是严格同帧性能基准，也尚未覆盖工业相机或长时间压力测试。
内置浏览器验证了真实底图、目标点选、图层开关、180° 旋转、390px 手机布局与停止数据后的
STALE/隐藏点位，未发现控制台错误；额外 Node 检查覆盖 Qt 投影、NaN/null、标定版本及同帧筛选。

现有主链在测试用 `timeout -s INT` 结束时会打印 `Cannot shutdown a ROS adapter that is
not running`，所有子进程仍正常退出；此为主链 launch 的收尾问题，本包未修改该逻辑。
