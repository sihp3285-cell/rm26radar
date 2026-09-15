# neural_sentry_decision —— 哨兵建议坐标 Shadow Mode

在**不修改、不替换原雷达决策链**的前提下，用当前训练好的哨兵 BC 模型并行推理，
把"建议前往的场地坐标"画到 Qt 地图页和 RViz 上。它只是显示器上的第二条信息，
不产生任何控制量。

## 边界与隔离

| 保证 | 实现方式 |
|---|---|
| 不参与原决策 | 独立可执行 `neural_sentry_node`（独立进程），只发布 `/neural_sentry/shadow/*` |
| 无控制输出 | 节点不创建任何 cmd/serial/fused 发布者；`package.xml` 不依赖控制类消息 |
| 话题命名强制 | 构造时校验 `suggestion_topic`/`marker_topic` 前缀必须是 `/neural_sentry/shadow/`，否则启动即失败 |
| 只读输入 | 只订阅 `/world_targets`（BestEffort，不反向影响发布者）与 `/flip_team` |
| 不占 GPU | 只启用 `CPUExecutionProvider`，单线程、`allow_spinning=0`、`nice 10`、地址空间上限 4 GB |
| 模型异常不停雷达 | 加载失败/校验失败→节点照常运行并持续发布 `valid:false`；连续 3 次推理异常→熔断并只停用建议显示 |
| 默认关闭 | `detect_pipeline.launch.py` 的 `neural_shadow_enabled` 默认 `false`，默认路径不启动本节点 |

`qt_display_node` 与 RViz 只**读**建议：Qt 在已缓存的底图副本上叠加绘制，RViz 用独立
MarkerArray 图层。`shadow` 关闭时 Qt 直接复用底图缓存，不做额外深拷贝。

## 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/world_targets` | `WorldTargetArray` | 订阅 | 唯一输入；只取 `valid && observed && position_source∈{MEASURED,TRACKED}` 的视觉目标 |
| `/flip_team` | `Bool` | 订阅 | `false`=蓝方视角（我方哨兵 107），`true`=红方视角（我方哨兵 7） |
| `/neural_sentry/shadow/suggestion` | `String`(JSON) | 发布 | 建议结果，含 `candidate_index/action/field_x/field_y/world_x/world_z/move_probability/ttl_s` |
| `/neural_sentry/shadow/markers` | `MarkerArray` | 发布 | 建议点球体 + ego→建议点连线 + 文字标签；失效时发 `DELETEALL` |

建议只在 `display_ttl_s`（默认 1.5 s）内有效，过期、阵营切换、输入中断或连续异常都会撤下显示。

## 坐标契约

固定场地坐标，**不做阵营镜像**：

```
field_x = world_z + 14      field_y = world_x + 7.5
world_z = field_x - 14      world_x = field_y - 7.5
```

`/flip_team` 只切换"要给哪个己方哨兵出建议"（7 或 107）和显示是否需要 180° 旋转，
不改变喂给网络的坐标。该契约与训练数据（数据库绝对场地坐标）及
`generated/RB2026_navgrid_v1.json` 的 `world_to_field` 一致，导出时写入
`manifest.json` 的 `coordinate_contract` 并在运行时强校验。

## 运行

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm_field tensorrt_detect_msgs position_prior tensorrt_detect neural_sentry_decision
source install/setup.bash

# 只起影子节点（需另有一个雷达在跑）
ros2 launch neural_sentry_decision shadow.launch.py
ros2 launch neural_sentry_decision shadow.launch.py initial_flip_team:=true

# 或随主链一起起（默认关闭）
ros2 launch tensorrt_detect detect_pipeline.launch.py neural_shadow_enabled:=true
ros2 launch tensorrt_detect detect_pipeline.launch.py neural_shadow_enabled:=true neural_shadow_bundle:=/path/to/bundle
```

RViz 图层 `Neural Sentry SHADOW (display only)` 已写入 `config/radar_debug.rviz`；
Qt 地图页顶部横幅显示 `SHADOW S<id> <MOVE|HOLD> / <candidate>`、建议场地坐标与 `P(MOVE)`，
不可用时显示 `SHADOW unavailable | 原决策继续运行` 与原因。

## 模型 bundle

`shadow.yaml` 默认指向 `models/neural_sentry/sentry_v2/`，包含
`manifest.json`（含各项 SHA-256 与特征契约）、`sentry.onnx`、`candidates.json`。
按仓库 `.gitignore` 约定（`models/`、`*.onnx`）二进制不进 Git，需要在每台机器上生成一次：

```bash
# 用训练环境（已装 torch/onnx/onnxruntime）从 best.pt 重新导出到空目录
python3 scripts/neural_sentry/export_onnx.py \
  --training-root /home/delphine/rm/train_shao \
  --checkpoint runs/sentry_v2/best.pt \
  --output /home/delphine/rm/tensorrt10_detect/models/neural_sentry/sentry_v2
```

导出脚本会校验 checkpoint 与数据集签名一致，并对真实留出哨兵样本比对
ONNX / PyTorch 的 logits 与最终动作，不一致直接失败；`--output` 已存在且非空时拒绝覆盖。

## 端到端自检

```bash
# 隔离域回放：把导出样本的视觉历史重新发布到 /world_targets，抓真实 ROS 链路输出
export ROS_DOMAIN_ID=77   # 必须是未占用的非 0 域，避免影响在跑的雷达
ros2 launch neural_sentry_decision shadow.launch.py
python3 scripts/neural_sentry/replay_shadow.py \
  --bundle models/neural_sentry/sentry_v2 --output log/shadow_replay.json --case 0
```

脚本要求：至少产生一条 `valid:true` 建议、输入停止后能观察到 stale 失效，
且建议 `candidate_index` 与导出参考一致，否则非零退出。

```bash
# 隔离性回归：输出话题约束、无控制发布者、熔断预算、降级存活、恢复出建议
python3 scripts/neural_sentry/test_shadow_isolation.py
```

## 已知限制

- 纯 Behavior Cloning 排序建议，**不构成战术成功或收益声明**，只作为操作员参考。
- 因果历史需要累计 `history_length`（10 s）的视觉观测后才开始出建议，启动初期显示
  `Warming up causal history`。
- 网络输入来自雷达视觉轨迹，训练数据来自裁判系统数据库，存在观测语义差异；
  因此本节点始终是旁路显示，不作为任何自动决策输入。
- 熔断后需重启影子节点才能恢复（`respawn=False`，避免异常时反复抢占 CPU）。
