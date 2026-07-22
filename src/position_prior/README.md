# position_prior

逐兵种条件位置先验的 ROS 2 shadow-mode 包。它订阅 `/world_targets`，只在目标真实观测丢失后发布独立的 `/prior_predictions`，不会覆盖 `WorldTarget`，也不会把猜点回灌 tracker。

## 在线约定

- 输入角色：英雄、工程、步兵 3、步兵 4、哨兵（模型名分别为 `hero`、`engineer`、`infantry3`、`infantry4`、`sentry`）。
- canonical 坐标固定为红方视角；蓝方使用 `(28-x, 15-y)` 中心对称。
- `2/5/10 s` 使用最邻近 horizon；物理可达距离使用真实 `lost_duration_s`，不使用离散 horizon 代替。
- 候选还必须位于最后可靠观测点的 `max_guess_distance_m` 硬距离阈值内；阈值内使用高斯距离权重温和偏向近点。
- `enabled_roles` 控制当前允许输出的兵种。首轮实测默认仅启用工程，其他兵种保留模型与代码能力，积累更多 shadow 数据后可按配置逐项开放。
- 最后可靠位置必须通过连续观测确认：默认连续 3 次检测位于 0.8 m 聚类半径内，单帧跳变不会刷新先验锚点，也不会结束正在进行的猜点。
- 猜点按“阵营 + 兵种”唯一化，同一方同一兵种最多输出和显示一个；实战运行时再由敌我门控只保留敌方结果。
- 猜点只服务敌方：蓝方视角只接受红方目标，红方视角只接受蓝方目标；己方和未知阵营不会建立或保留先验缓存。
- 达到 `guess_after_s` 后持续猜点；超过模型最大 horizon 时继续使用最邻近的 10 秒档，直到该车辆被真实重新观测。
- 真实观测会立即退出本轮猜点并刷新缓存；机器人被明确判死、输入时间非法、时间倒退或阵营翻转会清除缓存。
- tracker 的 `DEAD/INVALID` 仅表示短期跟踪生命周期结束，不清除最后一次可靠观测。
- 模型缺失、版本不符、geometry 不符或 SHA-256 不符时，节点保持运行但安全禁用输出。

## 输出

`PriorPredictionArray` 中每个结果包含主猜点、Top-K 候选、置信度、样本量、熵、fallback 层级、运动门控状态和拒绝原因。地图只以洋红菱形和空心候选圆显示这些结果；`RadarMap` 与 `MapAnalyzer` 仍只使用 tracker 数据。

shadow CSV 记录 `PREDICTION` 和 `REACQUIRED` 两类事件。后者包含重识别位置与最终误差，可用于后续实战评估。

## 模型与测试

模型目录 `position_prior_toolkit/` 在仓库根目录的 `.gitignore` 中。默认模型：

```text
position_prior_toolkit/run_v1/04_rmuc2026_position_prior_v1.yaml
SHA-256: b1a7384042939aca3fcfa3d45a5bbd3cd8b7e3bdeaa658aac24d17f36e2df53e
```

`test_model_golden` 会加载真实模型，并逐项核对 `06_golden_cases.json` 中全部 15 个 Python golden cases。

首次构建 ROS 接口前应退出与系统 ROS 版本不一致的 Conda 环境。Jazzy 的 `ros2` CLI 使用系统 Python 3.12；若误用其他 Python 生成 `rosidl` 绑定，C++ 节点仍可运行，但 `ros2 topic` 无法加载自定义消息。
