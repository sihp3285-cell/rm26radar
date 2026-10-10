# 全局战况 1.0

`DecisionNode` 从 `/world_targets` 计算 `/map_tactics`，继续输出原有 `/radar_map`。
已移除旧的大攻、半场矩形和堡垒接近规则。新 `MapTactics` 与旧版本不兼容，
修改后须重编译消息及其消费者并重启主链、Qt 和 Dashboard。

## 区域与计数

- bringup 传入 `assets/regions`，加载其中 `.yaml` 的 `regions` 或 `blind_zones`。
- 排除 `engineer_home.yaml`、`other_home.yaml`；文件名作为稳定 ID，避免旧配置
  共用 `enemy_blind_zone` 名称。当前 7 份文件生成 8 个区域（含对称沟区）。
- 全部顶点按固定场地 field(x,y) 米制坐标解释；只按文件的 `mirror_centrally`
  扩展对称区域。切换己方只改变敌我身份，显示旋转只改变绘图。
- 多边形内部或距离任意边不超过 `boundary_margin_m` 即计入，默认 0.75 m，
  允许 0.5–1.0 m。不是包围盒膨胀；场外目标不计入。
- 统计当前区域内数量，不是累计进入次数。区域重叠时默认分别统计，不可加总；
  配置了重叠优先级的组合例外，重合处只计入优先区域。
- `region_overlap_priority` 用 `"优先区域>被覆盖区域"` 声明重叠归属，按区域文件
  基名匹配，对称与多段实例一并生效。当前为 `gully>middle`：中心岛（中央区）与
  沟区在 0.75 m 容差下重合约 20 m²（占沟区约 40%），重合处计入沟区，
  因此中央区计数与沟区计数之和不再重复统计同一目标。
  规则里的区域名必须在已装载区域中存在，否则 `loadRegions()` 直接报错。
- 使用 `stable_class_id` 区分英雄、工程、步兵3、步兵4、哨兵；排除建筑、死亡、
  无效位置、无效身份、LOST，以及失联超过 `max_prediction_age_s`（默认 1 s）
  的目标。允许短期 Kalman 预测，不使用 Position Prior 猜点。
- 同阵营同兵种保留优先真实观测、其次身份置信度高的一项；每区域物理轨迹去重。
- `engineer.yaml` 作为本版工程岛边界，敌我工程都检测同一几何区域；只在文件
  明确要求镜像时扩展。两个工程上岛标志独立，可同时为 true。
- 数量最多的区域保留全部并列项；全部为零时不选择赢家。

## 数据与界面

`MapTactics.regions` 包含名称、field/world 两套顶点、数量、兵种 ID 和轨迹 ID。
两套顶点由 decision 统一转换，Qt/Web 不独立解析地图区域，保证统计与显示一致。
`input_fresh=false` 时计数和工程标志清空，消费者应显示未知而不是安全。
无输入 1 s 后由墙钟定时器发布未知状态；每 100 ms 刷新，收到坐标时也立即刷新。
保留输入 header 和 calibration_version，发布前尚无输入时仍提供区域几何。

Qt 状态栏显示敌车最多区域及工程上岛信息，地图显示全部区域、计数和浅色容差带。
Web 表格显示所有区域（含零计数）、兵种/轨迹、并列最多标记及两个工程标志；
地图图层可关闭，旋转与目标保持一致。Web 容差带按米制宽度绘制，Qt 按平均像素
比例近似绘制；精确判定均以 decision 为准。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to radar27_decision radar27_visualization radar27_dashboard radar27_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select radar27_decision --event-handlers console_direct+
```

独立测试包含排除文件、边界内外、拐角距离、重叠与并列、重叠优先级归属、
双工程、阵营切换、世界轴反转、重复目标、过期/死亡/NaN 过滤。几何代码不修改共享 YAML 顶点。
