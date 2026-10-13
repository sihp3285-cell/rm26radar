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
本分支已合入 `clean/web` 的兼容 Dashboard，使用新版区域消息统一构建。Web 展示
区域表格（含零计数）、兵种/轨迹、并列最多区域及双方工程上岛状态，地图提供
可开关的区域与米制容差带，随地图旋转。过期时保留区域几何，计数与状态显示未知。Qt 容差带按平均像素
比例近似绘制；精确判定以 decision 为准。

## 验证

```bash
# 在仓库根目录、新终端执行，勿先 source 其他分支的 install。
source /opt/ros/jazzy/setup.bash
colcon --log-base log/decision build --build-base build/decision \
  --install-base install/decision \
  --packages-select rm_field radar27_interfaces radar27_decision radar27_visualization radar27_dashboard \
  --cmake-args -DBUILD_TESTING=ON
# 本机 Open3D 安装于自定义目录时，构建参数需追加：
# -DOpen3D_DIR=/实际安装路径/lib/cmake/Open3D
source install/decision/setup.bash
colcon --log-base log/decision test --build-base build/decision --install-base install/decision \
  --packages-select radar27_decision radar27_dashboard --event-handlers console_direct+
colcon test-result --test-result-base build/decision --verbose
```

独立测试包含排除文件、边界内外、拐角距离、重叠与并列、重叠优先级归属、
双工程、阵营切换、世界轴反转、重复目标、过期/死亡/NaN 过滤。几何代码不修改共享 YAML 顶点。

## 分支与构建隔离

`clean/decision` 跟踪 `origin/clean/decision`，`clean/web` 跟踪 `origin/clean/web`，
decision 已合入 web 的兼容修复；web 仍保留独立开发分支。`backup/*` 只是旧历史备份，不作为开发或构建入口。
Git 切换分支不会清理 `build/`、`install/`，也不会重置当前终端已加载的 ROS 环境。
因此源码已经切换而 `ros2` 仍使用旧消息/旧节点，甚至找到当前分支不存在的包，都是可能的。

以上命令只验证 decision、Dashboard 与 Qt，不包含完整感知链。运行完整主链前，
在干净终端使用同样的 `--build-base build/decision --install-base install/decision`
执行全工作区 `colcon build`，重编译所有消息消费者，随后仅加载该安装目录。
Web 分支对应使用 `build/web`、`install/web`、`log/web`。
不同分支同时开发建议使用独立 Git worktree，并分别构建；不要叠加两套接口环境。
