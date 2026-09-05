# 68g1 / g2 / g3 规则修正重测

2026-09-05：三份功能对照报告已另行完成10次有效回放并替换，方法与复生成命令见 [COMPARISON.md](COMPARISON.md)，结果见仓库根目录 `测试交接文档.md` 最新节。本文下方“本轮只更新3份主报告”描述的是9月4日轮次；本次对照没有修改这三份主报告。

本夹具只属于测试工具。通过 class_loader 加载现有 DetectNode、PoseNode、MapNode 组件库，PositionPriorNode 使用现有可执行文件。不修改核心源码、模型、场地配置或生产参数文件；仅本次启动关闭 debug image 和 Qt，并把 shadow CSV 定向到对应运行目录。

## 数据与时间

- 数据库：`/home/delphine/下载/rmuc_2026_region_dataset.sqlite`，只读打开。
- 06.mp4 = 68g1 / 1778913533396；07.mp4 = 68g2 / 1778914365365；08.mp4 = 68g3 / 1778915208150。
- 帧号按原视频顺序保存，文件内容秒 = 帧号 / 20。
- `discovery_g*` 是加速、每4帧取1帧的**时间对齐预录**，不是易伤测试，不能使用其墙钟延迟/频率推导比赛效果。
- 在预录轨迹上识别对应局号，偶数10秒内容块拟合仿射时间映射，奇数块检验；正式重放前冻结 `log/rerun68/alignment.json`，不根据新结果重拟合。
- 时间映射来自轨迹，不是官方时钟；保留分段偏移检验，并在报告中给出时间偏移敏感性。坐标残差不等于时间精度。
- `realtime_g*` 按映射后的比赛节奏发布原图。单独源线程按稳态时钟排程，迟于一个帧周期的源帧会丢弃，避免把负载拥塞变成慢放。每帧记录计划时间、实际发布时刻、是否丢弃。
- 原图通过与生产相同的进程内通讯送给既有组件；输出和先验保存在完整 rosbag。记录端的接收时刻用作离线模拟的本地收包代理，**不是裁判服务器实际收包时间**。

## A/B 提交策略

两者均以 `/radar_map` 到达记录端生成候选融合帧，再由固定5Hz发送器取最新整帧。5Hz对应已核验的官方2026通信协议V1.3.0第7页0x0305上限；第32–33页明确两轴均为0视为未发送、单位为厘米。参考原文链接在报告和 `protocol_reference.md`。

- A：非零跟踪坐标才提交。
- B：优先使用同次跟踪坐标；缺失时使用当时已经到达的最新有效敌方 top-1 猜点，原帧年龄不得超过0.5秒。
- 没有使用未来先验；DB血量不参与选点，只有评估复活事件和单列“存活易伤时间”时使用。
- 预测消息是变长数组，按条目 `slot_idx` 查找，而非把数组下标当槽位。报告记录旧查找方法在本轮会漏掉多少次有效提交。
- 主表与逐事件账本统一5Hz；坐标四舍五入为厘米整数，无法用uint16表示则拒发，两轴均为0则省略。未实际编码CRC/发送串口/连接裁判服务器。本次B是明确给定策略的模拟，不能声称已经部署到生产发送端。
- 报告另列4Hz、5Hz、10Hz与未限频；10Hz和未限频超过协议上限，仅作算法诊断。固定节拍取最新已融合整帧，缺失槽位清空，不沿用最后有效坐标；源帧超过0.5秒拒绝。
- 协议第4页给出正常约130ms延迟、丢包率低于1%，较差环境约200ms、约3%的参考量级。报告另用固定随机种子68，对整包一致丢失来测130ms/1%与200ms/3%情景；不能当作本机实际网络测量或置信区间。

## 规则和限制

见用户提供的 V2.0.1 第117–118页。`rules.py` 使用整数十分之一分累计 x/P，避免阈值浮点漂移；中断时钟只由实际提交重启，不被复活截断。复活保留 P，仅 x 清零。

同刻顺序约定为复活、中断、数据；恰好0.5秒时先中断。该边界顺序需未来用裁判实测确认。

报告区分：P阈值并集、逐车合计、存活期间并集/合计、条件下机会解锁时刻。DB“是否易伤”仅为1Hz原始状态基线，未证明全部来自雷达；死亡时该列也可能为1。双倍主动触发和持续30秒效果未模拟。

射频压制和定位模块离线状态缺失，主报告明确假设无压制且定位在线。DB坐标在采样点间线性插值、两端保留最近样本；急转和复活处有不确定性。死亡阶段是否进入官方机会累计不能由现有数据确认，所以同时列存活指标，机会仅称为条件估计。

目前固定的先验模型中 g2 是 train，g1/g3 是 validation。没有为了本测试重新训练模型，g2不作为泛化证明。

## 运行和复核

在仓库根目录运行，GPU/ROS需宿主环境访问权限：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cmake -S scripts/rerun68 -B log/rerun68/fixture_build -DCMAKE_BUILD_TYPE=Release
cmake --build log/rerun68/fixture_build -j4
python3 -m unittest discover -s scripts/rerun68 -p 'test_*.py'
/usr/bin/python3 scripts/rerun68/run.py discovery g1 g2 g3
python3 scripts/rerun68/align.py g1
python3 scripts/rerun68/align.py g2
python3 scripts/rerun68/align.py g3
/usr/bin/python3 scripts/rerun68/run.py realtime g1 g2 g3
python3 scripts/gen_report_combined.py g1 g2 g3
```

运行目录必须不存在，工具拒绝覆盖原始录制。再次完整重录请使用新 phase 名称并在分析时通过 `--phase` 指定；重新生成本轮报告则直接运行最后一行即可。

每个运行目录保留 `run.json`、`source.json`、`source_frames.csv`、各进程日志、完整 `bag/`、直接导出的 `events.jsonl` 及消息数核验 `export_audit.json`。正式评估另保存 `summary.json`、`per_robot.json` 和 `ledger_A/B.jsonl`。预录和正式录制共享固定 `alignment.json`，`runtime_manifest.json` 记录组件哈希和git提交，`core_before.json`用于核心文件不变性核验。

本轮只更新3份主报告。另3份历史配置对比不能混入本次不同条件的新数据；其数值保留，标记为旧口径。旧报告原文统一备份到 `log/rerun68/previous_reports`。
