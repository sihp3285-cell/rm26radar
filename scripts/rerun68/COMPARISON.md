# 2026-09-05 三组功能对照重测

目的：在修正后的5Hz计分与固定时间映射下，检查现有三个设计在指定录像中的收益。不是裁判链路实测，不把历史DB当同条件对照。

## 实验设计

- g1：生产动态选择器 vs 现有的 armor 优先安全回退（`projection_selector_enabled=false`）。不是强制禁止 car 的纯 armor 实验。
- g3：生产射线R vs 正常有效五射线投影固定R=diag(1,1)m²。保留无效射线回退；固定R不增加地形方差。R同时影响选择器和跟踪，不能只解释为Kalman单模块收益。
- g3：NavGrid＋盲区全开 vs 全关。驻留锚点缩放保持0.8。关闭网格后现有代码不加载盲区；未测单开组合。
- 每配置两次。g1顺序为生产1、armor1、armor2、生产2；g3为生产1、fixed1、全关1、全关2、fixed2、生产2。g3的两次生产基线在两个报告中共用，不能算作四个独立样本。
- 全部按同一冻结映射实时重放，Qt/debug image关闭；同一节点库、模型、源录像、生产配置。对照参数只保存在log，固定R通过独立静态库副本和独立Pose组件构建，生产文件不变。
- 总计10次完整回放。每次保留bag、参数读回、源帧来源及接收时间，不覆盖旧录制。

一次fixed第1轮尝试因检查工具对正在写入的bag做只读SQL抽查，引发SQLite锁冲突并使录制器退出。该不完整尝试移至`comparison_20260905/failed_runs/`，明确排除后原配置重跑。此后禁止查询正在写入的bag；实际R只在录制结束且导出完成后检查。运行中的参数查询使用ROS服务，不访问bag。

## 工具

均从仓库根目录运行，ROS命令前source `/opt/ros/jazzy/setup.bash` 和 `install/setup.bash`。

1. `comparison_build.py`：建立只写log的对照库/参数文件和生产哈希清单；目录已存在时拒绝覆盖。
2. `comparison_run.py`：串行运行十次；只允许跳过已完整录制和导出核验的运行目录。
3. `comparison_capture.py`：通过独立ROS域196，只读抓取每次正在运行的pose和prior实际参数。
4. `comparison_process.py`：只处理已完整导出的录制；运行评价和实际测量R核验。
5. `comparison_analyze.py`：重生成三份对照报告；不替换三个主报告。单次分析用 `key variant repeat` 参数。
6. `comparison_measurements.py`：直接读取bag中的`/world_targets`，验证实际测量R。
7. `comparison_verify.py`：生产不变性、原始来源、逐事件时长、参数、R和报告图表核验。

测试数据和补丁位于 `log/rerun68/comparison_20260905/`；单次运行位于 `log/rerun68/cmp05_{variant}_r{repeat}_{g1或g3}/`。重生成报告可运行：

```bash
python3 scripts/rerun68/comparison_analyze.py
python3 scripts/rerun68/comparison_verify.py
```

## 解读

图为两次均值和最小/最大范围，非置信区间。报告同时保留A/B、逐次差值、逐车时长、准确率、运行延迟、共同时间偏移和网络情景。仅两次不能证明统计显著，单局录像也不能证明跨场泛化。并集饱和时重点看单机合计。

沿用当前规则解释：实际发送的死亡后猜点继续判定，不按DB血量删除；复活P保留、x清零。存活覆盖是辅助分析，不冒称官方易伤累计口径。假设定位在线、无干扰压制，未模拟实际双倍触发或无敌期间伤害。时间映射、1Hz参考坐标、精确同刻边界与真实链路仍有局限。
