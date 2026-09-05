# 开发、离线工具与测试入口

从仓库根目录运行。ROS相关命令需先加载ROS及工作区环境。

| 目录/脚本 | 用途 |
|---|---|
| `rerun68/` | 规则模拟、回放夹具、对照组、报告与原始数据核验、单元测试 |
| `gen_report_combined.py` | 三份主报告的精简入口，使用当前5Hz因果评估器 |
| `export_defense_reports.py` | 将六份报告及直接链接的小型证据复制到版本管理目录 `reports/`，并生成 SHA-256 清单 |
| `offline/build_position_prior.py` | 离线模型构建及评估；参数、模型产物仍在根目录 `position_prior_toolkit/` |
| `record_referee_topic.sh` / `run_noqt_test.sh` | 录制和无Qt运行辅助 |
| `export_bag_to_csv.sh` / `csv_cleanup.py` | 通用bag CSV导出及清洗 |
| `merge_topics_csv.py` / `compare_layers.py` | 历史CSV诊断工具，不作为当前比赛易伤计分入口 |

```bash
python3 scripts/gen_report_combined.py g1 g2 g3
python3 scripts/rerun68/comparison_analyze.py
python3 scripts/rerun68/verify.py
python3 scripts/rerun68/comparison_verify.py
python3 -m unittest discover -s scripts/rerun68 -p 'test_*.py'
python3 scripts/offline/build_position_prior.py --help
```

2026-09-05清理：删除旧 `eval_vulnerability.py`、历史硬编码恢复器 `log/gen_report_cov_cmp_restore.py`、一次性 `rerun68/finalize.py`；`log/gen_report_combined.py` 中旧模拟和绘图实现删除，当前入口迁入本目录。历史报告与原始录制保留。

`log/` 只用于报告、录制和可追溯构建产物。里面的固定R源码副本与CMake编译探测文件属于生成的审计/构建材料，不是独立维护的测试源；负责生成它们的代码位于 `scripts/rerun68/`。请勿手动编辑或迁移这些带哈希的材料。
