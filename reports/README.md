# 答辩报告快照（2026-09-05）

本目录保存六份最终报告及报告直接链接的小型 JSON/补丁证据，原样复制自 `log/`，文件校验值见 `sha256.json`。不包含模型、视频、数据库、原始 bag 或构建产物；JSON 中记载的原始文件路径/哈希是溯源信息，不表示那些文件也在 Git 中。

| 报告 | 内容 |
|---|---|
| [68g1](report_68g1/report.html) | 第一局纯跟踪与跟踪加猜点 |
| [68g2](report_68g2/report.html) | 第二局纯跟踪与跟踪加猜点 |
| [68g3](report_68g3/report.html) | 第三局纯跟踪与跟踪加猜点 |
| [动态选点](report_68g1_armor_dyn/report.html) | 第一局动态选点对照 |
| [测量协方差](report_68g3_cov_cmp/report.html) | 第三局射线协方差对照 |
| [NavGrid 与盲区](report_68g3_navgrid_cmp/report.html) | 第三局导航网格与盲区对照 |

下载仓库后用浏览器打开相应 HTML 即可查看内嵌图表；网页代码托管平台可能只显示源代码。在线协议原文链接需要网络。

结论以仓库根目录 `测试交接文档.md` 顶部最新章节为准：结果是录像条件模拟，不是裁判链路实测；g2 属于先验训练集，NavGrid 尚未证明稳定正收益。

本机保留完整 `log/` 时，可用 `python3 scripts/export_defense_reports.py` 刷新快照。该命令只打包现有报告及直接链接，不重新计算实验结果。
