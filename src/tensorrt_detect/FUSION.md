# FusionNode

`/world_targets` + `/prior_predictions` → `/fused_targets` (`FusedTargetArray`)。

每条输出是最新 world 快照的十个固定机器人槽位，不包含前哨站和附加死亡装甲板。逐槽选择：

1. `valid && observed` 且来源为 measured/tracked：原样复制 world x/z、track_id、兵种、阵营和跟踪置信度。
2. 丢失（`observed=false`）：复制同槽位、同兵种/阵营/有效轨迹身份的有效 prior 主猜点及置信度。
3. 无可用来源：`valid=false`，坐标默认零；不沿用上次有效坐标。没有第三种 Kalman 外推回退。

prior 按 `slot_idx` 查找，不按变长数组下标查找。拒绝未来、超过 `max_prior_age_s=0.5` 的 prior、比最新真实观测更旧的锚点，以及非有限坐标。确认死亡不提交。Tracker 已清空 track_id=-1 时仍允许同角色槽位的合法持久先验。

两个输入回调都可以发布，prior 稍后到达会更新当前快照；因此同一 header 可能多次发布。`header` 继承 world 图像时间，逐目标 `source_stamp` 保留真正选中输入的图像时间，不能以融合接收时间延长源数据寿命。上游无更新时没有定时重复发布。

节点不平均坐标、不改变 Tracker、不编码串口、不主动发送比赛命令。后续发送节点应只选择敌方 `valid=true` 的目标，检查 source_stamp 时效，并独立限频；SOURCE_PREDICTED 仅保留消息定义，不由本节点产生。

默认 launch 已启动 FusionNode。独立使用：

```bash
ros2 run tensorrt_detect fusion_node --ros-args --params-file src/tensorrt_detect/config/ros2_params.yaml
```

参数：`world_input_topic`、`prior_input_topic`、`output_topic`、`max_prior_age_s`。
