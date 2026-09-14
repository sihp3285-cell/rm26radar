# header 是最新 world_targets 的图像时间戳；targets 固定为10个机器人槽位。
# 融合不限制发送频率；串口节点应按协议限频并检查各目标 source_stamp。
std_msgs/Header header
FusedTarget[] targets
