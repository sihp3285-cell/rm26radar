# DetectPipeline 各推理阶段的最近统计。耗时单位均为毫秒；fps 是 DetectNode
# 回调频率的平滑估计，end_to_end_ms 从原图时间戳计算到消息发布时刻。
std_msgs/Header header
float64 car_ms
float64 armor_ms
float64 cls_ms
float64 outpost_ms
float64 airplane_ms
float64 total_ms
float64 end_to_end_ms
float64 fps
