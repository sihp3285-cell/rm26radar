# DetectPipeline 各推理阶段的最近统计。耗时单位均为毫秒；fps 是 DetectNode
# 检测处理频率的平滑估计，end_to_end_ms 用主机单调时钟测量取帧到结果发布，
# 不包含传感器曝光/传输时间，也不使用视频合成时间戳计算耗时。
std_msgs/Header header
float64 car_ms
float64 armor_ms
float64 cls_ms
float64 outpost_ms
float64 total_ms
float64 end_to_end_ms
float64 fps
