# DetectPipeline 各推理阶段的最近统计。耗时单位均为毫秒；fps 是 DetectNode
# 检测处理频率的平滑估计，camera_fps 是采集线程成功 read() 的频率估计（相机模式
# 即相机出图速率），dropped_fps 是 FrameBuffer 覆盖待处理帧的速率、dropped_count
# 是其累计值。end_to_end_ms 用主机单调时钟测量取帧到结果发布，不包含传感器
# 曝光/传输时间，也不使用视频合成时间戳计算耗时。
std_msgs/Header header
float64 car_ms
float64 armor_ms
float64 cls_ms
float64 outpost_ms
float64 total_ms
float64 end_to_end_ms
float64 fps
float64 camera_fps
float64 dropped_fps
uint64 dropped_count
