# DetectNode 对一帧输入图像的检测结果。header.stamp 沿用原始图像时间戳，供
# PoseNode 用同一时间轴计算 Tracker dt；detections 为空也代表有效的“无目标”帧。
std_msgs/Header header
DetectionBox[] detections
