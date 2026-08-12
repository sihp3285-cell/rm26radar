# PoseNode 每帧发布的完整槽位快照。即使某些 slot 无效也保留固定位置，便于 Map、
# Position Prior 对同一兵种持续维护状态；header.stamp 继承 DetectionArray/原图。
std_msgs/Header header
WorldTarget[] targets
