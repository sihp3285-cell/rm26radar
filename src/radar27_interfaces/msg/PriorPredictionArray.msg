# 一帧 /world_targets 对应的先验批输出。model_enabled/status 是节点级健康状态；
# 单个 prediction.valid 仍可能因过早、无可靠观测或低置信度而为 false。
std_msgs/Header header
bool model_enabled
string model_status
PriorPrediction[] predictions
