# PoseNode 的统一世界目标输出。前 10 项通常是 Tracker official slots，索引 10
# 是 Outpost 直通项，之后可追加死亡装甲板；数组位置、idx、track_id 和 class_id
# 分别表达容器位置、兼容索引、物理轨迹与语义身份，不应混用。
int32 idx
int32 class_id
int32 team_id
float32 score
bool is_dead
bool valid

# 雷达世界坐标：地面运动使用 (world_x, world_z)，world_y 当前固定为 0；单位米。
float32 world_x
float32 world_y
float32 world_z

int32 bbox_x
int32 bbox_y
int32 bbox_w
int32 bbox_h

# BotIdentity 稳定身份（指数加权历史类别分布）
int32 stable_class_id
float32 stable_class_conf

# Tracker 生命周期状态。INVALID 表示当前 official slot 没有对应物理轨迹。
uint8 TRACKING_INVALID=0
uint8 TRACKING_ACTIVE=1
uint8 TRACKING_PREDICTED=2
uint8 TRACKING_LOST=3
uint8 TRACKING_DEAD=4

# 当前 position 字段的数据来源。第一阶段不会产生 PRIOR_GUESSED。
uint8 POSITION_INVALID=0
uint8 POSITION_MEASURED=1
uint8 POSITION_TRACKED=2
uint8 POSITION_PREDICTED=3
uint8 POSITION_PRIOR_GUESSED=4

# PhysicalTrack 唯一 ID；非 tracker 目标和空 slot 为 -1。
int32 track_id
uint8 tracking_state
uint8 position_source

# 当前消息对应的 tracker 更新中是否匹配到了真实检测。
bool observed

# 世界坐标速度，与 position 一致：x=world_x，z=world_z，y 固定为 0。
float32 velocity_x
float32 velocity_y
float32 velocity_z

# 状态 [x, z, vx, vz] 的 4x4 行优先协方差。
float64[16] state_covariance
bool covariance_valid

# detection_confidence 是最后一次真实检测置信度；score 暂时保持同值以兼容旧代码。
float32 detection_confidence
float32 tracking_confidence

builtin_interfaces/Time last_observed_time
float32 lost_duration_s

# 最近一次进入世界 Kalman 滤波器的原始测量位置（未平滑）与当时实际使用的
# 测量噪声 R（2x2 行主序，world xz 平面）。R 由像素误差经 5 射线反投影
# Jacobian 传播得到；传播输入无效时回退为滤波器固定 R。槽位尚未发生任何
# 测量更新、或为直通项（前哨站/死亡装甲板）时 valid 为 false。
float32 measurement_x
float32 measurement_z
float64[4] measurement_covariance
bool measurement_covariance_valid
