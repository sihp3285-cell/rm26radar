int32 idx
int32 class_id
int32 team_id
float32 score
bool is_dead
bool valid

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
