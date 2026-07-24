uint32 grid_index

float32 prior_probability
float32 fused_probability

float32 canonical_x
float32 canonical_y
float32 field_x
float32 field_y
float32 world_x
float32 world_z

bool reachable
bool blocked
# 由 home/gully（全兵种）或 engineer（工程兵专用）盲区注入。
bool from_blind_zone
# 最后可靠观测点的驻留候选，由模型 stay_probability 和盲区规则赋权。
bool stay_anchor
# mesh 上的最短路径距离；mesh 未启用时等于直线距离。
float32 distance_from_last_m
float32 straight_distance_from_last_m
