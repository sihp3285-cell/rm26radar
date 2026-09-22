# 一个离线先验候选经过在线 Gate 后的可解释结果。数组已按 fused_probability
# 排序并受 output_top_k 限制；它不是模型 query_top_k 的原始完整返回。
uint32 grid_index

# prior_probability 仅是模型/盲区/驻留锚点形成的候选质量；fused_probability
# 还乘入可达距离与运动方向权重，并在在线候选间重新归一化。
float32 prior_probability
float32 fused_probability

# 同一点的三套坐标：canonical 固定红方视角；field 是统一场地坐标；world
# 对应 PoseNode 的 world_x/world_z，可直接交给 MapNode。
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
