# PositionPriorNode 对一个 official slot 的猜点结果。slot_idx 是固定消息槽位，
# track_id 是当前物理轨迹身份，role_class_id 是稳定兵种；三者不可互换。
uint8 FALLBACK_LOCAL_ZONE=0
uint8 FALLBACK_GLOBAL_ROLE=1

uint8 REJECTION_NONE=0
uint8 REJECTION_TOO_EARLY=1
uint8 REJECTION_TOO_LATE=2
uint8 REJECTION_NO_OBSERVATION=3
uint8 REJECTION_UNSUPPORTED_ROLE=4
uint8 REJECTION_OUT_OF_FIELD=5
uint8 REJECTION_NO_DISTRIBUTION=6
uint8 REJECTION_UNREACHABLE=7
uint8 REJECTION_LOW_CONFIDENCE=8
uint8 REJECTION_MODEL_DISABLED=9

int32 slot_idx
int32 track_id
int32 team_id
int32 role_class_id

bool valid
# valid=false 时 rejection_code/reason 解释安全门拒绝原因；消息仍有诊断价值。
uint8 rejection_code
string rejection_reason

builtin_interfaces/Time last_observed_time
float32 lost_duration_s
uint32 horizon_seconds

# 最后可靠真实观测、当前 Tracker（可能已外推）和先验主猜点分别保存，便于
# UI/日志区分 observation、Kalman prediction 与 prior，且不覆盖 /world_targets。
float32 last_world_x
float32 last_world_z
float32 tracker_world_x
float32 tracker_world_z
float32 prior_world_x
float32 prior_world_z

float32 prior_field_x
float32 prior_field_y
float32 prior_canonical_x
float32 prior_canonical_y

# 整次预测的综合可信度（跟踪质量、样本量、熵、可达质量等），不是主候选的
# fused_probability，也不是“目标恰在该点”的校准概率。
float32 prior_confidence
float32 normalized_entropy
# Gate 前先验质量中最终可达的比例，衡量离线分布与在线物理约束是否相容。
float32 reachable_probability_mass
uint8 fallback_level
uint32 sample_count
bool motion_gated
bool mesh_used
bool blind_zone_biased
float32 blind_zone_probability_mass
float32 stay_anchor_probability_mass

PriorCandidate[] candidates
