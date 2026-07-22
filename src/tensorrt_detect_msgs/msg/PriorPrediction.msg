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
uint8 rejection_code
string rejection_reason

builtin_interfaces/Time last_observed_time
float32 lost_duration_s
uint32 horizon_seconds

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

float32 prior_confidence
float32 normalized_entropy
float32 reachable_probability_mass
uint8 fallback_level
uint32 sample_count
bool motion_gated

PriorCandidate[] candidates
