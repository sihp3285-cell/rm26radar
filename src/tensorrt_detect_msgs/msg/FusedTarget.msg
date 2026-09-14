# 固定机器人槽位的单一坐标来源；valid=false 时不得提交坐标。
int32 slot_idx
int32 track_id
int32 team_id
int32 class_id

bool valid

uint8 SOURCE_INVALID=0
uint8 SOURCE_TRACKED=1
uint8 SOURCE_PREDICTED=2
uint8 SOURCE_PRIOR=3

uint8 source

float32 world_x
float32 world_z

float32 confidence
float32 lost_duration_s

# 原始选中输入的图像时间戳，供串口发送端检查新鲜度。
builtin_interfaces/Time source_stamp
