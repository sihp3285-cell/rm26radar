# A single image-space detection projected into the configured world frame (metres).
DetectionBox detection
float32 world_x
float32 world_z
float32[4] covariance
bool covariance_valid
# False measurements are ignored; an empty array still advances tracking time.
bool valid
# Death-armor observation: suppress nearby tracks, never initialize/update a positive track.
bool is_negative
