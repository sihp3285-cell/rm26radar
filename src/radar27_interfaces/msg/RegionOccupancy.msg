# Field metres and the same polygon in radar world X/Z. Regions may overlap.
string id
string name
float64[] field_x
float64[] field_y
float64[] world_x
float64[] world_z
uint32 enemy_count
# Parallel arrays, stable class codes follow rm_field/robot_class.hpp.
int32[] enemy_class_ids
int32[] enemy_track_ids
