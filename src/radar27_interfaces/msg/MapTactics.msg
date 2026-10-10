# Battlefield v1: current occupancy (not cumulative entries).
std_msgs/Header header
uint64 calibration_version
int32 own_team
bool input_fresh
float64 boundary_margin_m
RegionOccupancy[] regions
# All tied maxima; empty if all counts are zero or input is stale.
string[] busiest_region_ids
bool our_engineer_on_island
bool enemy_engineer_on_island
