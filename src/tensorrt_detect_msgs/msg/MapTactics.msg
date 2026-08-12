# MapAnalyzer 从 /world_targets 派生的轻量战术量，供 Qt 状态栏/外部决策使用。
# 这些值不参与 Tracker 或 Position Prior 的反向更新。
std_msgs/Header header

bool engineer_on_island
bool opponent_attack
bool our_attack
bool opponent_near_fortress
