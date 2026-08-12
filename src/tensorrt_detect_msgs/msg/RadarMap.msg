# MapNode 发布给外部消费者的固定兵种地图槽位。坐标是 map.png 像素，不是米制
# world/field 坐标；缺失目标以 0 填充，上层不能把它直接解释为真实地图原点。
std_msgs/Header header

# 蓝方 1-5 号及哨兵(6)的地图坐标 (x, y)
float32[6] blue_x
float32[6] blue_y

# 红方 1-5 号及哨兵(6)的地图坐标 (x, y)
float32[6] red_x
float32[6] red_y
