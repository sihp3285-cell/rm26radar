# DetectPipeline 输出、PoseNode 输入的单个二维目标。
# idx 是 robot_id.hpp 中的语义类别，不是 track_id 或 official slot；此阶段尚未跟踪。
int32 idx
# 检测分支的目标置信度；class_conf/class_margin 则来自后续兵种分类器。
float32 confidence
# 装甲板/特殊目标框，单位为输入图像像素，原点在左上角。
int32 x
int32 y
int32 width
int32 height
int32 armor_color
# 当前主要用于前哨站/死亡装甲板语义，不表示 Tracker 因漏检而进入 DEAD。
bool is_dead
# 该装甲板所属车辆的外层框；PoseNode 在一般地形优先取车辆框底边落地点。
int32 car_x
int32 car_y
int32 car_width
int32 car_height
# 兼容字段：DetectNode 当前不在检测阶段求世界坐标；真实 world 坐标由 PoseNode
# 使用相机标定与 PLY Raycaster 计算。
float32 world_x
float32 world_y
float32 fps
float32 class_conf
# 分类 top1 与 top2 的置信度差，用于 BotIdentity 判断身份是否有明显优势。
float32 class_margin
