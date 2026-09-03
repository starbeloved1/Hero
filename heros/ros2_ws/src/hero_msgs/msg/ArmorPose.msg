# 单个装甲板的三维位姿。pose 与 header 使用同一坐标系，位置单位为米。
std_msgs/Header header
geometry_msgs/Pose pose
uint8 id
uint8 color
float32 confidence
bool is_large
float32 reprojection_error
