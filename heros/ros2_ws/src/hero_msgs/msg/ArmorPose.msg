# 单个装甲板的三维位姿。pose 与 header 使用同一坐标系，位置单位为米。
std_msgs/Header header
geometry_msgs/Pose pose
# 对应二维检测框的像素中心；仅用于保留 PnP 输入图像中的位置语义。
geometry_msgs/Point32 image_center
uint8 id
uint8 color
float32 confidence
bool is_large
float32 reprojection_error
