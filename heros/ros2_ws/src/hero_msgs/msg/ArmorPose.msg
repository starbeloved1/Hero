# 单个装甲板的三维位姿，pose 与 header 使用同一坐标系，位置单位为米
std_msgs/Header header
geometry_msgs/Pose pose
# pose.header.frame_id 的 XY 平面内到装甲板中心的水平距离，单位米，即 sqrt(x^2 + y^2)。
# 正常发布时 pose 位于 world，可与 pose.position.z（竖直方向）配合观察。
# 若该帧 TF 不可用而回退到相机坐标系，该值也随 pose.header.frame_id 回退。
float32 distance
# 对应二维检测框的像素中心；仅用于保留 PnP 输入图像中的位置语义
geometry_msgs/Point32 image_center
uint8 id
uint8 color
float32 confidence
bool is_large
float32 reprojection_error
