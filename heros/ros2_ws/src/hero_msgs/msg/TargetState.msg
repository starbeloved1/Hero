# 整车在 world 坐标系中的预测状态模型
std_msgs/Header header

# 目标id
uint8 id
bool tracking
bool converged
bool jumped

# 整车参数
geometry_msgs/Point center
geometry_msgs/Vector3 velocity

float64 yaw # 四装甲板模型基准朝向
float64 angular_velocity
float64 radius
float64 radius_offset
float64 height_offset

# 未来时刻预测的四块装甲板位姿
geometry_msgs/Pose[] predicted_armors
