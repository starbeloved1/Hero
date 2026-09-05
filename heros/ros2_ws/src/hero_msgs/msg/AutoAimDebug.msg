# mode 3 自瞄控制决策过程量，供 Foxglove 和录像回归使用
# header.stamp 为本次控制候选生成时刻 Tcontrol
std_msgs/Header header

# 原始图像采集时刻 T0、预测状态时刻 Tstate、预计命中时刻 Taim
builtin_interfaces/Time measurement_stamp
builtin_interfaces/Time state_stamp
builtin_interfaces/Time aim_stamp

bool valid
bool target_locked
uint8 target_id
int8 armor_index

# 选中面板在 Taim 的预测位置
geometry_msgs/Point aim_point

float32 raw_yaw
float32 raw_pitch
float32 command_yaw
float32 command_pitch
float32 flight_time_sec
float32 state_age_sec
float32 system_response_time_sec
float32 phase_error_rad
float32 yaw_error_rad
float32 pitch_error_rad
float32 planar_acceleration_mps2

bool target_converged
bool phase_ready
bool gimbal_ready
bool acceleration_safe
bool fire_enabled
bool shoot_status
