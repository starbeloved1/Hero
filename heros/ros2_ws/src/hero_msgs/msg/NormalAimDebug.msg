# 普通模式瞄准过程调试信息，header 表示参与本次决策的装甲板测量时刻
std_msgs/Header header
bool valid
bool held_target
uint32 lost_frames
uint8 target_id
geometry_msgs/Point target_position
float32 raw_yaw
float32 raw_pitch
float32 command_yaw
float32 command_pitch
float32 flight_time_sec
bool shoot_status
