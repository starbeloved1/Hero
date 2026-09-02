# 从英雄机器人主控接收的云台状态。
std_msgs/Header header

uint8 MODE_NORMAL=1
uint8 MODE_ANTI_TOP=2
uint8 MODE_AUTO_AIM=3
uint8 MODE_ANTI_BASE=4

float32 yaw
float32 pitch
uint8 mode
uint8 raw_mode_flag
#己方机器人颜色
uint8 robot_color
bool right_clicked
bool up
bool down

# +1/-1 表示旧协议 up/down 的上升沿；0 表示没有曝光调整。
int8 exposure_step
