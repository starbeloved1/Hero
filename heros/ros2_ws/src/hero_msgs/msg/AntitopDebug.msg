# mode 2 反前哨过程量，供 Foxglove 和录像回归使用。
# header.stamp 为产生本次控制候选的时刻 Tcontrol。
std_msgs/Header header

# 参与本次更新的原始装甲板观测时刻 T0。
builtin_interfaces/Time measurement_stamp

bool tracking_valid
bool center_valid
bool calibrated
uint8 target_id

# 当前连续跟踪的前哨装甲板和拟合出的旋转中心，均在 header.frame_id 中。
geometry_msgs/Point tracked_position
geometry_msgs/Point rotation_center

# 标定后使用最低层高度；未标定时使用当前观测高度。
float64 target_z
uint32 center_sample_count
uint32 calibration_sample_count
float64[3] z_layers

# 当前板与旋转中心在原始图像中的横向关系，以及由此得到的区域、周期、倒计时状态。
float32 tracked_image_x_px
float32 center_image_x_px
float32 center_image_distance_px
int8 rotation_direction
int8 matched_z_layer
float32 average_period_sec
bool in_shoot_zone
bool countdown_active
float32 countdown_remaining_sec

float32 raw_yaw
float32 raw_pitch
float32 command_yaw
float32 command_pitch
float32 flight_time_sec

bool pitch_locked
bool fire_enabled
bool shoot_ready
bool shoot_status
