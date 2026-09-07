# 与同名图像帧一致的最终时间戳和坐标系
std_msgs/Header header

# 大恒 SDK 原始设备计数及其读取到的频率；host 模式和视频输入均为 0
uint64 device_timestamp
uint64 device_timestamp_frequency_hz

# 图像从驱动获得及发布前的 ROS 时间，用于观测传输和颜色转换开销
builtin_interfaces/Time host_receive_stamp
builtin_interfaces/Time host_publish_stamp

# host 或 device
string timestamp_mode

# device 模式相对首帧设备时间映射到主机单调时间的低通残差
float64 device_to_host_offset_sec
uint32 reset_count
