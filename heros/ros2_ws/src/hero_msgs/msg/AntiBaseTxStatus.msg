# 反基地串口发送侧状态
# header.stamp 为本次状态采样时刻；last_send_stamp 为最近一个逻辑包实际开始写串口的 Tsend。
std_msgs/Header header
builtin_interfaces/Time last_send_stamp
uint64 sent_packets
uint32 pending_packets
float32 oldest_pending_ms
uint64 dropped_packets
