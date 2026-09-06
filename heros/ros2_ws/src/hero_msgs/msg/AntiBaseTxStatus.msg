# 反基地串口发送侧状态
# header.stamp 为本次状态采样时刻；last_send_stamp 为最近一个逻辑包实际开始写串口的 Tsend。
std_msgs/Header header
builtin_interfaces/Time last_send_stamp
uint64 sent_packets
uint32 pending_packets
float32 oldest_pending_ms
uint64 dropped_packets
# 最近一次 300B 逻辑包起始间隔及状态采样窗口内的实际发送频率。
# packet_gap_violation_count 仅统计起始间隔 <= 20ms 的包，非零即不合规。
float32 last_packet_gap_ms
float32 tx_rate_hz
uint64 packet_gap_violation_count
