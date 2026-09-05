# 本消息 header.stamp 表示数组内预测状态实际对应的时刻 Tstate
std_msgs/Header header

# 输入图像及 ArmorPoseArray 的采集时刻 T0
builtin_interfaces/Time measurement_stamp

hero_msgs/TargetState[] targets
