cmake_minimum_required(VERSION 3.8)
project(hero_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalState.msg"
  "msg/ControlCommand.msg"
  "msg/Armor.msg"
  "msg/ArmorArray.msg"
  "msg/ArmorPose.msg"
  "msg/ArmorPoseArray.msg"
  "msg/TargetState.msg"
  "msg/TargetStateArray.msg"
  "msg/AutoAimDebug.msg"
  "msg/NormalAimDebug.msg"
  "msg/AntitopDebug.msg"
  "msg/AntiBasePacket.msg"
  "msg/AntiBaseTxStatus.msg"
  "msg/AntiBaseDebug.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
