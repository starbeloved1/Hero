# Hero ROS 2 项目记忆

本文件记录跨对话仍有效的项目状态、已作出的设计决定、环境限制和下一步。稳定代码规范不在此重复，见 `STANDARD.md`。

## 项目上下文

- Git 根目录：`Hero/`；新 ROS 2 工程：`Hero/heros/ros2_ws/`；目标版本：ROS 2 Humble。
- `2026HeroAim/` 是旧 C++ 兼容性基准，只读。
- `../ly_aim/` 是参考实现，不是运行时依赖。
- 项目长期目标是英雄机器人 ROS 2 自瞄系统的开发、优化、测试、部署与维护；旧 C++ 迁移是当前阶段。
- 当前协作方式：以一个完整、可验证的小切片为单位实现；构建和测试后先进行代码讲解，再继续迁移。用户偏好直接、逐段的代码阅读，并会参与命名和结构决定。

## 当前功能包状态

| 功能包 | 状态 | 责任与已确定接口 |
| --- | --- | --- |
| `hero_msgs` | 已完成 | 公共消息包。`GimbalState` 对应旧 `SerialPortData`；`ControlCommand` 对应旧 `SerialPortWriteData`。 |
| `gimbal_driver` | 已完成 | 保留旧串口协议：16 字节接收帧、14 字节发送帧、反射 CRC-16（多项式 `0x8408`、初值 `0xffff`）。发布 `/hero/gimbal/state`，订阅 `/hero/gimbal/control`，并在 ROS 弧度与串口角度制之间转换。`enable_fire` 默认关闭；`allow_virtual_serial` 仅限离线测试。 |
| `hero_tf` | 已完成 | 维护 `world -> gimbal_link -> camera_link -> camera_optical_frame`。包名按当前决定保留；实现已整理为 `hero_tf_node.hpp`、`hero_tf_node.cpp` 与 `main.cpp`。 |
| `camera_router` | 已完成第一部分 | 按模式在主 8 mm 与基地相机之间选择并原样转发图像和内参。第二台 8 mm 尚未加入。 |
| `camera_driver` | 已完成 SDK 与本地视频后端 | 每路可通过 YAML 的 `source` 选择 `daheng` 或 `video`。大恒模式严格按 SN 打开主 8 mm 与基地相机，Bayer 图像转换为 `bgr8`；视频模式通过 OpenCV 回放本地文件。两种输入均按同一 YAML 内参和 topic 发布给 `camera_router`。已完成构建、输入源单元测试与本地视频发布验证，待真实相机接入验证。 |

## 坐标系与标定决定

- 动态变换保持旧 `Solver`：`R_world_gimbal = Rz(yaw) * Ry(-pitch)`。
- 主 8 mm 静态标定来自旧 `init.json`：平移 `(0.195, 0.0, 0.065)` m，yaw/pitch/roll 为 `(-0.8, -11.2, 0.0)` 度。
- 旧 `cameraRotationMatrix` 是 gimbal 到 camera；新 `makeCamera2Gimbal()` 返回其逆方向，用于 camera 到 gimbal。
- `camera_link` 为 FLU，`camera_optical_frame` 为 RDF；两者固定轴转换由 `makeFLU2RDF()` 表示。

## 相机选帧决定

- 输入：`/hero/camera/aim8mm/{image_raw,camera_info}` 与 `/hero/camera/base/{image_raw,camera_info}`。
- 输出：`/hero/camera/selected/{image_raw,camera_info}`。
- `MODE_ANTI_BASE`（模式 4）且 `base_camera_enabled: true` 时选择基地相机；其余模式选择主 8 mm。
- 路由器不修改 `Image` 或 `CameraInfo` 的 `header.stamp`、`header.frame_id`。
- 主 8 mm 内参来自旧 `init.json`；基地相机旧配置没有内参，因此当前 `CameraInfo` 的标定数组为零，不能用于基地相机 PnP，获得标定后必须补齐。

## 环境限制与下一步

- Galaxy Linux-x86 SDK `2.6.2606.9251` 已安装在 `/opt/galaxy_sdk`，系统已能加载 `/usr/lib/libgxiapi.so`；官方单相机示例已编译成功。
- 当前没有连接大恒 USB 或 GigE 相机，因此 `camera_driver` 尚不能做真实采图验证。接入相机后需要重新插拔或重启，再使用实际序列号运行 launch。
- 可通过 `camera_driver/config/camera_driver.local.yaml` 离线回放主 8 mm 视频；该配置要求填写本机录像路径，默认关闭基地相机。视频保持真实相机的 topic 与 `CameraInfo` 接口，可用于路由、检测和算法的离线开发；视频分辨率必须与配置标定一致。
- 下一步可使用本地视频迁移和验证检测模块；在上车前仍须验证两台真实相机的设备发现、SN 匹配、图像话题、时间戳和路由切换。
- 后续迁移顺序：相机采集 → 检测 → PnP/解算 → 普通/反陀螺控制 → 追踪预测 → 反基地与 MQTT → 全系统 launch、rosbag 回归、部署验证。

## 已验证命令

```bash
cd Hero/heros/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon test --packages-select gimbal_driver
colcon test --packages-select hero_tf
colcon test --packages-select camera_router
colcon build --packages-up-to camera_driver --symlink-install
colcon test-result --verbose
```

`gimbal_driver` 与 `camera_router` 已通过构建和测试；`hero_tf` 已在此前通过构建、单元测试和 TF 运行验证。
