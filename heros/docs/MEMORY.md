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
| `hero_msgs` | 已完成 | 公共消息包。`GimbalState` 对应旧 `SerialPortData`；`ControlCommand` 对应旧 `SerialPortWriteData`；`Armor` 与 `ArmorArray` 传递装甲板二维检测结果。 |
| `gimbal_driver` | 已完成 | 保留旧串口协议：16 字节接收帧、14 字节发送帧、反射 CRC-16（多项式 `0x8408`、初值 `0xffff`）。发布 `/hero/gimbal/state`，订阅 `/hero/gimbal/control`，并在 ROS 弧度与串口角度制之间转换。`port_name: virtual` 启用显式虚拟状态；真实串口打开失败即退出。 |
| `hero_tf` | 已完成 | 维护 `world -> gimbal_link -> camera_link -> camera_optical_frame`。包名按当前决定保留；实现已整理为 `hero_tf_node.hpp`、`hero_tf_node.cpp` 与 `main.cpp`。 |
| `camera_router` | 已完成第一部分 | 按模式在主 8 mm 与基地相机之间选择并原样转发图像和内参。第二台 8 mm 尚未加入。 |
| `camera_driver` | 已完成 SDK 与本地视频后端 | 每路可通过 YAML 的 `source` 选择 `daheng` 或 `video`。大恒模式严格按 SN 打开主 8 mm 与基地相机，Bayer 图像转换为 `bgr8`；视频模式通过 OpenCV 回放本地文件。两种输入均按同一 YAML 内参和 topic 发布给 `camera_router`。已完成构建、输入源单元测试与本地视频发布验证，待真实相机接入验证。 |
| `armor_detector` | 已完成第一版 | 迁移旧 `ArmorOneStage` 的 OpenVINO 0526 模型推理、颜色筛选、类别映射、NMS 与二维四角点输出。订阅 `/hero/camera/selected/image_raw`，发布 `/hero/detector/armors`（`ArmorArray`）；Foxglove 订阅时按限频发布 `/hero/detector/visualization` 调试图。0526 模型统一放在 `heros/model/0526.onnx`；已完成构建、纯逻辑单测和模型加载启动验证，待录像端到端验证。 |
| `armor_solver` | 已完成 PnP 第一版 | 订阅 `/hero/detector/armors` 和 `/hero/camera/selected/camera_info`，将四个二维角点用 IPPE PnP 解算为 `ArmorPoseArray`，发布 `/hero/solver/armor_poses`。默认按检测时间戳查 TF 并输出 `world`；查不到同刻 TF 时降级为相机光学坐标系输出。Foxglove 订阅时发布 `/hero/solver/markers`。已完成合成角点单元测试，待正确标定录像与真实相机端到端验证。 |

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

## 颜色与检测接口决定

- 串口接收帧第 12 字节在旧工程中名为 `color`；旧 `ArmorOneStage::setColorFlag()` 会将 `0` 和 `1` 翻转后再筛选模型输出，因此该字段的准确语义是己方机器人颜色。`GimbalState` 统一命名为 `robot_color`，不再使用容易误导的 `enemy_color`。
- `armor_detector` 的 `target_color: -1` 表示根据 `robot_color` 自动取相反颜色；`-2` 表示不做颜色筛选，适用于没有串口状态的离线录像；`0` 或 `1` 表示固定筛选颜色。
- `ArmorArray.header` 与其中每个 `Armor.header` 都继承输入图像的时间戳与相机 `frame_id`。每个装甲板输出四个像素角点、Hero 编号、模型颜色类别和置信度；三维 PnP 不属于本包，留给后续解算模块。
- `ArmorPoseArray` 与其中每个 `ArmorPose` 的 `header` 使用相同坐标系；`pose.position` 单位为米，`reprojection_error` 单位为像素。`armor_solver` 目前保持旧工程的装甲板尺寸、角点顺序和“仅 ID 1 为大装甲板”的映射，但 PnP 后端暂采用 OpenCV IPPE；与旧 PoseLib PnPL 的实际精度差异必须在正确标定的录像和实车上验证。

## 环境限制与下一步

- Galaxy Linux-x86 SDK `2.6.2606.9251` 已安装在 `/opt/galaxy_sdk`，系统已能加载 `/usr/lib/libgxiapi.so`；官方单相机示例已编译成功。
- 当前用户环境已有 OpenVINO C++ 运行时，位于用户本地安装目录。`heros/build.sh` 与 `heros/run.sh` 会自动定位其 CMake 配置和运行库；路径不得写入项目 YAML 或 CMake。若自动定位失败，可由用户显式设置 `OpenVINO_DIR`。
- 当前没有连接大恒 USB 或 GigE 相机，因此 `camera_driver` 尚不能做真实采图验证。接入相机后需要重新插拔或重启，再使用实际序列号运行 launch。
- 在 `camera_driver.yaml` 中将相机 `source` 设为 `video` 并填写录像路径即可离线回放；`data/文件名.avi` 表示项目根目录的 `heros/data/文件名.avi`。视频保持真实相机的 topic 与 `CameraInfo` 接口，可用于路由、检测和算法的离线开发；驱动自动将 `Image` 与 `CameraInfo` 的宽高设为视频实际分辨率，但 YAML 内参仍必须对应该分辨率。配合 `gimbal_driver.yaml` 中的 `port_name: virtual`，可完整提供本地云台状态。
- `heros/run.sh` 是当前已迁移功能包的一键入口。脚本直接依次启动 `gimbal_driver`、`hero_tf`、`camera_driver`、`camera_router`、`armor_detector`、`armor_solver` 与 `foxglove_bridge`，不使用额外的 `bringup` 功能包，也不覆盖任何 YAML 参数。是否读取视频、相机与串口参数均由各包 YAML 决定；Foxglove Desktop 连接 `ws://localhost:8765` 即可观察系统。
- 已在无真实相机的开发机上验证过已迁移节点的组合启动；因当前 YAML 选择大恒相机且开发机未接相机，`camera_driver` 正确报告“未发现大恒相机”并退出。上车前应确认相机序列号和串口设备名。`gimbal_driver` 的虚拟模式已完成构建、单元测试和 ROS 话题验证：默认发布普通模式/蓝色，动态切换到反基地/红色后由 `camera_router` 正确切到基地相机。
- 后续优化项：在完整链路和实时正确性验证后，评估将相机采集、相机路由和装甲板检测放入组件容器并使用进程内通信，以及将检测器改为 OpenVINO 双请求异步流水线。当前迁移阶段不为此重构，优先补齐完整功能链路。
- 多模式控制决定：模式唯一来源为 `/hero/gimbal/state` 的 `GimbalState.mode`。模式切换不重启节点；各策略发布各自的候选控制指令，后续由唯一的 `command_mux` 按 mode 选择并独占发布 `/hero/gimbal/control`，避免多发布者交错控制云台。
- 下一步使用下载的本地视频核对 `/hero/detector/armors` 与 `/hero/solver/armor_poses` 的时间戳、相机坐标系、编号、颜色、角点、PnP 深度和重投影误差；录像分辨率与内参不一致时，不得相信三维数值。随后迁移普通/反陀螺策略与追踪预测模块。在上车前仍须验证两台真实相机的设备发现、SN 匹配、图像话题、时间戳和路由切换。
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
