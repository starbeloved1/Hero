# Hero ROS 2 项目记忆

本文件记录跨对话仍有效的项目状态、已作出的设计决定、环境限制和下一步。稳定代码规范不在此重复，见 `STANDARD.md`。

## 项目上下文

- Git 根目录：`Hero/`；新 ROS 2 工程：`Hero/heros/ros2_ws/`；目标版本：ROS 2 Humble。瞄准域源码统一位于 `ros2_ws/src/hero_aim/`，其中每个子目录仍为独立 ROS package。
- `2026HeroAim/` 是旧 C++ 的只读、相对稳定行为基准。Hero 尚未完成稳定性验证时，必须持续对照其协议、标定含义、公开行为和算法结果。
- `../ly_aim/` 是优秀的 ROS 参考案例，不是运行时依赖；必须主动借鉴其处理思路、包拆分、参数、launch、测试与硬件生命周期，同时保持 Hero 的既定行为。
- ROS 功能块的初步迁移已完成，项目当前阶段不再是大规模迁移，而是熟悉学习代码和通信流程、以录像与实车验证检测稳定性、学习读取可视化与 debug 数据调参，并在实证基础上逐步优化现有代码。
- 当前协作方式：围绕可验证的问题推进；改动按风险完成构建、测试或实车/录像验证，并先带用户读懂相关代码。用户偏好直接、逐段的代码阅读，并会参与命名和结构决定。学习内容统一以 `heros/notes/` 下的 Markdown 文件沉淀；每次学习应更新或新增对应主题笔记，记录目标、相关代码与数据流、验证步骤、结论和下一步。

## 当前功能包状态

| 功能包 | 状态 | 责任与已确定接口 |
| --- | --- | --- |
| `hero_msgs` | 已完成 | 公共消息包。`GimbalState` 对应旧 `SerialPortData`；`ControlCommand` 对应旧 `SerialPortWriteData`；`Armor` 与 `ArmorArray` 传递装甲板二维检测结果；`NormalAimDebug` 与 `AutoAimDebug` 分别公开 mode 1/mode 3 过程量。临时 `AimTarget` 已删除。 |
| `gimbal_driver` | 已完成 | 保留旧串口协议：16 字节接收帧、14 字节发送帧、反射 CRC-16（多项式 `0x8408`、初值 `0xffff`）。发布 `/hero/gimbal/state`，订阅 `/hero/gimbal/control`，并在 ROS 弧度与串口角度制之间转换。`port_name: virtual` 启用显式虚拟状态；真实串口打开失败即退出。 |
| `hero_tf` | 已完成 | 维护 `world -> gimbal_link -> camera_link -> camera_optical_frame`。包名按当前决定保留；实现已整理为 `hero_tf_node.hpp`、`hero_tf_node.cpp` 与 `main.cpp`。 |
| `camera_driver` | 已完成 SDK 与本地视频后端 | 两路相机按职责拆分：aim8mm 只向 mode 1–3 发布图像和标定内参，base 只向 mode 4 发布图像给反基地。非活动大恒流持续排空原始 buffer、视频流持续 `grab`，但不做 BGR 转换或 ROS 发布；待真实相机接入验证。 |
| `armor_detector` | 已完成第一版 | 迁移旧 `ArmorOneStage` 的 OpenVINO 0526 模型推理、颜色筛选、类别映射、NMS 与二维四角点输出。订阅 `/hero/camera/aim8mm/image_raw`，发布 `/hero/detector/armors`（`ArmorArray`）；Foxglove 订阅时按限频发布 `/hero/detector/visualization` 调试图。0526 模型统一放在 `heros/model/0526.onnx`；已完成构建、纯逻辑单测和模型加载启动验证，待录像端到端验证。 |
| `armor_solver` | 已完成 PnP 第一版 | 订阅 `/hero/detector/armors` 和 `/hero/camera/aim8mm/camera_info`，将四个二维角点用 IPPE PnP 解算为 `ArmorPoseArray`，发布 `/hero/solver/armor_poses`。默认按检测时间戳查 TF 并输出 `world`；查不到同刻 TF 时降级为相机光学坐标系输出。Foxglove 订阅时发布 `/hero/solver/markers`。已完成合成角点单元测试，待正确标定录像与真实相机端到端验证。 |
| `aim_core` | 已完成第一版 | 无 ROS 依赖的瞄准核心 C++ 库，提供阻力弹道、角度等价连续化与旧 `BasicAimer` 风格的跳变阈值平滑；`config/ballistics.yaml` 是所有瞄准策略共用且唯一可信的弹丸和发射机构参数来源，缺失时节点启动失败。后续策略复用它，不通过公共 ROS 节点共享算法。 |
| `aim_normal` | 已完成第一版，已加入运行链路 | mode 1 普通瞄准迁移旧 `NormalAim` 的同编号空间连续保持、阻力弹道、大装甲板俯仰距离缩放与角度平滑。订阅 `/hero/solver/armor_poses`、`/hero/gimbal/state`，发布 `/hero/aim/normalaim/controller` 候选和按需 `/hero/aim/normalaim/debug`。默认 `enabled: false`、`enable_fire: false`；验证后分别显式开启。 |
| `aim_antitop` | 已完成第一版，已加入运行链路 | mode 2 反前哨。订阅 `/hero/solver/armor_poses`、`/hero/camera/aim8mm/camera_info` 与 `/hero/gimbal/state`，仅选择旧公开编号为 7 的前哨装甲板，连续选择目标并以其 XY 滑动均值拟合旋转中心；保持旧工程三层 Z 直方图标定参数。利用同刻 TF 将旋转中心投影回原始图像，按旧 20 px 跳变保护、方向窗口、25/35 px 区域滞回、0.5~1.0 s 周期范围和 `3 * 周期 - 系统延迟 - 飞行时间 + 方向偏置` 完成一次性开火倒计时。区域上升沿与周期在图像 `T0`（`Tzone`）时间轴测量，倒计时以绝对 `Tpermit` 对当前 `Tcontrol` 计算剩余时间，避免算法延迟污染周期。发布 `/hero/aim/antitop/controller` 候选、按需 `/hero/aim/antitop/debug` 与 `/hero/aim/antitop/markers`；Marker 用青色显示 T0 观测板/半径、蓝色方块和粉色箭头显示同相位三层旋转模型、橙色显示标定三层/旋转轴、绿色显示 Tcontrol 的弹道瞄点。默认 `enabled: false`、`enable_fire: false`；上车前必须用正确标定的前哨录像验证区域与时序。 |
| `hero_antibase` | 已完成第一版，已加入运行链路 | mode 4 反基地，位于功能包同级目录而非 `hero_aim`。仅在 mode4 消费 selected 基地图像，迁移旧工程中心裁剪/偏移、静态简化与拖影、低延迟 H.264、292B 数据分包、积压裁剪和自适应码率；发布 `/hero/antibase/packets`、`/hero/antibase/debug`，可选 `/hero/antibase/visualization`。不使用检测、PnP、TF、弹道或 yaw/pitch 控制。 |
| `aim_predictor` | 已完成第一版，默认启用 | mode 3 自瞄预测。订阅 `/hero/solver/armor_poses` 与 `/hero/gimbal/state`，以 Hero 编号分组维护四装甲板 EKF 状态，发布 `/hero/aim/autoaim/target_states`。不依赖旧 Hero 的整车 YOLO；会根据同刻相机 TF 将 PnP 平面法向统一为可见面方向，再进行面板关联。`header.stamp` 是预测状态时刻 `Tstate`，`measurement_stamp` 保存原始图像时刻 `T0`。同时按需发布 `/hero/aim/autoaim/predictor/markers`：青色预测四板与红色内向箭头代表 `Tstate`，黄色实际观测与橙色箭头代表 `T0`。 |
| `aim_auto` | 已完成第一版，已加入运行链路 | mode 3 独立控制策略。消费预测状态与云台状态，持续锁车、失效后最近回退；以同一预测装甲板计算 yaw/pitch，迭代补偿飞行时间与机构响应，通过相位、云台误差、连续帧和高加速度四道门控制开火。发布 `/hero/aim/autoaim/controller` 与 `/hero/aim/autoaim/debug`。按需发布 `/hero/aim/autoaim/controller/markers`：紫蓝四板与粉色内向箭头、绿色选板和洋红色瞄点均表示 `Taim`，只对应当前锁定的一台车。默认计算候选但 `enable_fire: false`。 |
| `command_mux` | 已完成多入口安全仲裁 | 唯一发布 `/hero/gimbal/control` 的安全仲裁节点。按 mode 接收 `aim_normal`、`aim_antitop`、`aim_auto` 三路独立候选，候选 topic 固定为：`/hero/aim/normalaim/controller`、`/hero/aim/antitop/controller`、`/hero/aim/autoaim/controller`；只转发当前 mode 对应且未超过 `max_command_age_sec` 的候选命令。mode4 不存在控制候选；模式切换清空全部缓存。 |

## 坐标系与标定决定

- 动态变换保持旧 `Solver`：`R_world_gimbal = Rz(yaw) * Ry(-pitch)`。
- 主 8 mm 静态标定来自旧 `init.json`：平移 `(0.195, 0.0, 0.065)` m，yaw/pitch/roll 为 `(-0.8, -11.2, 0.0)` 度。
- 旧 `cameraRotationMatrix` 是 gimbal 到 camera；新 `makeCamera2Gimbal()` 返回其逆方向，用于 camera 到 gimbal。
- `camera_link` 为 FLU，`camera_optical_frame` 为 RDF；两者固定轴转换由 `makeFLU2RDF()` 表示。

## 相机职责决定

- aim8mm 在 mode 1–3 发布 `/hero/camera/aim8mm/{image_raw,camera_info}`，图像 frame 固定为 `camera_optical_frame`，供检测、PnP 和反前哨使用。
- base 仅在 mode 4 发布 `/hero/camera/base/image_raw`，frame 为 `base_camera_optical_frame`，只供 `hero_antibase` 编码，不生成 CameraInfo，也不参与 PnP、TF 或检测。
- 两台设备均保持打开和取流；非活动大恒流仅排空原始 buffer，非活动视频仅推进解码器，避免旧帧积压并节省 BGR 转换与 ROS 传输。

## 颜色与检测接口决定

- 串口接收帧第 12 字节在旧工程中名为 `color`；旧 `ArmorOneStage::setColorFlag()` 会将 `0` 和 `1` 翻转后再筛选模型输出，因此该字段的准确语义是己方机器人颜色。`GimbalState` 统一命名为 `robot_color`，不再使用容易误导的 `enemy_color`。
- `armor_detector` 的 `target_color: -1` 表示根据 `robot_color` 自动取相反颜色；`-2` 表示不做颜色筛选，适用于没有串口状态的离线录像；`0` 或 `1` 表示固定筛选颜色。
- `ArmorArray.header` 与其中每个 `Armor.header` 都继承输入图像的时间戳与相机 `frame_id`。每个装甲板输出四个像素角点、Hero 编号、模型颜色类别和置信度；三维 PnP 不属于本包，留给后续解算模块。
- `ArmorPoseArray` 与其中每个 `ArmorPose` 的 `header` 使用相同坐标系；`pose.position` 单位为米，`reprojection_error` 单位为像素。`ArmorPose.image_center` 保留其 PnP 输入四角点的原始像素中心，供 `aim_antitop` 与同刻重投影的旋转中心比较。`armor_solver` 目前保持旧工程的装甲板尺寸、角点顺序和“仅 ID 1 为大装甲板”的映射，但 PnP 后端暂采用 OpenCV IPPE；与旧 PoseLib PnPL 的实际精度差异必须在正确标定的录像和实车上验证。

## 环境限制与下一步

- Galaxy Linux-x86 SDK `2.6.2606.9251` 已安装在 `/opt/galaxy_sdk`，系统已能加载 `/usr/lib/libgxiapi.so`；官方单相机示例已编译成功。
- 当前用户环境已有 OpenVINO C++ 运行时，位于用户本地安装目录。`heros/build.sh` 与 `heros/run.sh` 会自动定位其 CMake 配置和运行库；路径不得写入项目 YAML 或 CMake。若自动定位失败，可由用户显式设置 `OpenVINO_DIR`。
- 当前没有连接大恒 USB 或 GigE 相机，因此 `camera_driver` 尚不能做真实采图验证。接入相机后需要重新插拔或重启，再使用实际序列号运行 launch。
- 在 `camera_driver.yaml` 中将相机 `source` 设为 `video` 并填写录像路径即可离线回放；`data/文件名.avi` 表示项目根目录的 `heros/data/文件名.avi`。视频保持真实相机的 topic 与 `CameraInfo` 接口，可用于路由、检测和算法的离线开发；驱动自动将 `Image` 与 `CameraInfo` 的宽高设为视频实际分辨率，但 YAML 内参仍必须对应该分辨率。配合 `gimbal_driver.yaml` 中的 `port_name: virtual`，可完整提供本地云台状态。
- `heros/run.sh` 是当前已迁移功能包的一键入口。脚本直接依次启动 `gimbal_driver`、`hero_tf`、`camera_driver`、`hero_antibase`、`armor_detector`、`armor_solver`、`aim_predictor`、`aim_auto`、`aim_normal`、`aim_antitop`、`command_mux` 与 `foxglove_bridge`，不使用额外的 `bringup` 功能包，也不覆盖任何 YAML 参数。mode3 默认计算候选但不开火；普通瞄准与反前哨默认不发候选且不开火，待录像和实车验证后由 YAML 显式开启。是否读取视频、相机与串口参数均由各包 YAML 决定；Foxglove Desktop 连接 `ws://localhost:8765` 即可观察系统。
- 已在无真实相机的开发机上验证过已迁移节点的组合启动；因当前 YAML 选择大恒相机且开发机未接相机，`camera_driver` 正确报告“未发现大恒相机”并退出。上车前应确认相机序列号和串口设备名。`gimbal_driver` 的虚拟模式已完成构建、单元测试和 ROS 话题验证。
- 后续优化项：在录像与实车确认完整链路和实时正确性后，评估将相机采集、相机路由和装甲板检测放入组件容器并使用进程内通信，以及将检测器改为 OpenVINO 双请求异步流水线。优化必须先有测量与对照结果，不凭推测重构。
- 可视化与性能调优的当前优先级：使用已有 Foxglove 话题和调试图学习链路数据，建立检测稳定性、延迟和丢帧的观察方法；再根据录像和实车证据完善布局、频率、延迟统计与参数，避免仅为观测而扰动行为。
- 最终多模式控制决定：模式唯一来源为 `/hero/gimbal/state` 的 `GimbalState.mode`，模式切换不重启节点。三个功能包固定命名为 `aim_normal`、`aim_antitop`、`aim_auto`；topic 中完整写出模式名，固定为 `/hero/aim/normalaim/controller`、`/hero/aim/antitop/controller`、`/hero/aim/autoaim/controller`。mode 1 的 `aim_normal` 迁移旧 `NormalAim`，自身完成目标选择、弹道、角度平滑并发布普通候选；mode 2 的 `aim_antitop` 迁移旧 `AntiTop`，自身完成前哨跟踪、旋转中心/Z 标定、射击状态机、弹道并发布反陀螺候选；mode 3 的 `aim_predictor` 独立维护纯三维预测状态，`aim_auto` 负责锁车、选中预测装甲板、飞行时间迭代与安全开火决策并发布自瞄候选。mode3 不迁移旧工程的整车 YOLO 硬门槛；高加速度第一版仅禁火，不使用依赖二维 Tracker 面板编号的旧观测回退。反基地因相机、目标和逻辑独立而保留独立入口。所有策略都只发布 `ControlCommand` 候选，`command_mux` 按 mode 放行正确入口并独占发布 `/hero/gimbal/control`，避免多发布者交错控制云台；候选命令必须包含生成时刻，仲裁器只接受未超过 `max_command_age_sec` 的候选命令。公共弹道、角度归一化、控制消息辅助函数放入无 ROS 依赖的 `aim_core` C++ 库，不以一个公共 ROS 节点强行共享。
- 当前验证与学习顺序：先用下载的本地视频理解相机、路由、检测、PnP、各 mode 及其通信 topic，核对 `/hero/detector/armors`、`/hero/solver/armor_poses` 与各策略 debug/markers 的时间戳、坐标系、编号、颜色、角点、深度和重投影误差；再以录像和 `2026HeroAim` 对照检测稳定性；最后上车验证两台真实相机的设备发现、SN 匹配、图像话题、时间戳、路由切换、云台控制和各模式行为。录像分辨率与内参不一致时，不得相信三维数值。
- mode4 串口协议决定：`AntiBasePacket` 保留源图像 `T0`，含 8B 小端 `sequence_id` 和 292B H.264 数据；`gimbal_driver` 是唯一串口拥有者，写为 5 个 `'#' + chunk_index + 60B + CRC16` 的 64B 帧，分片间隔 2 ms、逻辑包起始间隔 21 ms。其 `/hero/antibase/tx_status` 的 `last_send_stamp` 是实际写出时刻 `Tsend`。mode4 自动发送码流，普通 14B 云台控制帧在该模式被抑制；切换模式会清空未发送包。虚拟串口中发送按逻辑成功计数，可用于离线链路验证。
- 反基地 MQTT 裁判闭环已实现：`gimbal_driver` 在最终 21 ms 节拍且五个串口分片均成功后，可选地镜像同一 300B 逻辑包至 UDP `127.0.0.1:9999`（默认关闭，实车保持关闭）；`tools/hero_judge/hero_judge.yaml` 是 UDP/MQTT 监听地址、端口、topic、QoS、间隔规则和统计周期的唯一默认配置，`hero_judge.py` 可用命令行临时覆盖它。裁判以单调时钟比较相邻**收到**包，间隔 `<= 20 ms` 的当前包直接丢弃且更新“上一个收到包”时刻，放行包以比赛格式转发至本机 Mosquitto。根目录 `hero_judge.sh` 启动仅监听 `127.0.0.1:3333` 的 broker 和裁判；修改 YAML 的 MQTT host/port 时须同步更新该目录的 `mosquitto_hero_judge.conf`。MQTT 必须为 `CustomByteBlock` topic、QoS 1、303B `0x0A 0xAC 0x02 + 300B`，其中 300B 为 8B 小端序号加 292B H.264。`Hero_client/src/sub_win.py` 是实际比赛客户端的协议解析、序号/缺包处理、H.264 组帧解码基准，不能另写不兼容接收器。`2026HeroAim/src/Mqtt_pub/antibase_receiver_mqtt.py` 是只读协议参考。开发机已安装 `mosquitto`、`mosquitto-clients`、`python3-paho-mqtt`；独立端到端验证已确认 303B 封装正确，25 ms 级包放行、5.1 ms 包丢弃。
- 当前工作顺序：熟悉代码与通信流程 → 录像验证检测和各 debug/markers 的稳定性 → 实车验证相机、云台和各模式行为 → 基于证据调参及优化；全过程持续对照 `2026HeroAim` 的稳定行为，并借鉴 `ly_aim` 的处理思路。临时 `AimTarget` 与“公共 aim_controller”已移除，不得将新算法接回这类过渡接口。

## 时间戳链路

- 只有 aim8mm 具有 `timestamp_mode` 与 `timestamp_offset`。默认 `host`，以 `GXDQBuf` 返回后、BGR 转换前记录的主机时刻作为图像近似 T0；视频输入只允许此模式。base 固定使用同一主机取帧时刻，不提供时间偏移或设备映射。
- aim8mm 的真实大恒输入可显式使用 `timestamp_mode: device`。驱动读取 SDK 时间戳频率，先以首帧建立设备计数到主机单调时间的相对映射，再用 0.02 系数低通修正。映射减少主机处理抖动，但其偏移仍包含相机到主机的传输延迟，不等同于精确曝光时刻。设备计数回退、频率变化、时间戳不递增或 ROS 时钟相对单调时钟跳变超过 100 ms 时，设备模式重置映射并丢弃跨边界帧；`use_sim_time` 不支持设备模式。
- `gimbal_driver` 的真实串口状态时间在有效帧回调入口记录，配置 `state_timestamp_offset` 可进行人工补偿，单位为秒，默认 0。虚拟串口不使用该补偿。`hero_tf` 原样使用 `GimbalState.header.stamp`，不得再叠加补偿。
- 未接大恒硬件时只验证默认视频／虚拟串口路径。上车后先在 host/device 两种相机模式下记录固定目标、云台左右转动和停稳数据，再根据世界坐标稳定性、TF 查询失败率和帧间抖动决定是否启用 device 与设置人工补偿。

## 已验证命令

```bash
cd Hero/heros/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon test --packages-select gimbal_driver
colcon test --packages-select hero_tf
colcon build --packages-up-to camera_driver --symlink-install
colcon test-result --verbose
```

`gimbal_driver` 已通过构建和测试；`hero_tf` 已在此前通过构建、单元测试和 TF 运行验证。

## 2026-09-07 代码评估补充

- 本次在既有构建产物上执行 `colcon test --event-handlers console_cohesion+ --return-code-on-test-failure`，14 个包完成，退出码 0；未重新构建，也未进行实车验证
- 源码审阅发现待验证及修复边界：`gimbal_driver::sendCommand()` 未检查缓存控制命令年龄，仲裁器停止输出后仍会重发末条命令；`aim_normal` 未检查输入观测年龄，仅为生成的候选写当前时间，仲裁器的候选超时不能替代观测超时
- 大恒后端读取了 `device_timestamp`，但上层未传递该字段，图像在取图和 BGR 转换完成后以 `now()` 打时间戳；实际采集时刻与 ROS 时钟的映射仍需实现及硬件验证
- `resolveModelPath()` 与 `resolveVideoPath()` 向上查找目录时未处理根目录的父目录等于自身；相对资源路径不存在时可能持续循环，需补充终止条件
