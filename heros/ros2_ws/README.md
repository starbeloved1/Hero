# Hero ROS 2 工作区

`ros2_ws` 是英雄机器人独立的 ROS 2 Humble 工作区。迁移期间，`2026HeroAim` 始终作为只读的行为参考；新工程按“完成一个可验证模块，再迁移下一个模块”的节奏推进。

## 约定

- 新功能包使用职责名，不加 `hero_` 前缀；`hero_msgs` 是公共消息包的固定例外。英雄机器人话题统一位于 `/hero` 命名空间下。
- ROS 边界的角度使用弧度、距离使用米；测量消息的时间戳表示数据采集时刻，控制命令的时间戳表示命令生成或仲裁输出时刻。
- 串口仍保持旧协议的角度制和字节布局，只有 `gimbal_driver` 可以在串口协议与 ROS 消息间转换。
- Node 只承担通信和硬件适配；算法将保持为可测试、无 ROS 依赖的 C++ 库。
- 高频状态和控制话题使用深度为 1 的 best-effort QoS，只处理最新数据，不积压旧数据。
- `src/hero_aim/` 是瞄准算法域的源码分组，内部的 `aim_core`、`aim_normal`、`aim_antitop`、`aim_predictor`、`aim_auto` 等仍是独立 ROS package；目录分组不改变包名或 topic。

构建当前工作区时，推荐在 `heros/` 目录执行：

```bash
./build.sh
```

脚本会自动加载 ROS 2，并定位本机用户目录下的 OpenVINO C++ 环境。也可向它透传普通 colcon 参数，例如只构建检测器：

```bash
./build.sh --packages-select armor_detector
```

当前已迁移的功能包：

- `hero_msgs`：云台状态、控制命令、装甲板检测/位姿和策略调试信息的公共消息定义。
- `gimbal_driver`：旧串口协议、CRC、云台状态发布与控制命令下发。
- `hero_tf`：云台姿态驱动的动态坐标变换，以及相机标定得到的静态坐标变换。
- `camera_router`：按云台模式选择主 8mm 或基地相机的图像与相机内参。
- `camera_driver`：通过大恒 SDK 或本地视频发布相机图像与标定参数。
- `armor_detector`：使用 0526 OpenVINO 模型输出装甲板二维四角点、编号、颜色和置信度。
- `armor_solver`：根据装甲板四角点和 `CameraInfo` 执行 PnP，输出三维装甲板位姿。
- `aim_core`：无 ROS 依赖的瞄准核心库，提供弹道求解、角度连续化和角度平滑。
- `aim_normal`：mode 1 普通瞄准，完成连续目标选择、弹道、角度平滑，并向 `/hero/aim/normalaim/controller` 发布候选控制。
- `aim_predictor`：mode 3 自瞄预测，维护每个目标车的四装甲板 EKF 状态，并向 `/hero/aim/autoaim/target_states` 发布预测结果；不直接控制云台。
- `aim_auto`：mode 3 自瞄控制，锁定目标车、预测命中时刻的装甲板、求弹道并向 `/hero/aim/autoaim/controller` 发布候选；`/hero/aim/autoaim/debug` 用于查看选择与开火门。
- `aim_antitop`：mode 2 反前哨。连续选择前哨板、拟合 XY 旋转中心、标定三层 Z 高度；利用同刻 TF 与 `CameraInfo` 的内参、畸变参数，将旋转中心重投影回原始图像，完成方向识别、射击区域、周期统计和倒计时开火。候选发布到 `/hero/aim/antitop/controller`，过程量发布到 `/hero/aim/antitop/debug`，三维过程量发布到 `/hero/aim/antitop/markers`；默认禁用且不开火。
- `hero_antibase`：mode 4 反基地。仅消费 selected 基地画面，完成旧工程的预处理、H.264 编码、292B 数据分包与自适应码率；不使用检测、PnP、TF、弹道或 yaw/pitch 控制。
- `command_mux`：按云台模式仲裁 mode1、mode2、mode3 三路候选，并独占发布 `/hero/gimbal/control`。mode4 没有控制候选，云台驱动只发送反基地码流。

`aim_core/config/ballistics.yaml` 保存所有瞄准策略共用的弹速、阻力、重力、弹丸尺寸/质量、枪口偏移和迭代次数。每个策略的 launch 都应先加载此文件，再加载自身 YAML；策略自身只保存目标选择、控制、话题和安全开关等差异参数。

相机驱动的原始 topic 按物理来源命名，例如 `aim8mm`、`base`；它们的 `frame_id` 同样带来源前缀。`camera_router` 输出的 `/hero/camera/selected/*` 则是后续算法唯一使用的逻辑相机接口，统一使用 `camera_optical_frame`，并保留原始采集时间戳。

启动已完成的云台通信与坐标系部分：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py
ros2 launch hero_tf hero_tf.launch.py
ros2 launch camera_router camera_router.launch.py
ros2 launch armor_detector armor_detector.launch.py
```

## 本地视频回放

`camera_driver` 的每一路输入可通过 `<相机名>.source` 选择 `daheng` 或 `video`。选择
`video` 后，节点使用 OpenCV 读取本地视频，但仍发布与真实相机完全相同的图像和标定话题，
所以可以在不连接机器人时验证路由、检测和后续算法。

以 8mm 相机为例，在 `camera_driver.yaml` 中将 `aim8mm.source` 改为 `video`，填写
`aim8mm.video_path: data/你的录像.avi`，并关闭不使用的相机。该路径相对项目根目录 `heros/`，
对应 `heros/data/你的录像.avi`。视频输入会自动读取视频实际宽高，
不使用 YAML 的 `width`、`height`。焦距、主点和畸变参数仍由 YAML 提供，因此录像分辨率应当
与这套内参标定时的分辨率一致。然后启动：

```bash
ros2 launch camera_driver camera_driver.launch.py \
  config_file:=/你的本地路径/camera_driver.yaml
```

`video_rate_hz: 0.0` 表示按录像记录的 FPS 回放，`video_loop: true` 表示读到末尾后从头循环。
本地视频只是离线输入，不替代真实相机、时间同步和硬件链路验证。

本地回放时，将 `gimbal_driver.yaml` 的 `port_name` 改为 `virtual`，虚拟云台会持续发布零 yaw、零 pitch 的状态。
它的默认模式为普通模式、己方颜色为蓝色；检测器 `target_color: -1` 会据此自动筛选红色目标。运行中可直接切换：

```bash
ros2 param set /gimbal_driver_node virtual_mode 4
ros2 param set /gimbal_driver_node virtual_robot_color 1
```

`virtual_mode` 取值为 `1` 普通、`2` 反陀螺、`3` 自瞄、`4` 反基地；`virtual_robot_color` 中 `0` 为蓝色、`1` 为红色。
真实车端必须保留实际设备名，例如 `/dev/ttyACM0`；设备打开失败会使节点启动失败，不会自动进入虚拟模式。
若不希望按颜色筛选，仍可将 `armor_detector.yaml` 的 `target_color` 改为 `-2`。

## 一键启动当前系统

在 `heros/` 目录执行：

```bash
./run.sh
```

脚本会直接启动目前已迁移的 `gimbal_driver`、`hero_tf`、`camera_driver`、`camera_router`、`hero_antibase`、`armor_detector`、`armor_solver`、`aim_predictor`、`aim_auto`、`aim_normal`、`aim_antitop`、`command_mux`，以及 `foxglove_bridge`；不额外使用总启动功能包。
该命令不覆盖任何参数，全部行为以各功能包 `config/` 目录中的 YAML 为准：车上使用串口和大恒相机；需要本地回放时再由你把对应 YAML 改为视频源。
运行后在 Foxglove Desktop 中连接 `ws://localhost:8765`，即可查看话题和 TF。若此前已手动启动 Bridge，应先停止它，避免端口 `8765` 冲突。

`aim_normal` 默认只计算与发布 `/hero/aim/normalaim/debug` 调试量，不发布候选、更不会开火；这是为当前录像、内参与实车弹道尚未完成统一验证设置的安全默认值。验证完成后在 `aim_normal.yaml` 中把 `enabled` 设为 `true` 才会向仲裁器提供控制候选，`enable_fire` 需要单独显式设为 `true`。Foxglove 的 **Raw Messages** 或 **Plot** 面板可查看其目标点、原始/平滑 yaw、pitch、飞行时间与开火状态。

mode 3 的 `aim_predictor` 与 `aim_auto` 默认会在模式 3 下计算、发布预测状态和控制候选，但 `aim_auto.enable_fire: false`，因此候选命令的 `shoot_status` 固定为 0。`TargetStateArray.header.stamp` 是预测状态时刻 `Tstate`，新增的 `measurement_stamp` 是原始图像时刻 `T0`；预测器已经完成 `T0 → Tstate` 的外推。Foxglove 中优先观察 `/hero/aim/autoaim/target_states` 与 `/hero/aim/autoaim/debug`，后者会显示目标锁定、选中面板、命中时刻、弹道、相位、云台误差及每道开火门。

在 Foxglove 新建 **3D** 面板并选择 `/hero/aim/autoaim/predictor/markers`，可同时查看 Predictor 的四板模型与本帧实际 PnP 观测：青色方块为每台已跟踪车辆在 `Tstate` 的四块预测装甲板，红色箭头为板指向车辆中心的内向方向；黄色方块和橙色箭头为 `T0` 的实际观测。该话题只有 3D 面板订阅时才发布，便于检查四板模型是否跳错板、半径是否发散或朝向是否颠倒。

另一个 3D 话题 `/hero/aim/autoaim/controller/markers` 用于检查控制决策：紫蓝方块和粉色箭头表示**已锁定的一台车**在预计命中时刻 `Taim` 的四块装甲板及其内向方向；绿色方块是最终选中的装甲板，洋红色球是用于解弹道的 `aim_point`。建议先为 Predictor 和 Controller 分别建立两个 3D 面板；若叠加到同一面板，会同时出现 `T0`、`Tstate`、`Taim` 三个时刻的数据，信息较多但便于分析整段预测链路。

反前哨可在独立 **3D** 面板选择 `/hero/aim/antitop/markers`。青色球是本帧 `T0` 的连续跟踪前哨板，青色线段表示它相对拟合旋转轴的水平半径；同一相位的蓝色方块和粉色内向箭头是由该观测重建的前哨三层板模型，会随连续帧更新而绕轴转动。完成 Z 标定后，橙色竖线与三个橙色球表示旋转轴和低、中、高三层高度。绿色球是本次控制时刻 `Tcontrol` 实际交给弹道解算的瞄点，即“旋转中心 XY + 当前目标 Z”。旋转中心本身只拟合 XY，故图中不会把 `rotation_center.z = 0` 误画成物理高度。

## 检测可视化

在 Foxglove 新建 **Image** 面板并选择 `/hero/detector/visualization`，可直接查看主 8 mm 或当前选中相机画面上的装甲板四角、编号、颜色和置信度。
该图是调试专用副本：原始 `/hero/camera/selected/image_raw` 与结构化检测结果 `/hero/detector/armors` 不会被修改。发布上限由 `visualization_rate_hz` 配置，且只有 Image 面板订阅该话题时才复制、绘制和发布图像；实战或性能测试时可将 `visualization_enabled` 设为 `false`。

在 Foxglove 新建 **3D** 面板并选择 `/hero/solver/markers`，可查看 PnP 后位于 `world` 坐标系的三维装甲板。结构化位姿数据位于 `/hero/solver/armor_poses`，其中 `reprojection_error` 越小表示该帧二维角点与 PnP 结果越一致。录像分辨率与内参不一致时，Marker 只能用于检查链路，不可视为真实空间位置。

## Mode4 反基地

云台状态切到 `mode: 4` 后，`camera_router` 将基地相机切到 selected，`hero_antibase` 开始生成 H.264 码流逻辑包，`gimbal_driver` 自动按旧协议写入真实串口：每个逻辑包为 8B 小端序号加 292B H.264 数据，拆成 5 个 `'#' + 分片序号 + 60B + CRC16` 的 64B 帧，分片间隔 2 ms，完整包起始间隔为 21 ms（约 47.62 Hz）。反基地要求使用 1 Mbaud 串口；320 个实际串口字节在 115200 baud 下至少需要约 27.8 ms，物理上无法达到 50 Hz。

`/hero/aim/antibase/packets.header.stamp` 始终是源图像时刻 `T0`；`/hero/aim/antibase/tx_status.last_send_stamp` 是实际串口开始发送时刻 `Tsend`。Foxglove 可查看 `/hero/aim/antibase/debug` 的编码字节数、码率、运动比例、缓存和丢包统计；`tx_status.tx_rate_hz`、`last_packet_gap_ms` 与 `packet_gap_violation_count` 分别用于确认实际频率、最近完整包间隔和是否出现过不合规间隔。将 `hero_antibase.yaml` 的 `visualization_enabled` 设为 `true` 后，可在 `/hero/aim/antibase/visualization` 查看实际送入编码器的 320×320 预处理图像。UDP/MQTT 本地调试镜像尚未迁移。
