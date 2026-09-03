# Hero ROS 2 工作区

`ros2_ws` 是英雄机器人独立的 ROS 2 Humble 工作区。迁移期间，`2026HeroAim` 始终作为只读的行为参考；新工程按“完成一个可验证模块，再迁移下一个模块”的节奏推进。

## 约定

- 新功能包使用职责名，不加 `hero_` 前缀；`hero_msgs` 是公共消息包的固定例外。英雄机器人话题统一位于 `/hero` 命名空间下。
- ROS 边界的角度使用弧度、距离使用米，时间戳表示数据采集时刻。
- 串口仍保持旧协议的角度制和字节布局，只有 `gimbal_driver` 可以在串口协议与 ROS 消息间转换。
- Node 只承担通信和硬件适配；算法将保持为可测试、无 ROS 依赖的 C++ 库。
- 高频状态和控制话题使用深度为 1 的 best-effort QoS，只处理最新数据，不积压旧数据。

构建当前工作区时，推荐在 `heros/` 目录执行：

```bash
./build.sh
```

脚本会自动加载 ROS 2，并定位本机用户目录下的 OpenVINO C++ 环境。也可向它透传普通 colcon 参数，例如只构建检测器：

```bash
./build.sh --packages-select armor_detector
```

当前已迁移的功能包：

- `hero_msgs`：云台状态与控制命令的公共消息定义。
- `gimbal_driver`：旧串口协议、CRC、云台状态发布与控制命令下发。
- `hero_tf`：云台姿态驱动的动态坐标变换，以及相机标定得到的静态坐标变换。
- `camera_router`：按云台模式选择主 8mm 或基地相机的图像与相机内参。
- `camera_driver`：通过大恒 SDK 或本地视频发布相机图像与标定参数。
- `armor_detector`：使用 0526 OpenVINO 模型输出装甲板二维四角点、编号、颜色和置信度。

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

脚本会直接启动目前已迁移的 `gimbal_driver`、`hero_tf`、`camera_driver`、`camera_router`、`armor_detector`，以及 `foxglove_bridge`；不额外使用总启动功能包。
该命令不覆盖任何参数，全部行为以各功能包 `config/` 目录中的 YAML 为准：车上使用串口和大恒相机；需要本地回放时再由你把对应 YAML 改为视频源。
运行后在 Foxglove Desktop 中连接 `ws://localhost:8765`，即可查看话题和 TF。若此前已手动启动 Bridge，应先停止它，避免端口 `8765` 冲突。

## 检测可视化

在 Foxglove 新建 **Image** 面板并选择 `/hero/detector/visualization`，可直接查看主 8 mm 或当前选中相机画面上的装甲板四角、编号、颜色和置信度。
该图是调试专用副本：原始 `/hero/camera/selected/image_raw` 与结构化检测结果 `/hero/detector/armors` 不会被修改。默认最多发布 10 Hz，且只有 Image 面板订阅该话题时才复制、绘制和发布图像；实战或性能测试时可将 `visualization_enabled` 设为 `false`。
