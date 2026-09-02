# Hero ROS 2 工作区

`ros2_ws` 是英雄机器人独立的 ROS 2 Humble 工作区。迁移期间，`2026HeroAim` 始终作为只读的行为参考；新工程按“完成一个可验证模块，再迁移下一个模块”的节奏推进。

## 约定

- 新功能包使用职责名，不加 `hero_` 前缀；`hero_msgs` 是公共消息包的固定例外。英雄机器人话题统一位于 `/hero` 命名空间下。
- ROS 边界的角度使用弧度、距离使用米，时间戳表示数据采集时刻。
- 串口仍保持旧协议的角度制和字节布局，只有 `gimbal_driver` 可以在串口协议与 ROS 消息间转换。
- Node 只承担通信和硬件适配；算法将保持为可测试、无 ROS 依赖的 C++ 库。
- 高频状态和控制话题使用深度为 1 的 best-effort QoS，只处理最新数据，不积压旧数据。

构建当前工作区：

```bash
cd Hero/heros/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

当前已迁移的功能包：

- `hero_msgs`：云台状态与控制命令的公共消息定义。
- `gimbal_driver`：旧串口协议、CRC、云台状态发布与控制命令下发。
- `hero_tf`：云台姿态驱动的动态坐标变换，以及相机标定得到的静态坐标变换。
- `camera_router`：按云台模式选择主 8mm 或基地相机的图像与相机内参。
- `camera_driver`：通过大恒 SDK 或本地视频发布相机图像与标定参数。

启动已完成的云台通信与坐标系部分：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py
ros2 launch hero_tf hero_tf.launch.py
ros2 launch camera_router camera_router.launch.py
```

## 本地视频回放

`camera_driver` 的每一路输入可通过 `<相机名>.source` 选择 `daheng` 或 `video`。选择
`video` 后，节点使用 OpenCV 读取本地视频，但仍发布与真实相机完全相同的图像和标定话题，
所以可以在不连接机器人时验证路由、检测和后续算法。

以 8mm 相机为例，先复制 `src/camera_driver/config/camera_driver.local.yaml` 到不提交 Git 的
本地位置，填写 `aim8mm.video_path` 为实际录像路径；录像分辨率必须与 `width`、`height` 和
标定参数匹配。然后启动：

```bash
ros2 launch camera_driver camera_driver.launch.py \
  config_file:=/你的本地路径/camera_driver.local.yaml
```

`video_rate_hz: 0.0` 表示按录像记录的 FPS 回放，`video_loop: true` 表示读到末尾后从头循环。
本地视频只是离线输入，不替代真实相机、时间同步和硬件链路验证。
