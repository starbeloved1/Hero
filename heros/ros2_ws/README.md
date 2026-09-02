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

启动已完成的云台通信与坐标系部分：

```bash
ros2 launch gimbal_driver gimbal_driver.launch.py
ros2 launch hero_tf hero_tf.launch.py
ros2 launch camera_router camera_router.launch.py
```
