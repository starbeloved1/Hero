# Hero ROS 2 自瞄系统

面向 RoboMaster 英雄机器人的 ROS 2 工程，包含双相机驱动、装甲板检测、PnP 解算、TF、三种瞄准策略、云台串口通信和反基地视频传输。


## 工作模式

| Mode | 功能 | 数据链路 |
| --- | --- | --- |
| 1 | 普通瞄准 | aim8mm → detector → solver → aim_normal |
| 2 | 反前哨 | aim8mm → detector → solver → aim_antitop |
| 3 | 预测自瞄 | aim8mm → detector → solver → predictor → aim_auto |
| 4 | 反基地 | base → hero_antibase → gimbal_driver |

三个瞄准策略的候选命令由 `command_mux` 按 mode 仲裁，最终通过 `/hero/gimbal/control` 发送给 `gimbal_driver`。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- OpenVINO C++ Runtime
- Daheng Galaxy SDK，默认位于 `/opt/galaxy_sdk`
- OpenCV、Eigen3、Boost、GStreamer 和 `foxglove_bridge`

## 构建

```bash
cd Hero/heros
./build.sh
```

## 运行

```bash
cd Hero/heros
./run.sh
```

Foxglove Desktop 连接：ws://localhost:8765


各节点参数位于 `ros2_ws/src/<package>/config/`。本地录像验证时，将 `camera_driver.yaml` 的相机源设为 `video`，并将 `gimbal_driver.yaml` 的 `port_name` 设为 `virtual`

运行时切换虚拟 mode：

```bash
ros2 param set /gimbal_driver_node virtual_mode 1
ros2 param set /gimbal_driver_node virtual_mode 2
ros2 param set /gimbal_driver_node virtual_mode 3
ros2 param set /gimbal_driver_node virtual_mode 4
```

启动反基地本地裁判模拟器：

```bash
./hero_judge.sh
```
