# Hero ROS 2 自瞄系统

面向 RoboMaster 英雄机器人的 ROS 2 工程，包含双相机驱动、装甲板检测、PnP 解算、TF、三种瞄准策略、云台串口通信和反基地视频传输。

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

在电脑的 Foxglove Desktop 中添加 Foxglove WebSocket 连接：

```text
ws://192.168.2.101:8765
```

Bridge 在 NUC 的所有网络接口上监听 `8765` 端口，不需要在电脑安装 ROS

## NUC 开机自启

先构建项目，再在 NUC 的 `heros` 目录执行：

```bash
./install-autostart.sh
sudo systemctl enable --now hero-aim.service
```

常用操作：

```bash
sudo systemctl restart hero-aim.service
sudo systemctl stop hero-aim.service
sudo systemctl status hero-aim.service
journalctl -u hero-aim.service -f
```

安装前查看将生成的 service，不修改系统：

```bash
./install-autostart.sh --dry-run
```


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
./hero-judge.sh
```
