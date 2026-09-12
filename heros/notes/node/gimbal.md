# 云台驱动

## 定位

`gimbal_driver` 是主控串口与 ROS 之间的边界：把主控状态变成 `GimbalState`，把最终 `ControlCommand` 写回主控。它不计算瞄准点。

```text
主控状态 ──串口──> gimbal_driver ──> /hero/gimbal/state
command_mux ──> /hero/gimbal/control ──> gimbal_driver ──串口──> 主控
```

## 接口

| Topic | 类型 | 方向 | 含义 |
| --- | --- | --- | --- |
| `/hero/gimbal/state` | `GimbalState` | 发布 | 当前云台、模式、己方颜色和按键状态 |
| `/hero/gimbal/control` | `ControlCommand` | 订阅 | `command_mux` 选出的唯一最终控制 |
| `/hero/antibase/packets` | `AntiBasePacket` | 订阅 | mode 4 专用串口包 |
| `/hero/antibase/tx_status` | `AntiBaseTxStatus` | 发布 | mode 4 串口发送状态 |

高频状态和控制均为 `KeepLast(1) + best_effort`：只关心最新值，不积压旧命令。

## 原始串口协议

帧头固定为 ASCII `!`（`0x21`）。浮点数直接按 4 字节 `float` 写入；当前实现运行在小端平台，因此低字节在前。CRC16 初值 `0xFFFF`、反射多项式 `0x8408`，校验字节低位在前。

### 主控 → 上位机：状态帧，16 B

| 字节 | 原始字段 | 类型 | 含义 |
| --- | --- | --- | --- |
| 0 | start | `uint8` | 固定 `!` |
| 1 | `mode_flag` | `uint8` | 1 normal、2 anti-top、3 auto-aim、4 anti-base；非法值在 ROS 侧按 normal 处理 |
| 2–5 | `pitch_deg` | `float32` | pitch，单位度 |
| 6–9 | `yaw_deg` | `float32` | yaw，单位度；驱动接收后归一化到 `[-180, 180]` |
| 10 | `up` | `uint8` | 非零为按下 |
| 11 | `down` | `uint8` | 非零为按下 |
| 12 | `robot_color` | `uint8` | 主控原始颜色值；项目约定 0 蓝、1 红 |
| 13 | `right_clicked` | `uint8` | 非零为按下 |
| 14–15 | crc | `uint16` | 前 14 B 的 CRC16，低字节在前 |

### 上位机 → 主控：控制帧，14 B

| 字节 | 原始字段 | 类型 | 含义 |
| --- | --- | --- | --- |
| 0 | start | `uint8` | 固定 `!` |
| 1 | `command_flag` | `uint8` | 控制命令标记，由参数提供 |
| 2–5 | `pitch_deg` | `float32` | 目标 pitch，单位度 |
| 6–9 | `yaw_deg` | `float32` | 目标 yaw，单位度 |
| 10 | `shoot_status` | `uint8` | 射击状态；仅 `enable_fire` 启用时透传 |
| 11 | `target_id` | `uint8` | 目标 Hero 编号 |
| 12–13 | crc | `uint16` | 前 12 B 的 CRC16，低字节在前 |

ROS 输入输出角度均为弧度：收状态时 `度 → 弧度`，写控制帧时 `弧度 → 度`。`hero_tf` 消费转换后的状态，在同一时间戳发布动态 `world → gimbal_link`；它不是本节点发布的 TF。

## 时间与发送

- 真实串口没有下位机采样时刻；驱动在读帧回调入口记录主机接收时刻，并在之后的定时发布中保留它，不能改成发布时刻。
- `state_timestamp_offset` 是人工校正量，单位秒。它解决状态时刻与相机 `T0` 的固定偏差，详见 [time.md](../project/time.md)。
- 虚拟串口每次发布用当前 `now()`；`virtual_mode` 和 `virtual_robot_color` 仅在 `port_name: virtual` 时可修改。
- 每个发送周期取最近一次 `/hero/gimbal/control`，将 yaw/pitch 转为度后写串口。当前实现不检查命令年龄：mux 停止后，最后一条命令仍会被重复发送。
- `enable_fire: false` 时无条件把 `shoot_status` 写为 0；启用后才会透传射击状态。


## 参考文件

1. `gimbal_driver/src/gimbal_driver_node.cpp`：状态、控制、mode 4 分流
2. `gimbal_driver/config/gimbal_driver.yaml`：串口和频率配置
3. `gimbal_driver/include/gimbal_driver/serial_protocol.hpp`、`src/serial_protocol.cpp`：原始帧定义与编解码
4. `hero_tf/src/hero_tf_node.cpp`：由状态生成 TF 的位置
