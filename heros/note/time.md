# 时间与预测状态

本文约定适用于 Hero ROS 2 自瞄链路。重点是区分：原始观测发生的时间、跟踪器内部正式保存状态的时间，以及发布给下游的预测状态时间。

## 时间代号

| 代号 | 含义 | 当前链路中的位置 |
| --- | --- | --- |
| `T0` | 原始传感器采集时刻 | 相机拍摄图像，之后的检测、PnP、`ArmorPoseArray` 均保留它 |
| `Tstate` | 预测状态实际代表的时刻 | `aim_predictor` 发布 `TargetStateArray` 时的状态时刻 |
| `Tcontrol` | 生成本次控制候选的时刻 | `aim_auto` 计算 yaw、pitch、射击许可的时刻 |
| `Tmux` | `command_mux` 仲裁输出时刻 | 唯一 `/hero/gimbal/control` 的输出时刻 |
| `Tsend` | 串口实际写出协议帧的时刻 | `gimbal_driver` 写入下位机的时刻 |
| `Taim` | 预计弹丸命中目标的时刻 | `aim_auto` 用于重建目标未来装甲板位置的时刻 |

## 一条图像的时间线

```text
T0 = 10.000 s：相机拍到图像
        │
        ▼
检测、PnP 完成
        │
T1 = 10.025 s：aim_predictor 收到 ArmorPoseArray
        │
        ├── Tracker 先推到 T0，再用该图像观测修正
        │       内部正式状态变为 S(T0)
        │
        ▼
Tstate = 10.027 s：临时将 S(T0) 推到 S(Tstate)，发布 TargetStateArray
        │
        ▼
Tcontrol：aim_auto 收到状态，继续推算目标未来状态
        │
        ▼
Taim = Tcontrol + 机构响应时间 + 弹丸飞行时间
```

`T1` 只是“程序此刻处理到消息”的墙钟时刻，通常不需要写入消息；真正有算法语义的是 `T0`、`Tstate`、`Tcontrol` 与 `Taim`。

## 模型、状态与观测

三者不是同一个概念：

```text
观测 Z(T0)
    一帧图像解出的某块装甲板位置、朝向
        │
        ▼
跟踪模型
    匀速平移 + 匀角速度旋转 + 四装甲板空间结构
        │
        ▼
状态 S(t)
    车辆中心、速度、yaw、角速度、半径和四块板的位置
```

- **模型**是长期有效的推理规则，不属于某一个时刻。
- **状态**是模型对目标在某一个具体时刻的估计；因此必须说明它的时间。
- **观测**是传感器在 `T0` 得到的事实，存在噪声且通常只能看见其中一块板。

## Tracker 的内部记忆与发布消息

`TargetTracker` 内部保存：

```cpp
StateVector state_;
StateMatrix covariance_;
double last_stamp_sec_;
```

它们构成跟踪器的**正式记忆**。在处理一帧 `T0` 图像后，正式记忆应表示 `S(T0)`：

```text
旧正式状态 S(旧 T0)
        │
predict(新 T0)
        ▼
预测状态 Ŝ(新 T0)
        │
update(新图像的观测 Z(新 T0))
        ▼
新正式状态 S(新 T0)
```

`predict(stamp_sec)` 会改写内部记忆：`state_`、`covariance_` 和 `last_stamp_sec_` 都会变成新时刻的值。它用于让正式状态与即将融合的观测处于同一时刻。

而发布前调用的 `estimate(Tstate)` 是临时推演：

```text
内部仍保存 S(T0)
        │
临时外推 Tstate - T0
        ▼
返回 TargetEstimate(Tstate)
        │
发布后临时结果丢弃，内部仍是 S(T0)
```

因此 `estimate()` 不会把 `state_` 和 `last_stamp_sec_` 改到 `Tstate`。这是必要的：下一张图像的 `T0` 往往早于刚发布的 `Tstate`；若把正式记忆提前推进到未来，下一帧观测就会让 EKF 时间倒流。

## TargetStateArray 的时间语义

当前接口规定：

```text
TargetStateArray.header.stamp = Tstate
TargetStateArray.header.frame_id = world
TargetStateArray.measurement_stamp = T0
```

每个 `TargetState` 的中心、速度、yaw 和 `predicted_armors` 都表示 `Tstate` 的目标状态。`measurement_stamp` 明确保留它基于哪一帧图像，便于计算检测链路延迟和调试。

## 从 Tstate 推到 Taim

`aim_auto` 不应修改 `aim_predictor` 内部的 Tracker。它接到 `TargetState(Tstate)` 后，仅为本次控制临时预测：

```text
S(Tstate)
   │
   ├── 先补偿 Tcontrol - Tstate
   ├── 再补偿机构响应时间
   └── 再补偿弹丸飞行时间
   ▼
S(Taim)
   ▼
重建四块板，选择可打装甲板，得到 aim_point
```

`Taim` 需要迭代求解：目标未来位置影响弹道飞行时间，飞行时间又影响目标未来位置。收敛后，`AutoAimDebug` 记录 `state_stamp`、`aim_stamp` 与最终 `aim_point`。

## 最重要的检查原则

1. 传感器、图像、检测与 PnP 的 `header.stamp` 应始终保留 `T0`。
2. 预测状态消息的 `header.stamp` 必须等于其内容实际代表的 `Tstate`，不能仍写 `T0`。
3. 一条消息若同时需要观测时间和状态时间，主 `header.stamp` 表示主要数据时刻，另一个时间使用语义明确的独立字段保存。
4. 预测到未来只用于本次推理或控制，不可把 Tracker 的正式记忆直接改到未来。
