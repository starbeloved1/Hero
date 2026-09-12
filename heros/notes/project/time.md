# 时间语义

时间戳回答的不是“消息何时到达”，而是“消息内容代表系统的何时”。物理测量、预测状态、控制决策和串口发送必须使用不同的时间。

## 时间代号

| 代号 | 含义 | 项目位置 |
| --- | --- | --- |
| `T0` | 一帧图像实际代表的采集时刻 | 图像链的 `header.stamp`：`Image → ArmorArray → ArmorPoseArray` |
| `Tstate` | 预测器发布状态实际代表的时刻 | `TargetStateArray.header.stamp` |
| `Tcontrol` | 策略生成一次控制候选的时刻 | 各策略候选 `ControlCommand.header.stamp` |
| `Taim` | 算法预计弹丸命中目标的时刻 | `AutoAimDebug.aim_stamp` |
| `Tzone` | 周期目标进入射击区时对应图像的 `T0` | `AntitopDebug.zone_stamp` |
| `Tpermit` | 反前哨计划允许开火的绝对时刻 | `AntitopDebug.permit_stamp` |
| `Tmux` | `command_mux` 输出最终控制帧的时刻 | `/hero/gimbal/control` 的 `header.stamp` |
| `Tsend` | 驱动实际开始写出一帧或逻辑包的时刻 | `AntiBaseTxStatus.last_send_stamp` |

回调收到消息的 `now()` 只是处理时刻；除生成新控制、预测新状态或记录发送外，不能用它替代上游消息的时间。

## 统一规则

1. `header.stamp` 只表示该消息**主要数据**的时刻；需要多个时刻时，增加名称明确的字段。
2. 以 `T0` 进行 TF 查询、目标运动拟合和周期测量；以 `Tcontrol` 生成本次命令；以 `Taim` 重建命中时的目标。
3. 同一帧派生的图像、检测、PnP 和可视化应保留同一个 `T0`，不能因处理耗时重打时间戳。
4. ROS 时间用于跨节点语义；反基地队列的节拍使用单调时钟，不能把两者直接相减。
5. 时间戳必须递增；相机设备时钟回退、映射失效或 ROS 时间大跳变时，驱动重置映射并丢弃跨边界帧。

## 相机、检测、PnP 与 TF

```text
相机采集 T0
  → Image / CameraInfo(T0)
  → ArmorArray(T0)
  → ArmorPoseArray(T0)
  → 各瞄准策略
```

### `aim8mm.timestamp_mode`：决定 `T0` 怎样取得

| 模式 | `T0` 来源 | 适用范围 | 关键限制 |
| --- | --- | --- | --- |
| `host` | 驱动取到原始帧的主机时刻 | 视频和真实相机 | 受 USB、解码和主机调度抖动影响 |
| `device` | 相机设备计数映射到 ROS 时间 | 真实 aim8mm 大恒相机 | 仍含传输延迟；不支持视频和 `use_sim_time` |

- 当前 `camera_driver.yaml` 配置为 `aim8mm.timestamp_mode: host`。
- `device` 会读取设备时钟频率，以首帧建立映射，并持续低通修正；它的目标是降低主机取帧时刻的抖动，不是直接取得精确曝光时间。
- 切换模式应修改 `aim8mm.timestamp_mode` 后重启 `camera_driver`；`base` 相机不支持该参数。
- `base` 相机只使用主机取帧时刻近似 `T0`，不提供设备时间映射。
- `armor_detector` 对每张输入图像发布一个同 `T0`、同坐标系的 `ArmorArray`；无检测也应保留该帧的 `T0`。
- `armor_solver` 对 `T0` 查询 TF；成功时在目标坐标系发布 `ArmorPoseArray(T0)`，失败时保留相机坐标系发布，时间不变。
- `hero_tf` 的动态 `world → gimbal_link` 使用 `GimbalState.header.stamp`；静态相机外参不代表一次测量，不参与运动时间推断。

`T0` 与云台状态时间不一致会直接造成世界坐标误差；因此相机时间、串口状态时间和 TF 必须在实车一起验证。

## 云台状态

- `GimbalState.header.stamp` 必须表示该 yaw/pitch 实际成立的时刻；虚拟云台以发布时刻作为近似。
- `hero_tf` 原样采用这个时间戳，不能在 TF 或下游再次补偿。
- 当前策略缓存最新云台状态，不与每个 `ArmorPoseArray(T0)` 做消息同步；观察快速运动时，必须把这个状态年龄视为误差来源。

## 时间戳校正

这两项是项目中仅有的人工传感器时间校正；当前配置均为 `0.0`。

```text
真实图像采集 ── 相机传输 Δcamera ── 主机收到图像
真实云台采样 ── 串口传输 Δstate  ── 主机收到状态
```

| 参数 | 校正对象 | 当前值（秒） |
| --- | --- | --- |
| `aim8mm.timestamp_offset` | 图像最终 `T0` 的 `Δcamera` | `0.0` |
| `state_timestamp_offset` | `GimbalState.header.stamp` 的 `Δstate` | `0.0` |

- 两个参数均以“收到时刻 + offset”写出最终时间戳；负值提前，正值推后。
- `timestamp_mode: device` 是自动的相机时钟映射，不是人工校正；它降低帧间抖动，但不自动消除 `Δcamera`。
- 两项校正让图像与 TF 中的云台姿态指向同一真实时刻；它们分别作用于相机和串口，不能互相替代。
- 没有额外的 detector、PnP 或 TF 时间偏移参数。
- 参考 `ly_aim` 没有云台状态补偿参数：它在串口回调直接写 `state.stamp = now()`，等价于 Hero 设为 `0.0`。
- `ly_aim` 的 TF 查询偏移当前为 `0.0`；它在 PnP 解算时将图像 `T0` 偏移后查询 TF，不是云台状态时间补偿。
- 当 `ly_aim.use_current_time_for_tf: false` 时，它查询的时间为 `T0 + tf_timestamp_offset`；当前值为 `0.0`，所以实际仍查询 `T0`。
- Hero 的 `armor_solver` 已直接以 `T0` 查询 TF，不需要增加 `tf_timestamp_offset`。
- 若相机与串口状态存在固定时间差，应在 `state_timestamp_offset` 修正状态的物理时间；这会让所有 TF 使用者一致受益。把同一差值加在 solver 查询时间只会形成局部补丁，并使其他使用 TF 的节点仍然错误。

## Mode 1：普通瞄准

```text
ArmorPoseArray(T0) + 最新 GimbalState
  → aim_normal
  → NormalAimDebug(T0)
  → ControlCommand(Tcontrol)
```

- `aim_normal` 的目标选择、弹道和角度平滑均基于本次 `ArmorPoseArray(T0)`。
- `NormalAimDebug.header.stamp` 是所用装甲板观测的 `T0`。
- `aim_normal` 发布候选时写入当前 `Tcontrol`；它不是观测时刻。
- mode 1 当前未单独检查观测年龄；候选新鲜不代表输入观测新鲜。

## Mode 2：反前哨

```text
ArmorPoseArray(T0)
  → 前哨跟踪与旋转中心拟合
  → Tzone、period
  → Tpermit
  → ControlCommand(Tcontrol)
```

- 前哨板、旋转中心、分层高度和周期都由图像观测的 `T0` 更新。
- `Tzone` 必须是装甲板进入射击区那帧的 `T0`，周期必须是相邻 `Tzone` 的差；不能用回调时刻 `Tcontrol` 测周期。
- 控制器在当前 `Tcontrol` 计算 `Tpermit = Tzone + 3 × period - system_delay - flight_time + direction_bias`，并只计算剩余时间 `Tpermit - Tcontrol`。
- `AntitopDebug.header.stamp` 是 `Tcontrol`，`measurement_stamp` 是本帧 `T0`，`zone_stamp` 是 `Tzone`，`permit_stamp` 是 `Tpermit`。
- mode 2 候选的 `ControlCommand.header.stamp` 是 `Tcontrol`；反前哨 Marker 中观测与旋转模型表示 `T0`，弹道瞄点表示当前控制结果。

## Mode 3：预测自瞄

```text
ArmorPoseArray(T0)
  → Tracker 正式状态 S(T0)
  → TargetStateArray(Tstate, measurement_stamp=T0)
  → aim_auto 在 Tcontrol 临时外推
  → 目标 S(Taim)
  → ControlCommand(Tcontrol)
```

- `aim_predictor` 先将每个 Tracker 的正式状态预测到 `T0`，再融合该帧观测；其正式记忆最终是 `S(T0)`。
- 发布前，预测器仅临时将 `S(T0)` 外推到 `Tstate`；`estimate(Tstate)` 不得改写 Tracker 的正式状态或 `last_stamp_sec_`。
- 将正式记忆推进到 `Tstate` 会使下一帧通常更早的 `T0` 发生时间倒流，破坏 EKF。
- `TargetStateArray.header.stamp = Tstate`，`measurement_stamp = T0`；数组内中心、速度、yaw 和预测装甲板全部表示 `Tstate`。
- `aim_auto` 先补偿 `Tcontrol - Tstate`，再迭代机构响应和飞行时间，得到 `Taim` 的目标与瞄点。
- `AutoAimDebug.header.stamp` 是 `Tcontrol`，并同时给出 `measurement_stamp`、`state_stamp` 与 `aim_stamp`；其控制 Marker 表示 `Taim`。
- mode 3 候选的 `ControlCommand.header.stamp` 是 `Tcontrol`，不应伪装成 `Taim`。

## 仲裁与普通云台串口

```text
策略候选(Tcontrol)
  → command_mux 检查年龄与模式
  → 最终 ControlCommand(Tmux)
  → gimbal_driver 缓存并写串口
```

- `command_mux` 用 `Tmux - Tcontrol` 检查候选是否超过 `max_command_age_sec`，只放行当前 mode 对应的候选。
- 仲裁器输出时将 `ControlCommand.header.stamp` 改写为 `Tmux`；下游无法从最终控制话题恢复原始 `Tcontrol`，应查看策略 debug。
- 无候选、候选过期或模式不匹配时，仲裁器输出安全命令，其时间仍为 `Tmux`。
- `gimbal_driver` 以固定周期发送最新缓存的普通控制命令，串口协议不携带 ROS 时间戳。
- 当前驱动未按控制命令年龄清空缓存；仲裁器停止输出后，驱动仍可能重发最后一条命令。这是已知安全边界，不得误认为 `max_command_age_sec` 已保护到 `Tsend`。

## Mode 4：反基地

```text
base Image(T0)
  → hero_antibase 编码与分包
  → AntiBasePacket(T0)
  → gimbal_driver 队列
  → 逻辑包开始发送(Tsend)
```

- `AntiBasePacket.header.stamp` 始终保留源基地图像的 `T0`，即使编码、排队和分片发生在更晚时刻。
- mode 4 不发送普通云台控制帧；`gimbal_driver` 只按最小逻辑包间隔和分片间隔发送 H.264 数据。
- `AntiBaseTxStatus.header.stamp` 是状态采样时刻，`last_send_stamp` 是最近完整逻辑包实际开始写串口的 `Tsend`。
- `Tsend - T0` 是反基地视觉到链路发送的端到端延迟；它不能由普通控制链路的时间戳代替。

## 调试时只计算这些量

| 量 | 含义 | 用途 |
| --- | --- | --- |
| `Tstate - T0` | 观测到预测状态的链路延迟 | 检查 mode 3 预测器滞后 |
| `Tcontrol - T0` | 观测到本次策略决策的延迟 | 检查 mode 1、2、3 的实时性 |
| `Taim - Tcontrol` | 控制向前看的总时间 | 检查弹道与机构补偿 |
| `Tmux - Tcontrol` | 候选在仲裁前等待的时间 | 检查仲裁频率与候选时效 |
| `Tsend - T0` | 基地图像到实际发送的延迟 | 检查 mode 4 编码、排队和发送 |

最终控制话题的时间已经是 `Tmux`；分析策略延迟时必须使用各策略 debug 中保留的 `T0`、`Tstate` 与 `Tcontrol`，不能把 `Tmux` 当成策略生成时刻。
