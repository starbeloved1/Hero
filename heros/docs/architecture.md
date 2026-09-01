# Hero ROS 2 架构约定

## 迁移规则

`../2026HeroAim` 是兼容性基准，迁移切片中不得修改。每个切片只把一个职责迁入 `ros2_ws/src`：保持它的公开行为、补充自动化验证、完成代码讲解和学习复盘后，才开始下一个切片。`ly_aim` 只作为 ROS 设计参考，不是运行时依赖。

## 包与接口规则

- 包名统一为 `hero_*`；公共接口只定义在 `hero_msgs`。
- Node 负责 ROS 通信和硬件生命周期。可复用的视觉、自瞄算法保持为普通 C++ 库，禁止包含 `rclcpp`。
- 参数归属于实际使用它的包，并从包内 YAML 加载。资源路径通过包 share 目录解析，禁止使用开发者主目录的绝对路径。
- 硬件状态消息必须包含采集时间戳；涉及坐标系时必须设置 `frame_id`。ROS 中角度使用弧度、长度使用米。
- 串口仅在传输边界保留旧实现的角度制、帧布局和 CRC；字节行为必须与 `2026HeroAim` 兼容。

## 初始接口

`hero_msgs/GimbalState` 对应旧代码的 `SerialPortData`：包含当前 yaw/pitch、原始模式 flag、映射后的操作模式、敌方颜色、右键和曝光按键。`exposure_step` 是从旧 `consumeExposureAdjustmentSteps` 语义推导的单次上升沿事件。

`hero_msgs/ControlCommand` 对应旧代码的 `SerialPortWriteData`：包含 yaw/pitch 设定值、射击状态和目标 ID。只有 `hero_gimbal_driver` 可以把它编码并写入物理串口；只有部署 YAML 显式设置 `enable_fire: true` 时才允许发射。

## 高频数据规则

图像、云台状态和控制话题使用深度为 1、best-effort、volatile 的 QoS。处理节点必须优先处理最新数据，不能积压过期数据；回调函数不得阻塞等待相机、串口、推理或网络 I/O。

## 当前坐标系约定

`hero_tf` 当前维护一条最小坐标树：`world -> gimbal_link -> camera_link -> camera_optical_frame`。`world -> gimbal_link` 由 `hero_msgs/GimbalState` 的 yaw、pitch 动态发布，并严格保留旧 `Solver` 的 `Rz(yaw) * Ry(-pitch)` 方向约定。

`gimbal_link -> camera_link` 使用旧 `init.json` 中 8 mm 相机的平移和 yaw/pitch/roll 标定参数；旧矩阵描述的是“云台坐标到相机坐标”，TF 发布时必须使用其逆旋转。`camera_link` 使用前、左、上（FLU）约定，`camera_optical_frame` 使用 ROS 光学坐标的右、下、前（RDF）约定。枪口、底部相机与第二相机将在对应硬件迁移切片中加入。

## 迁移顺序

1. 公共接口与云台串口驱动。
2. TF 与标定坐标系。
3. 多相机驱动与按模式选帧。
4. 检测、位姿解算、普通/反陀螺控制、追踪预测、反基地与 MQTT。
5. 全系统启动、rosbag 回归和部署验证。
