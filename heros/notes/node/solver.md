# 装甲板位姿解算

## 定位

`armor_solver` 将 detector 给出的四个像素角点解为装甲板三维位姿；先在相机光学坐标系做 PnP，再尽量变换到 `world`。它查询 TF，但不发布 TF。

```text
/hero/detector/armors(T0, camera_optical_frame) + CameraInfo + 同刻 TF
  → armor_solver
  → /hero/solver/armor_poses(T0, world 或 camera_optical_frame)
```

## 接口

| Topic | 类型 | 方向 | 含义 |
| --- | --- | --- | --- |
| `/hero/detector/armors` | `ArmorArray` | 订阅 | 同一 `T0` 的二维角点集合 |
| `/hero/camera/aim8mm/camera_info` | `sensor_msgs/CameraInfo` | 订阅 | 相机矩阵 `K` 与畸变 `D` |
| `/hero/solver/armor_poses` | `ArmorPoseArray` | 发布 | PnP 成功的三维装甲板集合 |
| `/hero/solver/markers` | `visualization_msgs/MarkerArray` | 可选发布 | 装甲板立方体可视化 |

| `ArmorPose` 字段 | 含义 |
| --- | --- |
| `header` | 继承检测帧的 `T0`；`frame_id` 与 `pose` 使用同一坐标系 |
| `pose.position` | 装甲板中心位置，单位米 |
| `pose.orientation` | 装甲板朝向四元数 |
| `image_center` | 原始图像四角点的像素中心，仅保留二维语义 |
| `id` / `color` / `confidence` | 从 detector 原样传递 |
| `is_large` | 仅 `id = 1` 为 true |
| `reprojection_error` | PnP 投影回四角点的平均像素误差；越小越可信 |

## 节点前置条件

| 类别 | 已知量 | 要求 |
| --- | --- | --- |
| `solvePnP` 输入 | `corners[4]` | detector 输出的四个有限像素角点；顺序不能改变 |
| `solvePnP` 输入 | 相机内参 | `CameraInfo.k` 的完整矩阵 `K`，其中 `fx = K[0] > 0`、`fy = K[4] > 0` |
| `solvePnP` 输入 | 畸变参数 | `CameraInfo.d` 的完整数组 `D` |
| `solvePnP` 输入 | 装甲板尺寸 | `id = 1` 为 `230 × 56 mm`；其他 ID 为 `135 × 56 mm` |
| 坐标变换输入 | 检测帧 | `ArmorArray.header.stamp = T0`，`frame_id` 当前为 `camera_optical_frame` |
| 坐标变换输入 | 目标坐标系 | `target_frame_id` 当前为 `world` |
| 坐标变换输入 | TF | 要求链路 `world → gimbal_link → camera_link → camera_optical_frame` 完整存在。缺失时仍发布相机坐标结果 |

内参只缓存最新一份；没有有效内参时整帧不发布。

## 输出与发布

| 类别 | 结果 | 含义 |
| --- | --- | --- |
| `solvePnP` 输出 | `rvec` | 装甲板相对相机的 Rodrigues 旋转向量；发布前转为四元数 |
| `solvePnP` 输出 | `tvec` | 装甲板中心相对 `camera_optical_frame` 的位置，单位米 |
| `solvePnP` 输出 | `reprojection_error` | 候选位姿投影回图像后，四个角点的平均像素误差 |
| 发布 | `ArmorPose.pose` | 有同刻 TF：`T_world_armor = T_world_camera × T_camera_armor`；否则为原始 `T_camera_armor` |
| 发布 | `ArmorPose.header` | `stamp` 始终为检测帧 `T0`；`frame_id` 为 `world` 或输入 `source_frame` |
| 发布 | `ArmorPoseArray` | 同一帧所有 PnP 成功项，发布到 `/hero/solver/armor_poses` |

`SOLVEPNP_IPPE` 从候选中保留 `tvec.z > 0` 且平均重投影误差最小的一组。solver 查询 TF，但不发布 TF。

## 可视化

`visualization_enabled: true` 且存在订阅者时发布 marker。小/大装甲板分别画为 `135 × 56 × 10 mm`、`230 × 56 × 10 mm` 的立方体；marker 生命周期为 0.2 秒，并会显式删除上一帧遗留的 marker。

## 参考文件

1. `armor_solver/src/armor_solver_node.cpp`：Topic、TF 查询、消息组装
2. `armor_solver/src/armor_pnp.cpp`：装甲板尺寸、IPPE 与误差选择
3. `armor_solver/config/armor_solver.yaml`：目标坐标系和 TF 等待时间
4. `hero_msgs/msg/ArmorPose.msg`、`ArmorPoseArray.msg`：输出接口
