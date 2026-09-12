# 装甲板检测

## 定位

`armor_detector` 将 aim8mm 的一张 `bgr8` 图像变为同一帧内的二维装甲板集合。它不计算距离、姿态或世界坐标；这些由后续 `armor_solver` 负责。

```text
/hero/camera/aim8mm/image_raw(T0, camera_optical_frame)
  → armor_detector
  → /hero/detector/armors(T0, camera_optical_frame)
```

## 接口

| Topic | 类型 | 方向 | 作用 |
| --- | --- | --- | --- |
| `/hero/camera/aim8mm/image_raw` | `sensor_msgs/Image` | 订阅 | 输入图像，必须为 `bgr8` |
| `/hero/gimbal/state` | `GimbalState` | 订阅 | 提供 mode 与己方颜色 |
| `/hero/detector/armors` | `ArmorArray` | 发布 | 输出二维装甲板，交给 `armor_solver` |
| `/hero/detector/visualization` | `sensor_msgs/Image` | 可选发布 | 检测结果图 |


| `Armor` 字段 | 含义 |
| --- | --- |
| `corners[4]` | 四个像素角点，原点在图像左上，单位像素；顺序固定，后续 PnP 不得自行重排 |
| `id` | Hero 公开编号 |
| `color` | 模型颜色类别 |
| `confidence` | 该候选的 sigmoid 置信度 |

## 一帧如何处理

1. 图像回调只覆盖缓存的 `latest_image`；独立工作线程只取最新帧，推理慢时旧帧会被主动丢弃。
2. mode 4 时直接跳过检测，不发布装甲板结果。
3. 将图像缩放为 `640 × 640`，以 BGR 输入 OpenVINO 0526 模型，再把四角点缩回原图像尺寸。
4. 置信度低于 `confidence_threshold`、颜色不匹配、类别未知或长宽比不在 `1–6` 的候选被丢弃。
5. 对剩余候选按 `nms_threshold` 执行 NMS，保留不重叠的结果。

## 颜色与编号

| `target_color` | 行为 |
| --- | --- |
| `-2` | 不按颜色过滤，当前离线录像配置 |
| `-1` | 自动取 `robot_color` 的相反颜色；己方蓝 `0` 时检测红 `1`，反之亦然 |
| `0` / `1` | 固定只保留蓝 / 红目标 |


| 0526 模型类别 | 发布的 Hero `id` | 含义 |
| --- | --- | --- |
| `0` | `6` | 基地 |
| `1` | `1` | 英雄 |
| `2` | `2` | 工程 |
| `3` / `4` / `5` | `3` / `4` / `5` | 步兵 |
| `6` | `7` | 前哨 |
| `7` | `0` | 哨兵 |

颜色 `0` 为蓝、`1` 为红

## 空结果与可视化

- 无检测、颜色未知或推理异常时，发布同 `T0` 的空 `ArmorArray`。
- mode 4 时不发布检测结果，因为基地相机只交给 `hero_antibase`。
- 可视化只在 `visualization_enabled: true` 且存在订阅者时发布；`visualization_rate_hz: 0` 表示每个已处理输入都可视化。

## 参考文件

1. `config/armor_detector.yaml`：话题、模型、阈值与颜色策略
2. `src/armor_detector_node.cpp`：ROS 回调、最新帧策略、mode 与颜色分支
3. `src/armor_detector.cpp`：模型输入、筛选、角点缩放与 NMS
4. `src/armor_utils.cpp`：颜色筛选和模型类别到 Hero ID 的转换
5. `hero_msgs/msg/Armor.msg`、`ArmorArray.msg`：输出接口
