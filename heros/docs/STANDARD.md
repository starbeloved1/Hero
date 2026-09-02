# Hero ROS 2 工程规范

本文件保存长期稳定的工程约定。它不记录当前迁移进度、硬件环境或待办事项；这些信息只写入 `MEMORY.md`。

## 命名

- 新功能包采用英文职责名，不加 `hero_` 前缀，例如 `camera_driver`、`camera_router`、`armor_detector`、`solver`、`tracker`、`controller`。
- `hero_msgs` 是公共自定义消息包，固定保留原名。
- ROS 话题统一位于 `/hero/...` 命名空间；消息字段、参数、坐标系、C++ 标识符均使用英文。
- 包内文件采用 `snake_case`，不加 `hero_` 前缀。例如 `main.cpp`、`camera_router_node.cpp`、`camera_router.yaml`、`camera_router.launch.py`。
- 推理模型统一放在 `heros/model/`，不用 `models/`；功能包通过参数引用模型，不复制模型文件。
- 命名空间通常与包名一致；结束注释为 `}  // package_name`，匿名命名空间为 `}  // namespace`。
- 自定义坐标方向转换函数使用 `2`，例如 `makeCamera2Gimbal`、`makeFLU2RDF`。

既有包的改名是独立的无行为变化任务，必须同步更新源码、配置、launch、文档和验证命令；不得在无关任务中顺带重命名。

## 功能包结构

新建功能包或实施结构整理时，含 ROS 节点的功能包按以下结构组织，其中名称替换为实际包名：

```text
package/
├── include/package/
│   ├── package_node.hpp
│   └── domain_type.hpp
├── src/
│   ├── main.cpp
│   ├── package_node.cpp
│   └── domain_type.cpp
├── config/
├── launch/
└── test/
```

- `main.cpp` 只负责 ROS 初始化、创建节点、`spin`、异常日志、关闭和退出码。
- `package_node.hpp` 声明 Node 类、回调和长期持有的 ROS 成员；`package_node.cpp` 实现参数、通信、定时器、硬件生命周期和调度。
- 可复用、需要单元测试或不应依赖 ROS 的逻辑实现为独立 C++ 库；该库不得包含 `rclcpp`。
- 仅在单一 `.cpp` 使用的小工具函数置于匿名命名空间；除模板或必要 `inline` 外，头文件不放实现。

## ROS 接口与职责

- `hero_msgs` 是唯一的 Hero 公共自定义消息包。
- Node 负责 ROS 通信、参数、硬件适配、生命周期和调度；视觉、自瞄、协议解析和选择规则等可复用逻辑保持普通 C++。
- ROS 边界中，角度使用弧度、长度使用米；`header.stamp` 表示数据采集时刻；涉及坐标系的数据必须设置 `frame_id`。
- 串口协议在串口边界保留旧工程的角度制、帧布局和 CRC；只有对应硬件驱动包可以负责 ROS 消息与串口帧的转换。
- 图像、云台状态和控制等高频话题使用 `KeepLast(1) + best_effort + volatile`，优先处理最新数据，回调不得阻塞等待硬件、推理或网络 I/O。
- 参数归属于实际使用它的功能包，保存在包内 YAML；资源通过 package share 目录解析，禁止写开发者主目录绝对路径。

## 文档、注释与验证

- `heros/` 下的 README、Markdown 和新增代码注释使用中文。
- 每个独立纯 C++ 规则至少具有单元测试，覆盖正常、边界和降级路径。
- 修改 ROS 通信时，除构建和单测外，按风险补充话题或 launch 集成验证。
- 使用项目配置的格式化与静态检查工具保持一致风格；结构整理必须保持外部行为不变，并按包为单位完成验证。
