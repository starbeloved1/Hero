# AntiBase 裁判 / MQTT 闭环

该工具只用于本地验证，绝不能作为车端协议或裁判系统替代品。

链路为：

```text
gimbal_driver 最终串口分片成功
  -> UDP 300B 镜像 (127.0.0.1:9999)
  -> hero_judge.py
  -> Mosquitto (127.0.0.1:3333)
  -> CustomByteBlock 303B
  -> Hero_client
```

`gimbal_driver` 只在五个 64B 分片均已成功写出后才镜像同一个逻辑包。裁判模拟器使用收到 UDP datagram 的单调时间；与前一个收到包的间隔小于或等于 20 ms 时，当前包直接丢弃，且该拒包仍更新比较基准。放行包的 MQTT payload 固定为 `0A AC 02 + 300B`，与 `Hero_client` 比赛接收格式一致。

## 配置

`hero_judge.yaml` 是裁判程序的默认配置，包含 UDP 监听地址和端口、MQTT 地址和端口、topic、QoS、20 ms 丢包规则及统计周期。直接运行 Python 时会自动读取它：

```bash
python3 tools/hero_judge/hero_judge.py
```

需要临时覆盖某一项时可以传命令行参数，例如：

```bash
python3 tools/hero_judge/hero_judge.py --udp-port 10000
```

`mosquitto_hero_judge.conf` 是 Mosquitto 自身的配置。若修改 YAML 的 MQTT host 或 port，必须同步修改该文件，或自行启动对应的 broker。

## 启动

安装一次依赖：

```bash
sudo apt install mosquitto mosquitto-clients python3-paho-mqtt python3-yaml
```

在 `gimbal_driver.yaml` 中仅为本地测试设为：

```yaml
antibase_udp_mirror_enabled: true
```

重启 ROS 进程后，另开终端运行：

```bash
cd Hero/heros
./tools/hero_judge/run_hero_judge.sh
```

裁判日志每秒显示收到、放行、间隔丢弃、间隔 min/avg/max 和 MQTT 发布确认；`[Judge DROP]` 表示该包未进入 MQTT，也不会到达客户端。

最后将 `Hero_client/config.py` 的 `MQTT_CONFIG` 临时改为 `broker: "127.0.0.1"`、`mqtt_port: 3333`，保留 `topic_video: "CustomByteBlock"` 与 `qos_level: 1`，再启动原 `Hero_client/src/sub_win.py`。测试结束后把 UDP 镜像参数恢复为 `false`，并恢复客户端比赛地址。
