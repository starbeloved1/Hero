#!/usr/bin/env python3
"""本地反基地裁判模拟器：UDP 300B 输入，MQTT 303B 比赛格式输出。"""

from __future__ import annotations

import argparse
import signal
import socket
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import paho.mqtt.client as mqtt
except ImportError as error:
    raise SystemExit(
        "缺少 paho-mqtt；请安装 python3-paho-mqtt 或 pip install paho-mqtt"
    ) from error

try:
    import yaml
except ImportError as error:
    raise SystemExit(
        "缺少 PyYAML；请安装 python3-yaml 或 pip install pyyaml"
    ) from error


PACKET_BYTES = 300
MQTT_PREFIX = bytes((0x0A, 0xAC, 0x02))  # protobuf bytes field, varint length=300
MQTT_PACKET_BYTES = len(MQTT_PREFIX) + PACKET_BYTES
DEFAULT_CONFIG_PATH = Path(__file__).with_name("hero_judge.yaml")
DEFAULT_CONFIG: dict[str, object] = {
    "udp_host": "127.0.0.1",
    "udp_port": 9999,
    "mqtt_host": "127.0.0.1",
    "mqtt_port": 3333,
    "mqtt_topic": "CustomByteBlock",
    "mqtt_client_id": "hero_judge",
    "mqtt_qos": 1,
    "min_packet_gap_ms": 20.0,
    "stats_interval_sec": 1.0,
}


@dataclass
class WindowStats:
    received: int = 0
    accepted: int = 0
    judge_dropped: int = 0
    malformed: int = 0
    mqtt_queued: int = 0
    mqtt_acked: int = 0
    mqtt_errors: int = 0
    gaps_ms: list[float] | None = None

    def __post_init__(self) -> None:
        if self.gaps_ms is None:
            self.gaps_ms = []


class LocalJudge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.running = True
        self.last_received_ns: int | None = None
        self.last_sequence: int | None = None
        self.total_accepted = 0
        self.total_judge_dropped = 0
        self.total_malformed = 0
        self.window = WindowStats()
        self.last_stats_time = time.monotonic()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((args.udp_host, args.udp_port))
        self.socket.settimeout(0.2)

        # paho-mqtt 1.x（Ubuntu 包）和 2.x（比赛客户端常用）都能运行
        try:
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=args.client_id,
            )
        except AttributeError:
            self.mqtt_client = mqtt.Client(client_id=args.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_publish = self.on_publish
        self.mqtt_connected = False

    def on_connect(self, _client, _userdata, _flags, reason_code, _properties=None) -> None:
        failed = getattr(reason_code, "is_failure", False)
        if failed or reason_code not in (0, None):
            print(f"[Judge MQTT] connect failed: {reason_code}", flush=True)
            self.mqtt_connected = False
            return
        self.mqtt_connected = True
        print(
            f"[Judge MQTT] connected {self.args.mqtt_host}:{self.args.mqtt_port} "
            f"topic={self.args.mqtt_topic} qos={self.args.qos}",
            flush=True,
        )

    def on_disconnect(self, _client, _userdata, _disconnect_flags=None,
                      reason_code=None, _properties=None) -> None:
        self.mqtt_connected = False
        if reason_code not in (0, None):
            print(f"[Judge MQTT] disconnected: {reason_code}", flush=True)

    def on_publish(self, _client, _userdata, _mid, _reason_code=None,
                   _properties=None) -> None:
        self.window.mqtt_acked += 1

    def start_mqtt(self) -> None:
        self.mqtt_client.connect_async(
            self.args.mqtt_host, self.args.mqtt_port, keepalive=30
        )
        self.mqtt_client.loop_start()

    def stop(self) -> None:
        self.running = False
        try:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        except Exception:
            pass
        self.socket.close()

    def print_stats_if_due(self) -> None:
        now = time.monotonic()
        if now - self.last_stats_time < self.args.stats_interval_sec:
            return
        gaps = self.window.gaps_ms
        gap_text = "-/-/-" if not gaps else (
            f"{min(gaps):.3f}/{sum(gaps) / len(gaps):.3f}/{max(gaps):.3f}"
        )
        print(
            "[Judge] "
            f"rx={self.window.received} accept={self.window.accepted} "
            f"drop_gap={self.window.judge_dropped} malformed={self.window.malformed} "
            f"gap_ms(min/avg/max)={gap_text} "
            f"mqtt(queue/ack/error)={self.window.mqtt_queued}/"
            f"{self.window.mqtt_acked}/{self.window.mqtt_errors} "
            f"total(accept/drop)={self.total_accepted}/{self.total_judge_dropped}",
            flush=True,
        )
        self.window = WindowStats()
        self.last_stats_time = now

    def handle_packet(self, packet: bytes) -> None:
        if len(packet) != PACKET_BYTES:
            self.window.malformed += 1
            self.total_malformed += 1
            return

        received_ns = time.monotonic_ns()
        self.window.received += 1
        sequence = struct.unpack_from("<Q", packet, 0)[0]
        gap_ms: float | None = None
        if self.last_received_ns is not None:
            gap_ms = (received_ns - self.last_received_ns) / 1_000_000.0
            self.window.gaps_ms.append(gap_ms)

        # 严格模拟：拒绝包也成为下一包的比较基准，连续突发会连续被拒
        self.last_received_ns = received_ns
        if gap_ms is not None and gap_ms <= self.args.min_packet_gap_ms:
            self.window.judge_dropped += 1
            self.total_judge_dropped += 1
            print(
                f"[Judge DROP] seq={sequence} gap={gap_ms:.3f}ms "
                f"<= {self.args.min_packet_gap_ms:.3f}ms",
                flush=True,
            )
            self.last_sequence = sequence
            return

        if self.last_sequence is not None and sequence > self.last_sequence + 1:
            missing = sequence - self.last_sequence - 1
            print(f"[Judge] sequence gap before seq={sequence}: {missing}", flush=True)
        self.last_sequence = sequence
        message = MQTT_PREFIX + packet
        assert len(message) == MQTT_PACKET_BYTES
        info = self.mqtt_client.publish(self.args.mqtt_topic, message, qos=self.args.qos)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.window.mqtt_errors += 1
            print(f"[Judge MQTT] publish failed: rc={info.rc}", flush=True)
            return
        self.window.accepted += 1
        self.window.mqtt_queued += 1
        self.total_accepted += 1

    def run(self) -> None:
        print(
            f"[Judge] UDP listen {self.args.udp_host}:{self.args.udp_port}; "
            f"drop when gap <= {self.args.min_packet_gap_ms:.3f}ms; "
            f"MQTT output is {MQTT_PACKET_BYTES}B",
            flush=True,
        )
        self.start_mqtt()
        while self.running:
            self.print_stats_if_due()
            try:
                packet, _source = self.socket.recvfrom(PACKET_BYTES + 1)
            except socket.timeout:
                continue
            except OSError:
                break
            self.handle_packet(packet)


def load_config(config_path: Path) -> dict[str, object]:
    try:
        with config_path.open(encoding="utf-8") as config_file:
            loaded = yaml.safe_load(config_file)
    except OSError as error:
        raise SystemExit(f"无法读取裁判配置 {config_path}: {error}") from error
    except yaml.YAMLError as error:
        raise SystemExit(f"裁判配置 YAML 格式错误 {config_path}: {error}") from error

    if loaded is None:
        loaded = {}
    if not isinstance(loaded, dict):
        raise SystemExit(f"裁判配置必须是键值映射: {config_path}")
    unknown = set(loaded) - set(DEFAULT_CONFIG)
    if unknown:
        raise SystemExit(f"裁判配置包含未知字段: {', '.join(sorted(unknown))}")

    config = DEFAULT_CONFIG.copy()
    config.update(loaded)
    try:
        config["udp_host"] = str(config["udp_host"])
        config["udp_port"] = int(config["udp_port"])
        config["mqtt_host"] = str(config["mqtt_host"])
        config["mqtt_port"] = int(config["mqtt_port"])
        config["mqtt_topic"] = str(config["mqtt_topic"])
        config["mqtt_client_id"] = str(config["mqtt_client_id"])
        config["mqtt_qos"] = int(config["mqtt_qos"])
        config["min_packet_gap_ms"] = float(config["min_packet_gap_ms"])
        config["stats_interval_sec"] = float(config["stats_interval_sec"])
    except (TypeError, ValueError) as error:
        raise SystemExit(f"裁判配置字段类型错误: {error}") from error
    return config


def parse_args() -> argparse.Namespace:
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    config_args, remaining_args = config_parser.parse_known_args()
    config = load_config(config_args.config)

    parser = argparse.ArgumentParser(description="Hero AntiBase judge simulator")
    parser.add_argument("--config", type=Path, default=config_args.config,
                        help="裁判 YAML 配置文件")
    parser.add_argument("--udp-host", default=config["udp_host"])
    parser.add_argument("--udp-port", type=int, default=config["udp_port"])
    parser.add_argument("--mqtt-host", default=config["mqtt_host"])
    parser.add_argument("--mqtt-port", type=int, default=config["mqtt_port"])
    parser.add_argument("--mqtt-topic", default=config["mqtt_topic"])
    parser.add_argument("--client-id", default=config["mqtt_client_id"])
    parser.add_argument("--qos", type=int, choices=(0, 1), default=config["mqtt_qos"])
    parser.add_argument("--min-packet-gap-ms", type=float,
                        default=config["min_packet_gap_ms"])
    parser.add_argument("--stats-interval-sec", type=float,
                        default=config["stats_interval_sec"])
    args = parser.parse_args(remaining_args)
    if not 1 <= args.udp_port <= 65535 or not 1 <= args.mqtt_port <= 65535:
        parser.error("端口必须在 1..65535")
    if args.min_packet_gap_ms < 0.0 or args.stats_interval_sec <= 0.0:
        parser.error("间隔阈值必须非负，统计周期必须为正")
    return args


def main() -> int:
    judge = LocalJudge(parse_args())

    def stop_handler(_signal_number, _frame) -> None:
        judge.running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        judge.run()
    finally:
        judge.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
