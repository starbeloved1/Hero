#!/usr/bin/env python3
"""
订阅 MQTT 并在终端打印每个 303B 包的完整内容（十六进制）。
"""

import argparse
import os
import struct
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("需要安装 paho-mqtt: pip install paho-mqtt", flush=True)
    sys.exit(1)

FRAME_SIZE = 303
DATA_SIZE = 300
PB_TAG = 0x0A


CONFIG = {
    "broker": "192.168.12.1",
    "port": 3333,
    "topic": "CustomByteBlock",
    "qos": 1,
    # 设置固定 client_id；若留空且 auto_client_id_on_empty=True，则自动生成唯一ID。
    "client_id": "101",
    "auto_client_id_on_empty": False,
    # 0 表示无限打印。
    "max_packets": 0,
    # 是否允许命令行参数覆盖上面配置。
    "enable_cli_override": True,
}


def parse_varint(payload: bytes, start_idx: int = 1):
    idx = start_idx
    length = 0
    shift = 0
    while True:
        if idx >= len(payload):
            raise ValueError("varint 越界")
        byte = payload[idx]
        length |= (byte & 0x7F) << shift
        idx += 1
        if byte < 0x80:
            break
        shift += 7
        if shift > 28:
            raise ValueError("varint 过长")
    return length, idx


def hexdump(data: bytes, width: int = 16) -> str:
    lines = []
    for offset in range(0, len(data), width):
        chunk = data[offset: offset + width]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        lines.append(f"{offset:04X}: {hex_part}")
    return "\n".join(lines)


class PacketDumper:
    def __init__(self, broker: str, port: int, topic: str, qos: int, client_id: str, max_packets: int):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.qos = qos
        self.client_id = client_id
        self.max_packets = max_packets

        self.packet_count = 0
        self.last_seq = None

        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
        )
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"[MQTT] 连接失败: {reason_code}", flush=True)
            return
        print(f"[MQTT] 已连接 {self.broker}:{self.port}", flush=True)
        client.subscribe(self.topic, qos=self.qos)
        print(f"[MQTT] 订阅: topic={self.topic} qos={self.qos}", flush=True)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"[MQTT] 异常断开: {reason_code}", flush=True)
        else:
            print("[MQTT] 已断开", flush=True)

    def _print_303_packet(self, payload: bytes):
        seq = None
        pb_len = None
        parse_error = None

        try:
            if payload[0] != PB_TAG:
                raise ValueError(f"protobuf tag 错误: 0x{payload[0]:02X}")
            pb_len, data_start = parse_varint(payload, 1)
            if pb_len != DATA_SIZE:
                raise ValueError(f"protobuf data 长度异常: {pb_len}")

            data = payload[data_start:data_start + DATA_SIZE]
            if len(data) != DATA_SIZE:
                raise ValueError(f"data 区长度异常: {len(data)}")

            seq = struct.unpack("<Q", data[0:8])[0]
        except Exception as exc:
            parse_error = str(exc)

        print("=" * 90, flush=True)
        print(f"[303B] 包#{self.packet_count} 接收时间={time.strftime('%H:%M:%S')} len={len(payload)}", flush=True)

        if parse_error is None:
            gap = "-"
            if self.last_seq is not None and seq is not None:
                gap = str(seq - self.last_seq - 1)
            print(f"[303B] pb_tag=0x{payload[0]:02X} pb_len={pb_len} seq={seq} gap={gap}", flush=True)
            self.last_seq = seq
        else:
            print(f"[303B] 解析失败: {parse_error}", flush=True)

        print("[303B] 完整内容(十六进制):", flush=True)
        print(hexdump(payload), flush=True)

    def on_message(self, client, userdata, msg):
        self.packet_count += 1
        payload = msg.payload

        if len(payload) == FRAME_SIZE:
            self._print_303_packet(payload)
        else:
            print("=" * 90, flush=True)
            print(f"[非303B] 包#{self.packet_count} len={len(payload)} topic={msg.topic}", flush=True)
            print(hexdump(payload), flush=True)

        if self.max_packets > 0 and self.packet_count >= self.max_packets:
            print(f"[结束] 已达到 max_packets={self.max_packets}", flush=True)
            client.disconnect()

    def run(self):
        print("[启动] MQTT 303B 包打印器", flush=True)
        print(
            f"[配置] broker={self.broker}:{self.port} topic={self.topic} "
            f"qos={self.qos} client_id={self.client_id} max_packets={self.max_packets}",
            flush=True,
        )
        self.client.connect(self.broker, self.port, keepalive=60)
        self.client.loop_forever()


def build_client_id(user_client_id: str | None, auto_on_empty: bool):
    if user_client_id:
        return user_client_id
    if not auto_on_empty:
        # 配置了空 client_id 且不自动生成时，使用一个保底唯一ID避免连接被 broker 拒绝。
        return f"dump303-fallback-{os.getpid()}"
    # 避免 broker 因重复 client_id 拒绝连接。
    return f"dump303-{os.getpid()}-{int(time.time()) % 100000}"


def main():
    runtime_cfg = dict(CONFIG)
    if CONFIG.get("enable_cli_override", True):
        parser = argparse.ArgumentParser(description="订阅 MQTT 并打印每个 303B 包完整内容")
        parser.add_argument("--broker", default=runtime_cfg["broker"], help="MQTT broker")
        parser.add_argument("--port", type=int, default=runtime_cfg["port"], help="MQTT TCP 端口")
        parser.add_argument("--topic", default=runtime_cfg["topic"], help="订阅话题")
        parser.add_argument("--qos", type=int, choices=[0, 1], default=runtime_cfg["qos"], help="订阅 QoS")
        parser.add_argument("--client-id", default=runtime_cfg["client_id"], help="MQTT client_id")
        parser.add_argument("--max-packets", type=int, default=runtime_cfg["max_packets"], help="最多打印包数，0=不限")
        args = parser.parse_args()

        runtime_cfg.update(
            {
                "broker": args.broker,
                "port": args.port,
                "topic": args.topic,
                "qos": args.qos,
                "client_id": args.client_id,
                "max_packets": max(0, args.max_packets),
            }
        )

    dumper = PacketDumper(
        broker=runtime_cfg["broker"],
        port=runtime_cfg["port"],
        topic=runtime_cfg["topic"],
        qos=runtime_cfg["qos"],
        client_id=build_client_id(runtime_cfg.get("client_id"), bool(runtime_cfg.get("auto_client_id_on_empty", True))),
        max_packets=max(0, int(runtime_cfg.get("max_packets", 0))),
    )

    try:
        dumper.run()
    except KeyboardInterrupt:
        print("\n[退出] 用户中断", flush=True)
    except Exception as exc:
        print(f"[异常] {exc}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
