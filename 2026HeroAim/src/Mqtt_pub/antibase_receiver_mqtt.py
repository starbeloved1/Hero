#!/usr/bin/env python3
"""
AntiBase UDP -> MQTT 发布端

职责：
1) 从 UDP 接收 300B 固定包
2) 不做业务层序列化，直接发布与裁判系统一致的 303B 消息格式
   (`0A + varint(300) + data[300]`)
3) 本地打印统计
"""

import argparse
import collections
import socket
import struct
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print('需要安装 paho-mqtt: pip install paho-mqtt')
    sys.exit(1)

PACKET_SIZE = 300
HEADER_FMT = '<Q'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
STATS_INTERVAL_S = 1.0
JUDGE_MIN_PACKET_GAP_MS = 20.0

CONFIG = {
    'udp_bind_host': '127.0.0.1',
    'udp_port': 9999,
    'broker': '127.0.0.1',
    'mqtt_port': 1884,
    'client_id': 'antibase_bridge_pub',
    'topic_video': 'CustomByteBlock',
    'qos_level': 1,
    # 模拟裁判：相邻收到的 300B 包间隔必须严格大于 20ms，否则该包不转发。
    'judge_sim_enabled': True,
    'judge_min_packet_gap_ms': JUDGE_MIN_PACKET_GAP_MS,
    'enable_cli_override': False,
}


class UdpToMqttBridge:
    def __init__(
        self,
        udp_port: int,
        broker: str,
        mqtt_port: int,
        client_id: str,
        topic_video: str,
        topic_stats: str,
        udp_bind_host: str,
        qos_level: int,
        judge_sim_enabled: bool,
        judge_min_packet_gap_ms: float,
    ):
        self.udp_port = udp_port
        self.udp_bind_host = udp_bind_host
        self.broker = broker
        self.mqtt_port = mqtt_port
        self.topic_video = topic_video
        self.topic_stats = topic_stats
        self.qos_level = qos_level
        self.judge_sim_enabled = bool(judge_sim_enabled)
        self.judge_min_packet_gap_ms = max(0.0, float(judge_min_packet_gap_ms))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.udp_bind_host, self.udp_port))
        self.sock.settimeout(1.0)

        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
        )
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_publish = self.on_publish

        self.total_pkts = 0
        self.total_bytes = 0
        self.total_udp_lost = 0
        self.total_udp_duplicate = 0
        self.total_judge_dropped = 0
        self.last_seq = None

        self.win_pkts = collections.deque()
        self.win_bytes = collections.deque()
        self.win_gaps_ms = collections.deque()
        self.last_udp_recv_monotonic = None
        self.last_print = time.time()
        self.last_delay_print = time.time()
        self.running = True
        self.mqtt_pub_try = 0
        self.mqtt_pub_ack = 0
        self.latest_seq = None
        self.latest_bridge_pub_ms = 0.0
        self.latest_mqtt_ack_ms = -1.0
        self.latest_pub_mid = None
        self.pub_start_by_mid = {}

    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        if reason_code.is_failure:
            print(f'[MQTT] 连接失败: {reason_code}')
        else:
            print(f'[MQTT] 已连接 {self.broker}:{self.mqtt_port}')
            print(f'[MQTT] 发布话题: {self.topic_video}, qos={self.qos_level}')

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        if reason_code.is_failure:
            print(f'[MQTT] 异常断开: {reason_code}')

    def on_publish(self, client, userdata, mid, reason_code, properties):
        self.mqtt_pub_ack += 1
        t0 = self.pub_start_by_mid.pop(mid, None)
        if t0 is not None:
            self.latest_mqtt_ack_ms = max(0.0, (time.time_ns() - t0) / 1e6)

    def connect_mqtt(self):
        delay_s = 1.0
        while self.running:
            try:
                self.client.connect(self.broker, self.mqtt_port, keepalive=60)
                self.client.loop_start()
                return True
            except (TimeoutError, socket.timeout, OSError) as e:
                print(f'[MQTT] 连接超时/失败: {e}, {delay_s:.1f}s 后重试')
                time.sleep(delay_s)
                delay_s = min(delay_s * 2.0, 10.0)
        return False

    def disconnect_mqtt(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass

    @staticmethod
    def _encode_varint(value: int) -> bytes:
        if value < 0:
            raise ValueError('varint 仅支持非负整数')
        out = bytearray()
        while True:
            to_write = value & 0x7F
            value >>= 7
            if value:
                out.append(to_write | 0x80)
            else:
                out.append(to_write)
                break
        return bytes(out)

    def publish_video_packet(self, payload_300: bytes):
        if len(payload_300) != PACKET_SIZE:
            return

        pub_begin_ns = time.time_ns()
        mqtt_payload = bytes([0x0A]) + self._encode_varint(PACKET_SIZE) + payload_300
        self.mqtt_pub_try += 1
        info = self.client.publish(self.topic_video, mqtt_payload, qos=self.qos_level)
        self.latest_pub_mid = info.mid
        self.pub_start_by_mid[info.mid] = pub_begin_ns
        self.latest_bridge_pub_ms = max(0.0, (time.time_ns() - pub_begin_ns) / 1e6)

    def run(self):
        print(f'[Bridge] 监听 UDP: {self.udp_bind_host}:{self.udp_port}')
        print(f'[Bridge] 包结构: 总长={PACKET_SIZE}B, 头={HEADER_SIZE}B, payload={PACKET_SIZE - HEADER_SIZE}B')
        if self.judge_sim_enabled:
            print(
                f'[Judge SIM] 已启用：相邻收到包间隔 <= {self.judge_min_packet_gap_ms:.1f}ms '
                '直接丢弃，不发布 MQTT'
            )

        if not self.connect_mqtt():
            print('[Bridge] MQTT 未建立连接，程序退出')
            return

        while self.running:
            now = time.time()

            if now - self.last_print >= STATS_INTERVAL_S:
                cutoff = now - STATS_INTERVAL_S
                while self.win_pkts and self.win_pkts[0] < cutoff:
                    self.win_pkts.popleft()
                while self.win_bytes and self.win_bytes[0][0] < cutoff:
                    self.win_bytes.popleft()
                while self.win_gaps_ms and self.win_gaps_ms[0][0] < cutoff:
                    self.win_gaps_ms.popleft()

                pps = len(self.win_pkts)
                kbps = sum(b for _, b in self.win_bytes) / 1024.0
                seen = self.total_pkts + self.total_udp_lost
                loss_pct = (self.total_udp_lost / seen * 100.0) if seen > 0 else 0.0
                gaps_ms = [gap for _, gap in self.win_gaps_ms]
                min_gap = min(gaps_ms) if gaps_ms else 0.0
                avg_gap = sum(gaps_ms) / len(gaps_ms) if gaps_ms else 0.0
                max_gap = max(gaps_ms) if gaps_ms else 0.0
                unsafe_gap = sum(gap <= self.judge_min_packet_gap_ms for gap in gaps_ms)

                stats_line = (
                    f'[Bridge] 速率={kbps:.2f} kB/s 包/秒={pps} '
                    f'间隔ms(min/avg/max)={min_gap:.2f}/{avg_gap:.2f}/{max_gap:.2f} '
                    f'<={self.judge_min_packet_gap_ms:.1f}ms={unsafe_gap} '
                    f'累计放行={self.total_pkts}包/{self.total_bytes / 1024.0:.1f} kB '
                    f'裁判模拟丢弃={self.total_judge_dropped} '
                    f'UDP源丢包={self.total_udp_lost}({loss_pct:.1f}%) UDP重复过滤={self.total_udp_duplicate} 最新seq={self.last_seq} '
                    f'MQTT发布尝试/确认={self.mqtt_pub_try}/{self.mqtt_pub_ack}'
                )
                if unsafe_gap > 0:
                    print(f'\033[33m[Bridge LIMIT VIOLATION] {stats_line}\033[0m')
                else:
                    print(stats_line)
                print(
                    f'[延迟][Bridge] seq={self.latest_seq} '
                    f'bridge_pub={self.latest_bridge_pub_ms:.2f}ms '
                    f'mqtt_ack={self.latest_mqtt_ack_ms:.2f}ms'
                )
                self.last_print = now
                self.last_delay_print = now

            try:
                data, _addr = self.sock.recvfrom(PACKET_SIZE + 64)
            except socket.timeout:
                continue

            if len(data) != PACKET_SIZE:
                continue

            (seq_id,) = struct.unpack_from(HEADER_FMT, data, 0)

            # UDP debug 镜像或上游链路可能出现相邻同序号重发。该 300B 包的
            # 内容由序号唯一标识，重复发布只会浪费 QoS 1 带宽并制造客户端
            # 重复计数；在 bridge 入口直接忽略，不影响正常序号缺口统计。
            if self.last_seq is not None and seq_id == self.last_seq:
                self.total_udp_duplicate += 1
                continue

            self.latest_seq = int(seq_id)

            if self.last_seq is not None:
                gap = int(seq_id) - int(self.last_seq) - 1
                if gap > 0:
                    self.total_udp_lost += gap
            self.last_seq = seq_id

            recv_monotonic = time.monotonic()
            recv_t = time.time()
            gap_ms = None
            if self.last_udp_recv_monotonic is not None:
                gap_ms = (recv_monotonic - self.last_udp_recv_monotonic) * 1000.0
                self.win_gaps_ms.append((recv_t, gap_ms))
            self.last_udp_recv_monotonic = recv_monotonic

            # 按裁判描述的“相邻收到的两个包”做检测。被拒包也更新上一个收到
            # 包的时刻，因此连续高频突发会连续被拒，和检测端逐包比较一致。
            if (
                self.judge_sim_enabled
                and gap_ms is not None
                and gap_ms <= self.judge_min_packet_gap_ms
            ):
                self.total_judge_dropped += 1
                print(
                    f'\033[33m[Judge SIM DROP] seq={seq_id} gap={gap_ms:.2f}ms '
                    f'<= {self.judge_min_packet_gap_ms:.1f}ms，未发布 MQTT\033[0m'
                )
                continue

            self.total_pkts += 1
            self.total_bytes += PACKET_SIZE
            self.win_pkts.append(recv_t)
            self.win_bytes.append((recv_t, PACKET_SIZE))

            self.publish_video_packet(data)

    def stop(self):
        self.running = False
        self.disconnect_mqtt()
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    runtime_cfg = dict(CONFIG)
    if CONFIG.get('enable_cli_override', False):
        parser = argparse.ArgumentParser(description='AntiBase UDP->MQTT 发布端')
        parser.add_argument('--udp-bind-host', type=str, default=CONFIG['udp_bind_host'], help='UDP 绑定地址')
        parser.add_argument('--udp-port', type=int, default=CONFIG['udp_port'], help='UDP 监听端口')
        parser.add_argument('--broker', type=str, default=CONFIG['broker'], help='MQTT broker')
        parser.add_argument('--mqtt-port', type=int, default=CONFIG['mqtt_port'], help='MQTT TCP 端口')
        parser.add_argument('--client-id', type=str, default=CONFIG['client_id'], help='MQTT client_id')
        parser.add_argument('--topic-video', type=str, default=CONFIG['topic_video'], help='话题')
        parser.add_argument('--qos', type=int, default=CONFIG['qos_level'], choices=[0, 1], help='MQTT QoS')
        args = parser.parse_args()
        runtime_cfg.update(
            {
                'udp_bind_host': args.udp_bind_host,
                'udp_port': args.udp_port,
                'broker': args.broker,
                'mqtt_port': args.mqtt_port,
                'client_id': args.client_id,
                'topic_video': args.topic_video,
                'qos_level': args.qos,
            }
        )

    bridge = UdpToMqttBridge(
        runtime_cfg['udp_port'],
        runtime_cfg['broker'],
        runtime_cfg['mqtt_port'],
        runtime_cfg['client_id'],
        runtime_cfg['topic_video'],
        None,
        runtime_cfg['udp_bind_host'],
        runtime_cfg['qos_level'],
        runtime_cfg['judge_sim_enabled'],
        runtime_cfg['judge_min_packet_gap_ms'],
    )
    try:
        bridge.run()
    except KeyboardInterrupt:
        print('\n[Bridge] 已停止')
    finally:
        bridge.stop()


if __name__ == '__main__':
    main()
