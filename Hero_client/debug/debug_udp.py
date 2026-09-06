#!/usr/bin/env python3
"""
UDP 3334 图传包调试工具:
1) 监听自定义客户端 UDP 图传端口 3334
2) 打印每个 UDP 包的长度、包头字段、payload 前缀/完整 hex
3) 退出时汇总帧编号、分片序号、大小分布和疑似问题

规则包头：
  frame_id      2 bytes
  fragment_id   2 bytes
  frame_size    4 bytes
  payload       remaining bytes
"""

import argparse
import collections
import socket
import struct
import time


CONFIG = {
    "host": "192.168.12.2",
    "port": 3334,
    "max_packet_bytes": 65535,
    "recv_buffer_bytes": 4 * 1024 * 1024,
    "print_limit": 0,          # 0 means unlimited
    "hex_preview_bytes": 48,
    "stats_interval_s": 1.0,
    "default_endian": "little",   # protocol/network order unless proved otherwise
}


class FrameStats:
    def __init__(self, frame_id: int, total_size: int):
        self.frame_id = frame_id
        self.total_size = total_size
        self.first_ts = time.time()
        self.last_ts = self.first_ts
        self.fragments = {}
        self.packet_sizes = []
        self.payload_bytes = 0
        self.duplicate_fragments = 0
        self.total_size_changes = 0

    def add(self, fragment_id: int, total_size: int, payload_len: int, packet_len: int):
        now = time.time()
        self.last_ts = now
        if total_size != self.total_size:
            self.total_size_changes += 1
        if fragment_id in self.fragments:
            self.duplicate_fragments += 1
        else:
            self.fragments[fragment_id] = payload_len
        self.payload_bytes += payload_len
        self.packet_sizes.append(packet_len)

    def fragment_range_text(self):
        if not self.fragments:
            return "--"
        keys = sorted(self.fragments)
        return f"{keys[0]}..{keys[-1]}"

    def missing_fragments(self):
        if not self.fragments:
            return []
        keys = sorted(self.fragments)
        start = 0 if 0 in self.fragments else keys[0]
        end = keys[-1]
        return [i for i in range(start, end + 1) if i not in self.fragments]

    def is_complete_by_bytes(self):
        unique_payload = sum(self.fragments.values())
        return unique_payload >= self.total_size


class UdpDebugReceiver:
    def __init__(self, args):
        self.host = args.host
        self.port = args.port
        self.endian = "little" if args.little_endian else args.endian
        self.header = struct.Struct((">" if self.endian == "big" else "<") + "HHI")
        self.max_packet_bytes = args.max_packet_bytes
        self.recv_buffer_bytes = args.recv_buffer_bytes
        self.hex_preview_bytes = args.hex_preview_bytes
        self.full_hex = args.full_hex
        self.print_limit = args.print_limit
        self.stats_interval_s = args.stats_interval_s

        self.total_packets = 0
        self.total_bytes = 0
        self.total_payload_bytes = 0
        self.bad_packets = 0
        self.last_print_ts = time.time()
        self.start_ts = time.time()
        self.packet_size_counter = collections.Counter()
        self.payload_size_counter = collections.Counter()
        self.frame_size_counter = collections.Counter()
        self.source_counter = collections.Counter()
        self.frames = {}
        self.recent_packet_ts = collections.deque()
        self.recent_bytes = collections.deque()

    @staticmethod
    def _hex(data: bytes, max_bytes: int):
        if max_bytes <= 0:
            return ""
        shown = data[:max_bytes]
        text = " ".join(f"{b:02X}" for b in shown)
        if len(data) > max_bytes:
            text += f" ...(+{len(data) - max_bytes}B)"
        return text

    @staticmethod
    def _ascii_preview(data: bytes, max_bytes: int):
        shown = data[:max_bytes]
        return "".join(chr(b) if 32 <= b < 127 else "." for b in shown)

    def _maybe_print_window_stats(self):
        now = time.time()
        if now - self.last_print_ts < self.stats_interval_s:
            return
        self.last_print_ts = now
        cutoff = now - 1.0
        while self.recent_packet_ts and self.recent_packet_ts[0] < cutoff:
            self.recent_packet_ts.popleft()
        while self.recent_bytes and self.recent_bytes[0][0] < cutoff:
            self.recent_bytes.popleft()
        pps = len(self.recent_packet_ts)
        kbps = sum(size for _, size in self.recent_bytes) / 1024.0
        print(
            f"[窗口] pps={pps} rate={kbps:.2f}kB/s "
            f"total={self.total_packets} bad={self.bad_packets} frames={len(self.frames)}"
        )

    def _print_packet(self, addr, packet_len, frame_id, fragment_id, frame_size, payload):
        if self.print_limit > 0 and self.total_packets > self.print_limit:
            return
        payload_len = len(payload)
        hex_len = len(payload) if self.full_hex else self.hex_preview_bytes
        print(
            f"[包] #{self.total_packets} from={addr[0]}:{addr[1]} "
            f"len={packet_len} payload={payload_len} "
            f"frame={frame_id} frag={fragment_id} frame_size={frame_size} "
            f"endian={self.endian}"
        )
        if hex_len > 0:
            print(f"     hex={self._hex(payload, hex_len)}")
            print(f"     ascii={self._ascii_preview(payload, min(hex_len, 96))}")

    def _record_packet(self, addr, data: bytes):
        packet_len = len(data)
        self.total_packets += 1
        self.total_bytes += packet_len
        self.packet_size_counter[packet_len] += 1
        self.source_counter[f"{addr[0]}:{addr[1]}"] += 1
        now = time.time()
        self.recent_packet_ts.append(now)
        self.recent_bytes.append((now, packet_len))

        if packet_len < self.header.size:
            self.bad_packets += 1
            print(f"[坏包] #{self.total_packets} from={addr[0]}:{addr[1]} len={packet_len} < 8")
            return

        frame_id, fragment_id, frame_size = self.header.unpack_from(data, 0)
        payload = data[self.header.size:]
        payload_len = len(payload)
        self.total_payload_bytes += payload_len
        self.payload_size_counter[payload_len] += 1
        self.frame_size_counter[frame_size] += 1

        frame = self.frames.get(frame_id)
        if frame is None:
            frame = FrameStats(frame_id, frame_size)
            self.frames[frame_id] = frame
        frame.add(fragment_id, frame_size, payload_len, packet_len)

        if frame_size == 0 or frame_size > 64 * 1024 * 1024:
            self.bad_packets += 1
            print(
                f"[疑似包头异常] #{self.total_packets} frame={frame_id} frag={fragment_id} "
                f"frame_size={frame_size} payload={payload_len} "
                f"提示: 若 frame_size 离谱，试试 --little-endian"
            )

        self._print_packet(addr, packet_len, frame_id, fragment_id, frame_size, payload)

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.recv_buffer_bytes)
        sock.bind((self.host, self.port))
        sock.settimeout(0.5)
        print(f"[启动] UDP监听 {self.host}:{self.port} endian={self.endian}")
        print("[提示] 默认只打印 payload 前缀；需要完整 hex 用 --full-hex，包很多时建议 --print-limit N")

        try:
            while True:
                try:
                    data, addr = sock.recvfrom(self.max_packet_bytes)
                except socket.timeout:
                    self._maybe_print_window_stats()
                    continue
                self._record_packet(addr, data)
                self._maybe_print_window_stats()
        except KeyboardInterrupt:
            print("\n[退出] 收到 Ctrl+C")
        finally:
            sock.close()
            self.print_summary()

    @staticmethod
    def _counter_text(counter, limit=8):
        if not counter:
            return "--"
        return ", ".join(f"{key}:{value}" for key, value in counter.most_common(limit))

    def print_summary(self):
        elapsed = max(1e-6, time.time() - self.start_ts)
        complete_frames = sum(1 for frame in self.frames.values() if frame.is_complete_by_bytes())
        duplicate_fragments = sum(frame.duplicate_fragments for frame in self.frames.values())
        size_changes = sum(frame.total_size_changes for frame in self.frames.values())
        incomplete_frames = len(self.frames) - complete_frames
        print("\n========== UDP 3334 调试汇总 ==========")
        print(f"运行时间: {elapsed:.2f}s")
        print(f"总包数: {self.total_packets}, 坏/疑似异常包: {self.bad_packets}")
        print(f"总字节: {self.total_bytes}B, payload总字节: {self.total_payload_bytes}B")
        print(f"平均包率: {self.total_packets / elapsed:.2f} pkt/s")
        print(f"平均速率: {self.total_bytes / elapsed / 1024.0:.2f} kB/s")
        print(f"来源: {self._counter_text(self.source_counter)}")
        print(f"UDP包大小分布: {self._counter_text(self.packet_size_counter)}")
        print(f"payload大小分布: {self._counter_text(self.payload_size_counter)}")
        print(f"frame_size分布: {self._counter_text(self.frame_size_counter)}")
        print(
            f"帧数: {len(self.frames)}, 按字节完整帧: {complete_frames}, "
            f"不完整帧: {incomplete_frames}, 重复分片: {duplicate_fragments}, "
            f"同帧frame_size变化: {size_changes}"
        )

        if not self.frames:
            return

        print("\n最近/前若干帧详情:")
        for frame_id in sorted(self.frames)[-20:]:
            frame = self.frames[frame_id]
            missing = frame.missing_fragments()
            unique_payload = sum(frame.fragments.values())
            duration_ms = max(0.0, (frame.last_ts - frame.first_ts) * 1000.0)
            missing_text = "none" if not missing else (
                ",".join(map(str, missing[:12])) + ("..." if len(missing) > 12 else "")
            )
            print(
                f"  frame={frame_id} total={frame.total_size} unique_payload={unique_payload} "
                f"frags={len(frame.fragments)} range={frame.fragment_range_text()} "
                f"complete={int(frame.is_complete_by_bytes())} missing={missing_text} "
                f"dup={frame.duplicate_fragments} span={duration_ms:.1f}ms"
            )


def parse_args():
    parser = argparse.ArgumentParser(description="UDP 3334 图传包调试工具")
    parser.add_argument("--host", default=CONFIG["host"], help="UDP监听地址")
    parser.add_argument("--port", type=int, default=CONFIG["port"], help="UDP监听端口")
    parser.add_argument("--endian", choices=["big", "little"], default=CONFIG["default_endian"], help="包头解析字节序")
    parser.add_argument("--little-endian", action="store_true", help="等价于 --endian little")
    parser.add_argument("--max-packet-bytes", type=int, default=CONFIG["max_packet_bytes"], help="单包最大接收字节")
    parser.add_argument("--recv-buffer-bytes", type=int, default=CONFIG["recv_buffer_bytes"], help="socket接收缓冲")
    parser.add_argument("--hex-preview-bytes", type=int, default=CONFIG["hex_preview_bytes"], help="每包payload预览字节数")
    parser.add_argument("--full-hex", action="store_true", help="打印每包完整payload hex，输出会很大")
    parser.add_argument("--print-limit", type=int, default=CONFIG["print_limit"], help="最多逐包打印多少包，0为不限")
    parser.add_argument("--stats-interval-s", type=float, default=CONFIG["stats_interval_s"], help="窗口统计打印周期")
    return parser.parse_args()


def main():
    args = parse_args()
    UdpDebugReceiver(args).run()


if __name__ == "__main__":
    main()
