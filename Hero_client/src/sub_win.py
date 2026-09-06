"""
Mode4 MQTT/H.264 接收端。
收包、协议解析、H.264 组帧、解码和预览
独立运行模式和集成运行共用
"""

from __future__ import annotations

import json
import queue
import struct
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Iterable, Optional

import av
import cv2
import numpy as np
import paho.mqtt.client as mqtt

try:
    from .control_protocol import encode_custom_control
except ImportError:
    from control_protocol import encode_custom_control

try:
    from config import MQTT_CONFIG as CONFIG
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from config import MQTT_CONFIG as CONFIG


FRAME_SIZE = 303
DATA_SIZE = 300
PACKET_HEADER_SIZE = 8
PB_TAG = 0x0A
MAX_H264_BUFFER_BYTES = 1024 * 1024


def make_status_frame(width: int, height: int, text: str) -> np.ndarray:
    width, height = max(640, int(width)), max(360, int(height))
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    scale = max(0.8, min(width, height) / 700.0)
    thickness = max(1, round(scale * 2))
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    cv2.putText(frame, text, ((width - tw) // 2, max(th, (height + th) // 2)),
                cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return frame


class PreviewState:
    """独立窗口与集成面板共用的最新 JPEG 和指标快照。"""

    def __init__(self) -> None:
        self.condition = threading.Condition()
        self._metrics_lock = threading.Lock()
        self._metrics: dict = {}
        self.latest_jpeg: Optional[bytes] = None

    def update_metrics(self, metrics: dict) -> None:
        with self._metrics_lock:
            self._metrics = dict(metrics)

    def snapshot_metrics(self) -> dict:
        with self._metrics_lock:
            return dict(self._metrics)

    def set_jpeg_frame(self, jpeg: bytes) -> None:
        with self.condition:
            self.latest_jpeg = bytes(jpeg)
            self.condition.notify_all()


def make_preview_handler(state: PreviewState, preview_fps: int = 20):
    refresh_ms = max(15, round(1000 / max(1, int(preview_fps))))

    class PreviewHandler(BaseHTTPRequestHandler):
        def log_message(self, *_args) -> None:
            return

        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                html = f'''<!doctype html><meta charset="utf-8"><title>Mode4 MQTT</title>
<style>html,body{{margin:0;background:#080a0c;color:#eee;font-family:sans-serif;overflow:hidden}}#v{{width:100vw;height:100vh;object-fit:contain;display:block}}#grid{{position:fixed;inset:0;width:100vw;height:100vh;pointer-events:none}}#grid path{{fill:none;stroke:#00d25a;stroke-width:1;vector-effect:non-scaling-stroke;opacity:.85;shape-rendering:crispEdges}}#s{{position:fixed;top:12px;left:12px;background:#000a;padding:7px 10px;font-size:12px;font-weight:700}}</style>
<img id="v"><svg id="grid" viewBox="0 0 6 6" preserveAspectRatio="none" aria-hidden="true"><path d="M1 0V6 M2 0V6 M3 0V6 M4 0V6 M5 0V6 M0 1H6 M0 2H6 M0 3H6 M0 4H6 M0 5H6"/></svg><span id="s">MQTT · WAITING</span><script>
const v=document.querySelector('#v'),g=document.querySelector('#grid'),s=document.querySelector('#s');
function fitGrid(){{
  if(!v.naturalWidth||!v.naturalHeight)return;
  const r=v.getBoundingClientRect(),scale=Math.min(r.width/v.naturalWidth,r.height/v.naturalHeight);
  const w=v.naturalWidth*scale,h=v.naturalHeight*scale;
  g.style.left=(r.left+(r.width-w)/2)+'px';g.style.top=(r.top+(r.height-h)/2)+'px';
  g.style.width=w+'px';g.style.height=h+'px';g.style.right='auto';g.style.bottom='auto';
}}
v.onload=fitGrid;window.addEventListener('resize',fitGrid);
setInterval(()=>v.src='/preview.jpg?t='+Date.now(),{refresh_ms});
setInterval(async()=>{{try{{const m=await (await fetch('/api/status')).json();s.textContent='MQTT · '+(m.packet_active&&m.mqtt_connected?'VIDEO ACTIVE':'WAITING FOR PACKETS')}}catch(e){{s.textContent='MQTT · DISCONNECTED'}}}},500);
</script>'''.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return
            if path == "/api/status":
                body = json.dumps(state.snapshot_metrics(), ensure_ascii=False).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/preview.jpg":
                with state.condition:
                    jpeg = state.latest_jpeg
                if not jpeg:
                    self.send_response(503)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                try:
                    self.wfile.write(jpeg)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            self.send_error(404)

    return PreviewHandler


class AnnexBAssembler:
    """增量 Annex-B 缓冲：提取访问单元并缓存 SPS/PPS。"""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.sps: Optional[bytes] = None
        self.pps: Optional[bytes] = None

    @staticmethod
    def starts(data: bytes | bytearray) -> list[tuple[int, int]]:
        result, i, size = [], 0, len(data)
        while i + 2 < size:
            if i + 3 < size and data[i:i + 4] == b"\x00\x00\x00\x01":
                result.append((i, 4)); i += 4
            elif data[i:i + 3] == b"\x00\x00\x01":
                result.append((i, 3)); i += 3
            else:
                i += 1
        return result

    @classmethod
    def nals(cls, data: bytes | bytearray) -> Iterable[bytes]:
        positions = cls.starts(data)
        for index, (start, prefix) in enumerate(positions):
            end = positions[index + 1][0] if index + 1 < len(positions) else len(data)
            if end > start + prefix:
                yield bytes(data[start + prefix:end])

    @staticmethod
    def is_vcl(nal_type: int) -> bool:
        return nal_type in (1, 2, 5)

    @staticmethod
    def _rbsp(data: bytes) -> bytes:
        out, zeros = bytearray(), 0
        for value in data:
            if zeros >= 2 and value == 3:
                zeros = 0
                continue
            out.append(value)
            zeros = zeros + 1 if value == 0 else 0
        return bytes(out)

    @staticmethod
    def _ue(data: bytes, bit: int) -> tuple[Optional[int], int]:
        total, zeros = len(data) * 8, 0
        while bit < total and not ((data[bit // 8] >> (7 - bit % 8)) & 1):
            zeros += 1; bit += 1
        if bit >= total:
            return None, bit
        bit += 1
        value = 1
        for _ in range(zeros):
            if bit >= total:
                return None, bit
            value = (value << 1) | ((data[bit // 8] >> (7 - bit % 8)) & 1)
            bit += 1
        return value - 1, bit

    @classmethod
    def first_mb(cls, nal: bytes) -> Optional[int]:
        if len(nal) <= 1:
            return None
        return cls._ue(cls._rbsp(nal[1:]), 0)[0]

    @classmethod
    def is_intra(cls, nal: bytes) -> bool:
        if len(nal) <= 1 or not cls.is_vcl(nal[0] & 0x1F):
            return False
        first, bit = cls._ue(cls._rbsp(nal[1:]), 0)
        if first is None:
            return False
        slice_type, _ = cls._ue(cls._rbsp(nal[1:]), bit)
        return slice_type is not None and slice_type % 5 in (2, 4)

    def append(self, chunk: bytes) -> None:
        self.buffer.extend(chunk)

    def clear(self) -> None:
        self.buffer.clear()

    def resync(self) -> None:
        positions = self.starts(self.buffer)
        if positions:
            if positions[0][0]:
                del self.buffer[:positions[0][0]]
        elif len(self.buffer) > 4096:
            del self.buffer[:-4]

    def next_access_unit(self) -> Optional[bytes]:
        positions = self.starts(self.buffer)
        if len(positions) < 2:
            return None
        has_vcl = False
        split = None
        for index, (start, prefix) in enumerate(positions):
            end = positions[index + 1][0] if index + 1 < len(positions) else len(self.buffer)
            nal = bytes(self.buffer[start + prefix:end])
            if not nal:
                continue
            nal_type = nal[0] & 0x1F
            if index and has_vcl and (nal_type == 9 or (self.is_vcl(nal_type) and self.first_mb(nal) == 0)):
                split = start
                break
            has_vcl = has_vcl or self.is_vcl(nal_type)
        if split is None:
            return None
        au = bytes(self.buffer[:split])
        del self.buffer[:split]
        return au or None

    def update_parameters(self, au: bytes) -> None:
        for nal in self.nals(au):
            nal_type = nal[0] & 0x1F
            if nal_type == 7 and len(nal) >= 4:
                self.sps = nal
            elif nal_type == 8 and len(nal) >= 2:
                self.pps = nal

    def add_cached_parameters(self, au: bytes, nal_types: list[int]) -> bytes:
        if 7 in nal_types or 8 in nal_types or not self.sps or not self.pps:
            return au
        return b"\x00\x00\x00\x01" + self.sps + b"\x00\x00\x00\x01" + self.pps + au


class MqttVideoSubscriber:
    """Mode4 MQTT 接收器；构造参数与旧版 sub_win.py 保持兼容。"""

    def __init__(self, broker, mqtt_port, width, fps, client_id, topic_video, qos_level,
                 topic_deploy_status='', topic_event='', damage_event_callback: Optional[Callable[[str, float], None]] = None,
                 topic_control='', control_qos_level=1, control_minus_value=0, control_plus_value=1,
                 mqtt_reconnect_min_delay_s=1, mqtt_reconnect_max_delay_s=4,
                 resync_seq_on_reconnect=True, enter_wait_idr_on_disconnect=True,
                 payload_has_embedded_header=True, web_preview_enable=True,
                 web_preview_host='0.0.0.0', web_preview_port=8099, web_preview_fps=20,
                 web_jpeg_quality=80, web_preview_width=960, auto_open_browser=True,
                 auto_open_delay_s=0.8, ingress_queue_size=60, severe_gap_threshold=30,
                 reset_cooldown_ms=250, decode_error_burst_limit=10, stall_recovery_ms=900,
                 stall_watchdog_startup_only=True, recover_on_any_gap=False,
                 drop_to_idr_on_gap=False, drop_to_idr_min_gap=5, gap_guard_min_gap=4,
                 gap_guard_total_gap=16, strict_idr_gate=True, allow_intra_slice_gate=True,
                 wait_idr_fallback_au=120, prepend_cached_sps_pps_on_idr=True,
                 startup_require_params=True, trim_h264_padding_zeros=True,
                 log_metrics_interval_s=1.0, queue_alert_ratio=.75,
                 log_packet_preview_count=10, log_nal_preview_count=30,
                 packet_timeout_s=1.0, signal_check_interval_s=.2,
                 cv_display_enable=True, web_server_enable=True, frame_queue_enable=True):
        self.broker, self.mqtt_port, self.width, self.fps = broker, int(mqtt_port), int(width), max(1, int(fps))
        self.client_id, self.topic_video, self.qos_level = client_id, topic_video, int(qos_level)
        self.topic_deploy_status = str(topic_deploy_status or '')
        self.topic_event = str(topic_event or '')
        self.damage_event_callback = damage_event_callback
        self.topic_control = str(topic_control or '')
        self.control_qos_level = max(0, min(1, int(control_qos_level)))
        self.control_payloads = {
            'left': encode_custom_control(control_minus_value),
            'right': encode_custom_control(control_plus_value),
        }
        if self.topic_control:
            for direction, payload in self.control_payloads.items():
                if not payload or len(payload) > 30:
                    raise ValueError(f'invalid control payload for {direction}')
        self.web_preview_enable = bool(web_preview_enable)
        self.web_preview_host, self.web_preview_port = web_preview_host, int(web_preview_port)
        self.web_preview_fps, self.web_jpeg_quality = max(1, int(web_preview_fps)), min(95, max(20, int(web_jpeg_quality)))
        self.web_preview_width = max(0, int(web_preview_width))
        self.auto_open_browser, self.auto_open_delay_s = bool(auto_open_browser), max(0., float(auto_open_delay_s))
        self.cv_display_enable, self.frame_queue_enable = bool(cv_display_enable), bool(frame_queue_enable)
        self.web_server_enable = bool(web_server_enable)
        self.preview_required = self.web_preview_enable or not self.frame_queue_enable
        self.ingress_queue_size = max(20, int(ingress_queue_size))
        self.severe_gap_threshold, self.reset_cooldown_ms = max(1, int(severe_gap_threshold)), max(0, int(reset_cooldown_ms))
        self.decode_error_burst_limit, self.stall_recovery_ms = max(1, int(decode_error_burst_limit)), max(100, int(stall_recovery_ms))
        self.stall_watchdog_startup_only, self.recover_on_any_gap = bool(stall_watchdog_startup_only), bool(recover_on_any_gap)
        self.drop_to_idr_on_gap, self.drop_to_idr_min_gap = bool(drop_to_idr_on_gap), max(1, int(drop_to_idr_min_gap))
        self.gap_guard_min_gap, self.gap_guard_total_gap = max(1, int(gap_guard_min_gap)), max(1, int(gap_guard_total_gap))
        self.strict_idr_gate, self.allow_intra_slice_gate = bool(strict_idr_gate), bool(allow_intra_slice_gate)
        self.wait_idr_fallback_au = max(0, int(wait_idr_fallback_au))
        self.prepend_cached_sps_pps_on_idr, self.startup_require_params = bool(prepend_cached_sps_pps_on_idr), bool(startup_require_params)
        self.trim_h264_padding_zeros = bool(trim_h264_padding_zeros)
        self.log_metrics_interval_s, self.queue_alert_ratio = max(.2, float(log_metrics_interval_s)), min(.98, max(.3, float(queue_alert_ratio)))
        self.log_packet_preview_count, self.log_nal_preview_count = max(0, int(log_packet_preview_count)), max(0, int(log_nal_preview_count))
        self.packet_timeout_s, self.signal_check_interval_s = max(.3, float(packet_timeout_s)), max(.05, float(signal_check_interval_s))
        self.resync_seq_on_reconnect, self.enter_wait_idr_on_disconnect = bool(resync_seq_on_reconnect), bool(enter_wait_idr_on_disconnect)
        self.payload_has_embedded_header = bool(payload_has_embedded_header)
        self.running, self.service_started, self.last_error = True, False, ''
        self.mqtt_connected = False
        self.mqtt_subscribed = False
        self.deployment_lock = threading.Lock()
        self.deployment_status: Optional[bool] = None
        self.deployment_last_update_ts = 0.0
        self.deployment_message_count = 0
        self.deployment_last_error = ''
        self.damage_lock = threading.Lock()
        self.damage_value: Optional[str] = None
        self.damage_last_update_ts = 0.0
        self.damage_message_count = 0
        self.damage_last_error = ''
        self.control_lock = threading.Lock()
        self.control_publish_count = 0
        self.control_last_direction = ''
        self.control_last_error = ''
        self.signal_lock = threading.Lock(); self.signal_active = False; self.signal_seen_once = False; self.resume_pending = False
        self.log_lock = threading.Lock()
        self.last_packet_ts = self.last_frame_ts = 0.
        self.last_seq: Optional[int] = None; self.consecutive_gap = 0; self.waiting_for_idr = True; self.wait_idr_skip_count = 0
        # MQTT 回调在进入解码队列前只过滤“紧邻且相同”的 sequence。不能在
        # 这里丢弃较小序号，否则会破坏解码线程对发布端重启的识别。
        self.last_ingress_sequence: Optional[int] = None
        # 倒退序号通常是重发或旧流，不能直接接收。若出现低序号连续递增，
        # 则确认发布端已经重启，主动切换到新流，避免永久卡在旧序号上。
        self.backward_sequence_active = False; self.sequence_restart_suspect_gap = 100
        self.sequence_restart_low_seq_max = 64; self.sequence_restart_confirm_packets = 3
        self.restart_candidate_start: Optional[int] = None
        self.restart_candidate_last: Optional[int] = None
        self.restart_candidate_count = 0
        self.assembler = AnnexBAssembler(); self.codec = self._new_decoder()
        self.preview_state = PreviewState(); self.preview_server = None
        # MQTT 回调、解码、预览、显示均使用独立的“只保留最新项”队列，
        # 慢消费者不会反向阻塞上游，也不会累计陈旧画面。
        self.ingress_queue: queue.Queue = queue.Queue(maxsize=self.ingress_queue_size)
        self.frame_queue: queue.Queue = queue.Queue(maxsize=1)
        self.preview_queue: queue.Queue = queue.Queue(maxsize=1)
        self.packet_times, self.accepted_packet_times, self.frame_times, self.web_frame_times = deque(), deque(), deque(), deque()
        for name in ('video_pkt_count','parsed_packet_count','accepted_packet_count','frame_count','display_frame_count','display_drop_count','web_output_frame_count','web_skip_count','last_web_jpeg_bytes','ingress_drop_count','gap_drop','gap_event_count','dup_drop','seq_repeat_drop','seq_old_drop','mqtt_protocol_dup_count','seq_restart_suspect_count','seq_restart_confirmed_count','reset_count','reset_cooldown_skip_count','buf_overflow_reset','h264_padding_trim_count','h264_padding_trim_bytes','decode_error_streak','nal_log_count'):
            setattr(self, name, 0)
        self.frame_width = self.frame_height = 0; self.last_web_preview_ts = self.last_reset_ts = self.last_log_ts = 0.
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=self.client_id)
        self.client.on_connect, self.client.on_message, self.client.on_disconnect = self.on_connect, self.on_message, self.on_disconnect
        try: self.client.reconnect_delay_set(min_delay=max(1, int(mqtt_reconnect_min_delay_s)), max_delay=max(1, int(mqtt_reconnect_max_delay_s)))
        except Exception: pass
        self.preview_thread = threading.Thread(target=self._preview_loop, daemon=True); self.preview_thread.start()
        self._show_status('WAITING FOR PACKETS')
        self.decoder_thread = threading.Thread(target=self._decode_loop, daemon=True); self.decoder_thread.start()
        self.signal_thread = threading.Thread(target=self._signal_loop, daemon=True); self.signal_thread.start()
        if self.web_server_enable: self._start_web_server()
        self._publish_metrics()

    @staticmethod
    def _new_decoder():
        codec = av.CodecContext.create('h264', 'r'); codec.thread_type = 'FRAME'
        try: codec.flags |= av.codec.context.Flags.LOW_DELAY
        except Exception: pass
        return codec

    @staticmethod
    def _parse_message(payload: bytes) -> tuple[int, bytes]:
        if len(payload) != FRAME_SIZE or payload[0] != PB_TAG: raise ValueError('invalid 303B protobuf frame')
        index = 1; value = shift = 0
        while True:
            if index >= len(payload) or shift > 28: raise ValueError('invalid protobuf varint')
            byte = payload[index]; index += 1; value |= (byte & 0x7f) << shift
            if byte < 0x80: break
            shift += 7
        if value != DATA_SIZE or len(payload) < index + value: raise ValueError('invalid data size')
        data = payload[index:index + value]
        return struct.unpack_from('<Q', data)[0], data[PACKET_HEADER_SIZE:]

    @staticmethod
    def _parse_deploy_status(payload: bytes) -> bool:
        """解析 2.2.27 DeployModeStatusSync 的 protobuf：optional uint32 status = 1。"""
        if not payload or payload[0] != 0x08:
            raise ValueError('expected protobuf field 1 uint32')
        value, index = MqttVideoSubscriber._parse_varint(payload, 1)
        if index != len(payload):
            raise ValueError('unexpected trailing protobuf data')
        if value not in (0, 1):
            raise ValueError(f'invalid deployment status: {value}')
        return bool(value)

    @staticmethod
    def _parse_varint(payload: bytes, index: int) -> tuple[int, int]:
        value = shift = 0
        while True:
            if index >= len(payload) or shift > 63:
                raise ValueError('invalid protobuf varint')
            byte = payload[index]
            index += 1
            value |= (byte & 0x7F) << shift
            if byte < 0x80:
                return value, index
            shift += 7

    @classmethod
    def _parse_event(cls, payload: bytes) -> tuple[int, str]:
        """解析 Event：field 1 为 int32 event_id，field 2 为 UTF-8 string param。"""
        event_id: Optional[int] = None
        parameter: Optional[str] = None
        index = 0
        while index < len(payload):
            key, index = cls._parse_varint(payload, index)
            field, wire_type = key >> 3, key & 0x07
            if field == 1:
                if wire_type != 0:
                    raise ValueError('Event.event_id has invalid wire type')
                event_id, index = cls._parse_varint(payload, index)
            elif field == 2:
                if wire_type != 2:
                    raise ValueError('Event.param has invalid wire type')
                length, index = cls._parse_varint(payload, index)
                if length > len(payload) - index:
                    raise ValueError('Event.param length exceeds payload')
                try:
                    parameter = payload[index:index + length].decode('utf-8')
                except UnicodeDecodeError as exc:
                    raise ValueError('Event.param is not UTF-8') from exc
                index += length
            else:
                raise ValueError(f'unsupported Event field: {field}')
        if event_id is None or parameter is None:
            raise ValueError('Event requires event_id and param')
        return event_id, parameter

    @staticmethod
    def _put_latest(target: queue.Queue, item) -> bool:
        try: target.put_nowait(item); return False
        except queue.Full:
            try: target.get_nowait()
            except queue.Empty: pass
            try: target.put_nowait(item); return True
            except queue.Full: return True

    def _set_active(self, active: bool, reason: str) -> None:
        with self.signal_lock:
            if self.signal_active == active: return
            if not active and self.signal_seen_once: self.resume_pending = True
            self.signal_active = active; self.signal_seen_once |= active
        self._show_status('WAITING FOR VIDEO' if active else 'WAITING FOR PACKETS')
        print(f'[信号] {"开始收包" if active else "等待收包"}: {reason}')

    def _signal_loop(self) -> None:
        while self.running:
            self._set_active(self.last_packet_ts > 0 and time.time() - self.last_packet_ts <= self.packet_timeout_s, 'packet timeout')
            self._publish_metrics(); time.sleep(self.signal_check_interval_s)

    def _recover(self, reason: str, now: float, sequence_id: int, force: bool = False) -> bool:
        if not force and now - self.last_reset_ts < self.reset_cooldown_ms / 1000:
            self.reset_cooldown_skip_count += 1; return False
        self.assembler.clear(); self.codec = self._new_decoder(); self.waiting_for_idr = True; self.wait_idr_skip_count = 0
        self.decode_error_streak = 0; self.last_reset_ts = now; self.reset_count += 1
        print(f'[恢复] decoder reset: {reason}, seq={sequence_id}')
        return True

    def _wait_for_idr(self, reason: str, sequence_id: int) -> None:
        self.assembler.clear(); self.waiting_for_idr = True; self.wait_idr_skip_count = 0; self.decode_error_streak = 0
        print(f'[恢复] waiting for keyframe: {reason}, seq={sequence_id}')

    def _clear_restart_candidate(self) -> None:
        self.restart_candidate_start = self.restart_candidate_last = None
        self.restart_candidate_count = 0

    def _try_confirm_sequence_restart(self, sequence_id: int, now: float) -> bool:
        """仅接受低序号连续递增的新流，避免把单个旧包误判为重启。"""
        if sequence_id > self.sequence_restart_low_seq_max:
            self._clear_restart_candidate(); return False
        if self.restart_candidate_last is None:
            self.restart_candidate_start = self.restart_candidate_last = sequence_id
            self.restart_candidate_count = 1
            return False
        if sequence_id != self.restart_candidate_last + 1:
            self.restart_candidate_start = self.restart_candidate_last = sequence_id
            self.restart_candidate_count = 1
            return False
        self.restart_candidate_last = sequence_id
        self.restart_candidate_count += 1
        if self.restart_candidate_count < self.sequence_restart_confirm_packets:
            return False

        old_sequence = self.last_seq
        self.last_seq = sequence_id; self.consecutive_gap = 0; self.backward_sequence_active = False
        self._clear_restart_candidate(); self.seq_restart_confirmed_count += 1
        self._recover(f'confirmed publisher restart {old_sequence}->{sequence_id}', now, sequence_id, True)
        print(f'\033[33m[序号] 已切换到确认的新流: {old_sequence}->{sequence_id}\033[0m')
        return True

    def _accept_sequence(self, sequence_id: int, now: float) -> bool:
        if self.last_seq is None:
            self.last_seq = sequence_id; self.backward_sequence_active = False; self._clear_restart_candidate(); return True
        if sequence_id <= self.last_seq:
            self.dup_drop += 1
            if sequence_id == self.last_seq:
                self.seq_repeat_drop += 1; self._clear_restart_candidate()
            else:
                self.seq_old_drop += 1
                rollback = self.last_seq - sequence_id
                if rollback >= self.sequence_restart_suspect_gap and not self.backward_sequence_active:
                    self.backward_sequence_active = True; self.seq_restart_suspect_count += 1
                    print(f'\033[33m[序号 WARN] 疑似发布端重启或第二发布者: last={self.last_seq}, incoming={sequence_id}, rollback={rollback}\033[0m')
                if rollback >= self.sequence_restart_suspect_gap and self._try_confirm_sequence_restart(sequence_id, now):
                    return True
                if rollback < self.sequence_restart_suspect_gap:
                    self._clear_restart_candidate()
            return False
        self.backward_sequence_active = False; self._clear_restart_candidate()
        gap = sequence_id - self.last_seq - 1; self.last_seq = sequence_id
        if not gap: self.consecutive_gap = 0; return True
        self.gap_drop += gap; self.gap_event_count += 1; self.consecutive_gap += gap
        print(f'[丢包] seq gap={gap}, total={self.consecutive_gap}')
        if self.drop_to_idr_on_gap and not self.waiting_for_idr and gap >= self.drop_to_idr_min_gap:
            self._wait_for_idr('sequence gap', sequence_id)
        if self.recover_on_any_gap and not self.waiting_for_idr and (gap >= self.gap_guard_min_gap or self.consecutive_gap >= self.gap_guard_total_gap):
            self._recover('gap guard', now, sequence_id)
        elif gap > self.severe_gap_threshold or self.consecutive_gap > self.severe_gap_threshold:
            self._recover('severe sequence gap', now, sequence_id)
        return True

    def _trim_padding(self, chunk: bytes) -> bytes:
        if not self.trim_h264_padding_zeros: return chunk
        valid = chunk.rstrip(b'\0')
        if len(valid) != len(chunk):
            self.h264_padding_trim_count += 1; self.h264_padding_trim_bytes += len(chunk) - len(valid)
        return valid

    def _decode_access_unit(self, au: bytes, sequence_id: int, now: float) -> bool:
        nals = list(AnnexBAssembler.nals(au)); types = [nal[0] & 0x1f for nal in nals if nal]
        self.assembler.update_parameters(au)
        has_vcl, has_idr = any(AnnexBAssembler.is_vcl(t) for t in types), 5 in types
        has_intra = self.allow_intra_slice_gate and any(AnnexBAssembler.is_intra(nal) for nal in nals)
        has_params = (7 in types and 8 in types) or (self.assembler.sps and self.assembler.pps)
        key = has_idr or has_intra
        if self.waiting_for_idr:
            allowed = ((key and (has_params or not (self.startup_require_params and self.frame_count == 0)))
                       if self.strict_idr_gate else (key or 7 in types or 8 in types))
            fallback = has_vcl and has_params and self.frame_count == 0 and self.wait_idr_skip_count >= self.wait_idr_fallback_au > 0
            if not allowed and not fallback:
                self.wait_idr_skip_count += 1; return False
            self.waiting_for_idr = False; self.wait_idr_skip_count = 0; print('[I帧] decoder unlocked')
        if not has_vcl: return False
        source = self.assembler.add_cached_parameters(au, types) if self.prepend_cached_sps_pps_on_idr and (has_idr or self.frame_count == 0) else au
        try:
            packets = self.codec.parse(source) or [av.Packet(source)]
            self.parsed_packet_count += len(packets); produced = False
            for packet in packets:
                for frame in self.codec.decode(packet): self._handle_frame(frame); produced = True
            if produced: self.decode_error_streak = 0
            return produced
        except Exception as exc:
            self.decode_error_streak += 1
            if self.decode_error_streak >= self.decode_error_burst_limit: self._recover(f'decode errors: {exc}', now, sequence_id)
            return False

    def _handle_frame(self, frame) -> None:
        if frame is None or not self.signal_active: return
        image = frame.to_ndarray(format='bgr24')
        if image is None or not image.size: return
        now = time.time(); self.frame_count += 1; self.last_frame_ts = now; self.frame_times.append(now)
        self.frame_height, self.frame_width = image.shape[:2]
        self._queue_preview(image)
        if self.frame_queue_enable and self._put_latest(self.frame_queue, image.copy()): self.display_drop_count += 1

    @staticmethod
    def _draw_display_grid(image: np.ndarray) -> None:
        """仅在 OpenCV 窗口显示前叠加细绿色 6×6 网格，不写入 JPEG。"""
        height, width = image.shape[:2]
        if width < 2 or height < 2:
            return
        color, divisions = (0, 210, 0), 6
        for index in range(1, divisions):
            x = round(width * index / divisions)
            y = round(height * index / divisions)
            cv2.line(image, (x, 0), (x, height - 1), color, 1, cv2.LINE_AA)
            cv2.line(image, (0, y), (width - 1, y), color, 1, cv2.LINE_AA)

    def _resize(self, image: np.ndarray, width: int) -> np.ndarray:
        if width <= 0 or image.shape[1] <= width: return image
        return cv2.resize(image, (width, max(1, image.shape[0] * width // image.shape[1])), interpolation=cv2.INTER_AREA)

    def _queue_preview(self, image: np.ndarray) -> None:
        """把待编码预览图交给预览线程；队满时直接替换为最新图。"""
        if self.preview_required and self._put_latest(self.preview_queue, image.copy()):
            self.web_skip_count += 1

    def _preview_loop(self) -> None:
        """独立执行缩放和 JPEG 编码，避免拖慢 H.264 解码线程。"""
        next_encode_ts = 0.0
        while self.running:
            try:
                image = self.preview_queue.get(timeout=.2)
            except queue.Empty:
                continue
            if image is None:
                return
            delay = next_encode_ts - time.time()
            if delay > 0:
                time.sleep(delay)
                # 等待限帧期间若来了新图，只编码最新的一张。
                while True:
                    try:
                        latest = self.preview_queue.get_nowait()
                    except queue.Empty:
                        break
                    if latest is None:
                        return
                    image = latest
                    self.web_skip_count += 1
            now = time.time()
            next_encode_ts = now + 1.0 / self.web_preview_fps
            ok, encoded = cv2.imencode(
                '.jpg', self._resize(image, self.web_preview_width),
                [int(cv2.IMWRITE_JPEG_QUALITY), self.web_jpeg_quality],
            )
            if ok:
                self.preview_state.set_jpeg_frame(encoded.tobytes())
                self.last_web_jpeg_bytes = len(encoded)
                self.web_output_frame_count += 1
                self.web_frame_times.append(now)

    def _show_status(self, text: str) -> None:
        image = make_status_frame(self.frame_width or self.width or 1280, self.frame_height or max(360, (self.width or 1280) * 3 // 4), text)
        if self.frame_queue_enable: self._put_latest(self.frame_queue, image)
        if self.preview_required: self._queue_preview(image)

    def on_connect(self, _client, _userdata, _flags, reason_code, _properties) -> None:
        if reason_code.is_failure:
            self.last_error = f'MQTT connect failed: {reason_code}'; return
        self.mqtt_connected = True; self.last_error = ''
        try:
            topics = [(self.topic_video, self.qos_level)]
            if self.topic_deploy_status and self.topic_deploy_status != self.topic_video:
                topics.append((self.topic_deploy_status, self.qos_level))
            if self.topic_event and all(topic != self.topic_event for topic, _ in topics):
                topics.append((self.topic_event, self.qos_level))
            self.client.subscribe(topics)
            self.mqtt_subscribed = True
            print(f'[MQTT] subscribed {self.broker}:{self.mqtt_port} {", ".join(topic for topic, _ in topics)}, qos={self.qos_level}')
        except Exception as exc: self.last_error = str(exc); self.mqtt_subscribed = False

    def on_disconnect(self, _client, _userdata, _flags, reason_code, _properties) -> None:
        self.mqtt_connected = self.mqtt_subscribed = False
        # 主动断开无需等待收包超时，立即切换到无包状态；下一包会触发恢复流程。
        if self.running:
            self._set_active(False, f'mqtt disconnect {reason_code}')
        if self.enter_wait_idr_on_disconnect: self._wait_for_idr(f'mqtt disconnect {reason_code}', -1)

    def publish_control(self, direction: str) -> None:
        """通过视频订阅所使用的同一 MQTT 连接发布控制消息。"""
        if direction not in self.control_payloads or not self.topic_control:
            raise ValueError(f'unsupported control direction: {direction}')
        if not self.mqtt_connected or not self.mqtt_subscribed:
            raise RuntimeError('MQTT video connection is unavailable')
        with self.control_lock:
            info = self.client.publish(
                self.topic_control, self.control_payloads[direction], qos=self.control_qos_level)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                self.control_last_error = f'MQTT publish failed: rc={info.rc}'
                raise RuntimeError(self.control_last_error)
            self.control_publish_count += 1
            self.control_last_direction = direction
            self.control_last_error = ''
        print(f'[MQTT control] {direction} -> {self.topic_control}')

    def control_snapshot(self) -> dict:
        with self.control_lock:
            return {
                'connected': self.mqtt_connected and self.mqtt_subscribed,
                'topic_control': self.topic_control,
                'last_direction': self.control_last_direction,
                'publish_count': self.control_publish_count,
                'last_error': self.control_last_error,
            }

    def on_message(self, _client, _userdata, message) -> None:
        if message.topic == self.topic_deploy_status:
            try:
                status = self._parse_deploy_status(message.payload)
                with self.deployment_lock:
                    self.deployment_status = status
                    self.deployment_last_update_ts = time.time()
                    self.deployment_message_count += 1
                    self.deployment_last_error = ''
            except ValueError as exc:
                with self.deployment_lock:
                    self.deployment_last_error = str(exc)
            return
        if message.topic == self.topic_event:
            try:
                event_id, value = self._parse_event(message.payload)
                if event_id != 5:
                    return
                value = value.strip()
                if not value:
                    raise ValueError('Event.param is empty')
                now = time.time()
                with self.damage_lock:
                    self.damage_value = value
                    self.damage_last_update_ts = now
                    self.damage_message_count += 1
                    self.damage_last_error = ''
                if self.damage_event_callback is not None:
                    self.damage_event_callback(value, now)
            except ValueError as exc:
                with self.damage_lock:
                    self.damage_last_error = str(exc)
            return
        if message.topic != self.topic_video or len(message.payload) != FRAME_SIZE: return
        now = time.time(); self.last_packet_ts = now; self._set_active(True, 'MQTT packet')
        # Paho 暴露的是 MQTT PUBLISH 的 DUP 标志；它与“序号重复”并非同一概念。
        mqtt_protocol_dup = bool(getattr(message, 'dup', False))
        try:
            sequence, chunk = self._parse_message(message.payload)
        except ValueError:
            return

        # 保持原有日志口径：pkt 是所有合法 MQTT 到包，ok 是真正送入解码链路的
        # 唯一序号包。因此即使在入口去掉重复包，pkt/ok 仍会分别显示。
        self.video_pkt_count += 1; self.parsed_packet_count += 1; self.packet_times.append(now)
        if mqtt_protocol_dup: self.mqtt_protocol_dup_count += 1
        if sequence == self.last_ingress_sequence:
            self.dup_drop += 1; self.seq_repeat_drop += 1
            return
        self.last_ingress_sequence = sequence
        if self._put_latest(self.ingress_queue, (sequence, chunk, now)): self.ingress_drop_count += 1

    def _decode_loop(self) -> None:
        while self.running:
            try: item = self.ingress_queue.get(timeout=.2)
            except queue.Empty: continue
            if item is None: return
            sequence, chunk, now = item
            try:
                with self.signal_lock: resumed, self.resume_pending = self.resume_pending, False
                if resumed:
                    self.last_seq = None; self.consecutive_gap = 0; self.backward_sequence_active = False; self._clear_restart_candidate()
                    self._recover('stream resumed', now, sequence, True)
                if not self._accept_sequence(sequence, now): continue
                self.accepted_packet_count += 1; self.accepted_packet_times.append(now)
                self.assembler.append(self._trim_padding(chunk)); self.assembler.resync()
                if len(self.assembler.buffer) > MAX_H264_BUFFER_BYTES:
                    self.buf_overflow_reset += 1; self._recover('h264 buffer overflow', now, sequence, True); continue
                produced = False
                while (au := self.assembler.next_access_unit()) is not None: produced |= self._decode_access_unit(au, sequence, now)
                if (not produced and self.last_frame_ts > 0 and not self.waiting_for_idr
                    and (not self.stall_watchdog_startup_only or self.frame_count == 0)
                    and now - self.last_frame_ts > self.stall_recovery_ms / 1000):
                    self._recover('decode stall', now, sequence)
            except Exception as exc:
                self.last_error = str(exc)
            self._publish_metrics()

    def _publish_metrics(self) -> None:
        now = time.time()
        for values in (self.packet_times, self.accepted_packet_times, self.frame_times, self.web_frame_times):
            while values and now - values[0] > 1: values.popleft()
        metrics = {
            'broker': self.broker, 'mqtt_port': self.mqtt_port, 'client_id': self.client_id,
            'topic': self.topic_video, 'topic_deploy_status': self.topic_deploy_status,
            'topic_event': self.topic_event, 'topic_control': self.topic_control,
            'qos_level': self.qos_level,
            'mqtt_connected': self.mqtt_connected, 'mqtt_subscribed': self.mqtt_subscribed,
            'last_error': self.last_error, 'packet_active': self.signal_active,
            'packet_timeout_s': self.packet_timeout_s, 'video_pkt_count': self.video_pkt_count,
            'parsed_packet_count': self.parsed_packet_count, 'frame_count': self.frame_count,
            'display_frame_count': self.display_frame_count, 'display_drop_count': self.display_drop_count,
            'web_output_frame_count': self.web_output_frame_count, 'web_skip_count': self.web_skip_count,
            'web_preview_fps_limit': self.web_preview_fps, 'web_preview_fps': float(len(self.web_frame_times)),
            'web_jpeg_quality': self.web_jpeg_quality, 'web_preview_width': self.web_preview_width,
            'last_web_jpeg_bytes': self.last_web_jpeg_bytes,
            'ingress_queue_size_limit': self.ingress_queue_size, 'ingress_queue_size': self.ingress_queue.qsize(),
            'ingress_queue_alert_threshold': max(1, round(self.ingress_queue_size * self.queue_alert_ratio)),
            'ingress_drop_count': self.ingress_drop_count, 'reset_count': self.reset_count,
            'reset_cooldown_skip_count': self.reset_cooldown_skip_count,
            'decode_error_streak': self.decode_error_streak, 'severe_gap_threshold': self.severe_gap_threshold,
            'reset_cooldown_ms': self.reset_cooldown_ms, 'decode_error_burst_limit': self.decode_error_burst_limit,
            'stall_recovery_ms': self.stall_recovery_ms, 'gap_drop': self.gap_drop,
            'gap_event_count': self.gap_event_count, 'dup_drop': self.dup_drop,
            'seq_repeat_drop': self.seq_repeat_drop, 'seq_old_drop': self.seq_old_drop,
            'mqtt_protocol_dup_count': self.mqtt_protocol_dup_count,
            'seq_restart_suspect_count': self.seq_restart_suspect_count,
            'seq_restart_confirmed_count': self.seq_restart_confirmed_count,
            'buf_overflow_reset': self.buf_overflow_reset,
            'h264_padding_trim_count': self.h264_padding_trim_count,
            'h264_padding_trim_bytes': self.h264_padding_trim_bytes,
            'buffer_bytes': len(self.assembler.buffer), 'packet_fps': float(len(self.packet_times)),
            'accepted_packet_count': self.accepted_packet_count,
            'accepted_packet_fps': float(len(self.accepted_packet_times)),
            'decode_fps': float(len(self.frame_times)),
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'last_frame_age_ms': (now - self.last_frame_ts) * 1000 if self.last_frame_ts else -1,
        }
        with self.deployment_lock:
            metrics.update({
                'deployment_status': self.deployment_status,
                'deployment_last_update_age_ms': (
                    (now - self.deployment_last_update_ts) * 1000
                    if self.deployment_last_update_ts else -1
                ),
                'deployment_message_count': self.deployment_message_count,
                'deployment_last_error': self.deployment_last_error,
            })
        with self.damage_lock:
            metrics.update({
                'damage_value': self.damage_value,
                'damage_last_update_age_ms': (
                    (now - self.damage_last_update_ts) * 1000 if self.damage_last_update_ts else -1
                ),
                'damage_message_count': self.damage_message_count,
                'damage_last_error': self.damage_last_error,
            })
        self.preview_state.update_metrics(metrics)
        self._log_metrics(metrics, now)

    def _log_metrics(self, metrics: dict, now: float) -> None:
        """每秒输出一行接收端摘要；异常或丢弃时以黄色突出显示。"""
        with self.log_lock:
            if now - self.last_log_ts < self.log_metrics_interval_s:
                return
            self.last_log_ts = now
        warning = any((metrics['ingress_queue_size'] >= metrics['ingress_queue_alert_threshold'],
                       metrics['ingress_drop_count'], metrics['gap_drop'], metrics['dup_drop'],
                       metrics['decode_error_streak'], metrics['buf_overflow_reset'], metrics['last_error']))
        prefix = '[MQTT rx WARN]' if warning else '[MQTT rx]'
        text = (
            f"{prefix} state={'ON' if metrics['packet_active'] else 'WAIT'} "
            f"mqtt={'OK' if metrics['mqtt_connected'] and metrics['mqtt_subscribed'] else 'DOWN'} "
            f"pkt={metrics['packet_fps']:.1f}/s ok={metrics['accepted_packet_fps']:.1f}/s dec={metrics['decode_fps']:.1f}/s "
            f"total={metrics['video_pkt_count']} frame={metrics['frame_count']} "
            f"queue={metrics['ingress_queue_size']}/{metrics['ingress_queue_size_limit']} "
            f"drop(in/gap/dup)={metrics['ingress_drop_count']}/{metrics['gap_drop']}/{metrics['dup_drop']} "
            f"dup(rep/old/mqtt)={metrics['seq_repeat_drop']}/{metrics['seq_old_drop']}/{metrics['mqtt_protocol_dup_count']} "
            f"restart(sus/ok)={metrics['seq_restart_suspect_count']}/{metrics['seq_restart_confirmed_count']} "
            f"reset={metrics['reset_count']} buf={metrics['buffer_bytes']}B "
            f"age={metrics['last_frame_age_ms']:.0f}ms"
        )
        if metrics['last_error']:
            text += f" err={metrics['last_error'][:120]}"
        print(f'\033[33m{text}\033[0m' if warning else text)

    def _start_web_server(self) -> None:
        if not self.web_preview_enable: return
        try:
            self.preview_server = ThreadingHTTPServer((self.web_preview_host, self.web_preview_port), make_preview_handler(self.preview_state, self.web_preview_fps))
            threading.Thread(target=self.preview_server.serve_forever, daemon=True).start()
            url = f'http://127.0.0.1:{self.web_preview_port}/'; print(f'[Web预览] {url}')
            if self.auto_open_browser:
                timer = threading.Timer(self.auto_open_delay_s, lambda: webbrowser.open(url, new=2)); timer.daemon = True; timer.start()
        except Exception as exc: self.last_error = f'web preview: {exc}'

    def _display_loop(self) -> None:
        cv2.namedWindow('Mode4 Video', cv2.WINDOW_NORMAL)
        if self.width: cv2.resizeWindow('Mode4 Video', self.width, max(1, self.width * 3 // 4))
        # 显示时钟与解码/到包时刻解耦。只取队列中最新帧；若这一周期无新帧，
        # 则重复上一帧。这样不会积压，也不会把成帧突发直接表现为画面跳动。
        interval_s = 1.0 / self.fps
        next_present_ts = time.monotonic()
        current_image = None
        while self.running:
            now = time.monotonic()
            timeout_s = min(.01, max(0., next_present_ts - now))
            got_image = False
            try:
                image = self.frame_queue.get(timeout=timeout_s)
                got_image = True
            except queue.Empty:
                image = None
            if got_image:
                if image is None: break
                current_image = image
                # 队列容量为 1，但仍排空以兼容未来调整容量后的“只显示最新帧”语义。
                while True:
                    try: current_image = self.frame_queue.get_nowait()
                    except queue.Empty: break
                    if current_image is None: break
                if current_image is None: break

            now = time.monotonic()
            if current_image is not None and now >= next_present_ts:
                display = self._resize(current_image, self.width)
                self._draw_display_grid(display)
                cv2.imshow('Mode4 Video', display)
                self.display_frame_count += 1
                next_present_ts += interval_s
                if next_present_ts <= now:
                    next_present_ts = now + interval_s
            if cv2.waitKey(1) & 0xff == ord('q'): break
        self.stop()

    def start_service(self) -> None:
        if self.service_started: return
        if not self.running: raise RuntimeError('MQTT receiver has already been stopped')
        self.client.connect_async(self.broker, self.mqtt_port, keepalive=60); self.client.loop_start(); self.service_started = True; self._publish_metrics()

    def run(self) -> None:
        self.start_service()
        if self.cv_display_enable: self._display_loop()
        else:
            while self.running: time.sleep(.2)

    def stop(self) -> None:
        if not self.running: return
        self.running = False; self.mqtt_connected = self.mqtt_subscribed = False
        self._put_latest(self.ingress_queue, None); self._put_latest(self.frame_queue, None); self._put_latest(self.preview_queue, None)
        for thread in (self.decoder_thread, self.signal_thread, self.preview_thread): thread.join(timeout=1)
        if self.service_started:
            try: self.client.disconnect(); self.client.loop_stop()
            except Exception: pass
            self.service_started = False
        if self.preview_server:
            self.preview_server.shutdown(); self.preview_server.server_close(); self.preview_server = None
        self._publish_metrics(); print(f'[Subscriber] stopped packets={self.video_pkt_count} frames={self.frame_count} gaps={self.gap_drop}')


def make_receiver(cfg: dict) -> MqttVideoSubscriber:
    """根据 config.py 创建独立运行使用的接收器。"""
    return MqttVideoSubscriber(
        broker=cfg['broker'], mqtt_port=cfg['mqtt_port'], width=cfg['display_width'],
        fps=cfg['display_fps_hint'], cv_display_enable=cfg['cv_display_enable'],
        client_id=cfg['client_id'], topic_video=cfg['topic_video'], qos_level=cfg['qos_level'],
        topic_deploy_status=cfg.get('topic_deploy_status', ''),
        topic_event=cfg.get('topic_event', ''),
        topic_control=cfg.get('topic_control', ''),
        control_qos_level=cfg['qos_level'],
        control_minus_value=cfg.get('control_minus_value', 0),
        control_plus_value=cfg.get('control_plus_value', 1),
        web_preview_enable=cfg['web_preview_enable'], web_preview_port=cfg['web_preview_port'],
        web_preview_fps=cfg['web_preview_fps'], web_jpeg_quality=cfg['web_jpeg_quality'],
        web_preview_width=cfg['web_preview_width'],
        trim_h264_padding_zeros=cfg.get('trim_h264_padding_zeros', True),
        drop_to_idr_on_gap=cfg.get('drop_to_idr_on_gap', False),
        drop_to_idr_min_gap=cfg.get('drop_to_idr_min_gap', 5),
        auto_open_browser=cfg['auto_open_browser'], ingress_queue_size=cfg['ingress_queue_size'],
    )


def main() -> None:
    """独立启动入口；保持旧版命令行参数兼容。"""
    cfg = dict(CONFIG)
    if CONFIG.get('enable_cli_override', False):
        parser = argparse.ArgumentParser(description='Mode4 MQTT H.264 接收端')
        parser.add_argument('--broker', default=cfg['broker'])
        parser.add_argument('--mqtt-port', type=int, default=cfg['mqtt_port'])
        parser.add_argument('--width', type=int, default=cfg['display_width'])
        parser.add_argument('--fps', type=int, default=cfg['display_fps_hint'])
        parser.add_argument('--display', action=argparse.BooleanOptionalAction,
                            default=cfg['cv_display_enable'])
        parser.add_argument('--client-id', default=cfg['client_id'])
        parser.add_argument('--topic-video', default=cfg['topic_video'])
        parser.add_argument('--qos', type=int, choices=(0, 1), default=cfg['qos_level'])
        args = parser.parse_args()
        cfg.update(
            broker=args.broker, mqtt_port=args.mqtt_port, display_width=args.width,
            display_fps_hint=args.fps, cv_display_enable=args.display,
            client_id=args.client_id, topic_video=args.topic_video, qos_level=args.qos,
        )
    receiver = make_receiver(cfg)
    try:
        receiver.run()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()


if __name__ == '__main__':
    main()
