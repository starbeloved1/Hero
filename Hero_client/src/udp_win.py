#!/usr/bin/env python3
"""
Mode4 UDP HEVC 图传接收端

规则链路：
1) 自定义客户端监听 UDP 3334 端口
2) 每个 UDP 包前 8 字节为固定头：
   - frame_id:      2 bytes
   - fragment_id:   2 bytes
   - frame_size:    4 bytes，当前帧 HEVC 码流总字节数
3) 8 字节之后为当前帧分片数据

为了比赛实时率，接收线程只做重组和入队，解码线程只保留最新帧。
"""

import argparse
import json
import queue
import socket
import struct
import sys
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    from config import UDP_CONFIG as CONFIG
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from config import UDP_CONFIG as CONFIG

try:
    import av
    import cv2
    import numpy as np
except ImportError:
    print("需要安装 av、opencv 和 numpy: pip install av opencv-python numpy")
    sys.exit(1)


def make_status_frame(width: int, height: int, text: str):
    """生成 OpenCV/Web 共用的黑底白字英文状态画面。"""
    width = max(640, int(width))
    height = max(360, int(height))
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    scale = max(0.8, min(width, height) / 700.0)
    thickness = max(1, int(round(scale * 2)))
    (text_width, text_height), _ = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        thickness,
    )
    cv2.putText(
        frame,
        text,
        (max(0, (width - text_width) // 2), max(text_height, (height + text_height) // 2)),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )
    return frame


class PreviewState:
    def __init__(self):
        self.condition = threading.Condition()
        self.metrics_lock = threading.Lock()
        self.metrics = {}
        self.latest_jpeg = None

    def update_metrics(self, metrics):
        with self.metrics_lock:
            self.metrics = dict(metrics)

    def snapshot_metrics(self):
        with self.metrics_lock:
            return dict(self.metrics)

    def set_jpeg_frame(self, jpeg_bytes: bytes):
        with self.condition:
            self.latest_jpeg = bytes(jpeg_bytes)
            self.condition.notify_all()


def make_preview_handler(state: PreviewState, preview_fps: int = 20):
    jpeg_poll_ms = max(15, int(round(1000.0 / max(1, int(preview_fps)))))

    class PreviewHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                html = (
                    '<!doctype html><html><head><meta charset="utf-8">'
                    '<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">'
                    "<title>HUSTLY-HERO UDP</title>"
                    "<style>"
                    ":root{color-scheme:dark;--bg:#080a0c;--panel:rgba(10,14,18,.78);--line:rgba(255,255,255,.14);"
                    "--text:#eef3f5;--muted:#95a0a6;--ok:#2fd17c;--warn:#f0b84f;--bad:#ff5b5f;--cyan:#56c7ff;}"
                    "*{box-sizing:border-box}html,body{width:100%;height:100%;margin:0;overflow:hidden;background:var(--bg);"
                    'font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color:var(--text);}'
                    ".app{position:relative;width:100vw;height:100vh;background:#050607;}"
                    ".video{position:absolute;inset:0;width:100%;height:100%;object-fit:contain;background:#050607;}"
                    ".shade{position:absolute;inset:0;pointer-events:none;background:linear-gradient(180deg,rgba(0,0,0,.56),transparent 24%,transparent 68%,rgba(0,0,0,.58));}"
                    ".top{position:absolute;left:0;right:0;top:0;display:grid;grid-template-columns:minmax(180px,1fr) auto minmax(180px,1fr);"
                    "gap:12px;align-items:start;padding:12px max(12px,env(safe-area-inset-right)) 0 max(12px,env(safe-area-inset-left));}"
                    ".brand{display:flex;align-items:center;gap:10px;min-width:0}.mark{width:12px;height:12px;border-radius:50%;background:var(--bad);box-shadow:0 0 18px var(--bad);}"
                    ".mark.ok{background:var(--ok);box-shadow:0 0 18px var(--ok)}.mark.warn{background:var(--warn);box-shadow:0 0 18px var(--warn)}"
                    ".title{font-size:17px;font-weight:750;letter-spacing:0}.sub{font-size:12px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;margin-top:1px;}"
                    ".pillrow{display:flex;gap:8px;justify-content:center;flex-wrap:wrap}.pill{height:34px;min-width:74px;padding:0 11px;border:1px solid var(--line);"
                    "background:var(--panel);backdrop-filter:blur(10px);display:flex;flex-direction:column;justify-content:center;align-items:center;}"
                    ".pill b{font-size:15px;line-height:16px}.pill span{font-size:10px;color:var(--muted);line-height:12px;text-transform:uppercase;}"
                    ".net{justify-self:end;text-align:right;min-width:0}.net .main{font-size:13px;font-weight:650}.net .minor{font-size:11px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:42vw;}"
                    ".side{position:absolute;right:max(12px,env(safe-area-inset-right));top:74px;width:214px;border:1px solid var(--line);"
                    "background:var(--panel);backdrop-filter:blur(10px);padding:10px 12px;display:grid;gap:9px;}"
                    ".row{display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center;font-size:12px}.row .k{color:var(--muted)}.row .v{font-variant-numeric:tabular-nums;font-weight:650;}"
                    ".bar{height:5px;background:rgba(255,255,255,.12);overflow:hidden}.bar i{display:block;height:100%;width:0;background:var(--cyan);transition:width .18s ease;}"
                    ".bottom{position:absolute;left:max(12px,env(safe-area-inset-left));right:max(12px,env(safe-area-inset-right));bottom:max(10px,env(safe-area-inset-bottom));"
                    "display:grid;grid-template-columns:1fr auto;gap:12px;align-items:end}.strip{border:1px solid var(--line);background:var(--panel);backdrop-filter:blur(10px);"
                    "height:42px;display:flex;align-items:center;gap:18px;padding:0 12px;min-width:0}.metric{display:flex;gap:7px;align-items:baseline;min-width:0}.metric .k{font-size:11px;color:var(--muted);white-space:nowrap}.metric .v{font-size:14px;font-weight:700;font-variant-numeric:tabular-nums;}"
                    ".clock{height:42px;min-width:82px;border:1px solid var(--line);background:var(--panel);backdrop-filter:blur(10px);display:flex;align-items:center;justify-content:center;font-size:16px;font-weight:750;font-variant-numeric:tabular-nums;}"
                    ".lost{position:absolute;inset:0;display:none;align-items:center;justify-content:center;background:#000;font-size:32px;font-weight:800;letter-spacing:0;}"
                    ".lost.show{display:flex}.bad{color:var(--bad)}.warn{color:var(--warn)}.ok{color:var(--ok)}"
                    ".pillrow,.net,.side,.bottom,.shade{display:none}"
                    "@media(max-width:760px){.top{grid-template-columns:1fr auto;padding-top:9px}.pillrow{grid-column:1/3;order:3;justify-content:flex-start}.net{display:none}.side{display:none}"
                    ".bottom{grid-template-columns:1fr}.clock{display:none}.strip{height:auto;min-height:44px;flex-wrap:wrap;gap:10px 14px;padding:8px 10px}.metric .v{font-size:13px}.title{font-size:15px}}"
                    "</style></head><body><main class=\"app\">"
                    '<img id="videoImage" class="video" alt="preview"/>'
                    '<div class="shade"></div>'
                    '<section class="top">'
                    '<div class="brand"><i id="mark" class="mark"></i><div><div class="title">HUSTLY-HERO UDP</div><div id="stateText" class="sub">WAITING FOR VIDEO</div></div></div>'
                    '<div class="pillrow">'
                    '<div class="pill"><b id="fps">0.0</b><span>预览帧率</span></div>'
                    '<div class="pill"><b id="pps">0.0</b><span>包率</span></div>'
                    '<div class="pill"><b id="age">--</b><span>延迟</span></div>'
                    "</div>"
                    '<div class="net"><div id="netMain" class="main">UDP</div><div id="netSub" class="minor">--</div></div>'
                    "</section>"
                    '<aside class="side">'
                    '<div class="row"><span class="k">解码帧</span><span id="decoded" class="v">0</span></div>'
                    '<div class="row"><span class="k">接收包</span><span id="packets" class="v">0</span></div>'
                    '<div class="row"><span class="k">完整帧</span><span id="complete" class="v">0</span></div>'
                    '<div class="row"><span class="k">缓冲</span><span id="buffer" class="v">0 B</span></div>'
                    '<div class="bar"><i id="bufferBar"></i></div>'
                    '<div class="row"><span class="k">显示丢帧</span><span id="drop" class="v">0</span></div>'
                    '<div class="row"><span class="k">Rx-&gt;Display</span><span id="rxlat" class="v">--</span></div>'
                    '<div class="row"><span class="k">预览跳帧</span><span id="webskip" class="v">0</span></div>'
                    '<div class="row"><span class="k">重置</span><span id="reset" class="v">0</span></div>'
                    '<div class="row"><span class="k">分辨率</span><span id="size" class="v">--</span></div>'
                    '<div class="row"><span class="k">单帧大小</span><span id="jpgsize" class="v">--</span></div>'
                    "</aside>"
                    '<section class="bottom"><div class="strip">'
                    '<div class="metric"><span class="k">UDP</span><span id="udp" class="v">--</span></div>'
                    '<div class="metric"><span class="k">端口</span><span id="port" class="v">--</span></div>'
                    '<div class="metric"><span class="k">限帧</span><span id="limit" class="v">--</span></div>'
                    '<div class="metric"><span class="k">质量</span><span id="quality" class="v">--</span></div>'
                    '<div class="metric"><span class="k">重复</span><span id="dup" class="v">0</span></div>'
                    '<div class="metric"><span class="k">坏包</span><span id="bad" class="v">0</span></div>'
                    "</div>"
                    '<div id="clock" class="clock">--:--</div></section>'
                    '<div id="lost" class="lost">WAITING FOR PACKETS</div>'
                    "</main><script>"
                    'const imgEl=document.getElementById("videoImage");'
                    'function tickPreview(){imgEl.src="/preview.jpg?t="+Date.now();}'
                    "setInterval(tickPreview," + str(jpeg_poll_ms) + ");tickPreview();"
                    'const $=id=>document.getElementById(id);'
                    "const fmt=n=>Number(n||0).toLocaleString();"
                    "function cls(el,c){el.className=c;}"
                    'async function tick(){try{const r=await fetch("/api/status",{cache:"no-store"});const s=await r.json();'
                    'const age=s.last_frame_age_ms;const receiving=!!s.packet_active;let health=receiving&&age>=0&&age<500?"ok":receiving&&age>=0&&age<1500?"warn":"bad";'
                    'const waitText=receiving?"WAITING FOR VIDEO":"WAITING FOR PACKETS";'
                    'cls($("mark"),"mark "+health);$("stateText").textContent=health==="ok"?"VIDEO ACTIVE":health==="warn"?"VIDEO DELAYED":waitText;'
                    '$("lost").textContent=waitText;$("lost").className="lost "+(health==="bad"?"show":"");$("fps").textContent=(s.web_preview_fps||0).toFixed(1);'
                    '$("pps").textContent=(s.packet_fps||0).toFixed(1);$("age").textContent=age>=0?Math.round(age)+"ms":"--";'
                    '$("decoded").textContent=fmt(s.frame_count);$("packets").textContent=fmt(s.video_pkt_count);$("complete").textContent=fmt(s.complete_frame_count);'
                    '$("buffer").textContent=fmt(s.pending_bytes)+" B";$("bufferBar").style.width=Math.min(100,(s.pending_bytes||0)/20000*100)+"%";'
                    '$("drop").textContent=fmt(s.display_drop_count);$("webskip").textContent=fmt(s.web_skip_count);$("reset").textContent=fmt(s.decoder_reset_count);$("dup").textContent=fmt(s.dup_drop);$("bad").textContent=fmt(s.bad_packet_count);'
                    '$("rxlat").textContent=s.udp_to_display_latency_ms>=0?Math.round(s.udp_to_display_latency_ms)+"ms":"--";'
                    '$("size").textContent=s.frame_width&&s.frame_height?s.frame_width+"x"+s.frame_height:"--";'
                    '$("jpgsize").textContent=s.last_web_jpeg_bytes?Math.round(s.last_web_jpeg_bytes/1024)+" KB":"--";'
                    '$("udp").textContent=s.udp_active?"接收中":"等待";$("udp").className="v "+(s.udp_active?"ok":"bad");'
                    '$("port").textContent=s.udp_port||"--";$("limit").textContent=(s.web_preview_fps_limit||"--")+"fps";$("quality").textContent=s.web_jpeg_quality??"--";'
                    '$("netMain").textContent=(s.udp_host||"--")+":"+(s.udp_port||"--");$("netSub").textContent="HEVC / JPEG Web";'
                    '}catch(e){cls($("mark"),"mark bad");$("stateText").textContent="WAITING FOR VIDEO";$("lost").textContent="WAITING FOR PACKETS";$("lost").className="lost show";}}'
                    'setInterval(tick,500);tick();setInterval(()=>{$("clock").textContent=new Date().toTimeString().slice(0,5)},1000);'
                    "</script></body></html>"
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return

            if path == "/api/status":
                body = json.dumps(state.snapshot_metrics(), ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == "/preview.jpg":
                with state.condition:
                    jpg = state.latest_jpeg
                if not jpg:
                    self.send_response(503)
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                try:
                    self.wfile.write(jpg)
                except (BrokenPipeError, ConnectionResetError):
                    return
                return

            self.send_error(404)

    return PreviewHandler


@dataclass
class FrameAssembly:
    frame_id: int
    total_size: int
    created_ts: float
    updated_ts: float
    fragments: dict[int, bytes] = field(default_factory=dict)
    received_bytes: int = 0

    def add_fragment(self, fragment_id: int, payload: bytes, now: float) -> bool:
        if fragment_id in self.fragments:
            return False
        self.fragments[fragment_id] = bytes(payload)
        self.received_bytes += len(payload)
        self.updated_ts = now
        return True


class UdpHevcReceiver:
    def __init__(
        self,
        host: str,
        port: int,
        width: int,
        fps: int,
        header_endian: str,
        web_preview_enable: bool,
        web_preview_host: str,
        web_preview_port: int,
        web_preview_fps: int,
        web_preview_width: int,
        web_jpeg_quality: int,
        auto_open_browser: bool,
        auto_open_delay_s: float,
        recv_buffer_bytes: int,
        max_udp_packet_bytes: int,
        max_frame_bytes: int,
        max_pending_frames: int,
        frame_timeout_ms: int,
        wait_keyframe_after_loss: bool,
        encoded_queue_size: int,
        decoded_queue_size: int,
        log_interval_s: float,
        packet_timeout_s: float,
        signal_check_interval_s: float,
        cv_display_enable: bool = True,
        web_server_enable: bool = True,
        frame_queue_enable: bool = True,
    ):
        self.host = host
        self.port = int(port)
        self.width = int(width)
        self.fps = max(1, int(fps))
        self.cv_display_enable = bool(cv_display_enable)
        self.header = struct.Struct((">" if header_endian == "big" else "<") + "HHI")
        self.web_preview_enable = bool(web_preview_enable)
        self.web_preview_host = web_preview_host
        self.web_preview_port = int(web_preview_port)
        self.web_preview_fps = max(1, int(web_preview_fps))
        self.web_preview_width = max(0, int(web_preview_width))
        self.web_jpeg_quality = min(95, max(20, int(web_jpeg_quality)))
        self.auto_open_browser = bool(auto_open_browser)
        self.auto_open_delay_s = max(0.0, float(auto_open_delay_s))
        self.recv_buffer_bytes = max(256 * 1024, int(recv_buffer_bytes))
        self.max_udp_packet_bytes = max(1200, int(max_udp_packet_bytes))
        self.max_frame_bytes = max(64 * 1024, int(max_frame_bytes))
        self.max_pending_frames = max(1, int(max_pending_frames))
        self.frame_timeout_s = max(0.02, int(frame_timeout_ms) / 1000.0)
        self.wait_keyframe_after_loss = bool(wait_keyframe_after_loss)
        self.log_interval_s = max(0.2, float(log_interval_s))
        self.packet_timeout_s = max(0.3, float(packet_timeout_s))
        self.signal_check_interval_s = max(0.05, float(signal_check_interval_s))
        self.web_server_enable = bool(web_server_enable)
        self.frame_queue_enable = bool(frame_queue_enable)

        self.running = True
        self.service_started = False
        self.last_error = ""
        self.sock = None
        self.pending: dict[int, FrameAssembly] = {}
        self.pending_lock = threading.Lock()
        self.encoded_queue = queue.Queue(maxsize=max(1, int(encoded_queue_size)))
        self.frame_queue = queue.Queue(maxsize=max(1, int(decoded_queue_size)))
        self.raw_frame_lock = threading.Lock()
        self.latest_raw_frame = None
        self.latest_raw_frame_generation = 0
        self.latest_raw_frame_ts = 0.0
        self.last_web_encoded_generation = -1

        self.codec = self._create_decoder("startup")
        self.last_log_ts = 0.0
        self.last_packet_ts = 0.0
        self.last_frame_ts = 0.0
        self.last_web_preview_ts = 0.0
        self.last_udp_to_display_latency_ms = -1.0
        self.packet_times = deque()
        self.complete_times = deque()
        self.decode_times = deque()
        self.web_frame_times = deque()

        self.udp_packet_count = 0
        self.payload_bytes = 0
        self.bad_packet_count = 0
        self.duplicate_fragment_count = 0
        self.completed_frame_count = 0
        self.decoded_frame_count = 0
        self.display_frame_count = 0
        self.display_drop_count = 0
        self.web_output_frame_count = 0
        self.web_skip_count = 0
        self.last_web_jpeg_bytes = 0
        self.incomplete_drop_count = 0
        self.queue_drop_count = 0
        self.decode_error_count = 0
        self.decoder_reset_count = 0
        self.loss_recovery_count = 0
        self.wait_keyframe_skip_count = 0
        self.keyframe_count = 0
        self.frame_gap_count = 0
        self.frame_width = 0
        self.frame_height = 0
        self.last_completed_frame_id = None
        self.waiting_for_keyframe = True
        self.decoder_needs_reset = False
        self.recovery_lock = threading.Lock()
        self.signal_active = False
        self.signal_seen_once = False
        self.signal_lock = threading.Lock()
        self.hevc_vps = None
        self.hevc_sps = None
        self.hevc_pps = None

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.decode_thread = threading.Thread(target=self._decode_loop, daemon=True)
        self.jpeg_thread = threading.Thread(target=self._jpeg_loop, daemon=True)
        self.preview_state = PreviewState()
        self.preview_server = None
        self._show_status_frame("WAITING FOR PACKETS")
        self.signal_thread = threading.Thread(target=self._signal_watchdog_loop, daemon=True)

    def _create_decoder(self, reason: str):
        codec = av.CodecContext.create("hevc", "r")
        # FRAME 线程通常能吃满 CPU，LOW_DELAY 降低内部缓存；这和 sub_win 的 PyAV 路线一致。
        codec.thread_type = "FRAME"
        try:
            codec.flags |= av.codec.context.Flags.LOW_DELAY
        except Exception:
            pass
        print(f"[解码] HEVC 解码器初始化 ({reason})")
        return codec

    def _reset_decoder(self, reason: str):
        self.decoder_reset_count += 1
        self.codec = self._create_decoder(reason)

    def _enter_keyframe_recovery(self, reason: str):
        if not self.wait_keyframe_after_loss:
            return
        with self.recovery_lock:
            was_waiting = self.waiting_for_keyframe
            self.waiting_for_keyframe = True
            self.decoder_needs_reset = True
            self.loss_recovery_count += 1
            if not was_waiting:
                self.wait_keyframe_skip_count = 0
                print(f"[恢复] {reason}，等待下一个HEVC关键帧")

    def _consume_decoder_reset_flag(self) -> bool:
        with self.recovery_lock:
            if not self.decoder_needs_reset:
                return False
            self.decoder_needs_reset = False
            return True

    def _show_status_frame(self, text: str):
        width = self.frame_width if self.frame_width > 0 else (self.width if self.width > 0 else 1280)
        height = self.frame_height if self.frame_height > 0 else max(360, int(width * 3 / 4))
        img = make_status_frame(width, height, text)
        self._set_latest_raw_frame(img, -1, time.time())
        if self.frame_queue_enable:
            while True:
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    break
            try:
                self.frame_queue.put_nowait((img, -1, 0.0))
            except queue.Full:
                pass

    def _set_signal_active(self, active: bool, reason: str):
        active = bool(active)
        with self.signal_lock:
            if self.signal_active == active:
                return False
            self.signal_active = active
            seen_before = self.signal_seen_once
            if active:
                self.signal_seen_once = True

        if active:
            print(f"[信号] 开始收包: {reason}")
            self._show_status_frame("WAITING FOR VIDEO")
        else:
            print(f"[信号] 超过 {self.packet_timeout_s:.1f}s 未收包，切换等待状态")
            if seen_before:
                with self.pending_lock:
                    self.pending.clear()
                while True:
                    try:
                        self.encoded_queue.get_nowait()
                    except queue.Empty:
                        break
                self.last_completed_frame_id = None
                self._enter_keyframe_recovery("UDP收包超时后等待恢复")
            self._show_status_frame("WAITING FOR PACKETS")
        self._publish_metrics()
        return True

    def _signal_watchdog_loop(self):
        while self.running:
            now = time.time()
            active = self.last_packet_ts > 0 and (now - self.last_packet_ts) <= self.packet_timeout_s
            if not active:
                self._set_signal_active(False, "packet timeout")
            self._publish_metrics()
            time.sleep(self.signal_check_interval_s)

    def _keyframe_gate_state(self, hevc_bytes: bytes):
        nal_types = self._hevc_nal_types(hevc_bytes)
        has_vcl = any(0 <= nal_type <= 31 for nal_type in nal_types)
        has_keyframe = any(nal_type in (19, 20, 21) for nal_type in nal_types)
        has_params = all(nal_type in nal_types for nal_type in (32, 33, 34))
        return nal_types, has_vcl, has_keyframe, has_params

    def _update_hevc_params(self, hevc_bytes: bytes):
        for nal in self._iter_annexb_nals(hevc_bytes):
            if len(nal) < 2:
                continue
            nal_type = (nal[0] >> 1) & 0x3F
            if nal_type == 32:
                self.hevc_vps = bytes(nal)
            elif nal_type == 33:
                self.hevc_sps = bytes(nal)
            elif nal_type == 34:
                self.hevc_pps = bytes(nal)

    def _prepend_cached_hevc_params(self, hevc_bytes: bytes, nal_types):
        if 32 in nal_types or 33 in nal_types or 34 in nal_types:
            return hevc_bytes
        if not (self.hevc_vps and self.hevc_sps and self.hevc_pps):
            return hevc_bytes
        prefix = bytearray()
        for nal in (self.hevc_vps, self.hevc_sps, self.hevc_pps):
            prefix.extend(b"\x00\x00\x00\x01")
            prefix.extend(nal)
        return bytes(prefix) + hevc_bytes

    def _hevc_nal_types(self, data: bytes):
        return [(nal[0] >> 1) & 0x3F for nal in self._iter_annexb_nals(data) if len(nal) >= 2]

    def _iter_annexb_nals(self, data: bytes):
        positions = self._find_start_codes(data)
        for idx, (start, prefix_len) in enumerate(positions):
            payload_start = start + prefix_len
            payload_end = positions[idx + 1][0] if idx + 1 < len(positions) else len(data)
            if payload_end > payload_start:
                yield data[payload_start:payload_end]

    def _find_start_codes(self, data: bytes):
        positions = []
        i = 0
        limit = len(data) - 2
        while i < limit:
            if i + 3 < len(data) and data[i:i + 4] == b"\x00\x00\x00\x01":
                positions.append((i, 4))
                i += 4
            elif data[i:i + 3] == b"\x00\x00\x01":
                positions.append((i, 3))
                i += 3
            else:
                i += 1
        return positions

    def _frame_gap(self, previous_id: int, current_id: int) -> int:
        expected = (previous_id + 1) & 0xFFFF
        if current_id == expected:
            return 0
        return (current_id - expected) & 0xFFFF

    def _start_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.recv_buffer_bytes)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(0.2)
        print(f"[UDP] 正在监听 {self.host}:{self.port}")

    def _recv_loop(self):
        buf = bytearray(self.max_udp_packet_bytes)
        view = memoryview(buf)
        while self.running:
            try:
                nbytes, _addr = self.sock.recvfrom_into(view)
            except socket.timeout:
                self._drop_stale_frames(time.time())
                continue
            except OSError:
                break

            now = time.time()
            if nbytes <= self.header.size:
                self.bad_packet_count += 1
                continue

            try:
                frame_id, fragment_id, total_size = self.header.unpack_from(view, 0)
            except struct.error:
                self.bad_packet_count += 1
                continue

            payload_len = nbytes - self.header.size
            if total_size <= 0 or total_size > self.max_frame_bytes or payload_len <= 0:
                self.bad_packet_count += 1
                if self.bad_packet_count <= 8:
                    print(
                        f"[UDP] 异常包 frame={frame_id} frag={fragment_id} "
                        f"total={total_size} payload={payload_len}"
                    )
                continue

            payload = bytes(view[self.header.size:nbytes])
            self.udp_packet_count += 1
            self.payload_bytes += payload_len
            self.last_packet_ts = now
            self.packet_times.append(now)
            self._set_signal_active(True, "UDP video packet")
            self._handle_fragment(frame_id, fragment_id, total_size, payload, now)
            self._log_stats(now)

    def _handle_fragment(self, frame_id: int, fragment_id: int, total_size: int, payload: bytes, now: float):
        completed = None
        with self.pending_lock:
            assembly = self.pending.get(frame_id)
            if assembly is None or assembly.total_size != total_size:
                assembly = FrameAssembly(frame_id, total_size, now, now)
                self.pending[frame_id] = assembly

            if not assembly.add_fragment(fragment_id, payload, now):
                self.duplicate_fragment_count += 1
                return

            completed = self._try_build_frame(assembly)
            if completed is not None:
                first_packet_ts = assembly.created_ts
                self.pending.pop(frame_id, None)
            else:
                first_packet_ts = now

            self._trim_pending_locked(now)

        if completed is not None:
            if self.last_completed_frame_id is not None:
                gap = self._frame_gap(self.last_completed_frame_id, frame_id)
                if gap:
                    self.frame_gap_count += gap
                    self._enter_keyframe_recovery(
                        f"检测到UDP帧号跳变 prev={self.last_completed_frame_id} "
                        f"current={frame_id} gap={gap}"
                    )
            self.last_completed_frame_id = frame_id
            self.completed_frame_count += 1
            self.complete_times.append(now)
            self._push_encoded(frame_id, completed, first_packet_ts)

    def _try_build_frame(self, assembly: FrameAssembly):
        if assembly.received_bytes < assembly.total_size:
            return None

        # 规则没有给出分片总数，只能按分片序号连续拼接。兼容 0 起始和 1 起始。
        for base in (0, 1):
            chunks = []
            size = 0
            frag_id = base
            while frag_id in assembly.fragments and size < assembly.total_size:
                chunk = assembly.fragments[frag_id]
                chunks.append(chunk)
                size += len(chunk)
                frag_id += 1
            if size >= assembly.total_size:
                return b"".join(chunks)[:assembly.total_size]
        return None

    def _drop_stale_frames(self, now: float):
        with self.pending_lock:
            self._trim_pending_locked(now)

    def _trim_pending_locked(self, now: float):
        stale_ids = [
            frame_id
            for frame_id, assembly in self.pending.items()
            if now - assembly.updated_ts > self.frame_timeout_s
        ]
        overflow_ids = []
        if len(self.pending) - len(stale_ids) > self.max_pending_frames:
            live = sorted(
                (
                    (assembly.created_ts, frame_id)
                    for frame_id, assembly in self.pending.items()
                    if frame_id not in stale_ids
                )
            )
            overflow_count = len(self.pending) - len(stale_ids) - self.max_pending_frames
            overflow_ids = [frame_id for _ts, frame_id in live[:overflow_count]]

        dropped = 0
        for frame_id in stale_ids + overflow_ids:
            if self.pending.pop(frame_id, None) is not None:
                self.incomplete_drop_count += 1
                dropped += 1
        if dropped:
            self._enter_keyframe_recovery(f"丢弃不完整UDP帧 {dropped} 个")

    def _push_latest(self, target_queue: queue.Queue, item, label: str):
        try:
            target_queue.put_nowait(item)
            return
        except queue.Full:
            pass
        dropped = 0
        while True:
            try:
                target_queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        self.queue_drop_count += dropped
        if label == "display":
            self.display_drop_count += dropped
        try:
            target_queue.put_nowait(item)
        except queue.Full:
            self.queue_drop_count += 1
            if label == "display":
                self.display_drop_count += 1

    def _push_encoded(self, frame_id: int, hevc_bytes: bytes, recv_ts: float):
        try:
            self.encoded_queue.put_nowait((frame_id, hevc_bytes, recv_ts))
            return
        except queue.Full:
            self.queue_drop_count += 1

        dropped = 0
        while True:
            try:
                self.encoded_queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        if dropped:
            self._enter_keyframe_recovery(f"解码队列丢弃encoded帧 {dropped} 个")

        try:
            self.encoded_queue.put_nowait((frame_id, hevc_bytes, recv_ts))
        except queue.Full:
            self.queue_drop_count += 1

    def _decode_loop(self):
        while self.running:
            try:
                item = self.encoded_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break

            frame_id, hevc_bytes, recv_ts = item
            nal_types, has_vcl, has_keyframe, _has_params = self._keyframe_gate_state(hevc_bytes)
            if nal_types:
                self._update_hevc_params(hevc_bytes)
                if has_keyframe:
                    self.keyframe_count += 1
                with self.recovery_lock:
                    waiting_for_keyframe = self.waiting_for_keyframe
                    needs_reset = self.decoder_needs_reset
                if waiting_for_keyframe:
                    if not has_keyframe:
                        self.wait_keyframe_skip_count += 1
                        if self.wait_keyframe_skip_count % 60 == 0:
                            print(f"[恢复] 等待HEVC关键帧，已跳过 {self.wait_keyframe_skip_count} 帧")
                        continue
                    if needs_reset:
                        self._reset_decoder(f"loss recovery frame={frame_id}")
                    with self.recovery_lock:
                        self.waiting_for_keyframe = False
                        self.decoder_needs_reset = False
                        self.wait_keyframe_skip_count = 0
                    print(f"[恢复] 收到HEVC关键帧 frame={frame_id} types={nal_types[:8]}，恢复解码")
                elif self._consume_decoder_reset_flag():
                    self._reset_decoder(f"delayed recovery frame={frame_id}")
                if not has_vcl:
                    continue
                if has_keyframe:
                    hevc_bytes = self._prepend_cached_hevc_params(hevc_bytes, nal_types)
            elif self._consume_decoder_reset_flag():
                self._reset_decoder(f"loss recovery frame={frame_id}")
            try:
                decoded_any = False
                for packet in self.codec.parse(hevc_bytes):
                    for frame in self.codec.decode(packet):
                        decoded_any = True
                        self._handle_decoded_frame(frame, frame_id, recv_ts)
                if not decoded_any and self.decoded_frame_count == 0:
                    # 启动阶段参数集/IDR 未到时很常见，不打印刷屏。
                    pass
            except Exception as exc:
                self.decode_error_count += 1
                if self.decode_error_count <= 8:
                    print(f"[解码] frame={frame_id} 失败: {exc}")
                self._enter_keyframe_recovery(f"HEVC解码错误 frame={frame_id}")

    def _handle_decoded_frame(self, frame, frame_id: int, recv_ts: float):
        if frame is None or frame.width <= 0 or frame.height <= 0:
            return
        if not self.signal_active:
            return

        img = frame.to_ndarray(format="bgr24")
        if img is None or img.size == 0:
            return

        self.decoded_frame_count += 1
        now = time.time()
        self.last_frame_ts = now
        self.decode_times.append(now)
        self.frame_height, self.frame_width = img.shape[:2]
        latency_ms = (now - recv_ts) * 1000.0
        self.last_udp_to_display_latency_ms = latency_ms
        self._set_latest_raw_frame(img, frame_id, now)
        if self.frame_queue_enable:
            self._push_latest(self.frame_queue, (img, frame_id, latency_ms), "display")

    def _set_latest_raw_frame(self, img, frame_id: int, now: float):
        if not self.web_preview_enable or img is None or img.size == 0:
            return
        with self.raw_frame_lock:
            if (self.latest_raw_frame is not None
                    and self.latest_raw_frame_generation > self.last_web_encoded_generation):
                self.web_skip_count += 1
            self.latest_raw_frame = img
            self.latest_raw_frame_generation += 1
            self.latest_raw_frame_ts = now

    def _jpeg_loop(self):
        last_encoded_generation = -1
        while self.running:
            if not self.web_preview_enable:
                time.sleep(0.1)
                continue

            now = time.time()
            min_interval = 1.0 / self.web_preview_fps
            if self.last_web_preview_ts > 0 and now - self.last_web_preview_ts < min_interval:
                time.sleep(min(0.005, min_interval))
                continue

            with self.raw_frame_lock:
                img = self.latest_raw_frame
                generation = self.latest_raw_frame_generation
            if img is None or img.size == 0:
                time.sleep(0.01)
                continue
            if generation == last_encoded_generation:
                time.sleep(0.005)
                continue

            if self._encode_web_preview(img, now):
                last_encoded_generation = generation
                self.last_web_encoded_generation = generation
            else:
                time.sleep(0.005)

    def _encode_web_preview(self, img, now=None):
        if not self.web_preview_enable or img is None or img.size == 0:
            return False
        try:
            now = time.time() if now is None else now
            self.last_web_preview_ts = now
            web_img = self._resize_to_width(img, self.web_preview_width)
            ok, jpg = cv2.imencode(
                ".jpg",
                web_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.web_jpeg_quality)],
            )
            if ok:
                jpg_bytes = jpg.tobytes()
                self.preview_state.set_jpeg_frame(jpg_bytes)
                self.last_web_jpeg_bytes = len(jpg_bytes)
                self.web_output_frame_count += 1
                self.web_frame_times.append(now)
                return True
            else:
                self.last_web_jpeg_bytes = 0
                return False
        except Exception:
            return False

    def _resize_to_width(self, img, target_width):
        if target_width <= 0 or img is None or img.size == 0:
            return img
        h, w = img.shape[:2]
        if w <= 0 or h <= 0 or w <= target_width:
            return img
        target_h = max(1, int(h * target_width / w))
        return cv2.resize(img, (target_width, target_h), interpolation=cv2.INTER_AREA)

    def _resize_for_display(self, img):
        if self.width <= 0 or img is None or img.size == 0:
            return img
        h, w = img.shape[:2]
        if w <= 0 or h <= 0 or w == self.width:
            return img
        target_h = max(1, int(h * self.width / w))
        return cv2.resize(img, (self.width, target_h), interpolation=cv2.INTER_AREA)

    def _prune_times(self, values: deque, now: float):
        while values and now - values[0] > 1.0:
            values.popleft()

    def _publish_metrics(self):
        now = time.time()
        self._prune_times(self.packet_times, now)
        self._prune_times(self.complete_times, now)
        self._prune_times(self.decode_times, now)
        self._prune_times(self.web_frame_times, now)
        with self.pending_lock:
            pending_count = len(self.pending)
            pending_bytes = sum(item.received_bytes for item in self.pending.values())

        last_packet_age_ms = (now - self.last_packet_ts) * 1000.0 if self.last_packet_ts > 0 else -1.0
        last_frame_age_ms = (now - self.last_frame_ts) * 1000.0 if self.last_frame_ts > 0 else -1.0
        self.preview_state.update_metrics({
            "udp_host": self.host,
            "udp_port": self.port,
            "udp_active": self.signal_active,
            "packet_active": self.signal_active,
            "packet_timeout_s": self.packet_timeout_s,
            "last_error": self.last_error,
            "video_pkt_count": self.udp_packet_count,
            "complete_frame_count": self.completed_frame_count,
            "parsed_packet_count": self.completed_frame_count,
            "frame_count": self.decoded_frame_count,
            "display_frame_count": self.display_frame_count,
            "display_drop_count": self.display_drop_count,
            "web_output_frame_count": self.web_output_frame_count,
            "web_skip_count": self.web_skip_count,
            "web_preview_fps_limit": self.web_preview_fps,
            "web_preview_fps": float(len(self.web_frame_times)),
            "web_jpeg_quality": self.web_jpeg_quality,
            "web_preview_width": self.web_preview_width,
            "last_web_jpeg_bytes": self.last_web_jpeg_bytes,
            "packet_fps": float(len(self.packet_times)),
            "complete_fps": float(len(self.complete_times)),
            "decode_fps": float(len(self.decode_times)),
            "last_packet_age_ms": last_packet_age_ms,
            "last_frame_age_ms": last_frame_age_ms,
            "udp_to_display_latency_ms": self.last_udp_to_display_latency_ms,
            "frame_width": self.frame_width,
            "frame_height": self.frame_height,
            "pending_count": pending_count,
            "pending_bytes": pending_bytes,
            "bad_packet_count": self.bad_packet_count,
            "dup_drop": self.duplicate_fragment_count,
            "gap_drop": self.incomplete_drop_count,
            "queue_drop_count": self.queue_drop_count,
            "decoder_reset_count": self.decoder_reset_count,
            "loss_recovery_count": self.loss_recovery_count,
            "waiting_for_keyframe": self.waiting_for_keyframe,
            "wait_keyframe_skip_count": self.wait_keyframe_skip_count,
            "keyframe_count": self.keyframe_count,
            "frame_gap_count": self.frame_gap_count,
            "server_time": now,
        })

    def _log_stats(self, now: float):
        if now - self.last_log_ts < self.log_interval_s:
            return
        self.last_log_ts = now
        self._publish_metrics()
        with self.pending_lock:
            pending_count = len(self.pending)
            pending_bytes = sum(item.received_bytes for item in self.pending.values())
        frame_age_ms = (now - self.last_frame_ts) * 1000.0 if self.last_frame_ts > 0 else -1.0
        print(
            f"[统计] udp={self.udp_packet_count} pkt/s={len(self.packet_times)} "
            f"complete={self.completed_frame_count}({len(self.complete_times)}/s) "
            f"decode={self.decoded_frame_count}({len(self.decode_times)}/s) "
            f"display={self.display_frame_count} size={self.frame_width}x{self.frame_height} "
            f"pending={pending_count}/{pending_bytes}B drop(incomplete/queue/display)="
            f"{self.incomplete_drop_count}/{self.queue_drop_count}/{self.display_drop_count} "
            f"bad={self.bad_packet_count} dup={self.duplicate_fragment_count} "
            f"gap={self.frame_gap_count} "
            f"age={frame_age_ms:.0f}ms reset={self.decoder_reset_count} "
            f"wait_key={int(self.waiting_for_keyframe)} recover={self.loss_recovery_count}"
        )

    def _start_web_preview(self):
        if not self.web_preview_enable:
            return
        try:
            handler = make_preview_handler(self.preview_state, self.web_preview_fps)
            self.preview_server = ThreadingHTTPServer(
                (self.web_preview_host, self.web_preview_port),
                handler,
            )
            preview_thread = threading.Thread(
                target=self.preview_server.serve_forever,
                daemon=True,
            )
            preview_thread.start()
            print(f"[Web预览] 已启动: http://127.0.0.1:{self.web_preview_port}/")
            if self.web_preview_host == "0.0.0.0":
                print(f"[Web预览] 局域网访问: http://<本机IP>:{self.web_preview_port}/")
            self._schedule_open_browser()
        except Exception as exc:
            print(f"[Web预览] 启动失败: {exc}")

    def _schedule_open_browser(self):
        if not self.auto_open_browser:
            return
        url = f"http://127.0.0.1:{self.web_preview_port}/"

        def open_browser():
            try:
                if webbrowser.open(url, new=2):
                    print(f"[Web预览] 已自动打开浏览器: {url}")
                else:
                    print(f"[Web预览] 自动打开浏览器失败，请手动访问: {url}")
            except Exception as exc:
                print(f"[Web预览] 自动打开浏览器失败: {exc}，请手动访问: {url}")

        timer = threading.Timer(self.auto_open_delay_s, open_browser)
        timer.daemon = True
        timer.start()

    def _display_loop(self):
        window_name = "Mode4 UDP HEVC"
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            if self.width > 0:
                cv2.resizeWindow(window_name, self.width, max(1, int(self.width * 3 / 4)))
            print("[显示] OpenCV窗口已创建，按 q 退出")
        except Exception as exc:
            print(f"[显示] 创建OpenCV窗口失败: {exc}")
            print("[显示] 无GUI环境请使用Web预览")
            self._headless_loop()
            return

        wait_ms = max(1, int(1000 / self.fps))
        while self.running:
            try:
                img, frame_id, _latency_ms = self.frame_queue.get(timeout=0.05)
                if img is None:
                    break
                cv2.imshow(window_name, self._resize_for_display(img))
                if frame_id >= 0:
                    self.display_frame_count += 1
                if cv2.waitKey(wait_ms) & 0xFF == ord("q"):
                    print("[显示] 用户关闭窗口")
                    break
            except queue.Empty:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[显示] 用户关闭窗口")
                    break
            except Exception as exc:
                print(f"[显示] 错误: {exc}")
                break
        cv2.destroyAllWindows()

    def _headless_loop(self):
        if self.web_preview_enable:
            print(f"[显示] OpenCV显示已关闭，请使用浏览器访问 http://127.0.0.1:{self.web_preview_port}/")
        else:
            print("[显示] OpenCV和Web显示均已关闭，后台继续接收和解码")
        while self.running:
            try:
                img, _frame_id, _latency_ms = self.frame_queue.get(timeout=0.5)
                if img is None:
                    break
            except queue.Empty:
                self._publish_metrics()

    def start_service(self):
        """非阻塞启动UDP监听与工作线程，供统一客户端调用。"""
        if self.service_started:
            return
        if not self.running:
            raise RuntimeError("UDP receiver has already been stopped")
        self._start_socket()
        if self.web_server_enable:
            self._start_web_preview()
        self._publish_metrics()
        self.recv_thread.start()
        self.decode_thread.start()
        self.jpeg_thread.start()
        self.signal_thread.start()
        self.service_started = True
        self.last_error = ""

    def run(self):
        self.start_service()
        if self.cv_display_enable:
            self._display_loop()
        else:
            self._headless_loop()

    def stop(self):
        if not self.running:
            return
        self.running = False
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass
        try:
            self.encoded_queue.put_nowait(None)
        except Exception:
            pass
        try:
            self.frame_queue.put_nowait((None, -1, 0.0))
        except Exception:
            pass
        try:
            self.recv_thread.join(timeout=1.0)
            self.decode_thread.join(timeout=1.0)
            self.jpeg_thread.join(timeout=1.0)
            self.signal_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.preview_server is not None:
                self.preview_server.shutdown()
                self.preview_server.server_close()
        except Exception:
            pass
        self.service_started = False


def parse_args():
    parser = argparse.ArgumentParser(description="Mode4 UDP 3334 HEVC 图传接收端")
    parser.add_argument("--host", default=CONFIG["host"], help="UDP 监听地址")
    parser.add_argument("--port", type=int, default=CONFIG["port"], help="UDP 监听端口")
    parser.add_argument("--width", type=int, default=CONFIG["display_width"], help="显示窗口目标宽度，0为原始大小")
    parser.add_argument("--fps", type=int, default=CONFIG["display_fps_hint"], help="显示刷新帧率提示")
    parser.add_argument(
        "--display",
        action=argparse.BooleanOptionalAction,
        default=CONFIG["cv_display_enable"],
        help="是否创建OpenCV显示窗口",
    )
    parser.add_argument("--little-endian", action="store_true", help="按小端解析8字节UDP包头")
    parser.add_argument("--max-frame-bytes", type=int, default=CONFIG["max_frame_bytes"], help="允许的最大单帧码流字节数")
    parser.add_argument("--frame-timeout-ms", type=int, default=CONFIG["frame_timeout_ms"], help="不完整帧超时丢弃时间")
    parser.add_argument("--web-port", type=int, default=CONFIG["web_preview_port"], help="Web预览HTTP端口")
    parser.add_argument("--web-fps", type=int, default=CONFIG["web_preview_fps"], help="Web预览JPEG限帧")
    parser.add_argument("--web-width", type=int, default=CONFIG["web_preview_width"], help="Web预览JPEG目标宽度，0为原始大小")
    parser.add_argument("--web-quality", type=int, default=CONFIG["web_jpeg_quality"], help="Web预览JPEG质量")
    parser.add_argument("--no-web", action="store_true", help="关闭Web预览，只保留OpenCV窗口")
    parser.add_argument("--no-open-browser", action="store_true", help="启动后不自动打开浏览器")
    parser.add_argument("--web", action="store_true", help="enable web preview")
    return parser.parse_args()


def main():
    args = parse_args()
    receiver = UdpHevcReceiver(
        host=args.host,
        port=args.port,
        width=args.width,
        fps=args.fps,
        cv_display_enable=args.display,
        header_endian="little" if args.little_endian else CONFIG["header_endian"],
        web_preview_enable=args.web or (CONFIG["web_preview_enable"] and not args.no_web),
        web_preview_host=CONFIG["web_preview_host"],
        web_preview_port=args.web_port,
        web_preview_fps=args.web_fps,
        web_preview_width=args.web_width,
        web_jpeg_quality=args.web_quality,
        auto_open_browser=not args.no_open_browser,
        auto_open_delay_s=CONFIG["auto_open_delay_s"],
        recv_buffer_bytes=CONFIG["recv_buffer_bytes"],
        max_udp_packet_bytes=CONFIG["max_udp_packet_bytes"],
        max_frame_bytes=args.max_frame_bytes,
        max_pending_frames=CONFIG["max_pending_frames"],
        frame_timeout_ms=args.frame_timeout_ms,
        wait_keyframe_after_loss=CONFIG["wait_keyframe_after_loss"],
        encoded_queue_size=CONFIG["encoded_queue_size"],
        decoded_queue_size=CONFIG["decoded_queue_size"],
        log_interval_s=CONFIG["log_interval_s"],
        packet_timeout_s=CONFIG["packet_timeout_s"],
        signal_check_interval_s=CONFIG["signal_check_interval_s"],
    )
    try:
        receiver.run()
    except KeyboardInterrupt:
        print("\n[退出] 收到 Ctrl+C")
    finally:
        receiver.stop()
        print(
            f"[汇总] udp={receiver.udp_packet_count} complete={receiver.completed_frame_count} "
            f"decoded={receiver.decoded_frame_count} displayed={receiver.display_frame_count} "
            f"bad={receiver.bad_packet_count} incomplete_drop={receiver.incomplete_drop_count}"
        )


if __name__ == "__main__":
    main()
