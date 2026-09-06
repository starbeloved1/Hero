import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

from config import APP_CONFIG, MQTT_CONFIG, UDP_CONFIG
from src.sub_win import MqttVideoSubscriber, make_status_frame
from src.udp_win import UdpHevcReceiver


ROOT = Path(__file__).resolve().parent
DASHBOARD_PATH = ROOT / "src" / "dashboard.html"


def encode_status_jpeg(text: str) -> bytes:
    frame = make_status_frame(1280, 720, text)
    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return jpg.tobytes() if ok else b""


class IntegratedController:
    def __init__(self):
        self.operation_lock = threading.RLock()
        # 累计伤害属于本次面板进程的状态：重启 MQTT 服务不清零，重启程序才清零。
        self.damage_lock = threading.Lock()
        self.cumulative_damage = None
        self.damage_last_update_ts = 0.0
        self.damage_message_count = 0
        self.mqtt_grid_enabled = True
        self.udp_preview_enabled = bool(APP_CONFIG.get("udp_preview_enabled", True))
        self.mqtt_preview_enabled = bool(APP_CONFIG.get("mqtt_preview_enabled", True))
        self.channels = {
            "mqtt": {"state": "STOPPED", "receiver": None, "error": ""},
            "udp": {"state": "STOPPED", "receiver": None, "error": ""},
        }
        self.placeholders = {
            "STOPPED": encode_status_jpeg("SERVICE STOPPED"),
            "STARTING": encode_status_jpeg("SERVICE STARTING"),
            "STOPPING": encode_status_jpeg("SERVICE STOPPING"),
            "ERROR": encode_status_jpeg("SERVICE ERROR"),
            "WAITING": encode_status_jpeg("WAITING FOR PACKETS"),
        }

    def _record_damage_event(self, value: str, received_ts: float) -> None:
        with self.damage_lock:
            self.cumulative_damage = value
            self.damage_last_update_ts = received_ts
            self.damage_message_count += 1

    def _create_mqtt_receiver(self):
        cfg = MQTT_CONFIG
        receiver = MqttVideoSubscriber(
            broker=cfg["broker"],
            mqtt_port=cfg["mqtt_port"],
            width=cfg["display_width"],
            fps=cfg["display_fps_hint"],
            cv_display_enable=False,
            # 比赛 MQTT client id 由配置唯一确定；集成模式不得自行改写。
            client_id=cfg["client_id"],
            topic_video=cfg["topic_video"],
            topic_deploy_status=cfg.get("topic_deploy_status", ""),
            topic_event=cfg.get("topic_event", ""),
            damage_event_callback=self._record_damage_event,
            topic_control=cfg.get("topic_control", ""),
            control_qos_level=cfg["qos_level"],
            control_minus_value=cfg.get("control_minus_value", 0),
            control_plus_value=cfg.get("control_plus_value", 1),
            qos_level=cfg["qos_level"],
            # 集成面板需要 JPEG；HTTP 服务则由本文件统一提供。
            web_preview_enable=True,
            web_preview_port=cfg["web_preview_port"],
            web_preview_fps=cfg["web_preview_fps"],
            web_jpeg_quality=cfg["web_jpeg_quality"],
            web_preview_width=cfg["web_preview_width"],
            trim_h264_padding_zeros=cfg.get("trim_h264_padding_zeros", True),
            drop_to_idr_on_gap=cfg.get("drop_to_idr_on_gap", False),
            drop_to_idr_min_gap=cfg.get("drop_to_idr_min_gap", 5),
            auto_open_browser=False,
            ingress_queue_size=cfg["ingress_queue_size"],
            web_server_enable=False,
            frame_queue_enable=False,
        )
        return receiver

    @staticmethod
    def _create_udp_receiver():
        cfg = UDP_CONFIG
        return UdpHevcReceiver(
            host=cfg["host"],
            port=cfg["port"],
            width=cfg["display_width"],
            fps=cfg["display_fps_hint"],
            cv_display_enable=False,
            header_endian=cfg["header_endian"],
            web_preview_enable=True,
            web_preview_host=cfg["web_preview_host"],
            web_preview_port=cfg["web_preview_port"],
            web_preview_fps=cfg["web_preview_fps"],
            web_preview_width=cfg["web_preview_width"],
            web_jpeg_quality=cfg["web_jpeg_quality"],
            auto_open_browser=False,
            auto_open_delay_s=cfg["auto_open_delay_s"],
            recv_buffer_bytes=cfg["recv_buffer_bytes"],
            max_udp_packet_bytes=cfg["max_udp_packet_bytes"],
            max_frame_bytes=cfg["max_frame_bytes"],
            max_pending_frames=cfg["max_pending_frames"],
            frame_timeout_ms=cfg["frame_timeout_ms"],
            wait_keyframe_after_loss=cfg["wait_keyframe_after_loss"],
            encoded_queue_size=cfg["encoded_queue_size"],
            decoded_queue_size=cfg["decoded_queue_size"],
            log_interval_s=cfg["log_interval_s"],
            packet_timeout_s=cfg["packet_timeout_s"],
            signal_check_interval_s=cfg["signal_check_interval_s"],
            web_server_enable=False,
            frame_queue_enable=False,
        )

    def start(self, name: str):
        if name not in self.channels:
            raise ValueError(f"Unknown service: {name}")
        with self.operation_lock:
            channel = self.channels[name]
            if channel["state"] in ("STARTING", "RUNNING"):
                return
            channel.update(state="STARTING", error="", receiver=None)
            receiver = None
            try:
                receiver = self._create_mqtt_receiver() if name == "mqtt" else self._create_udp_receiver()
                receiver.start_service()
                channel.update(state="RUNNING", receiver=receiver, error="")
            except Exception as exc:
                if receiver is not None:
                    try:
                        receiver.stop()
                    except Exception:
                        pass
                channel.update(state="ERROR", receiver=None, error=str(exc))

    def stop(self, name: str):
        if name not in self.channels:
            raise ValueError(f"Unknown service: {name}")
        with self.operation_lock:
            channel = self.channels[name]
            if channel["state"] == "STOPPED":
                return
            receiver = channel["receiver"]
            channel["state"] = "STOPPING"
            try:
                if receiver is not None:
                    receiver.stop()
                channel.update(state="STOPPED", receiver=None, error="")
            except Exception as exc:
                channel.update(state="ERROR", receiver=None, error=str(exc))

    def restart(self, name: str):
        with self.operation_lock:
            self.stop(name)
            self.start(name)

    def start_all(self):
        with self.operation_lock:
            self.start("udp")
            self.start("mqtt")

    def stop_all(self):
        with self.operation_lock:
            self.stop("mqtt")
            self.stop("udp")

    def restart_all(self):
        with self.operation_lock:
            self.stop_all()
            self.start_all()

    def toggle_mqtt_grid(self) -> bool:
        with self.operation_lock:
            self.mqtt_grid_enabled = not self.mqtt_grid_enabled
            return self.mqtt_grid_enabled

    def toggle_udp_preview(self) -> bool:
        """只控制浏览器布局；UDP 接收服务和解码不受影响。"""
        with self.operation_lock:
            self.udp_preview_enabled = not self.udp_preview_enabled
            return self.udp_preview_enabled

    def toggle_mqtt_preview(self) -> bool:
        """只控制浏览器布局；MQTT 接收服务和解码不受影响。"""
        with self.operation_lock:
            self.mqtt_preview_enabled = not self.mqtt_preview_enabled
            return self.mqtt_preview_enabled

    def publish_crop_direction(self, direction: str) -> None:
        with self.operation_lock:
            receiver = self.channels["mqtt"]["receiver"]
            if receiver is None:
                raise RuntimeError("MQTT video service is not running")
            receiver.publish_control(direction)

    def control(self, target: str, action: str):
        if target == "all":
            getattr(self, f"{action}_all")()
        else:
            getattr(self, action)(target)

    def snapshot(self):
        result = {}
        publisher_snapshot = {
            "connected": False,
            "topic_control": MQTT_CONFIG.get("topic_control", ""),
            "last_direction": "",
            "publish_count": 0,
            "last_error": "MQTT video service is not running",
        }
        with self.operation_lock:
            for name, channel in self.channels.items():
                receiver = channel["receiver"]
                metrics = {}
                if receiver is not None:
                    try:
                        metrics = receiver.preview_state.snapshot_metrics()
                    except Exception as exc:
                        metrics = {"last_error": str(exc)}
                result[name] = {
                    "service_state": channel["state"],
                    "service_error": channel["error"],
                    **metrics,
                }
                if name == "mqtt" and receiver is not None:
                    publisher_snapshot = receiver.control_snapshot()
        result["app"] = {
            "default_main_stream": APP_CONFIG["default_main_stream"],
            "status_poll_ms": APP_CONFIG["status_poll_ms"],
            "mqtt_frame_poll_ms": max(15, round(1000 / max(1, MQTT_CONFIG["web_preview_fps"]))),
            "udp_frame_poll_ms": max(15, round(1000 / max(1, UDP_CONFIG["web_preview_fps"]))),
            "mqtt_grid_enabled": self.mqtt_grid_enabled,
            "udp_preview_enabled": self.udp_preview_enabled,
            "mqtt_preview_enabled": self.mqtt_preview_enabled,
            "publisher": publisher_snapshot,
        }
        with self.damage_lock:
            result["damage"] = {
                "subscribed": bool(
                    MQTT_CONFIG.get("topic_event")
                    and result["mqtt"].get("mqtt_connected")
                    and result["mqtt"].get("mqtt_subscribed")
                ),
                "available": self.cumulative_damage is not None,
                "value": self.cumulative_damage,
                "last_update_age_ms": (
                    (time.time() - self.damage_last_update_ts) * 1000
                    if self.damage_last_update_ts else -1
                ),
                "message_count": self.damage_message_count,
            }
        return result

    def frame(self, name: str) -> bytes:
        with self.operation_lock:
            channel = self.channels[name]
            receiver = channel["receiver"]
            state = channel["state"]
            if receiver is not None:
                try:
                    with receiver.preview_state.condition:
                        jpg = receiver.preview_state.latest_jpeg
                    if jpg:
                        return bytes(jpg)
                except Exception:
                    pass
            return self.placeholders.get(state, self.placeholders["WAITING"])


def make_handler(controller: IntegratedController):
    dashboard = DASHBOARD_PATH.read_text(encoding="utf-8")
    dashboard = dashboard.replace("__STATUS_POLL_MS__", str(APP_CONFIG["status_poll_ms"]))
    dashboard = dashboard.replace(
        "__MQTT_FRAME_POLL_MS__",
        str(max(15, round(1000 / max(1, MQTT_CONFIG["web_preview_fps"])))),
    )
    dashboard = dashboard.replace(
        "__UDP_FRAME_POLL_MS__",
        str(max(15, round(1000 / max(1, UDP_CONFIG["web_preview_fps"])))),
    )
    dashboard = dashboard.replace("__DEFAULT_MAIN_STREAM__", APP_CONFIG["default_main_stream"])
    dashboard_bytes = dashboard.encode("utf-8")

    class DashboardHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def _send_json(self, payload, status=200):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(dashboard_bytes)))
                self.end_headers()
                self.wfile.write(dashboard_bytes)
                return
            if path == "/api/status":
                self._send_json(controller.snapshot())
                return
            if path in ("/api/frame/mqtt.jpg", "/api/frame/udp.jpg"):
                name = "mqtt" if "mqtt" in path else "udp"
                body = controller.frame(name)
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            self.send_error(404)

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            if path == "/api/mqtt/grid/toggle":
                enabled = controller.toggle_mqtt_grid()
                self._send_json({"ok": True, "mqtt_grid_enabled": enabled, "status": controller.snapshot()})
                return
            if path == "/api/udp/preview/toggle":
                enabled = controller.toggle_udp_preview()
                self._send_json({"ok": True, "udp_preview_enabled": enabled, "status": controller.snapshot()})
                return
            if path == "/api/mqtt/preview/toggle":
                enabled = controller.toggle_mqtt_preview()
                self._send_json({"ok": True, "mqtt_preview_enabled": enabled, "status": controller.snapshot()})
                return
            if path in ("/api/pub/left", "/api/pub/right"):
                direction = path.rsplit("/", 1)[-1]
                try:
                    controller.publish_crop_direction(direction)
                    self._send_json({"ok": True, "direction": direction, "status": controller.snapshot()})
                except Exception as exc:
                    self._send_json({"ok": False, "error": str(exc)}, 503)
                return
            parts = path.strip("/").split("/")
            if len(parts) != 4 or parts[:2] != ["api", "control"]:
                self.send_error(404)
                return
            target, action = parts[2], parts[3]
            if target not in ("mqtt", "udp", "all") or action not in ("start", "stop", "restart"):
                self._send_json({"ok": False, "error": "Invalid control request"}, 400)
                return
            try:
                controller.control(target, action)
                self._send_json({"ok": True, "status": controller.snapshot()})
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, 500)

    return DashboardHandler


def main():
    controller = IntegratedController()
    server = ThreadingHTTPServer(
        (APP_CONFIG["host"], int(APP_CONFIG["port"])),
        make_handler(controller),
    )
    url = f'http://{APP_CONFIG["host"]}:{APP_CONFIG["port"]}/'
    print(f"[APP] Integrated dashboard: {url}")

    if APP_CONFIG["auto_open_browser"]:
        timer = threading.Timer(
            max(0.0, float(APP_CONFIG["auto_open_delay_s"])),
            lambda: webbrowser.open(url, new=2),
        )
        timer.daemon = True
        timer.start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[APP] Stopping integrated client")
    finally:
        controller.stop_all()
        server.server_close()


if __name__ == "__main__":
    main()
