"""集成面板使用的 MQTT 控制发布器；不提供独立运行入口。"""

from __future__ import annotations

import threading
from typing import Any

import paho.mqtt.client as mqtt

try:
    from .control_protocol import encode_custom_control
except ImportError:
    from control_protocol import encode_custom_control


class MqttControlPublisher:
    """维护一个 MQTT 连接，并向 CustomControl 发布 Protobuf 控制消息。"""

    def __init__(self, config: dict[str, Any]) -> None:
        self.broker = str(config["broker"])
        self.mqtt_port = int(config["mqtt_port"])
        self.client_id = str(config["client_id"])
        self.topic_control = str(config["topic_control"])
        self.qos_level = int(config.get("qos_level", 1))
        self.connect_timeout_s = max(0.1, float(config.get("connect_timeout_s", 2.0)))
        self.payloads = {
            "left": encode_custom_control(config.get("control_minus_value", 0)),
            "right": encode_custom_control(config.get("control_plus_value", 1)),
        }
        if not self.topic_control:
            raise ValueError("PUB_CONFIG.topic_control must not be empty")
        for direction, payload in self.payloads.items():
            if not payload:
                raise ValueError(f"PUB_CONFIG.payload_{direction} must not be empty")
            if len(payload) > 30:
                raise ValueError(f"PUB_CONFIG.payload_{direction} exceeds 30 bytes")

        self._lock = threading.RLock()
        self._connected = threading.Event()
        self._started = False
        self.last_error = ""
        self.last_direction = ""
        self.publish_count = 0
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
        )
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect

    def _on_connect(self, _client, _userdata, _flags, reason_code, _properties) -> None:
        if reason_code.is_failure:
            self.last_error = f"MQTT control connect failed: {reason_code}"
            self._connected.clear()
            return
        self.last_error = ""
        self._connected.set()
        print(f"[MQTT pub] connected {self.broker}:{self.mqtt_port}, topic={self.topic_control}")

    def _on_disconnect(self, _client, _userdata, _flags, reason_code, _properties) -> None:
        self._connected.clear()
        if reason_code:
            self.last_error = f"MQTT control disconnected: {reason_code}"

    def start(self) -> None:
        """由 integrated_client 启动；重复调用安全。"""
        with self._lock:
            if self._started:
                return
            try:
                self.client.connect_async(self.broker, self.mqtt_port, keepalive=60)
                self.client.loop_start()
                self._started = True
            except Exception as exc:
                self.last_error = f"MQTT control start failed: {exc}"

    def publish_direction(self, direction: str) -> None:
        if direction not in self.payloads:
            raise ValueError(f"Unsupported direction: {direction}")
        self.start()
        if not self._connected.wait(self.connect_timeout_s):
            detail = self.last_error or "broker connection timed out"
            raise RuntimeError(f"MQTT control unavailable: {detail}")
        with self._lock:
            info = self.client.publish(self.topic_control, self.payloads[direction], qos=self.qos_level)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish failed: rc={info.rc}")
            self.last_direction = direction
            self.publish_count += 1
            self.last_error = ""
        print(f"[MQTT pub] {direction} -> {self.topic_control}")

    def snapshot(self) -> dict[str, Any]:
        return {"connected": self._connected.is_set(), "topic_control": self.topic_control,
                "last_direction": self.last_direction, "publish_count": self.publish_count,
                "last_error": self.last_error}

    def stop(self) -> None:
        with self._lock:
            self._connected.clear()
            if self._started:
                try:
                    self.client.disconnect()
                    self.client.loop_stop()
                except Exception:
                    pass
            self._started = False
