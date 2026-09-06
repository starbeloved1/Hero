"""CustomControl 的 Protobuf 序列化边界。"""

from __future__ import annotations

try:
    from .custom_control_pb2 import CustomControl
except ImportError:  # 兼容直接执行 src/sub_win.py 的旧用法。
    from custom_control_pb2 import CustomControl


def encode_custom_control(value: int) -> bytes:
    """将面板控制值编码成 CustomControl 的 Protobuf 二进制。"""
    value = int(value)
    if value not in (0, 1):
        raise ValueError(f'CustomControl.value must be 0 or 1, got {value}')
    message = CustomControl()
    # value 是 optional 字段：即使为 0 也必须显式设置，确保发送 08 00 而非空包。
    message.value = value
    return message.SerializeToString()
