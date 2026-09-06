MQTT_CONFIG = {
    "broker": "127.0.0.1",
    "mqtt_port": 3333,
    "client_id": "101",

    "topic_video": "CustomByteBlock",
    "topic_deploy_status": "DeployModeStatusSync",
    "topic_event": "Event",
    "topic_control": "CustomControl",

    "control_minus_value": 0,
    "control_plus_value": 1,

    "qos_level": 1,

    "cv_display_enable": True,
    "display_width": 1280,

    "display_fps_hint": 30,

    "web_preview_enable": False,
    "web_preview_port": 8099,
    "web_preview_fps": 30,
    "web_preview_width": 0,
    "web_jpeg_quality": 60,

    "ingress_queue_size": 30,
    "trim_h264_padding_zeros": False,
    "drop_to_idr_on_gap": True,

    "drop_to_idr_min_gap": 3,

    "auto_open_browser": True,
    "enable_cli_override": False,
}


UDP_CONFIG = {
    "host": "0.0.0.0",
    "port": 3334,

    "cv_display_enable": False,
    "display_width": 1280,
    "display_fps_hint": 75,

    "web_preview_enable": True,
    "web_preview_host": "127.0.0.1",
    "web_preview_port": 8098,
    "web_preview_fps": 30,
    "web_preview_width": 0,
    "web_jpeg_quality": 30,
    "auto_open_browser": True,
    "auto_open_delay_s": 0.8,

    "header_endian": "big",
    "recv_buffer_bytes": 4 * 1024 * 1024,
    "max_udp_packet_bytes": 65535,
    "max_frame_bytes": 8 * 1024 * 1024,
    "max_pending_frames": 4,
    "frame_timeout_ms": 160,
    "wait_keyframe_after_loss": True,

    "decoded_queue_size": 1,
    "encoded_queue_size": 10,
    "log_interval_s": 1.0,
    "packet_timeout_s": 1.0,
    "signal_check_interval_s": 0.2,
}


APP_CONFIG = {
    "host": "127.0.0.1",
    "port": 8097,
    "auto_open_browser": True,
    "auto_open_delay_s": 0.8,
    "default_main_stream": "udp",
    "status_poll_ms": 500,
    # 关闭后 UDP 接收仍继续，只是浏览器不加载/占用 UDP 画面区域。
    "udp_preview_enabled": True,
    # 关闭后 MQTT 接收仍继续，只是浏览器不加载/占用 MQTT 画面区域。
    "mqtt_preview_enabled": True,
}
