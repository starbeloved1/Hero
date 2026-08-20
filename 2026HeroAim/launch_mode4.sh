#!/bin/bash
# Mode4 本地发送脚本：AutoAim -> UDP -> MQTT bridge。
# 前提：已单独启动 MQTT broker，且 build/AutoAim 已完成编译。

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
MQTT_BRIDGE_PY="$SCRIPT_DIR/src/Mqtt_pub/antibase_receiver_mqtt.py"
AUTOAIM_PID=""
MQTT_PID=""
STOPPING=0

stop_group() {
    local pid="$1"
    [ -z "$pid" ] && return
    kill -TERM -- "-$pid" 2>/dev/null || true
}

cleanup() {
    if [ "$STOPPING" -eq 1 ]; then
        return
    fi
    STOPPING=1

    echo "[launch] Stopping..."
    stop_group "$AUTOAIM_PID"
    stop_group "$MQTT_PID"

    [ -n "$AUTOAIM_PID" ] && wait "$AUTOAIM_PID" 2>/dev/null || true
    [ -n "$MQTT_PID" ] && wait "$MQTT_PID" 2>/dev/null || true

    echo "[launch] All processes stopped."
}
trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT

# 检查正式编译产物；本地测试与比赛均使用同一 AutoAim。
if [ ! -x "$BUILD_DIR/AutoAim" ]; then
    echo "Error: $BUILD_DIR/AutoAim not found. Build first."
    exit 1
fi

mkdir -p "$SCRIPT_DIR/logs"

echo "[launch] Starting UDP->MQTT bridge and AutoAim. Press Ctrl+C to stop."

# 先启动 bridge，避免发送端第一个 SPS/PPS/IDR 被错过。
start_mqtt_bridge() {
    setsid python3 -u "$MQTT_BRIDGE_PY" &
    MQTT_PID=$!
    echo "[launch] MQTT bridge PID=$MQTT_PID"
}

start_mqtt_bridge
sleep 1
if ! kill -0 "$MQTT_PID" 2>/dev/null; then
    echo "[launch] MQTT bridge exited early. Check broker at 127.0.0.1:1884."
    exit 1
fi

setsid bash -c "cd '$BUILD_DIR' && exec ./AutoAim" &
AUTOAIM_PID=$!

sleep 1
if ! kill -0 "$AUTOAIM_PID" 2>/dev/null; then
    echo "[launch] AutoAim exited early."
    exit 1
fi
echo "[launch] AutoAim PID=$AUTOAIM_PID"

# AutoAim 为主进程；bridge 异常退出时自动拉起
while kill -0 "$AUTOAIM_PID" 2>/dev/null; do
    if ! kill -0 "$MQTT_PID" 2>/dev/null; then
        echo "[launch] MQTT bridge exited, restarting in 1s..."
        sleep 1
        if kill -0 "$AUTOAIM_PID" 2>/dev/null; then
            start_mqtt_bridge
        fi
    fi
    sleep 1
done

AUTO_EXIT=0
wait "$AUTOAIM_PID" 2>/dev/null || AUTO_EXIT=$?
echo "[launch] AutoAim exited with code $AUTO_EXIT"
cleanup
exit "$AUTO_EXIT"
