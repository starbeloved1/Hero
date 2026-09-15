#!/usr/bin/env bash

set -euo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
tool_dir="${project_dir}/tools/hero_judge"

if ! command -v mosquitto >/dev/null 2>&1; then
  echo "未找到 mosquitto；请安装：sudo apt install mosquitto mosquitto-clients python3-paho-mqtt" >&2
  exit 1
fi
if ! python3 -c 'import paho.mqtt.client' >/dev/null 2>&1; then
  echo "当前 python3 缺少 paho-mqtt；请安装 python3-paho-mqtt 或 pip install paho-mqtt" >&2
  exit 1
fi

mosquitto -c "${tool_dir}/mosquitto_hero_judge.conf" &
broker_pid=$!
cleanup() {
  if kill -0 "${broker_pid}" 2>/dev/null; then
    kill "${broker_pid}" 2>/dev/null || true
    wait "${broker_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

sleep 0.2
if ! kill -0 "${broker_pid}" 2>/dev/null; then
  echo "裁判 Mosquitto 启动失败（3333 端口可能已被占用）" >&2
  exit 1
fi

python3 "${tool_dir}/hero_judge.py" "$@"
