#!/usr/bin/env bash

set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
source "${project_dir}/ros2_ws/install/setup.bash"

pids=()

cleanup() {
  for pid in "${pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT
trap 'cleanup; exit 0' INT TERM

# 各功能包自行读取其 config 目录下的 YAML，不在这里覆盖参数。
ros2 launch gimbal_driver gimbal_driver.launch.py &
pids+=("$!")

ros2 launch hero_tf hero_tf.launch.py &
pids+=("$!")

ros2 launch camera_driver camera_driver.launch.py &
pids+=("$!")

ros2 launch camera_router camera_router.launch.py &
pids+=("$!")

ros2 launch armor_detector armor_detector.launch.py &
pids+=("$!")

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
pids+=("$!")

wait
