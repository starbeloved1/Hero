#!/usr/bin/env bash

set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash

# 自动定位本机 OpenVINO；也允许用户在终端中显式指定 OpenVINO_DIR。
hero_openvino_dir="${OpenVINO_DIR:-}"
if [[ -z "${hero_openvino_dir}" || ! -f "${hero_openvino_dir}/OpenVINOConfig.cmake" ]]; then
  hero_openvino_config=$(find "${HOME}/.local" -type f \
    -path '*/openvino/cmake/OpenVINOConfig.cmake' -print -quit 2>/dev/null)
  if [[ -z "${hero_openvino_config}" ]]; then
    echo "未找到 OpenVINOConfig.cmake。请安装 OpenVINO，或先设置 OpenVINO_DIR。" >&2
    exit 1
  fi
  hero_openvino_dir=$(dirname "${hero_openvino_config}")
fi

export OpenVINO_DIR="${hero_openvino_dir}"
hero_openvino_root=$(cd -- "${OpenVINO_DIR}/.." && pwd)
if [[ -d "${hero_openvino_root}/libs" ]]; then
  export LD_LIBRARY_PATH="${hero_openvino_root}/libs:${LD_LIBRARY_PATH:-}"
fi
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

ros2 launch armor_solver armor_solver.launch.py &
pids+=("$!")

ros2 launch aim_normal aim_normal.launch.py &
pids+=("$!")

ros2 launch command_mux command_mux.launch.py &
pids+=("$!")

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
pids+=("$!")

wait
