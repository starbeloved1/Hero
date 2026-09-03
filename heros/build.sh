#!/usr/bin/env bash

set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
source "${project_dir}/setup_openvino.bash"

cd "${project_dir}/ros2_ws"
colcon build --symlink-install "$@"
