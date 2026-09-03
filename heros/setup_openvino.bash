#!/usr/bin/env bash

# 本文件只由 build.sh 和 run.sh source，用于定位本机安装的 OpenVINO。
# 允许用户显式设置 OpenVINO_DIR；未设置时自动搜索用户本地安装目录。

hero_openvino_dir="${OpenVINO_DIR:-}"
if [[ -z "${hero_openvino_dir}" || ! -f "${hero_openvino_dir}/OpenVINOConfig.cmake" ]]; then
  hero_openvino_config=$(find "${HOME}/.local" -type f \
    -path '*/openvino/cmake/OpenVINOConfig.cmake' -print -quit 2>/dev/null)
  if [[ -z "${hero_openvino_config}" ]]; then
    echo "未找到 OpenVINOConfig.cmake。请安装 OpenVINO，或先设置 OpenVINO_DIR。" >&2
    return 1
  fi
  hero_openvino_dir=$(dirname "${hero_openvino_config}")
fi

export OpenVINO_DIR="${hero_openvino_dir}"
hero_openvino_root=$(cd -- "${OpenVINO_DIR}/.." && pwd)
if [[ -d "${hero_openvino_root}/libs" ]]; then
  export LD_LIBRARY_PATH="${hero_openvino_root}/libs:${LD_LIBRARY_PATH:-}"
fi
