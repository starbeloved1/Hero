#!/usr/bin/env bash

set -euo pipefail

service_name="hero-aim.service"
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
template_file="${project_dir}/systemd/${service_name}.in"
service_file="/etc/systemd/system/${service_name}"
runtime_user="${SUDO_USER:-${USER}}"
dry_run=false

if [[ "${1:-}" == "--dry-run" ]]; then
  dry_run=true
elif [[ $# -ne 0 ]]; then
  echo "用法：$0 [--dry-run]" >&2
  exit 2
fi

if [[ ! -f "${template_file}" ]]; then
  echo "缺少 systemd 模板：${template_file}" >&2
  exit 1
fi
if [[ ! "${runtime_user}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then
  echo "无效的运行用户：${runtime_user}" >&2
  exit 1
fi
if [[ "${project_dir}" == *$'\n'* ]]; then
  echo "项目路径不能包含换行符" >&2
  exit 1
fi

escape_sed_replacement() {
  printf '%s' "$1" | sed 's/[&|\\]/\\&/g'
}

escaped_user=$(escape_sed_replacement "${runtime_user}")
escaped_project_dir=$(escape_sed_replacement "${project_dir}")
generated_service=$(mktemp)
trap 'rm -f "${generated_service}"' EXIT

sed \
  -e "s|@HERO_USER@|${escaped_user}|g" \
  -e "s|@HERO_PROJECT_DIR@|${escaped_project_dir}|g" \
  "${template_file}" > "${generated_service}"

if [[ "${dry_run}" == true ]]; then
  cat "${generated_service}"
  exit 0
fi

sudo install -m 0644 "${generated_service}" "${service_file}"
sudo systemctl daemon-reload

echo "已安装 ${service_name}"
echo "设置开机自启并立即启动：sudo systemctl enable --now ${service_name}"
echo "查看状态：sudo systemctl status ${service_name}"
echo "跟踪日志：journalctl -u ${service_name} -f"
