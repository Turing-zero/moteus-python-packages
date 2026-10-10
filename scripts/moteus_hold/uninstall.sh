#!/usr/bin/env bash
# 卸载 moteus-hold systemd 服务（保留 /etc/default/moteus-hold）
# 用法: sudo ./uninstall.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行" >&2
  exit 1
fi

systemctl disable --now moteus-hold.service 2>/dev/null || true
rm -f /etc/systemd/system/moteus-hold.service
systemctl daemon-reload
if [ -L /usr/local/bin/moteus-holdctl ]; then
  rm -f /usr/local/bin/moteus-holdctl
fi
echo "已卸载。参数文件 /etc/default/moteus-hold 保留，不需要可手动删除。"
