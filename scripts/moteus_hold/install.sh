#!/usr/bin/env bash
# 安装 moteus-hold systemd 服务（开机自启）
# 用法: sudo ./install.sh --user NAME [--python PATH] [--no-start]
#   --user    运行服务的用户（必填，该用户的 Python 需能 import moteus 和 python-can）
#   --python  Python 解释器（默认 /usr/bin/python3；用 venv 时填 venv 的 python）
#   --no-start  只安装并设置开机自启，不立即启动
# 本目录可以留在仓库里，也可以单独拷贝到部署机（此时使用 pip 安装的 moteus）。
# 服务直接运行本目录下的脚本，安装后不要移动或删除本目录。
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER=""
PYTHON=/usr/bin/python3
START=1

while [ $# -gt 0 ]; do
  case "$1" in
    --user) RUN_USER="$2"; shift 2 ;;
    --python) PYTHON="$2"; shift 2 ;;
    --no-start) START=0; shift ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
done

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行: sudo $0 $*" >&2
  exit 1
fi
if [ -z "$RUN_USER" ]; then
  echo "请用 --user 指定运行服务的用户，例如: sudo $0 --user robot" >&2
  exit 1
fi
if ! id "$RUN_USER" >/dev/null 2>&1; then
  echo "用户 $RUN_USER 不存在" >&2
  exit 1
fi
if [ "$RUN_USER" = root ]; then
  echo "不建议以 root 运行；服务只需 CAP_NET_ADMIN，请指定普通用户" >&2
  exit 1
fi
RUN_GROUP="$(id -gn "$RUN_USER")"

if ! sudo -u "$RUN_USER" test -r "$DIR/moteus_hold_daemon.py"; then
  echo "用户 $RUN_USER 无法读取 $DIR，请把本目录放到该用户可访问的位置（如 /opt/moteus_hold）" >&2
  exit 1
fi

echo "检查依赖（以 $RUN_USER 身份运行 $PYTHON）..."
if ! sudo -u "$RUN_USER" -H "$PYTHON" - "$DIR" <<'EOF'
import inspect, os, sys
repo = os.path.join(sys.argv[1], '..', '..', 'moteus')
if os.path.isfile(os.path.join(repo, 'moteus', '__init__.py')):
    sys.path.insert(0, repo)
import can, moteus
import moteus.pythoncan_device as pd
try:
    from importlib.metadata import version
    ver = version('moteus')
except Exception:
    ver = '?'
print(f'  moteus {ver}: {os.path.dirname(moteus.__file__)}')
print(f'  python-can {can.__version__}')
if 'restart_ms' not in inspect.signature(pd._maybe_start_socketcan).parameters:
    print('  警告: 该 moteus 版本不支持 restart_ms（bus-off 自动恢复），'
          '请安装包含 moteus_hold 支持的版本', file=sys.stderr)
EOF
then
  echo "依赖缺失：请先让 $RUN_USER 的 $PYTHON 能 import moteus 和 python-can" >&2
  exit 1
fi

chmod +x "$DIR/moteus_hold_daemon.py" "$DIR/moteus_holdctl.py"

sed -e "s|@USER@|$RUN_USER|g" -e "s|@GROUP@|$RUN_GROUP|g" \
    -e "s|@PYTHON@|$PYTHON|g" -e "s|@DIR@|$DIR|g" \
    "$DIR/moteus-hold.service" > /etc/systemd/system/moteus-hold.service
echo "已写入 /etc/systemd/system/moteus-hold.service"

if [ -f /etc/default/moteus-hold ]; then
  echo "保留已有的 /etc/default/moteus-hold"
else
  install -m 0644 "$DIR/moteus-hold.default" /etc/default/moteus-hold
  echo "已写入 /etc/default/moteus-hold"
fi

ln -sf "$DIR/moteus_holdctl.py" /usr/local/bin/moteus-holdctl
echo "已创建命令 moteus-holdctl"

systemctl daemon-reload
systemctl enable moteus-hold.service
if [ "$START" -eq 1 ]; then
  systemctl restart moteus-hold.service
  sleep 1
  systemctl --no-pager --lines=15 status moteus-hold.service || true
fi

echo
echo "完成。查看日志: journalctl -u moteus-hold -f"
echo "调试:         moteus-holdctl（需以 $RUN_USER 或其所在组的用户运行）"
