#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="sorting-pingpong.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
RUN_USER="${PINGPONG_SERVICE_USER:-$(id -un)}"
RUN_GROUP="${PINGPONG_SERVICE_GROUP:-$(id -gn)}"

usage() {
  cat <<'EOF'
用法:
  scripts/pingpong_service.sh install    安装、设置开机自启并立即启动
  scripts/pingpong_service.sh start      启动服务
  scripts/pingpong_service.sh stop       停止服务
  scripts/pingpong_service.sh restart    重启服务
  scripts/pingpong_service.sh status     查看服务状态
  scripts/pingpong_service.sh logs       实时查看日志
  scripts/pingpong_service.sh enable     设置开机自启
  scripts/pingpong_service.sh disable    取消开机自启
  scripts/pingpong_service.sh uninstall  停止、取消自启并删除服务

说明:
  正式服务只跑识别和 F407/TCP 链路，不打开 OpenCV/X11 预览窗口。
  现场配置仍然读取 config/pingpong_demo.env。
EOF
}

info() {
  echo "[INFO] $*"
}

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

need_sudo() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

check_repo() {
  [[ -x "$REPO_DIR/scripts/demo_pingpong.sh" ]] || die "找不到可执行启动脚本: $REPO_DIR/scripts/demo_pingpong.sh"
  [[ -f "$REPO_DIR/config/pingpong_demo.env" ]] || die "找不到现场配置: $REPO_DIR/config/pingpong_demo.env。请先复制并修改 config/pingpong_demo.env.example。"
  [[ -f "$REPO_DIR/ros2_ws/install/setup.sh" ]] || die "ROS2 工作区还没有 build: $REPO_DIR/ros2_ws/install/setup.sh"
}

service_file() {
  cat <<EOF
[Unit]
Description=Sorting robot pingpong demo
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
Group=$RUN_GROUP
WorkingDirectory=$REPO_DIR
Environment=PINGPONG_DEMO_CONFIG=$REPO_DIR/config/pingpong_demo.env
Environment=WORKSPACE_DIR=$REPO_DIR/ros2_ws
Environment=ROS_SETUP=/opt/ros/humble/setup.sh
ExecStart=$REPO_DIR/scripts/demo_pingpong.sh
Restart=on-failure
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
EOF
}

install_service() {
  check_repo
  info "安装 systemd 服务: $SERVICE_PATH"
  local tmp_file
  tmp_file="$(mktemp)"
  service_file >"$tmp_file"
  need_sudo install -m 0644 "$tmp_file" "$SERVICE_PATH"
  rm -f "$tmp_file"
  need_sudo systemctl daemon-reload
  need_sudo systemctl enable "$SERVICE_NAME"
  need_sudo systemctl restart "$SERVICE_NAME"
  info "已安装并启动: $SERVICE_NAME"
  info "查看状态: scripts/pingpong_service.sh status"
  info "查看日志: scripts/pingpong_service.sh logs"
}

uninstall_service() {
  info "停止并删除 systemd 服务: $SERVICE_NAME"
  need_sudo systemctl stop "$SERVICE_NAME" 2>/dev/null || true
  need_sudo systemctl disable "$SERVICE_NAME" 2>/dev/null || true
  need_sudo rm -f "$SERVICE_PATH"
  need_sudo systemctl daemon-reload
  info "已删除: $SERVICE_NAME"
}

command="${1:-install}"
case "$command" in
  install)
    install_service
    ;;
  start)
    check_repo
    need_sudo systemctl start "$SERVICE_NAME"
    ;;
  stop)
    need_sudo systemctl stop "$SERVICE_NAME"
    ;;
  restart)
    check_repo
    need_sudo systemctl restart "$SERVICE_NAME"
    ;;
  status)
    systemctl status "$SERVICE_NAME" --no-pager
    ;;
  logs)
    journalctl -u "$SERVICE_NAME" -f
    ;;
  enable)
    need_sudo systemctl enable "$SERVICE_NAME"
    ;;
  disable)
    need_sudo systemctl disable "$SERVICE_NAME"
    ;;
  uninstall)
    uninstall_service
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage
    die "未知命令: $command"
    ;;
esac
