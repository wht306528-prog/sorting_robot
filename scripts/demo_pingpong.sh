#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

CONFIG_FILE="${PINGPONG_DEMO_CONFIG:-$REPO_DIR/config/pingpong_demo.env}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$REPO_DIR/ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.sh}"

usage() {
  cat <<'EOF'
用法:
  scripts/demo_pingpong.sh

可选环境变量:
  PINGPONG_DEMO_CONFIG=/path/to/pingpong_demo.env
  CAMERA_PROFILE=auto|usb0|usb1|topic
  IMAGE_TOPIC=/your/image/topic
  F407_HOST=127.0.0.1
  F407_PORT=9000
  DEMO_DRY_RUN=1

首次配置:
  cp config/pingpong_demo.env.example config/pingpong_demo.env
EOF
}

print_quick_commands() {
  cat <<EOF

========== 现场常用命令 ==========
1. 创建/修改现场配置:
   cp config/pingpong_demo.env.example config/pingpong_demo.env
   nano config/pingpong_demo.env

2. 只检查配置，不启动:
   DEMO_DRY_RUN=1 scripts/demo_pingpong.sh

3. 查看 USB 相机:
   ls /dev/video*
   v4l2-ctl --list-devices

4. 查看 ROS2 图像和矩阵 topic:
   source /opt/ros/humble/setup.sh
   source $WORKSPACE_DIR/install/setup.sh
   ros2 topic list
   ros2 topic hz /image_raw
   ros2 topic echo /sorting/tray_matrix

5. 本机模拟 F407 接收:
   nc -l $F407_PORT

6. 真实 F407/W5500 地址配置:
   nano config/pingpong_demo.env
   # 修改 F407_HOST 和 F407_PORT

7. 当前将使用:
   F407_HOST=$F407_HOST
   F407_PORT=$F407_PORT
==================================

EOF
}

info() {
  echo "[INFO] $*"
}

warn() {
  echo "[WARN] $*" >&2
}

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  usage
  exit 0
fi

if [[ -f "$CONFIG_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
  info "已加载配置: $CONFIG_FILE"
else
  warn "未找到配置文件: $CONFIG_FILE"
  warn "将使用默认值；需要固定现场参数时，执行: cp config/pingpong_demo.env.example config/pingpong_demo.env"
fi

CAMERA_PROFILE="${CAMERA_PROFILE:-auto}"
IMAGE_TOPIC="${IMAGE_TOPIC:-}"
ACTIVE_TRAY_ID="${ACTIVE_TRAY_ID:-1}"
PROCESS_EVERY_N_FRAMES="${PROCESS_EVERY_N_FRAMES:-3}"
F407_HOST="${F407_HOST:-127.0.0.1}"
F407_PORT="${F407_PORT:-9000}"

[[ -f "$ROS_SETUP" ]] || die "找不到 ROS2 环境: $ROS_SETUP"
[[ -d "$WORKSPACE_DIR" ]] || die "找不到 ROS2 工作区: $WORKSPACE_DIR"
[[ -f "$WORKSPACE_DIR/install/setup.sh" ]] || die "工作区还没有 build 或 install/setup.sh 不存在。请先执行: cd $WORKSPACE_DIR && colcon build"
[[ -x "$REPO_DIR/scripts/run_pingpong_demo.sh" ]] || die "启动脚本不可执行: $REPO_DIR/scripts/run_pingpong_demo.sh"

PROFILE_TO_RUN="$CAMERA_PROFILE"
TOPIC_TO_RUN=""

case "$CAMERA_PROFILE" in
  auto)
    if [[ -e /dev/video0 ]]; then
      PROFILE_TO_RUN="usb0"
    elif [[ -e /dev/video1 ]]; then
      PROFILE_TO_RUN="usb1"
    elif [[ -n "$IMAGE_TOPIC" ]]; then
      PROFILE_TO_RUN="topic"
      TOPIC_TO_RUN="$IMAGE_TOPIC"
      warn "未找到 /dev/video0 或 /dev/video1，改用配置里的 IMAGE_TOPIC=$IMAGE_TOPIC"
    else
      die "未找到 USB 相机 /dev/video0 或 /dev/video1。若使用外部相机节点，请在配置中设置 CAMERA_PROFILE=topic 和 IMAGE_TOPIC。"
    fi
    ;;
  usb0)
    [[ -e /dev/video0 ]] || die "CAMERA_PROFILE=usb0，但 /dev/video0 不存在。可先执行: ls /dev/video*"
    ;;
  usb1)
    [[ -e /dev/video1 ]] || die "CAMERA_PROFILE=usb1，但 /dev/video1 不存在。可先执行: ls /dev/video*"
    ;;
  topic)
    [[ -n "$IMAGE_TOPIC" ]] || die "CAMERA_PROFILE=topic 需要设置 IMAGE_TOPIC，例如 /camera/camera/color/image_raw"
    PROFILE_TO_RUN="topic"
    TOPIC_TO_RUN="$IMAGE_TOPIC"
    ;;
  *)
    die "未知 CAMERA_PROFILE=$CAMERA_PROFILE，只支持 auto|usb0|usb1|topic"
    ;;
esac

info "启动配置:"
info "  profile=$PROFILE_TO_RUN"
if [[ "$PROFILE_TO_RUN" == "topic" ]]; then
  info "  image_topic=$TOPIC_TO_RUN"
elif [[ "$PROFILE_TO_RUN" == "usb0" ]]; then
  info "  usb_device=/dev/video0"
elif [[ "$PROFILE_TO_RUN" == "usb1" ]]; then
  info "  usb_device=/dev/video1"
else
  info "  usb_device=$PROFILE_TO_RUN"
fi
info "  active_tray_id=$ACTIVE_TRAY_ID"
info "  process_every_n_frames=$PROCESS_EVERY_N_FRAMES"
info "  f407=$F407_HOST:$F407_PORT"

print_quick_commands

export WORKSPACE_DIR ROS_SETUP ACTIVE_TRAY_ID PROCESS_EVERY_N_FRAMES F407_HOST F407_PORT

if [[ "${DEMO_DRY_RUN:-0}" == "1" ]]; then
  info "DEMO_DRY_RUN=1，只检查不启动。"
  if [[ "$PROFILE_TO_RUN" == "topic" ]]; then
    info "将执行: scripts/run_pingpong_demo.sh topic $TOPIC_TO_RUN"
  else
    info "将执行: scripts/run_pingpong_demo.sh $PROFILE_TO_RUN"
  fi
  exit 0
fi

if [[ "$PROFILE_TO_RUN" == "topic" ]]; then
  exec "$REPO_DIR/scripts/run_pingpong_demo.sh" topic "$TOPIC_TO_RUN"
fi

exec "$REPO_DIR/scripts/run_pingpong_demo.sh" "$PROFILE_TO_RUN"
