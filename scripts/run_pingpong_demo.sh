#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/wht/sorting_robot/ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.sh}"
PROFILE="${1:-${PINGPONG_CAMERA_PROFILE:-usb0}}"
TOPIC_ARG="${2:-}"
START_CAMERA="${START_CAMERA:-}"
VIDEO_DEVICE="${VIDEO_DEVICE:-}"
IMAGE_TOPIC="${IMAGE_TOPIC:-}"
ACTIVE_TRAY_ID="${ACTIVE_TRAY_ID:-1}"
PROCESS_EVERY_N_FRAMES="${PROCESS_EVERY_N_FRAMES:-3}"
F407_HOST="${F407_HOST:-127.0.0.1}"
F407_PORT="${F407_PORT:-9000}"

usage() {
  cat <<'EOF'
用法:
  scripts/run_pingpong_demo.sh usb0
  scripts/run_pingpong_demo.sh usb1
  scripts/run_pingpong_demo.sh topic /your/image/topic

模式:
  usb0    启动 v4l2_camera，读取 /dev/video0，识别 /image_raw
  usb1    启动 v4l2_camera，读取 /dev/video1，识别 /image_raw
  topic   不启动相机驱动，直接订阅已有 ROS2 图像 topic

常用覆盖:
  ACTIVE_TRAY_ID=1|2|3
  PROCESS_EVERY_N_FRAMES=3
  F407_HOST=127.0.0.1
  F407_PORT=9000
EOF
}

case "$PROFILE" in
  usb0)
    START_CAMERA="${START_CAMERA:-true}"
    VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
    IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
    ;;
  usb1)
    START_CAMERA="${START_CAMERA:-true}"
    VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video1}"
    IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
    ;;
  topic)
    if [[ -z "$TOPIC_ARG" && -z "$IMAGE_TOPIC" ]]; then
      usage
      echo "错误: topic 模式需要指定图像 topic，例如 /camera/camera/color/image_raw" >&2
      exit 2
    fi
    START_CAMERA="false"
    if [[ -n "$TOPIC_ARG" ]]; then
      IMAGE_TOPIC="$TOPIC_ARG"
    fi
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    usage
    echo "错误: 未知模式 '$PROFILE'" >&2
    exit 2
    ;;
esac

source "$ROS_SETUP"
cd "$WORKSPACE_DIR"
source install/setup.sh

echo "pingpong demo profile: $PROFILE"
echo "start_camera=$START_CAMERA video_device=$VIDEO_DEVICE image_topic=$IMAGE_TOPIC"
echo "tray=$ACTIVE_TRAY_ID tcp=$F407_HOST:$F407_PORT"

exec ros2 launch sorting_bringup pingpong_demo.launch.py \
  start_camera:="$START_CAMERA" \
  video_device:="$VIDEO_DEVICE" \
  image_topic:="$IMAGE_TOPIC" \
  active_tray_id:="$ACTIVE_TRAY_ID" \
  process_every_n_frames:="$PROCESS_EVERY_N_FRAMES" \
  f407_host:="$F407_HOST" \
  f407_port:="$F407_PORT"
