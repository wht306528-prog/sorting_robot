#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/wht/sorting_robot/ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.sh}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
ACTIVE_TRAY_ID="${ACTIVE_TRAY_ID:-1}"
PROCESS_EVERY_N_FRAMES="${PROCESS_EVERY_N_FRAMES:-3}"
F407_HOST="${F407_HOST:-127.0.0.1}"
F407_PORT="${F407_PORT:-9000}"

source "$ROS_SETUP"
cd "$WORKSPACE_DIR"
source install/setup.sh

exec ros2 launch sorting_bringup pingpong_demo.launch.py \
  video_device:="$VIDEO_DEVICE" \
  image_topic:="$IMAGE_TOPIC" \
  active_tray_id:="$ACTIVE_TRAY_ID" \
  process_every_n_frames:="$PROCESS_EVERY_N_FRAMES" \
  f407_host:="$F407_HOST" \
  f407_port:="$F407_PORT"
