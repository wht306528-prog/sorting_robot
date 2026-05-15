#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/wht/sorting_robot/ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.sh}"
PROFILE="${1:-${PINGPONG_CAMERA_PROFILE:-usb0}}"
TOPIC_ARG="${2:-}"
START_CAMERA="${START_CAMERA:-}"
START_REALSENSE="${START_REALSENSE:-false}"
REALSENSE_COLOR_PROFILE="${REALSENSE_COLOR_PROFILE:-640x480x15}"
REALSENSE_DEPTH_PROFILE="${REALSENSE_DEPTH_PROFILE:-640x480x15}"
VIDEO_DEVICE="${VIDEO_DEVICE:-}"
IMAGE_TOPIC="${IMAGE_TOPIC:-}"
COLOR_CAMERA_INFO_TOPIC="${COLOR_CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
USE_DEPTH="${USE_DEPTH:-}"
USE_UNDISTORT="${USE_UNDISTORT:-true}"
DEPTH_WINDOW_PX="${DEPTH_WINDOW_PX:-5}"
EXPECTED_TRAY_COUNT="${EXPECTED_TRAY_COUNT:-3}"
PROCESS_EVERY_N_FRAMES="${PROCESS_EVERY_N_FRAMES:-3}"
GEOMETRY_METHOD="${GEOMETRY_METHOD:-large_dark_rect}"
SPLIT_WIDE_LARGE_DARK_RECTS="${SPLIT_WIDE_LARGE_DARK_RECTS:-true}"
LARGE_DARK_MAX_SINGLE_WIDTH_RATIO="${LARGE_DARK_MAX_SINGLE_WIDTH_RATIO:-0.36}"
RELAX_SPLIT_STRUCTURE="${RELAX_SPLIT_STRUCTURE:-true}"
GEOMETRY_DARK_THRESHOLD="${GEOMETRY_DARK_THRESHOLD:-85}"
GEOMETRY_PROJECTION_ACTIVE_RATIO="${GEOMETRY_PROJECTION_ACTIVE_RATIO:-0.10}"
GEOMETRY_PROJECTION_SMOOTH_PX="${GEOMETRY_PROJECTION_SMOOTH_PX:-19}"
GEOMETRY_MIN_WIDTH_RATIO="${GEOMETRY_MIN_WIDTH_RATIO:-0.08}"
GEOMETRY_MIN_HEIGHT_RATIO="${GEOMETRY_MIN_HEIGHT_RATIO:-0.35}"
GEOMETRY_MIN_AREA_RATIO="${GEOMETRY_MIN_AREA_RATIO:-0.025}"
GEOMETRY_X_PADDING_PX="${GEOMETRY_X_PADDING_PX:-10}"
GEOMETRY_MORPHOLOGY_KERNEL_PX="${GEOMETRY_MORPHOLOGY_KERNEL_PX:-17}"
GEOMETRY_EDGE_ROI_PADDING_PX="${GEOMETRY_EDGE_ROI_PADDING_PX:-12}"
DARK_THRESHOLD="${DARK_THRESHOLD:-95}"
CLOSE_KERNEL_RATIO="${CLOSE_KERNEL_RATIO:-0.045}"
MIN_AREA_RATIO="${MIN_AREA_RATIO:-0.16}"
SIDE_BAND_RATIO="${SIDE_BAND_RATIO:-0.075}"
ROI_RADIUS_RATIO="${ROI_RADIUS_RATIO:-0.34}"
MIN_WHITE_RATIO="${MIN_WHITE_RATIO:-0.36}"
MIN_WHITE_COMPONENT_RATIO="${MIN_WHITE_COMPONENT_RATIO:-0.30}"
MIN_WHITE_SHAPE_COMPONENT_RATIO="${MIN_WHITE_SHAPE_COMPONENT_RATIO:-0.18}"
MIN_WHITE_COMPONENT_DIAMETER_RATIO="${MIN_WHITE_COMPONENT_DIAMETER_RATIO:-0.95}"
MIN_WHITE_CIRCULARITY="${MIN_WHITE_CIRCULARITY:-0.35}"
MAX_WHITE_CENTER_OFFSET_RATIO="${MAX_WHITE_CENTER_OFFSET_RATIO:-0.55}"
MIN_BALL_RATIO="${MIN_BALL_RATIO:-0.16}"
MIN_YELLOW_COMPONENT_RATIO="${MIN_YELLOW_COMPONENT_RATIO:-0.12}"
MIN_COLOR_MARGIN="${MIN_COLOR_MARGIN:-0.035}"
START_TCP_SENDER="${START_TCP_SENDER:-true}"
F407_HOST="${F407_HOST:-127.0.0.1}"
F407_PORT="${F407_PORT:-9000}"

usage() {
  cat <<'EOF'
用法:
  scripts/run_pingpong_demo.sh usb0
  scripts/run_pingpong_demo.sh usb1
  scripts/run_pingpong_demo.sh realsense
  scripts/run_pingpong_demo.sh topic /your/image/topic

模式:
  usb0    启动 v4l2_camera，读取 /dev/video0，识别 /image_raw
  usb1    启动 v4l2_camera，读取 /dev/video1，识别 /image_raw
  realsense 启动 realsense2_camera，识别 D435iF RGB，并采样对齐深度
  topic   不启动相机驱动，直接订阅已有 ROS2 图像 topic

常用覆盖:
  EXPECTED_TRAY_COUNT=1|2|3
  PROCESS_EVERY_N_FRAMES=3
  GEOMETRY_METHOD=large_dark_rect
  SPLIT_WIDE_LARGE_DARK_RECTS=true|false
  LARGE_DARK_MAX_SINGLE_WIDTH_RATIO=0.36
  RELAX_SPLIT_STRUCTURE=true|false
  GEOMETRY_DARK_THRESHOLD=85
  GEOMETRY_PROJECTION_ACTIVE_RATIO=0.10
  GEOMETRY_PROJECTION_SMOOTH_PX=19
  GEOMETRY_MIN_WIDTH_RATIO=0.08
  GEOMETRY_MIN_HEIGHT_RATIO=0.35
  GEOMETRY_MIN_AREA_RATIO=0.025
  GEOMETRY_X_PADDING_PX=10
  GEOMETRY_MORPHOLOGY_KERNEL_PX=17
  GEOMETRY_EDGE_ROI_PADDING_PX=12
  DARK_THRESHOLD=95
  CLOSE_KERNEL_RATIO=0.045
  MIN_AREA_RATIO=0.16
  SIDE_BAND_RATIO=0.075
  ROI_RADIUS_RATIO=0.34
  F407_HOST=127.0.0.1
  F407_PORT=9000
  USE_DEPTH=false|true
  USE_UNDISTORT=false|true
  REALSENSE_COLOR_PROFILE=1280x720x30
  REALSENSE_DEPTH_PROFILE=848x480x30
  COLOR_CAMERA_INFO_TOPIC=/camera/camera/color/camera_info
  DEPTH_IMAGE_TOPIC=/camera/camera/aligned_depth_to_color/image_raw
  MIN_WHITE_RATIO=0.36
  MIN_WHITE_COMPONENT_RATIO=0.30
  MIN_WHITE_SHAPE_COMPONENT_RATIO=0.18
  MIN_WHITE_COMPONENT_DIAMETER_RATIO=0.95
  MIN_WHITE_CIRCULARITY=0.35
  MAX_WHITE_CENTER_OFFSET_RATIO=0.55
  MIN_BALL_RATIO=0.16
  MIN_YELLOW_COMPONENT_RATIO=0.12
  MIN_COLOR_MARGIN=0.035
EOF
}

case "$PROFILE" in
  usb0)
    # usb0/usb1 会启动 v4l2_camera，把普通 USB 摄像头转成 ROS2 图像 topic。
    START_CAMERA="${START_CAMERA:-true}"
    VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
    IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
    START_REALSENSE="false"
    USE_DEPTH="${USE_DEPTH:-false}"
    ;;
  usb1)
    START_CAMERA="${START_CAMERA:-true}"
    VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video1}"
    IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
    START_REALSENSE="false"
    USE_DEPTH="${USE_DEPTH:-false}"
    ;;
  realsense)
    # realsense 模式由 launch 直接启动 D435iF 驱动，不需要另开终端。
    START_CAMERA="false"
    START_REALSENSE="true"
    VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
    IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/camera/color/image_raw}"
    COLOR_CAMERA_INFO_TOPIC="${COLOR_CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
    DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
    USE_DEPTH="${USE_DEPTH:-true}"
    ;;
  topic)
    # topic 模式假设相机驱动已经在别处启动，本脚本只订阅指定图像 topic。
    if [[ -z "$TOPIC_ARG" && -z "$IMAGE_TOPIC" ]]; then
      usage
      echo "错误: topic 模式需要指定图像 topic，例如 /camera/camera/color/image_raw" >&2
      exit 2
    fi
    START_CAMERA="false"
    START_REALSENSE="false"
    if [[ -n "$TOPIC_ARG" ]]; then
      IMAGE_TOPIC="$TOPIC_ARG"
    fi
    USE_DEPTH="${USE_DEPTH:-false}"
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

# 这里使用 setup.sh，符合当前工作区实际安装结果；不要默认写 setup.bash。
# ROS2 的 setup 脚本内部会引用未定义变量，source 时需要临时关闭 nounset。
set +u
source "$ROS_SETUP"
cd "$WORKSPACE_DIR"
source install/setup.sh
set -u

# 打印最终启动参数，方便无显示器现场通过终端确认相机、深度和 F407 地址。
echo "pingpong demo profile: $PROFILE"
echo "start_camera=$START_CAMERA start_realsense=$START_REALSENSE video_device=$VIDEO_DEVICE image_topic=$IMAGE_TOPIC"
echo "realsense_color_profile目标值=$REALSENSE_COLOR_PROFILE realsense_depth_profile目标值=$REALSENSE_DEPTH_PROFILE"
echo "use_depth=$USE_DEPTH depth_image_topic=$DEPTH_IMAGE_TOPIC"
echo "use_undistort=$USE_UNDISTORT color_camera_info_topic=$COLOR_CAMERA_INFO_TOPIC"
echo "expected_tray_count=$EXPECTED_TRAY_COUNT start_tcp_sender=$START_TCP_SENDER tcp=$F407_HOST:$F407_PORT"
echo "geometry_method=$GEOMETRY_METHOD"
echo "split_wide_large_dark_rects=$SPLIT_WIDE_LARGE_DARK_RECTS large_dark_max_single_width_ratio=$LARGE_DARK_MAX_SINGLE_WIDTH_RATIO relax_split_structure=$RELAX_SPLIT_STRUCTURE"
echo "geometry_dark_threshold=$GEOMETRY_DARK_THRESHOLD geometry_projection_active_ratio=$GEOMETRY_PROJECTION_ACTIVE_RATIO geometry_projection_smooth_px=$GEOMETRY_PROJECTION_SMOOTH_PX"
echo "geometry_min_width_ratio=$GEOMETRY_MIN_WIDTH_RATIO geometry_min_height_ratio=$GEOMETRY_MIN_HEIGHT_RATIO geometry_min_area_ratio=$GEOMETRY_MIN_AREA_RATIO"
echo "geometry_x_padding_px=$GEOMETRY_X_PADDING_PX geometry_morphology_kernel_px=$GEOMETRY_MORPHOLOGY_KERNEL_PX geometry_edge_roi_padding_px=$GEOMETRY_EDGE_ROI_PADDING_PX"
echo "dark_threshold=$DARK_THRESHOLD close_kernel_ratio=$CLOSE_KERNEL_RATIO min_area_ratio=$MIN_AREA_RATIO side_band_ratio=$SIDE_BAND_RATIO"
echo "roi_radius_ratio=$ROI_RADIUS_RATIO"
echo "min_white_ratio=$MIN_WHITE_RATIO min_white_component_ratio=$MIN_WHITE_COMPONENT_RATIO"
echo "min_white_shape_component_ratio=$MIN_WHITE_SHAPE_COMPONENT_RATIO min_white_component_diameter_ratio=$MIN_WHITE_COMPONENT_DIAMETER_RATIO"
echo "min_white_circularity=$MIN_WHITE_CIRCULARITY max_white_center_offset_ratio=$MAX_WHITE_CENTER_OFFSET_RATIO"
echo "min_ball_ratio=$MIN_BALL_RATIO min_yellow_component_ratio=$MIN_YELLOW_COMPONENT_RATIO min_color_margin=$MIN_COLOR_MARGIN"
echo "实际 RGB/Depth 分辨率和内参会由 ROS 节点首次收到图像后打印"

# launch 同时启动：可选 USB 相机节点、乒乓球实时识别节点、矩阵 TCP 发送节点。
exec ros2 launch sorting_bringup pingpong_demo.launch.py \
  start_camera:="$START_CAMERA" \
  start_realsense:="$START_REALSENSE" \
  realsense_color_profile:="$REALSENSE_COLOR_PROFILE" \
  realsense_depth_profile:="$REALSENSE_DEPTH_PROFILE" \
  video_device:="$VIDEO_DEVICE" \
  image_topic:="$IMAGE_TOPIC" \
  color_camera_info_topic:="$COLOR_CAMERA_INFO_TOPIC" \
  depth_image_topic:="$DEPTH_IMAGE_TOPIC" \
  use_depth:="$USE_DEPTH" \
  use_undistort:="$USE_UNDISTORT" \
  depth_window_px:="$DEPTH_WINDOW_PX" \
  expected_tray_count:="$EXPECTED_TRAY_COUNT" \
  process_every_n_frames:="$PROCESS_EVERY_N_FRAMES" \
  geometry_method:="$GEOMETRY_METHOD" \
  split_wide_large_dark_rects:="$SPLIT_WIDE_LARGE_DARK_RECTS" \
  large_dark_max_single_width_ratio:="$LARGE_DARK_MAX_SINGLE_WIDTH_RATIO" \
  relax_split_structure:="$RELAX_SPLIT_STRUCTURE" \
  geometry_dark_threshold:="$GEOMETRY_DARK_THRESHOLD" \
  geometry_projection_active_ratio:="$GEOMETRY_PROJECTION_ACTIVE_RATIO" \
  geometry_projection_smooth_px:="$GEOMETRY_PROJECTION_SMOOTH_PX" \
  geometry_min_width_ratio:="$GEOMETRY_MIN_WIDTH_RATIO" \
  geometry_min_height_ratio:="$GEOMETRY_MIN_HEIGHT_RATIO" \
  geometry_min_area_ratio:="$GEOMETRY_MIN_AREA_RATIO" \
  geometry_x_padding_px:="$GEOMETRY_X_PADDING_PX" \
  geometry_morphology_kernel_px:="$GEOMETRY_MORPHOLOGY_KERNEL_PX" \
  geometry_edge_roi_padding_px:="$GEOMETRY_EDGE_ROI_PADDING_PX" \
  dark_threshold:="$DARK_THRESHOLD" \
  close_kernel_ratio:="$CLOSE_KERNEL_RATIO" \
  min_area_ratio:="$MIN_AREA_RATIO" \
  side_band_ratio:="$SIDE_BAND_RATIO" \
  roi_radius_ratio:="$ROI_RADIUS_RATIO" \
  min_white_ratio:="$MIN_WHITE_RATIO" \
  min_white_component_ratio:="$MIN_WHITE_COMPONENT_RATIO" \
  min_white_shape_component_ratio:="$MIN_WHITE_SHAPE_COMPONENT_RATIO" \
  min_white_component_diameter_ratio:="$MIN_WHITE_COMPONENT_DIAMETER_RATIO" \
  min_white_circularity:="$MIN_WHITE_CIRCULARITY" \
  max_white_center_offset_ratio:="$MAX_WHITE_CENTER_OFFSET_RATIO" \
  min_ball_ratio:="$MIN_BALL_RATIO" \
  min_yellow_component_ratio:="$MIN_YELLOW_COMPONENT_RATIO" \
  min_color_margin:="$MIN_COLOR_MARGIN" \
  start_tcp_sender:="$START_TCP_SENDER" \
  f407_host:="$F407_HOST" \
  f407_port:="$F407_PORT"
