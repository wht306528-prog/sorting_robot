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
  CAMERA_PROFILE=auto|usb0|usb1|realsense|topic
  IMAGE_TOPIC=/your/image/topic
  COLOR_CAMERA_INFO_TOPIC=/your/color/camera_info
  REALSENSE_COLOR_PROFILE=1280x720x30
  REALSENSE_DEPTH_PROFILE=848x480x30
  USE_DEPTH=auto|true|false
  USE_UNDISTORT=true|false
  DEPTH_IMAGE_TOPIC=/your/aligned_depth/topic
  F407_HOST=192.168.1.50
  F407_PORT=9000
  CHECK_F407=1
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

3. 查看相机设备:
   ls /dev/video*
   v4l2-ctl --list-devices
   rs-enumerate-devices

4. 查看运行中的 ROS2 输出:
   source /opt/ros/humble/setup.sh
   source $WORKSPACE_DIR/install/setup.sh
   ros2 topic list
   ros2 topic hz /image_raw
   ros2 topic hz $DEPTH_IMAGE_TOPIC
   ros2 topic echo /sorting/tray_matrix

5. 修改真实 F407/W5500 地址:
   nano config/pingpong_demo.env
   # 修改 F407_HOST 和 F407_PORT

6. 当前将连接:
   F407_HOST=$F407_HOST
   F407_PORT=$F407_PORT
==================================

EOF
}

print_startup_summary() {
  local camera_line
  case "$PROFILE_TO_RUN" in
    topic)
      camera_line="订阅已有图像 topic: $TOPIC_TO_RUN"
      ;;
    realsense)
      camera_line="D435iF / RealSense"
      ;;
    usb0)
      camera_line="普通 USB 相机: /dev/video0"
      ;;
    usb1)
      camera_line="普通 USB 相机: /dev/video1"
      ;;
    *)
      camera_line="$PROFILE_TO_RUN"
      ;;
  esac

  cat <<EOF

========== 乒乓球演示启动配置 ==========
相机:
  profile: $PROFILE_TO_RUN
  设备/来源: $camera_line
  RGB topic: $IMAGE_TOPIC
  RGB CameraInfo topic: $COLOR_CAMERA_INFO_TOPIC
  RealSense RGB 目标值: $REALSENSE_COLOR_PROFILE
  RealSense Depth 目标值: $REALSENSE_DEPTH_PROFILE
  提示: 目标值不一定等于真实运行值；真实值会在 ROS 节点首次收到图像后打印。

深度与去畸变:
  use_depth: $USE_DEPTH_TO_RUN
  depth topic: $DEPTH_IMAGE_TOPIC
  depth_window_px: $DEPTH_WINDOW_PX
  use_undistort: $USE_UNDISTORT

识别:
  expected_tray_count: $EXPECTED_TRAY_COUNT
  process_every_n_frames: $PROCESS_EVERY_N_FRAMES
  split_wide_large_dark_rects: $SPLIT_WIDE_LARGE_DARK_RECTS
  large_dark_max_single_width_ratio: $LARGE_DARK_MAX_SINGLE_WIDTH_RATIO
  relax_split_structure: $RELAX_SPLIT_STRUCTURE
  min_white_ratio: $MIN_WHITE_RATIO
  min_white_component_ratio: $MIN_WHITE_COMPONENT_RATIO
  min_white_shape_component_ratio: $MIN_WHITE_SHAPE_COMPONENT_RATIO
  min_white_circularity: $MIN_WHITE_CIRCULARITY
  max_white_center_offset_ratio: $MAX_WHITE_CENTER_OFFSET_RATIO

F407/TCP:
  host: $F407_HOST
  port: $F407_PORT
  check_f407: $CHECK_F407
========================================

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

topic_exists() {
  local topic="$1"
  local topics
  if ! topics="$(timeout 4 ros2 topic list --no-daemon 2>/dev/null)"; then
    die "无法读取 ROS2 topic 列表。请确认 ROS2 环境可用、相机驱动已启动，或稍后重试。"
  fi
  grep -Fxq "$topic" <<<"$topics"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  usage
  exit 0
fi

# 优先读取现场配置文件；没有配置文件时只用默认值，方便首次试运行。
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
USE_DEPTH="${USE_DEPTH:-auto}"
DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
COLOR_CAMERA_INFO_TOPIC="${COLOR_CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
USE_UNDISTORT="${USE_UNDISTORT:-true}"
REALSENSE_COLOR_PROFILE="${REALSENSE_COLOR_PROFILE:-640x480x15}"
REALSENSE_DEPTH_PROFILE="${REALSENSE_DEPTH_PROFILE:-640x480x15}"
DEPTH_WINDOW_PX="${DEPTH_WINDOW_PX:-5}"
EXPECTED_TRAY_COUNT="${EXPECTED_TRAY_COUNT:-3}"
PROCESS_EVERY_N_FRAMES="${PROCESS_EVERY_N_FRAMES:-3}"
SPLIT_WIDE_LARGE_DARK_RECTS="${SPLIT_WIDE_LARGE_DARK_RECTS:-true}"
LARGE_DARK_MAX_SINGLE_WIDTH_RATIO="${LARGE_DARK_MAX_SINGLE_WIDTH_RATIO:-0.36}"
RELAX_SPLIT_STRUCTURE="${RELAX_SPLIT_STRUCTURE:-true}"
MIN_WHITE_RATIO="${MIN_WHITE_RATIO:-0.36}"
MIN_WHITE_COMPONENT_RATIO="${MIN_WHITE_COMPONENT_RATIO:-0.30}"
MIN_WHITE_SHAPE_COMPONENT_RATIO="${MIN_WHITE_SHAPE_COMPONENT_RATIO:-0.18}"
MIN_WHITE_CIRCULARITY="${MIN_WHITE_CIRCULARITY:-0.35}"
MAX_WHITE_CENTER_OFFSET_RATIO="${MAX_WHITE_CENTER_OFFSET_RATIO:-0.55}"
MIN_YELLOW_COMPONENT_RATIO="${MIN_YELLOW_COMPONENT_RATIO:-0.12}"
F407_HOST="${F407_HOST:-}"
F407_PORT="${F407_PORT:-9000}"
CHECK_F407="${CHECK_F407:-1}"

# 基础文件检查先做完，避免 launch 起来后才发现工作区或脚本缺失。
[[ -f "$ROS_SETUP" ]] || die "找不到 ROS2 环境: $ROS_SETUP"
[[ -d "$WORKSPACE_DIR" ]] || die "找不到 ROS2 工作区: $WORKSPACE_DIR"
[[ -f "$WORKSPACE_DIR/install/setup.sh" ]] || die "工作区还没有 build 或 install/setup.sh 不存在。请先执行: cd $WORKSPACE_DIR && colcon build"
[[ -x "$REPO_DIR/scripts/run_pingpong_demo.sh" ]] || die "启动脚本不可执行: $REPO_DIR/scripts/run_pingpong_demo.sh"

# 真实演示默认要求填 F407/W5500 地址，防止把矩阵发到本机模拟地址。
if [[ "$CHECK_F407" == "1" ]]; then
  [[ -n "$F407_HOST" ]] || die "未配置 F407_HOST。请先执行: cp config/pingpong_demo.env.example config/pingpong_demo.env，然后把 F407_HOST 改成真实 F407/W5500 IP。"
  [[ "$F407_HOST" != "127.0.0.1" && "$F407_HOST" != "localhost" ]] || die "当前 F407_HOST=$F407_HOST，这是本机地址，不适合真实硬件测试。请在 config/pingpong_demo.env 中填写 F407/W5500 的真实 IP。"
fi

# 这里先加载 ROS，只用于启动前检查 topic；真正启动时 run_pingpong_demo.sh 会再次 source。
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WORKSPACE_DIR/install/setup.sh"
set -u

PROFILE_TO_RUN="$CAMERA_PROFILE"
TOPIC_TO_RUN=""
USE_DEPTH_TO_RUN="$USE_DEPTH"

# 相机 profile 负责决定启动普通 USB、D435iF/RealSense，还是订阅外部已有图像 topic。
case "$CAMERA_PROFILE" in
  auto)
    if command -v rs-enumerate-devices >/dev/null 2>&1 && timeout 3 rs-enumerate-devices 2>/dev/null | grep -Eq 'D4|RealSense|Intel'; then
      PROFILE_TO_RUN="realsense"
      IMAGE_TOPIC="/camera/camera/color/image_raw"
      COLOR_CAMERA_INFO_TOPIC="/camera/camera/color/camera_info"
      USE_DEPTH_TO_RUN="true"
    elif [[ -e /dev/video0 ]]; then
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
  realsense)
    command -v ros2 >/dev/null 2>&1 || die "找不到 ros2 命令，请检查 ROS2 环境。"
    ros2 pkg prefix realsense2_camera >/dev/null 2>&1 || die "CAMERA_PROFILE=realsense 需要安装 realsense2_camera，例如 ros-humble-realsense2-camera。"
    IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/camera/color/image_raw}"
    COLOR_CAMERA_INFO_TOPIC="${COLOR_CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
    DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
    PROFILE_TO_RUN="realsense"
    ;;
  topic)
    [[ -n "$IMAGE_TOPIC" ]] || die "CAMERA_PROFILE=topic 需要设置 IMAGE_TOPIC，例如 /camera/camera/color/image_raw"
    PROFILE_TO_RUN="topic"
    TOPIC_TO_RUN="$IMAGE_TOPIC"
    ;;
  *)
    die "未知 CAMERA_PROFILE=$CAMERA_PROFILE，只支持 auto|usb0|usb1|realsense|topic"
    ;;
esac

# 深度输入自适应：USB 普通相机默认 z=0；D435iF/RealSense 默认启用对齐深度。
case "$USE_DEPTH_TO_RUN" in
  auto)
    if [[ "$PROFILE_TO_RUN" == "realsense" ]]; then
      USE_DEPTH_TO_RUN="true"
    elif [[ "$PROFILE_TO_RUN" == "topic" && -n "$DEPTH_IMAGE_TOPIC" ]]; then
      USE_DEPTH_TO_RUN="true"
    else
      USE_DEPTH_TO_RUN="false"
    fi
    ;;
  true|false)
    ;;
  *)
    die "未知 USE_DEPTH=$USE_DEPTH，只支持 auto|true|false"
    ;;
esac

# topic 模式下不负责启动相机，所以必须先确认图像 topic 已经存在。
if [[ "$PROFILE_TO_RUN" == "topic" ]]; then
  if ! topic_exists "$TOPIC_TO_RUN"; then
    die "未发现 RGB 图像 topic: $TOPIC_TO_RUN。请先启动相机驱动，或修改 IMAGE_TOPIC。"
  fi
fi

# 启用深度时，必须使用已经对齐到 RGB 的深度图，否则像素坐标和 z 对不上。
# realsense 模式的 topic 会由本脚本启动的驱动发布，所以启动前不检查 topic 是否已存在。
if [[ "$USE_DEPTH_TO_RUN" == "true" && "$PROFILE_TO_RUN" != "realsense" ]]; then
  if ! topic_exists "$DEPTH_IMAGE_TOPIC"; then
    die "USE_DEPTH=true，但未发现深度 topic: $DEPTH_IMAGE_TOPIC。请使用对齐到 RGB 的深度 topic，或设置 USE_DEPTH=false。"
  fi
fi

print_startup_summary

print_quick_commands

export WORKSPACE_DIR ROS_SETUP EXPECTED_TRAY_COUNT PROCESS_EVERY_N_FRAMES F407_HOST F407_PORT
export SPLIT_WIDE_LARGE_DARK_RECTS LARGE_DARK_MAX_SINGLE_WIDTH_RATIO RELAX_SPLIT_STRUCTURE
export MIN_WHITE_RATIO MIN_WHITE_COMPONENT_RATIO MIN_YELLOW_COMPONENT_RATIO
export MIN_WHITE_SHAPE_COMPONENT_RATIO MIN_WHITE_CIRCULARITY MAX_WHITE_CENTER_OFFSET_RATIO
export USE_DEPTH="$USE_DEPTH_TO_RUN" DEPTH_IMAGE_TOPIC DEPTH_WINDOW_PX
export USE_UNDISTORT COLOR_CAMERA_INFO_TOPIC
export REALSENSE_COLOR_PROFILE REALSENSE_DEPTH_PROFILE
if [[ "$CHECK_F407" == "1" ]]; then
  export START_TCP_SENDER="true"
else
  export START_TCP_SENDER="false"
fi

# 干跑模式只打印最终会执行什么，适合现场先核对配置。
if [[ "${DEMO_DRY_RUN:-0}" == "1" ]]; then
  info "DEMO_DRY_RUN=1，只检查不启动。"
  if [[ "$PROFILE_TO_RUN" == "topic" ]]; then
    info "将执行: scripts/run_pingpong_demo.sh topic $TOPIC_TO_RUN"
  else
    info "将执行: scripts/run_pingpong_demo.sh $PROFILE_TO_RUN"
  fi
  exit 0
fi

# 正式演示前先探测 F407/W5500 TCP 端口，地址错会在这里直接报错。
if [[ "$CHECK_F407" == "1" ]]; then
  info "正在检查 F407/W5500 TCP 连接: $F407_HOST:$F407_PORT"
  if ! timeout 2 bash -c "cat < /dev/null > /dev/tcp/$F407_HOST/$F407_PORT" 2>/dev/null; then
    die "无法连接 F407/W5500: $F407_HOST:$F407_PORT。请检查 IP、端口、网线/网络、F407 程序是否已启动监听。"
  fi
  info "F407/W5500 TCP 连接检查通过。"
fi

# 最后交给低层脚本统一启动 launch，避免两处维护 ros2 launch 参数。
if [[ "$PROFILE_TO_RUN" == "topic" ]]; then
  exec "$REPO_DIR/scripts/run_pingpong_demo.sh" topic "$TOPIC_TO_RUN"
fi

exec "$REPO_DIR/scripts/run_pingpong_demo.sh" "$PROFILE_TO_RUN"
