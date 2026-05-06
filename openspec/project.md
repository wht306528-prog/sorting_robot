# Sorting Robot OpenSpec Project

## Goal

构建基于 ROS2 的苗盘/乒乓球分拣视觉与控制链路：

```text
camera image
  -> vision recognition
  -> tray matrix
  -> F407 communication
  -> mechanical sorting action
```

短期演示目标是用黄/白乒乓球代替两类苗，验证视觉识别到矩阵输出的 ROS2 链路。

## Current System

ROS2 workspace:

```text
/home/wht/sorting_robot/ros2_ws
```

Important packages:

- `sorting_interfaces`
  - Defines `TrayCell`, `TrayMatrix`, and related messages.
- `sorting_vision`
  - Camera probes, offline debug tools, tray geometry, ping-pong recognition, realtime node.
- `sorting_driver`
  - Converts `TrayMatrix` into text protocol and sends it to F407/W5500 over TCP.
- `sorting_control`
  - Early control/planner skeleton.
- `sorting_bringup`
  - Launch package skeleton.

## Current Vision Pipeline

```text
RGB image
  -> tray outer border fitting
  -> perspective rectification
  -> 10 x 5 hole center generation
  -> per-hole ping-pong color classification
  -> debug image, JSON, and TrayMatrix
```

Main realtime node:

```text
sorting_vision.nodes.pingpong_realtime_node
```

Topics:

```text
subscribe /camera/camera/color/image_raw
publish   /sorting/pingpong/debug_image
publish   /sorting/pingpong/cells_json
publish   /sorting/tray_matrix
```

## Project Conventions

- Communicate and explain in Chinese unless user asks otherwise.
- Commit messages use `English summary 中文说明`.
- Debug outputs go under `samples/debug_/runs/<run_name>/`.
- Do not claim production readiness from debug images alone.

## Matrix Contract

Standard output for downstream control is:

```text
/sorting/tray_matrix
```

Message:

```text
sorting_interfaces/msg/TrayMatrix
```

Required content:

```text
150 TrayCell records
3 trays
10 rows per tray
5 cols per tray
```

Coordinate convention:

```text
tray_id=1 left tray
tray_id=2 middle tray
tray_id=3 right tray
row increases downward
col increases rightward
```

Current ping-pong class mapping:

```text
0 empty
1 white_ball
2 yellow_ball
```

## Known Gaps

- Full three-tray live recognition is not complete.
- Ping-pong realtime node currently writes one detected tray into `active_tray_id` and fills the other two trays as empty.
- Depth `z`, camera coordinates, mechanical-arm coordinates, and servo angles are not complete.
- F407 TCP link must be tested with real receiver or simulator.

