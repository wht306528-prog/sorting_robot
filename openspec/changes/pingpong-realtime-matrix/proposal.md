# Change: Ping-Pong Realtime Matrix

## Why

学长演示不能只看离线效果图。系统需要在 ROS2 端输出后续机械臂/F407 能使用的标准矩阵。

## What Changes

- Add realtime ping-pong recognition node.
- Subscribe to camera RGB image topic.
- Fit tray border, rectify tray, generate 10x5 hole centers, classify each hole.
- Publish human-readable debug image.
- Publish JSON matrix for debugging.
- Publish standard `/sorting/tray_matrix` for downstream driver/F407 testing.

## Current Scope

This change supports a single visible tray for ping-pong demonstration:

```text
active_tray_id = 1 by default
detected tray -> active_tray_id
other trays -> empty cells
```

The output still has 150 cells so the existing `matrix_tcp_sender` protocol path can validate it.

## Out of Scope

- Full three-tray recognition in one camera frame.
- True depth `z`.
- Camera coordinate conversion.
- Mechanical-arm coordinate conversion.
- Servo inverse kinematics and cylinder/actuator closed-loop control.

## Risks

- White ball classification can confuse light-colored empty-hole structures.
- Camera topic name may differ between machines.
- Real lighting and camera mounting may change thresholds.
- Matrix is classification-ready but not yet physically pick-coordinate-ready.

## Validation

Minimum validation:

```bash
cd /home/wht/sorting_robot/ros2_ws
source install/setup.sh
ros2 run sorting_vision pingpong_realtime_node
ros2 topic list
ros2 topic echo /sorting/tray_matrix
```

With camera connected:

- `/camera/camera/color/image_raw` must publish image frequency.
- `/sorting/pingpong/debug_image` must update.
- `/sorting/pingpong/cells_json` must include matrix fields.
- `/sorting/tray_matrix` must publish 150 `TrayCell` records.

