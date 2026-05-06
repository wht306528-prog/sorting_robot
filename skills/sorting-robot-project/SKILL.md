---
name: sorting-robot-project
description: Use this skill whenever working in /home/wht/sorting_robot or discussing the seedling tray / ping-pong ball sorting robot project, ROS2 nodes, LubanCat deployment, F407 communication, tray matrix outputs, vision debug runs, git commits, or project status. It preserves project-specific conventions, current architecture, known baselines, and anti-footgun rules from prior collaboration.
---

# Sorting Robot Project Skill

## Non-Negotiable Collaboration Rules

- Communicate in Chinese by default. Be direct, factual, patient, and do not overclaim.
- Do not treat debug visualization as final recognition. Always distinguish:
  - `debug baseline`: useful for inspection, not production.
  - `offline workflow`: works on saved images.
  - `realtime ROS2 node`: works on live image topics.
  - `F407/mechanical-arm ready`: publishes the standard matrix/control interface expected downstream.
- Do not casually replace a currently acceptable baseline. New algorithm changes should be isolated as a new run or small scoped patch.
- When the user is frustrated, answer the exact concern first, then continue work.
- The user dislikes hidden assumptions and self-directed style drift. If a convention is known, follow it.

## Git Rules

- Commit messages must follow the existing project style:
  - `English summary 中文说明`
  - Example: `Publish pingpong TrayMatrix 发布乒乓球标准矩阵`
- Before committing, run `git status --short` and avoid staging unrelated files.
- Do not rewrite history unless the user explicitly agrees. If rewriting local unpushed history, say so clearly.
- Never use destructive git commands unless explicitly requested.

## Debug Output Rules

- Put new debug outputs under `samples/debug_/runs/<run_name>/`.
- Put the most useful visual summaries under `samples/debug_/runs/<run_name>/final_effects/`.
- For multi-sample output, include an `_all_*.jpg` overview whenever practical.
- `samples/` is ignored by git; do not delete raw samples unless explicitly asked.
- Use `python3 scripts/archive_debug_outputs.py` after generating meaningful debug runs.

## Current Vision Architecture

Current offline/realtime vision pipeline:

```text
RGB image
  -> tray outer border fit
  -> perspective rectification
  -> 10 x 5 hole grid
  -> per-hole ping-pong classifier
  -> debug image / JSON / TrayMatrix
```

Important files:

- `sorting_vision/algorithms/tray_edge_fit.py`: single-tray outer border fitting and rectification.
- `sorting_vision/algorithms/tray_hole_grid.py`: rectified tray 10x5 hole center generation.
- `sorting_vision/algorithms/pingpong_detector.py`: OpenCV baseline for `empty / white_ball / yellow_ball`.
- `sorting_vision/debug_tools/pingpong_full_image_debug.py`: one-command offline whole-image ping-pong workflow.
- `sorting_vision/nodes/pingpong_realtime_node.py`: realtime ROS2 node for camera topic input and matrix output.

## Ping-Pong Class Mapping

For the current demo with ping-pong balls:

```text
0 = empty
1 = white_ball
2 = yellow_ball
```

If the user or senior student defines the color-to-class mapping differently, update the mapping immediately and call it out.

Current recognition is OpenCV-based, not trained YOLO:

- Yellow is relatively reliable using HSV yellow area and connected component checks.
- White is harder because empty tray holes can contain light-colored circular structures.
- Do not claim perfect recognition. Say it is a baseline unless tested live.

## ROS2 Runtime State

Main realtime node:

```bash
ros2 run sorting_vision pingpong_realtime_node
```

Default input:

```text
/camera/camera/color/image_raw
```

Outputs:

```text
/sorting/pingpong/debug_image
/sorting/pingpong/cells_json
/sorting/tray_matrix
```

`/sorting/pingpong/cells_json` includes:

- `matrix`: 10x5 string matrix.
- `matrix_ids`: 10x5 numeric matrix.
- `cells`: per-cell row/col/class/confidence.

`/sorting/tray_matrix` is the standard downstream interface:

- Message type: `sorting_interfaces/msg/TrayMatrix`.
- Contains 150 `TrayCell` records.
- Current ping-pong realtime node writes the detected single tray into configurable `active_tray_id` and fills the other trays as empty.
- This is valid for interface testing, but not full three-tray recognition yet.

Known limitation:

- Current ping-pong matrix has classification and rectified pixel centers.
- It does not yet provide true depth `z` or camera/mechanical-arm coordinates.
- Mechanical gripping still needs depth alignment, calibration, and coordinate conversion.

## Build and Source Notes

This workspace may provide `install/setup.sh`, not `install/setup.bash`.

Use:

```bash
cd /home/wht/sorting_robot/ros2_ws
source install/setup.sh
```

Build the vision package in this environment with:

```bash
colcon build --paths src/sorting_vision
```

If `ros2 run` cannot find a package, first check whether the install setup file was sourced.

If `rqt_image_view` is missing from shell, source ROS base first:

```bash
source /opt/ros/humble/setup.sh
source /home/wht/sorting_robot/ros2_ws/install/setup.sh
```

## Matrix and F407 Interface

Existing downstream chain:

```text
/sorting/tray_matrix
  -> sorting_driver.matrix_tcp_sender
  -> TCP
  -> F407 / W5500
```

The TCP sender expects exactly 150 cells:

```text
3 trays * 10 rows * 5 cols = 150
```

Tray numbering:

```text
tray_id=1 left tray
tray_id=2 middle tray
tray_id=3 right tray
```

Grid numbering:

```text
row 1 = top
row 10 = bottom
col 1 = left
col 5 = right
```

This matches the user document and existing `TrayCell.msg`.

## When Answering “Where Are We?”

Use this status framing:

- Offline ping-pong whole-image recognition: works as a demo baseline.
- Realtime ROS2 node: starts and publishes topics.
- Camera live data: only verified when a camera is connected and `/camera/camera/color/image_raw` has message frequency.
- Standard matrix output: present through `/sorting/tray_matrix`.
- Full mechanical-arm readiness: not complete until depth/camera calibration/mechanical coordinate conversion and F407 command integration are verified.

Never say the mechanical-arm task is finished just because the vision node starts.
