---
name: sorting-robot-project
description: 在 /home/wht/sorting_robot 内工作，或讨论苗盘/乒乓球分拣机器人、ROS2 节点、鲁班猫部署、F407 通信、矩阵输出、视觉调试、git 提交和项目进度时使用。它保存本项目约定、当前架构、已知基线和防误操作规则。
---

# 分拣机器人项目 Skill

## 不可违反的协作规则

- 默认用中文沟通。表达要直接、准确、耐心，不要夸大完成度。
- 不要把调试可视化当成最终识别。必须区分：
  - `调试基线`：便于检查，不等于生产可用。
  - `离线流程`：能处理保存图片。
  - `实时 ROS2 节点`：能处理实时图像 topic。
  - `F407/机械臂可联调`：已经发布下游期望的标准矩阵/控制接口。
- 不要随便替换当前可接受的基线。新算法改动应隔离为新 run 或小范围补丁。
- 用户情绪激动时，先回答具体担忧，再继续做事。
- 用户讨厌隐藏假设和自作主张。如果项目已有约定，必须遵守。

## Git 规则

- 提交信息必须符合项目现有风格：
  - `English summary 中文说明`
  - 示例：`Publish pingpong TrayMatrix 发布乒乓球标准矩阵`
- 提交前运行 `git status --short`，不要暂存无关文件。
- 未经用户明确同意，不要改写历史。如果要改写本地未推送历史，必须说清楚。
- 除非用户明确要求，不要使用破坏性 git 命令。

## 调试输出规则

- 新调试输出放在 `samples/debug_/runs/<run_name>/`。
- 最重要的可视化汇总放在 `samples/debug_/runs/<run_name>/final_effects/`。
- 多样本输出尽量包含 `_all_*.jpg` 总览图。
- `samples/` 被 git 忽略；除非用户明确要求，不要删除原始样本。
- 生成有意义的调试 run 后，使用 `python3 scripts/archive_debug_outputs.py` 归档。

## 项目阶段边界

项目有两个不同阶段，不要随便混在一起说：

- 当前近期演示阶段：识别并吸取乒乓球。目标是穴位里的黄/白乒乓球，下游重点是给 F407 吸取测试提供可靠的矩阵/TCP 接口。
- 最终项目阶段：识别并夹取苗盘作物。目标是作物/幼苗，下游动作是机械夹取或作物处理。

讨论算法、坐标、末端执行器或完成度时，必须说明对应哪个阶段。不能把乒乓球吸取完成说成最终作物夹取完成。

## 当前视觉架构

当前离线/实时视觉流程：

```text
RGB image
  -> 苗盘外框拟合
  -> 透视矫正
  -> 10 x 5 穴位网格
  -> 单穴位乒乓球分类
  -> debug image / JSON / TrayMatrix
```

重要文件：

- `sorting_vision/algorithms/tray_edge_fit.py`：单盘外框拟合和透视矫正。
- `sorting_vision/algorithms/tray_hole_grid.py`：矫正图中的 10x5 穴位中心生成。
- `sorting_vision/algorithms/pingpong_detector.py`：`empty / white_ball / yellow_ball` 的 OpenCV 基线。
- `sorting_vision/debug_tools/pingpong_full_image_debug.py`：离线整图乒乓球调试流程。
- `sorting_vision/nodes/pingpong_realtime_node.py`：订阅相机 topic 并发布矩阵的实时 ROS2 节点。

## 乒乓球类别映射

当前乒乓球演示阶段：

```text
0 = empty
1 = white_ball
2 = yellow_ball
```

如果用户或学长定义了不同的颜色到类别映射，必须立即更新并明确说明。

当前识别是 OpenCV 规则基线，不是训练后的 YOLO：

- 黄色主要依赖 HSV 黄色区域和连通域检查，相对更稳。
- 白色更难，因为空穴、反光或浅色结构可能干扰。
- 不要声称识别完美。没有现场验证前，只能说是基线。

## ROS2 运行状态

主实时节点：

```bash
ros2 run sorting_vision pingpong_realtime_node
```

默认输入：

```text
/camera/camera/color/image_raw
```

输出：

```text
/sorting/pingpong/debug_image
/sorting/pingpong/cells_json
/sorting/tray_matrix
```

`/sorting/pingpong/cells_json` 包含：

- `matrices_by_tray`：按 tray_id 组织的 10x5 字符串矩阵。
- `matrix_ids_by_tray`：按 tray_id 组织的 10x5 数字矩阵。
- `cells`：每个穴位的 row/col/class/confidence 等信息。

`/sorting/tray_matrix` 是下游标准接口：

- 消息类型：`sorting_interfaces/msg/TrayMatrix`。
- 固定包含 150 条 `TrayCell`。
- 当前乒乓球实时节点应从整帧中最多检测 3 个可见盘，按从左到右分配 tray_id，并把每个检测到的盘填入真实识别结果。
- 只有未检测到的盘才能补 empty，以保持下游固定 150 格格式。
- 不能再把单 active-tray 输出描述成当前乒乓球吸取阶段足够的识别方案。

已知限制：

- 当前乒乓球矩阵有分类和像素中心。
- 普通 RGB/USB 相机下 `z=0`；RGBD 且使用对齐到 RGB 的深度 topic 时，`z` 可写入相机深度。
- `z` 仍是相机深度，不是机械臂坐标。
- 吸取/夹取动作仍需要相机到执行机构的标定和坐标转换。

## 构建和 source 说明

本工作区可能提供的是 `install/setup.sh`，不是 `install/setup.bash`。

使用：

```bash
cd /home/wht/sorting_robot/ros2_ws
source install/setup.sh
```

在本环境构建视觉包：

```bash
colcon build --paths src/sorting_vision
```

如果 `ros2 run` 找不到包，先检查是否 source 了 install setup 文件。

如果 shell 里找不到 `rqt_image_view`，先 source ROS 基础环境：

```bash
source /opt/ros/humble/setup.sh
source /home/wht/sorting_robot/ros2_ws/install/setup.sh
```

## 矩阵和 F407 接口

当前下游链路：

```text
/sorting/tray_matrix
  -> sorting_driver.matrix_tcp_sender
  -> TCP
  -> F407 / W5500
```

TCP 发送端期望固定 150 格：

```text
3 trays * 10 rows * 5 cols = 150
```

盘编号：

```text
tray_id=1 左侧盘
tray_id=2 中间盘
tray_id=3 右侧盘
```

格子编号：

```text
row 1 = 顶部
row 10 = 底部
col 1 = 左侧
col 5 = 右侧
```

这与用户文档和现有 `TrayCell.msg` 保持一致。

## 回答“现在到哪了”时

使用下面的状态框架：

- 离线乒乓球整图识别：作为演示基线可用。
- 实时 ROS2 节点：可以启动并发布 topic。
- 相机实时数据：只有连接相机并确认图像 topic 有频率后才算验证。
- 标准矩阵输出：通过 `/sorting/tray_matrix` 发布。
- 完整机械臂可用：必须等深度/相机标定/机械坐标转换/F407 指令集成验证后才能说完成。

不要因为视觉节点能启动，就说机械臂任务已经完成。
