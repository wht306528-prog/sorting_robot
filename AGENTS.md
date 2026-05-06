# Sorting Robot Agent Instructions

本文件给所有 AI coding agent 使用。进入本仓库后，先遵守这里的项目规则。

## 沟通与判断

- 默认用中文和用户沟通。
- 不要把 debug 图、离线脚本、实时节点、机械臂闭环混为一谈。
- 不要自作主张替换已经能接受的视觉 baseline；新算法改动先做成独立实验或小范围补丁。
- 遇到用户质疑时，先回答质疑本身，再继续工作。

## Git 规则

- 提交信息使用双语格式：
  - `English summary 中文说明`
  - 示例：`Publish pingpong TrayMatrix 发布乒乓球标准矩阵`
- 提交前运行：
  - `git status --short`
- 不要提交 `samples/` 下的调试输出。
- 不要重写历史，除非用户明确同意。

## Debug 输出

- 新 debug run 放到：
  - `samples/debug_/runs/<run_name>/`
- 汇总图放到：
  - `samples/debug_/runs/<run_name>/final_effects/`
- 多样本结果尽量生成 `_all_*.jpg` 总览图。
- 生成重要 debug 输出后运行：
  - `python3 scripts/archive_debug_outputs.py`

## ROS2 运行约定

工作区可能只有 `install/setup.sh`，不要默认写 `setup.bash`。

常用命令：

```bash
cd /home/wht/sorting_robot/ros2_ws
source install/setup.sh
colcon build --paths src/sorting_vision
```

实时乒乓球节点：

```bash
ros2 run sorting_vision pingpong_realtime_node
```

默认订阅：

```text
/camera/camera/color/image_raw
```

默认发布：

```text
/sorting/pingpong/debug_image
/sorting/pingpong/cells_json
/sorting/tray_matrix
```

## 矩阵接口

`/sorting/tray_matrix` 是后续 F407/TCP/机械臂链路的标准接口，不是只给人看的调试 JSON。

消息类型：

```text
sorting_interfaces/msg/TrayMatrix
```

矩阵约定：

```text
3 trays * 10 rows * 5 cols = 150 TrayCell
tray_id=1 左盘
tray_id=2 中盘
tray_id=3 右盘
row=1 顶行，row=10 底行
col=1 左列，col=5 右列
```

乒乓球演示类别：

```text
0 = empty
1 = white_ball
2 = yellow_ball
```

当前限制必须说明：

- 乒乓球实时节点已发布标准 `TrayMatrix`。
- 当前只识别单盘，写入可配置 `active_tray_id`，其他盘补空。
- 当前 `z=0`，尚未完成深度、相机坐标、机械臂坐标转换。
- 不能声称机械臂/气缸闭环完成。

