# 当前项目状态

本文记录当前真实状态。若与早期文档冲突，以本文和当前代码为准。

## 1. 当前主线

项目已经完成 ROS 2 基础链路、RealSense RGB-D 输入验证和样本采集工具。
当前真正的难点不是环境，而是视觉几何识别：

```text
稳定识别 3 个苗盘
-> 每个苗盘得到可信的 5 x 10 穴位中心点
-> 从 aligned depth 读取 z
-> 后续再做单穴分类
```

旧路线“深色外框检测 + 机械均分 5 x 10”效果不够好。后续主线应优先推进
`tray_grid_debug`：用苗盘内部穴位结构拟合透视关系，再反投影出原图中心点。

## 2. 当前开发环境

主力开发环境：

```text
/home/wht/sorting_robot
/home/wht/sorting_robot/ros2_ws
```

开发原则：

- 虚拟机负责主要开发和离线样本调试。
- 鲁班猫用于阶段性实机验证和最终部署。
- 不要每个小实验都搬到鲁班猫上做。

当前 ROS 2 包：

```text
sorting_bringup
sorting_control
sorting_description
sorting_driver
sorting_interfaces
sorting_vision
```

## 3. 已验证能力

已经验证：

- ROS 2 Humble 可用。
- D435iF 可被系统识别。
- RealSense ROS 2 driver 可发布 RGB、Depth、Aligned Depth、CameraInfo。
- 640 x 480 x 15 模式稳定性优于 1280 x 720 x 30。
- `camera_input_probe` 可看到 RGB/Depth 图像。
- `camera_info_probe` 可读取相机内参。
- `depth_point_probe` 可在指定像素读取深度并反算相机坐标。
- `capture_rgbd_sample` 可采集 RGB-D 样本。

推荐 RealSense 启动命令：

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15 \
  align_depth.enable:=true
```

## 4. 当前样本和 debug

RGB-D 样本目录：

```text
samples/rgbd
```

当前有 10 组样本：

```text
sample_001_color.png ... sample_010_color.png
sample_001_depth.png ... sample_010_depth.png
sample_001_metadata.json ... sample_010_metadata.json
```

已整理 debug 查看入口：

```text
samples/debug_/_review/README.md
```

以后看算法效果优先看：

```text
samples/debug_/_review/contact_sheets/tray_grid__grid_overlay.jpg
samples/debug_/_review/contact_sheets/tray_grid__grid_rectified.jpg
samples/debug_/_review/contact_sheets/tray_grid__candidates.jpg
```

重新生成 debug 查看索引：

```bash
cd /home/wht/sorting_robot
python3 scripts/archive_debug_outputs.py
```

## 5. 当前重要工具

视觉侧 ROS 2 命令：

```text
camera_input_probe      检查 RGB/Depth 图像输入
camera_info_probe       检查 CameraInfo
depth_point_probe       单点深度探测
capture_rgbd_sample     采集 RGB-D 样本
real_matrix_publisher   临时 ROI 规则矩阵发布
grid_debug_publisher    临时 ROI 网格图调试
offline_tray_debug      旧离线调试工具
tray_grid_debug         当前主线离线网格调试工具
```

当前更推荐使用：

```bash
cd /home/wht/sorting_robot
python3 ros2_ws/src/sorting_vision/sorting_vision/tray_grid_debug.py \
  --input-dir samples/rgbd \
  --output-dir samples/debug_/runs
python3 scripts/archive_debug_outputs.py
```

`tray_grid_debug.py` 默认会在 `samples/debug_/runs/` 下新建时间戳子目录，
每次测试互不覆盖。

## 6. 当前代码风险

工作区存在未提交改动，主要包括：

- ROS 2 节点 Ctrl+C 退出修复。
- 新增 `capture_rgbd_sample.py`。
- 新增 `tray_grid_debug.py`。
- 新增 `scripts/archive_debug_outputs.py`。
- `offline_tray_debug.py` 被加入较多实验逻辑，文件已偏臃肿。

建议后续提交前先拆成几个清楚的提交：

1. ROS 节点 shutdown 修复。
2. RGB-D 样本采集工具。
3. Debug 归档脚本。
4. `tray_grid_debug` 离线网格原型。
5. 旧 `offline_tray_debug` 实验改动单独评估是否保留。

## 7. 下一步

短期目标：

- 不再继续修补外框均分路线。
- 把主线集中到 `tray_grid_debug.py`。
- 增强内部穴位检测过滤。
- 提高 homography 成功门槛。
- 输出每张样本、每个苗盘的质量评分。
- 明确失败原因，而不是强行输出看似正确的 5 x 10。

中期目标：

- 稳定输出 150 个穴位中心点 `u/v/z`。
- 再开始做单穴作物分类。
- 分类稳定后再迁移到实时节点。
