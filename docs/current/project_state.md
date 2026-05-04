# 当前项目状态

本文记录当前真实状态。若与早期文档冲突，以本文和当前代码为准。

## 1. 当前结论

项目基础链路已经具备，当前卡点不是 ROS 2、相机或通信，而是视觉几何：

```text
三苗盘真实边界识别
-> 单盘透视矫正
-> 稳定得到 5 x 10 穴位坐标
-> 再进入深度和 0/1/2 分类
```

最近的外框、孔中心、矩阵 overlay、Hough 边缘实验说明：当前算法还没有稳定完成
“苗盘边界识别 + 透视矫正”。在这个问题解决前，不应继续推进分类、矩阵美化或实机部署。

## 2. 开发环境

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

## 4. 样本和 Debug

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

当前 debug 查看入口：

```text
samples/debug_/_review/README.md
```

当前只保留两个直接对照 run：

```text
samples/debug_/runs/edge_segments_only_01
samples/debug_/runs/line_quad_01
```

历史实验已移动到：

```text
samples/debug_/archive
samples/debug_/_review/archive
```

`samples/debug` 是旧历史目录，新实验不要再写进去。

重新生成 debug 查看索引：

```bash
cd /home/wht/sorting_robot
python3 scripts/archive_debug_outputs.py
```

## 5. 当前关键工具

视觉侧 ROS 2 命令：

```text
camera_input_probe              检查 RGB/Depth 图像输入
camera_info_probe               检查 CameraInfo
depth_point_probe               单点深度探测
capture_rgbd_sample             采集 RGB-D 样本
real_matrix_publisher           临时 ROI 规则矩阵发布
grid_debug_publisher            临时 ROI 网格图调试
offline_tray_debug              旧离线调试工具，不建议继续堆新算法
tray_grid_debug                 内部网格拟合实验工具，仍未稳定
tray_rectify_debug              当前边界/矫正实验工具，未提交且仍在试错
tray_matrix_overlay_debug       矩阵 overlay 调试工具，当前不是主线
```

当前需要谨慎对待：

- `tray_rectify_debug.py` 很大，混入多轮失败实验，不能直接视为稳定主线。
- `offline_tray_debug.py` 已偏臃肿，应冻结为旧工具。
- `tray_grid_debug.py` 有内部网格拟合方向，但不能绕过外框/矫正质量问题。

## 6. 当前代码风险

工作区存在未提交改动：

```text
M  ros2_ws/src/sorting_vision/config/offline_tray_debug.yaml
M  ros2_ws/src/sorting_vision/setup.py
M  ros2_ws/src/sorting_vision/sorting_vision/offline_tray_debug.py
M  ros2_ws/src/sorting_vision/sorting_vision/tray_grid_debug.py
M  scripts/archive_debug_outputs.py
?? .claude/
?? ros2_ws/src/sorting_vision/sorting_vision/tray_matrix_overlay_debug.py
?? ros2_ws/src/sorting_vision/sorting_vision/tray_rectify_debug.py
```

不要把这些一次性全部提交。需要先整理：

1. 保留并提交 debug 归档脚本改动。
2. 单独评估 `tray_rectify_debug.py` 是否作为实验脚本保留。
3. 冻结或回退 `offline_tray_debug.py` 中不必要的实验堆叠。
4. 更新文档，明确当前卡点。

## 7. 下一步

短期只做一件事：

```text
稳定识别 3 个苗盘的真实边界，并把每个苗盘矫正到正视图。
```

验收方式：

- 原图 overlay 中边界必须贴合苗盘实物边缘。
- before/after 中 after 必须明显拉正。
- 不允许为了“看起来有输出”而画机械矩形、假网格或未验证矩阵。
- 算法失败时明确失败，不强行输出。

暂缓：

- 单穴 0/1/2 分类。
- 矩阵 overlay 美化。
- F407 联调。
- 鲁班猫部署优化。
- 手眼标定。
