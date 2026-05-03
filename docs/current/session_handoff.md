# 会话交接说明

本文档用于新开对话或换人接手时快速了解当前项目状态。

新对话建议先阅读：

```text
docs/current_project_state 当前项目状态.md
docs/session_handoff 会话交接说明.md
docs/system_design 系统设计说明.md
docs/runtime_configuration 运行参数配置说明.md
docs/glossary 术语对照表.md
```

注意：本文包含较多早期阶段记录。若硬件环境、相机分辨率、当前主线与
`docs/current_project_state 当前项目状态.md` 冲突，以当前状态文档和代码为准。

## 1. 项目目标

项目是智能苗盘分拣系统。工作区内有三条传送带：

- 1 号传送带：左侧待分拣苗盘。
- 2 号传送带：中间待分拣苗盘，同时作为 1 号苗盘的好苗补充来源。
- 3 号传送带：右侧空苗盘，用于按顺序接收 2 类苗。

每个苗盘为 `5 x 10 = 50` 个穴位。系统需要识别三个苗盘中每个穴位的状态，并输出 3 个苗盘共 150 行数据。

当前类别约定：

```text
0 = 空穴或无有效苗
1 = 好苗
2 = 弱苗或未出土苗
```

弱苗和未出土苗统一归为 2 类，因为机械处理方式相同。

## 2. 当前硬件和分工

上位机：

- 鲁班猫 5 / RK3588。
- Ubuntu 20.04。
- ROS 2 Humble。
- D435iF RGB-D 深度相机。

下位机：

- STM32F407ZGT6。
- 预计通过 W5500 以太网模块和鲁班猫通信。
- 负责五连杆逆解、气缸、吸盘、传送带、光电和 RS485 伺服。

分工边界：

- 鲁班猫侧输出前 8 列矩阵：`tray_id, col, row, class_id, confidence, u, v, z`。
- F407 侧后续处理相机坐标、机械臂坐标、伺服角度和运动控制。

## 3. 当前代码进度

已完成 ROS 2 接口：

```text
ros2_ws/src/sorting_interfaces/msg/TrayCell.msg
ros2_ws/src/sorting_interfaces/msg/TrayMatrix.msg
```

`TrayCell` 字段：

```text
tray_id
col
row
class_id
confidence
u
v
z
```

已完成视觉侧节点：

```text
mock_matrix_publisher
camera_input_probe
camera_info_probe
real_matrix_publisher
grid_debug_publisher
offline_tray_debug
```

说明：

- `mock_matrix_publisher`：发布模拟的 150 个穴位矩阵。
- `camera_input_probe`：订阅 RealSense RGB/Depth 图像，打印尺寸、编码和接收频率。
- `camera_info_probe`：订阅 RealSense CameraInfo，打印 `fx/fy/cx/cy`、K、P、D。
- `real_matrix_publisher`：订阅 RGB-D 图像，使用临时三苗盘 ROI 和 `5 x 10` 规则网格发布真实相机链路的 `TrayMatrix`。
- `grid_debug_publisher`：订阅 RGB 图像，发布画有三苗盘 ROI、网格线和穴位中心点的调试图像。
- `offline_tray_debug`：读取样本图片，检测或读取苗盘四角点，输出透视矫正和 `5 x 10` 网格调试图片。

已完成驱动侧节点：

```text
matrix_protocol_printer
matrix_tcp_sender
```

说明：

- `matrix_protocol_printer`：订阅 `/sorting/tray_matrix`，打印 F407 文本协议帧。
- `matrix_tcp_sender`：订阅 `/sorting/tray_matrix`，通过 TCP 发送 F407 文本协议帧。

已完成公共模块：

```text
sorting_vision/camera_model.py
```

说明：

- `CameraIntrinsics`：保存相机内参。
- `CameraIntrinsicsCache`：缓存 RGB 和 Depth 两路内参。
- `pixel_to_camera(u, v, z_mm)`：像素坐标和深度转相机坐标。

## 4. 已验证结果

模拟矩阵链路已验证：

```text
mock_matrix_publisher -> /sorting/tray_matrix -> ros2 topic echo
```

结果：

```text
每帧 cells=150
```

协议打印链路已验证：

```text
mock_matrix_publisher -> matrix_protocol_printer
```

结果：

```text
START frame_id=...
...
END checksum=...
```

D435iF 硬件识别已验证：

```text
lsusb 显示 Intel RealSense Depth Camera 435if
```

RealSense ROS 2 包已安装并可用：

```text
realsense2_camera
realsense2_camera_msgs
```

RealSense 节点已启动成功：

```text
RealSense Node Is Up!
```

当前检测到的相机话题：

```text
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/color/camera_info
/camera/camera/depth/camera_info
/camera/camera/extrinsics/depth_to_color
```

低分辨率模式下，项目节点实测：

```text
RGB:   640x480, rgb8, 约 14.3Hz, frame=camera_color_optical_frame
Depth: 640x480, 16UC1, 约 14.3Hz, frame=camera_depth_optical_frame
```

CameraInfo 实测：

```text
Color / RGB:
  size = 1280x720
  fx = 912.534119
  fy = 911.830383
  cx = 645.348755
  cy = 370.139221
  distortion_model = plumb_bob
  D = [0, 0, 0, 0, 0]

Depth:
  size = 848x480
  fx = 429.752228
  fy = 429.752228
  cx = 417.643738
  cy = 243.390274
  distortion_model = plumb_bob
  D = [0, 0, 0, 0, 0]
```

注意：CameraInfo 内参与当前图像流配置绑定。程序应从 CameraInfo 自动读取，不应在算法中手写固定内参。

## 5. 当前配置文件

视觉配置：

```text
ros2_ws/src/sorting_vision/config/mock_vision.yaml
```

包含：

```text
tray_matrix_topic
publish_period_sec
camera_frame_id
color_image_topic
depth_image_topic
color_camera_info_topic
depth_camera_info_topic
tray_1_roi
tray_2_roi
tray_3_roi
leaf_area_ratio_threshold
weak_area_ratio_threshold
depth_window_px
roi_inner_scale
color_sample_step_px
debug_image_topic
debug_line_width_px
debug_center_radius_px
```

离线样本调试配置：

```text
ros2_ws/src/sorting_vision/config/offline_tray_debug.yaml
```

说明：

- 当前视觉主线优先走离线样本图片调试。
- 固定 ROI 实时节点只作为调试和演示辅助。
- `offline_tray_debug` 第一版允许手工四角点作为过渡，但最终目标是自动检测苗盘外框。

驱动配置：

```text
ros2_ws/src/sorting_driver/config/driver.yaml
```

包含：

```text
tray_matrix_topic
f407_host
f407_port
send_timeout_sec
```

`f407_host=127.0.0.1` 只用于本机测试。真机联调时必须改为 F407/W5500 的实际 IP。

## 6. Git 状态和习惯

当前仓库主分支为：

```text
main
```

提交信息习惯：

```text
英文说明 中文说明
```

例如：

```bash
git commit -m "Add camera info probe 添加相机内参探测节点"
```

不要提交构建产物：

```text
build/
install/
log/
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
```

这些已在 `.gitignore` 中忽略。

## 7. 下一步建议

下一步优先做图像样本采集工具：

```text
capture_sample
```

目标：

- 保存 RGB 图。
- 保存 Depth 原始数组。
- 保存 Depth 可视化预览图。
- 保存 CameraInfo 和备注元数据。

样本用于后续离线调试：

- 苗盘外框检测。
- 透视矫正。
- 5x10 网格生成。
- ROI 裁剪。

如果暂时没有标准苗盘样本，也可以先用：

- 空苗盘。
- 三个空苗盘。
- 纸板画 5x10 网格。
- 小球/纸片模拟苗。

## 8. 注意事项

- 手机照片只能用于 RGB 几何算法调试，不能替代 D435iF 深度图。
- D435iF 的 RGB 和 Depth 是两路不同相机流，内参不同是正常现象。
- 相机内参可从 CameraInfo 自动读取。
- 手眼标定 R/T 不能由相机自动提供，必须在机械结构固定后现场标定。
- VMware 中相机帧率可能低于真机鲁班猫环境，最终性能应在鲁班猫实机上复测。
- 后续真实视觉算法不要手写 `fx/fy/cx/cy`，应复用 `CameraIntrinsicsCache`。

## 9. 协作偏好

用户是第一次完整参与项目开发，代码基础还在建立中。协作时需要注意：

- 不只给结论，也解释为什么这样做。
- 每一步尽量给清楚的命令、运行位置和预期输出。
- 不要一次塞太多新概念，复杂内容要拆成小步骤。
- 涉及 Git 提交时，commit message 使用“英文说明 中文说明”格式。
- 代码需要中文模块说明、类/函数说明和关键逻辑注释。
- 简单语句不需要机械逐行注释，但关键设计意图必须写清楚。
- 参数、IP、端口、话题名、相机内参等不要散落写死在代码里，应放配置文件或从 ROS 2 消息自动读取。
- 用户很在意后期维护成本，需要明确哪些是模拟值，哪些是真机联调时必须修改。
- 做完代码后要说明如何测试、如何判断成功、如何提交 Git。
- 如果出现大量 VSCode Git 变更，优先检查是否误把 `build/ install/ log/` 这类构建产物显示出来。
- 用户希望 AI 主动推进，但在关键架构取舍上解释清楚原因。
