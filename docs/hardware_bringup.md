# 硬件接入检查说明

本文档用于鲁班猫、D435iF 相机和后续 F407/W5500 接入前的检查。

目标不是立刻完成视觉算法，而是先确认硬件、系统驱动、ROS 2 数据流是否可用。

## 1. 当前优先级

建议按这个顺序推进：

1. 确认鲁班猫系统能正常启动并加载 ROS 2 Humble。
2. 确认 D435iF 能被系统识别。
3. 确认 D435iF 能输出 RGB 图和深度图。
4. 确认 ROS 2 中能看到相机话题。
5. 再开始写真实视觉节点。
6. 最后再接 F407/W5500 做 TCP 联调。

不要一开始同时接相机、视觉算法、F407 和机械结构。每一步单独验证，后面出问题才好定位。

## 2. D435iF 系统识别检查

插上 D435iF 后，在终端执行：

```bash
lsusb
```

正常情况下应该能看到 Intel RealSense 相关设备。

如果没有看到：

- 检查 USB 线是否支持数据传输。
- 尽量使用 USB 3.0 接口。
- 如果在 VMware 中测试，需要确认虚拟机已经接管 USB 设备。
- 如果在鲁班猫上测试，需要确认供电稳定。

## 3. RealSense 工具检查

检查是否安装 RealSense Viewer：

```bash
which realsense-viewer
```

如果已安装，可以运行：

```bash
realsense-viewer
```

在 Viewer 中需要确认：

- RGB 图像能显示。
- 深度图能显示。
- 帧率稳定。
- 图像没有频繁断流。
- 相机视野能覆盖三个苗盘工作区域。

## 4. Python SDK 检查

检查是否安装 `pyrealsense2`：

```bash
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 ok')"
```

如果提示 `ModuleNotFoundError`，说明当前 Python 环境还不能直接使用 RealSense SDK。

## 5. ROS 2 相机包检查

检查是否安装 RealSense ROS 2 包：

```bash
ros2 pkg list | grep realsense
```

如果安装成功，通常应能看到类似：

```text
realsense2_camera
realsense2_camera_msgs
```

启动相机节点的常见方式：

```bash
ros2 launch realsense2_camera rs_launch.py
```

启动后查看话题：

```bash
ros2 topic list
```

重点关注：

```text
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/color/camera_info
/camera/depth/camera_info
```

实际话题名以现场 `ros2 topic list` 输出为准。

## 6. 图像显示检查

如果有 `rqt_image_view`：

```bash
rqt_image_view
```

也可以查看图像话题信息：

```bash
ros2 topic info /camera/color/image_raw
ros2 topic info /camera/depth/image_rect_raw
```

如果话题存在但没有数据，检查：

- 相机节点是否仍在运行。
- USB 是否断开。
- 是否有权限访问相机。
- 是否在虚拟机里正确连接 USB。

## 7. 项目代码接入前的确认项

开始写真实视觉节点前，至少确认这些内容：

| 检查项 | 状态 |
| --- | --- |
| D435iF 在系统中可见 | 已确认，`lsusb` 显示 `Intel RealSense Depth Camera 435if` |
| RealSense ROS 2 包可用 | 已确认，存在 `realsense2_camera` 和 `realsense2_camera_msgs` |
| RealSense ROS 2 节点可启动 | 已确认，日志显示 `RealSense Node Is Up!` |
| RGB 图像可接收 | 已确认，`camera_input_probe` 收到 `640x480 rgb8` |
| 深度图可接收 | 已确认，`camera_input_probe` 收到 `640x480 16UC1` |
| ROS 2 中有 RGB 图像话题 | 已确认，`/camera/camera/color/image_raw` |
| ROS 2 中有深度图话题 | 已确认，`/camera/camera/depth/image_rect_raw` |
| RGB 图像频率 | VMware 中实测约 5-6Hz，低于相机配置 30FPS |
| 深度图频率 | VMware 中实测约 10-11Hz，低于相机配置 30FPS |
| 低分辨率 RGB 图像频率 | 项目节点实测约 14.3Hz |
| 低分辨率深度图频率 | 项目节点实测约 14.3Hz |
| RGB CameraInfo | 已确认，`1280x720`，`fx=912.534119`，`fy=911.830383`，`cx=645.348755`，`cy=370.139221` |
| Depth CameraInfo | 已确认，`848x480`，`fx=429.752228`，`fy=429.752228`，`cx=417.643738`，`cy=243.390274` |
| 相机视野能覆盖三个苗盘 | 待确认 |
| 相机固定方式稳定 | 待确认 |
| 光照环境稳定 | 待确认 |

## 8. 真机接入时不要写死的内容

以下内容必须通过配置文件或 ROS 参数管理，不应散落写死在代码中：

- 相机话题名。
- 深度图话题名。
- 相机内参来源。
- 苗盘外框识别阈值。
- ROI 尺寸。
- 叶片面积阈值。
- F407/W5500 IP 地址。
- F407/W5500 TCP 端口。

相关配置集中记录在：

```text
docs/runtime_configuration.md
```

## 9. 下一步代码目标

硬件确认后，下一步建议新增真实视觉输入节点：

```text
camera_matrix_publisher
```

初始版本只做：

- 订阅 RGB 图像。
- 订阅深度图。
- 打印图像尺寸和深度尺寸。
- 不做识别。
- 不输出矩阵。

确认图像输入稳定后，再继续实现苗盘外框识别、透视矫正和 5x10 网格生成。

## 10. 相机内参检查

D435iF 的 ROS 2 驱动会发布 CameraInfo，项目中已提供探测节点：

```text
camera_info_probe
```

运行方式：

```bash
ros2 run sorting_vision camera_info_probe --ros-args --params-file install/sorting_vision/share/sorting_vision/config/mock_vision.yaml
```

该节点会打印：

- 图像宽高。
- `fx`、`fy`、`cx`、`cy`。
- 畸变模型。
- `D` 畸变参数。
- `K` 内参矩阵。
- `P` 投影矩阵。

这些参数来自 D435iF 驱动发布的相机内参，不等于相机到机械臂基座的手眼标定参数。

当前已实现的第一版节点名为：

```text
camera_input_probe
```

运行方式：

```bash
ros2 run sorting_vision camera_input_probe --ros-args --params-file install/sorting_vision/share/sorting_vision/config/mock_vision.yaml
```
