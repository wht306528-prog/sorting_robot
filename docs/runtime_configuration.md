# 运行参数配置说明

本文档记录项目中“模拟阶段可以用默认值，但真机联调时可能需要修改”的参数。

原则：

- 不把现场 IP、端口、话题名等长期写死在代码里。
- 可变参数优先放在 `config/*.yaml`。
- 临时调试可以用 `--ros-args -p 参数名:=参数值` 覆盖。
- 修改参数含义时，同步更新 `docs/glossary.md` 和本文档。

## 1. 视觉模拟节点配置

配置文件：

```text
ros2_ws/src/sorting_vision/config/mock_vision.yaml
```

节点：

```text
mock_matrix_publisher
```

参数：

| 参数 | 当前默认值 | 真机注意事项 |
| --- | --- | --- |
| `tray_matrix_topic` | `/sorting/tray_matrix` | 后续真实视觉节点建议继续使用这个话题 |
| `publish_period_sec` | `1.0` | 真实相机方案可能改为苗盘到位后触发发布 |
| `camera_frame_id` | `mock_camera` | 接真实相机后应改为真实相机坐标系名称 |

运行示例：

```bash
ros2 run sorting_vision mock_matrix_publisher --ros-args --params-file install/sorting_vision/share/sorting_vision/config/mock_vision.yaml
```

## 2. 驱动节点配置

配置文件：

```text
ros2_ws/src/sorting_driver/config/driver.yaml
```

节点：

```text
matrix_protocol_printer
matrix_tcp_sender
```

参数：

| 参数 | 当前默认值 | 真机注意事项 |
| --- | --- | --- |
| `tray_matrix_topic` | `/sorting/tray_matrix` | 必须和视觉节点发布的话题一致 |
| `f407_host` | `127.0.0.1` | 只用于本机调试；真机必须改为 F407/W5500 实际 IP |
| `f407_port` | `9000` | 必须和 F407/W5500 程序监听端口一致 |
| `send_timeout_sec` | `2.0` | 网络不稳定时可以适当调大 |

本机调试示例：

```bash
nc -l 9000
ros2 run sorting_driver matrix_tcp_sender --ros-args --params-file install/sorting_driver/share/sorting_driver/config/driver.yaml
```

临时指定 F407 地址：

```bash
ros2 run sorting_driver matrix_tcp_sender --ros-args -p f407_host:=192.168.1.50 -p f407_port:=9000
```

## 3. 当前仍属于模拟的数据

以下内容目前只用于联调，不是真实视觉结果：

- `mock_matrix_publisher` 生成的 `u/v/z`。
- `mock_matrix_publisher` 生成的 `class_id`。
- `mock_matrix_publisher` 生成的 `confidence`。
- `camera_frame_id=mock_camera`。

真实相机接入后，这些值应由 D435iF 图像、深度图、苗盘识别、网格映射和分类算法产生。

## 4. 后续真实相机参数

真实相机接入后，应新增或确认以下参数，不要散落写死在代码中：

| 参数 | 说明 |
| --- | --- |
| `color_image_topic` | RGB 图像话题 |
| `depth_image_topic` | 深度图话题 |
| `color_camera_info_topic` | RGB 相机内参话题 |
| `depth_camera_info_topic` | 深度相机内参话题 |
| `tray_matrix_topic` | 输出苗盘矩阵话题 |
| `leaf_area_threshold` | 叶片面积占比阈值 |
| `empty_depth_threshold_mm` | 空穴深度判断阈值 |

当前已经加入第一版真实相机矩阵节点：

```text
real_matrix_publisher
```

它订阅 RGB-D 图像和 CameraInfo，使用临时配置的三个苗盘 ROI 生成 `5 x 10`
规则网格，并发布完整 `TrayMatrix`。当前分类规则是早期联调用的绿色面积占比
阈值，不代表最终苗盘识别算法。

当前 `camera_input_probe` 已使用：

```text
color_image_topic=/camera/camera/color/image_raw
depth_image_topic=/camera/camera/depth/image_rect_raw
report_period_sec=2.0
```

当前 `camera_info_probe` 已使用：

```text
color_camera_info_topic=/camera/camera/color/camera_info
depth_camera_info_topic=/camera/camera/depth/camera_info
```

代码约定：

```text
fx/fy/cx/cy 不写死在算法代码中。
真实视觉节点应从 CameraInfo 读取，并通过 CameraIntrinsicsCache 缓存。
```

`real_matrix_publisher` 当前使用：

| 参数 | 当前默认值 | 真机注意事项 |
| --- | --- | --- |
| `tray_matrix_topic` | `/sorting/tray_matrix` | 必须和 driver 节点订阅话题一致 |
| `publish_period_sec` | `1.0` | 后续建议改为苗盘到位后触发 |
| `camera_frame_id` | 空字符串 | 留空时使用 RGB 图像 `frame_id` |
| `color_image_topic` | `/camera/camera/color/image_raw` | 以现场 `ros2 topic list` 为准 |
| `depth_image_topic` | `/camera/camera/depth/image_rect_raw` | 最好使用对齐到 RGB 的深度话题 |
| `color_camera_info_topic` | `/camera/camera/color/camera_info` | 与 RGB 图像配置匹配 |
| `depth_camera_info_topic` | `/camera/camera/depth/camera_info` | 与深度图配置匹配 |
| `tray_1_roi` | `[20.0, 50.0, 185.0, 380.0]` | 1 号苗盘 ROI，格式 `[x, y, width, height]` |
| `tray_2_roi` | `[227.5, 50.0, 185.0, 380.0]` | 2 号苗盘 ROI，现场必须标定 |
| `tray_3_roi` | `[435.0, 50.0, 185.0, 380.0]` | 3 号苗盘 ROI，现场必须标定 |
| `leaf_area_ratio_threshold` | `0.20` | 大于等于该绿色面积占比暂判为 1 类 |
| `weak_area_ratio_threshold` | `0.03` | 介于弱苗和好苗阈值之间暂判为 2 类 |
| `depth_window_px` | `5` | 中心深度中位数采样窗口 |
| `roi_inner_scale` | `0.65` | 只取单穴中心区域做颜色统计，避免串格 |
| `color_sample_step_px` | `2` | 颜色采样步长，越小越细但 CPU 占用更高 |

运行示例：

```bash
ros2 run sorting_vision real_matrix_publisher --ros-args --params-file install/sorting_vision/share/sorting_vision/config/mock_vision.yaml
```
