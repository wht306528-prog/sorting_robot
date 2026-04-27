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

当前 `camera_input_probe` 已使用：

```text
color_image_topic=/camera/camera/color/image_raw
depth_image_topic=/camera/camera/depth/image_rect_raw
report_period_sec=2.0
```
