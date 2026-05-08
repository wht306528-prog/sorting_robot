# 术语和字段对照表

本文档用于统一项目中的英文变量、代号、中文名称和取值含义。后续新增字段或改名时，应优先更新本文档，避免鲁班猫端、F407 端和文档中的命名不一致。

## 1. 设备和系统名称

| 英文 / 代号 | 中文名称 | 说明 |
| --- | --- | --- |
| `sorting_robot` | 智能苗盘分拣机器人项目 | Git 仓库名和项目根目录名 |
| `Lubancat` | 鲁班猫开发板 | 上位计算平台 |
| `RK3588` | RK3588 处理器 | 鲁班猫 5 使用的处理器 |
| `ROS 2 Humble` | ROS 2 Humble 版本 | 鲁班猫侧软件框架 |
| `RGB camera` | 普通 RGB 相机 | 只提供彩色图像，不提供深度 |
| `RGB-D camera` | RGB-D 相机 | 同时提供彩色图像和深度图 |
| `D435iF` | Intel RealSense D435iF 深度相机 | 当前测试过的 RGB-D 相机型号，不是算法必须绑定的品牌 |
| `F407` | STM32F407ZGT6 控制板 | 下位运动控制器 |
| `W5500` | W5500 以太网模块 | F407 侧以太网通信模块 |
| `RS485` | RS485 总线 | 伺服电机通信方式 |
| `GPIO` | 通用输入输出口 | 控制传送带、气缸、吸盘或读取传感器 |
| `TCP` | TCP 网络通信 | 鲁班猫与 F407 的推荐通信方式 |

## 2. ROS 2 包名

| 包名 | 中文职责 | 说明 |
| --- | --- | --- |
| `sorting_interfaces` | 接口定义包 | 定义 ROS 2 消息 |
| `sorting_vision` | 视觉识别包 | 相机采集、苗盘识别、矩阵生成 |
| `sorting_driver` | 通信驱动包 | 与 F407 通信 |
| `sorting_control` | 控制逻辑包 | 预留给上位机控制、仿真或调度逻辑 |
| `sorting_bringup` | 启动包 | 一键启动节点 |
| `sorting_description` | 描述包 | 机械结构描述、可视化或仿真 |

## 3. 消息名称

| 英文名称 | 中文名称 | 说明 |
| --- | --- | --- |
| `TrayCell` | 单个苗盘穴位信息 | 一个穴位对应一条记录 |
| `TrayMatrix` | 三苗盘矩阵信息 | 一次完整识别结果，正常包含 150 个 `TrayCell` |
| `ObjectInfo` | 目标信息 | 当前作为 `TrayCell` 的兼容别名 |
| `SortCommand` | 分拣命令 | 预留给后续控制命令 |

## 3.1 节点和话题名称

| 英文 / 代号 | 中文名称 | 说明 |
| --- | --- | --- |
| `mock_matrix_publisher` | 模拟矩阵发布节点 | 不接相机，定时发布 150 个模拟穴位 |
| `camera_input_probe` | 相机输入探测节点 | 订阅 RGB 和可选 Depth 图像并打印尺寸、编码和接收频率 |
| `camera_info_probe` | 相机内参探测节点 | 订阅 RGB 和可选 Depth CameraInfo 并打印 fx/fy/cx/cy |
| `real_matrix_publisher` | 真实相机矩阵发布节点 | 订阅 RGB 和可选 Depth 图像，按临时 ROI 和规则网格发布 `TrayMatrix` |
| `grid_debug_publisher` | 网格调试图像发布节点 | 订阅 RGB 图像，绘制三苗盘 ROI、网格线和穴位中心点 |
| `offline_tray_debug` | 离线苗盘调试脚本 | 读取样本图片，检测或读取苗盘四角点，输出透视矫正和网格调试图片 |
| `CameraIntrinsics` | 相机内参结构 | 代码中统一保存 width/height/fx/fy/cx/cy 等字段 |
| `CameraIntrinsicsCache` | 相机内参缓存 | 后续真实视觉节点用于缓存 RGB 和 Depth 两路内参 |
| `matrix_protocol_printer` | 矩阵协议打印节点 | 订阅矩阵话题并打印 F407 文本协议帧 |
| `matrix_tcp_sender` | 矩阵 TCP 发送节点 | 订阅矩阵话题并通过 TCP 发送 F407 文本协议帧 |
| `/sorting/tray_matrix` | 苗盘矩阵话题 | 发布 `TrayMatrix` 消息 |
| `/sorting/debug/grid_image` | 网格调试图像话题 | 发布画有 ROI 和 5x10 网格的调试图像 |
| `/camera/camera/color/image_raw` | RealSense RGB 图像话题 | 当前 VMware 测试中检测到的话题名 |
| `/camera/camera/depth/image_rect_raw` | RealSense 深度图话题 | 当前 VMware 测试中检测到的话题名 |
| `/camera/camera/color/camera_info` | RealSense RGB 相机内参话题 | 当前 VMware 测试中检测到的话题名 |
| `/camera/camera/depth/camera_info` | RealSense 深度相机内参话题 | 当前 VMware 测试中检测到的话题名 |
| `use_depth` | 是否使用深度输入 | `true` 表示 RGB-D 模式；`false` 表示普通 RGB 相机模式，输出 `z=0` |

## 4. 苗盘和穴位编号

| 英文 / 代号 | 中文名称 | 取值 | 说明 |
| --- | --- | --- | --- |
| `tray` | 苗盘 | - | 一个矩形育苗盘 |
| `tray_id` | 苗盘序号 | `1..3` | 从左到右编号 |
| `tray_1` | 1 号苗盘 / 左侧苗盘 | `tray_id=1` | 1 号传送带上的苗盘 |
| `tray_2` | 2 号苗盘 / 中间苗盘 | `tray_id=2` | 2 号传送带上的苗盘 |
| `tray_3` | 3 号苗盘 / 右侧苗盘 | `tray_id=3` | 3 号传送带上的空苗盘或接收苗盘 |
| `cell` | 穴位 / 格子 | - | 苗盘中的一个方格位置 |
| `hole` | 穴位 / 孔位 | - | 与 `cell` 含义接近，文档中优先使用 `cell` |
| `row` | 行坐标 | `1..10` | 从上到下递增 |
| `col` | 列坐标 | `1..5` | 从左到右递增 |
| `index` | 顺序编号 | `0..49` 或 `1..50` | 如需使用，必须在文档中说明从 0 还是从 1 开始 |

注意：文档和代码中优先使用“穴位行坐标 / 穴位列坐标”，不要使用“空穴行坐标 / 空穴列坐标”表示全部格子。

## 5. 类别定义

| 英文 / 代号 | 中文名称 | 数值 | 说明 |
| --- | --- | --- | --- |
| `class_id` | 类别编号 | `0/1/2` | 当前分类结果 |
| `CLASS_EMPTY` | 空穴或无有效苗 | `0` | 空穴、无有效苗或无法作为好苗使用 |
| `CLASS_GOOD` | 好苗 | `1` | 可用于补充 1 号苗盘 |
| `CLASS_WEAK` | 弱苗或未出土苗 | `2` | 需要搬运到 3 号苗盘 |
| `confidence` | 类别置信度 | `0.0..1.0` | 分类可信程度 |

当前不单独定义 3 类。弱苗和未出土苗统一归为 `CLASS_WEAK=2`，因为机械处理方式相同。

## 6. 鲁班猫输出字段

鲁班猫输出矩阵的 8 列字段如下：

| 顺序 | 英文字段 | 中文名称 | 类型建议 | 取值 / 单位 |
| --- | --- | --- | --- | --- |
| 1 | `tray_id` | 苗盘序号 | `uint8` | `1..3` |
| 2 | `col` | 穴位列坐标 | `uint8` | `1..5` |
| 3 | `row` | 穴位行坐标 | `uint8` | `1..10` |
| 4 | `class_id` | 类别编号 | `uint8` | `0/1/2` |
| 5 | `confidence` | 类别置信度 | `float32` | `0.0..1.0` |
| 6 | `u` | 像素横坐标 | `float32` | pixel |
| 7 | `v` | 像素纵坐标 | `float32` | pixel |
| 8 | `z` | 深度值 | `float32` | mm |

## 7. 图像和坐标字段

| 英文 / 代号 | 中文名称 | 单位 | 说明 |
| --- | --- | --- | --- |
| `u` | 像素横坐标 | pixel | 图像坐标系 x 方向 |
| `v` | 像素纵坐标 | pixel | 图像坐标系 y 方向 |
| `z` / `Zc` | 深度值 | mm | D435iF 输出深度 |
| `fx` | 相机 x 方向焦距 | pixel | 相机内参 |
| `fy` | 相机 y 方向焦距 | pixel | 相机内参 |
| `cx` | 相机主点 x 坐标 | pixel | 相机内参 |
| `cy` | 相机主点 y 坐标 | pixel | 相机内参 |
| `Xc` | 相机坐标系 X | mm | F407 后续计算 |
| `Yc` | 相机坐标系 Y | mm | F407 后续计算 |
| `Zc` | 相机坐标系 Z | mm | 通常等于深度值 |
| `Xw` | 机械臂基座坐标系 X | mm | F407 后续计算 |
| `Yw` | 机械臂基座坐标系 Y | mm | F407 后续计算 |
| `R` | 旋转矩阵 | - | 相机坐标系到机械臂基座坐标系 |
| `T` | 平移向量 | mm | 相机光心到机械臂基座原点偏移 |
| `Tx` | X 方向平移 | mm | 平移向量 X 分量 |
| `Ty` | Y 方向平移 | mm | 平移向量 Y 分量 |

## 8. 机构和控制字段

| 英文 / 代号 | 中文名称 | 说明 |
| --- | --- | --- |
| `theta1` | 伺服电机 1 目标角度 | F407 根据坐标和五连杆逆解计算 |
| `theta2` | 伺服电机 2 目标角度 | F407 根据坐标和五连杆逆解计算 |
| `servo_1` | 伺服电机 1 | 五连杆主动杆驱动电机之一 |
| `servo_2` | 伺服电机 2 | 五连杆主动杆驱动电机之一 |
| `reduction_ratio` | 减速比 | 当前试验方案为 `1:10` |
| `cylinder` | 伸缩气缸 | 控制吸盘上下或伸缩 |
| `vacuum_cup` | 真空吸盘 | 抓取苗或小球 |
| `conveyor_1` | 1 号传送带 | 左侧传送带 |
| `conveyor_2` | 2 号传送带 | 中间传送带 |
| `conveyor_3` | 3 号传送带 | 右侧传送带 |
| `photoelectric_sensor` | 光电传感器 | 苗盘到位检测 |
| `limit_switch` | 限位开关 | 机械限位保护 |

## 9. 通信字段

| 英文 / 代号 | 中文名称 | 说明 |
| --- | --- | --- |
| `frame_id` | 帧编号 | 鲁班猫侧递增，用于排查丢帧 |
| `count` | 记录数量 | 正常为 150 |
| `payload` | 数据内容 | 通信帧中的矩阵数据 |
| `checksum` | 校验值 | 文本协议简单累加校验；只累加 150 行 CSV 数据行字节，不含 START/END/换行 |
| `crc16` | CRC16 校验 | 二进制协议推荐 |
| `START` | 文本帧开始标记 | 用于调试协议 |
| `END` | 文本帧结束标记 | 用于调试协议 |
| `ACK` | 确认 | F407 成功接收后可回传 |
| `NACK` | 否认 / 接收失败 | F407 校验失败后可回传 |
| `f407_host` | F407 主机地址参数 | `matrix_tcp_sender` 使用的目标 IP |
| `f407_port` | F407 端口参数 | `matrix_tcp_sender` 使用的目标 TCP 端口 |
| `send_timeout_sec` | 发送超时时间参数 | TCP 连接和发送超时时间 |
| `tray_matrix_topic` | 苗盘矩阵话题参数 | 控制节点订阅或发布的矩阵话题名 |
| `debug_image_topic` | 网格调试图像话题参数 | `grid_debug_publisher` 发布带网格图像的话题名 |
| `publish_period_sec` | 发布周期参数 | 模拟视觉节点发布周期，单位秒 |
| `camera_frame_id` | 相机坐标系名称参数 | `TrayMatrix.header.frame_id` 使用的坐标系名 |
| `color_image_topic` | RGB 图像话题参数 | `camera_input_probe` 订阅的 RGB 图像话题 |
| `depth_image_topic` | 深度图话题参数 | `camera_input_probe` 订阅的深度图话题 |
| `color_camera_info_topic` | RGB 相机内参话题参数 | `camera_info_probe` 订阅的 RGB CameraInfo 话题 |
| `depth_camera_info_topic` | 深度相机内参话题参数 | `camera_info_probe` 订阅的 Depth CameraInfo 话题 |
| `report_period_sec` | 状态打印周期参数 | `camera_input_probe` 打印状态的周期 |
| `tray_1_roi` | 1 号苗盘图像区域参数 | 临时真实视觉节点使用，格式 `[x, y, width, height]`，单位 pixel |
| `tray_2_roi` | 2 号苗盘图像区域参数 | 临时真实视觉节点使用，格式 `[x, y, width, height]`，单位 pixel |
| `tray_3_roi` | 3 号苗盘图像区域参数 | 临时真实视觉节点使用，格式 `[x, y, width, height]`，单位 pixel |
| `leaf_area_ratio_threshold` | 叶片面积占比阈值 | 早期规则分类中，大于等于该值暂判为 1 类 |
| `weak_area_ratio_threshold` | 弱苗面积占比阈值 | 早期规则分类中，介于该值和好苗阈值之间暂判为 2 类 |
| `depth_window_px` | 深度采样窗口参数 | 中心点附近取中位数的窗口大小，单位 pixel |
| `roi_inner_scale` | 单穴 ROI 内缩比例 | 颜色统计只取单穴中心区域，减少相邻穴位干扰 |
| `color_sample_step_px` | 颜色采样步长 | 早期规则分类中的像素采样间隔 |

## 10. 命名约定

- ROS 2 包名、话题名、Python 文件名使用小写加下划线，例如 `tray_matrix`.
- 消息名使用大驼峰，例如 `TrayCell`.
- 消息字段使用小写加下划线，例如 `tray_id`.
- 类别常量使用全大写加下划线，例如 `CLASS_GOOD`.
- 文档中第一次出现英文缩写时，应写清中文含义。
- 与 F407 对接的字段名一旦确认，不要随意改名；必须改名时，同时更新本文档、通信协议和双方代码。

## 11. 易混淆名称

| 不推荐名称 | 推荐名称 | 原因 |
| --- | --- | --- |
| 空穴行坐标 | 穴位行坐标 | 每个格子都输出，不只是空穴 |
| 空穴列坐标 | 穴位列坐标 | 每个格子都输出，不只是空穴 |
| 苗子种类 | 类别编号 / `class_id` | 代码中需要明确数值 |
| 矩阵第一列 | `tray_id` | 用字段名比用列号更清楚 |
| B 类 / C 类 | 2 类 / `CLASS_WEAK` | 当前弱苗和未出土统一处理 |
| 右侧空盘 | 3 号苗盘 / `tray_3` | 进入工作区后按苗盘编号描述 |
