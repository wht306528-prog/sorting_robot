# sorting_vision 结构整理计划

更新时间：2026-05-05

## 当前判断

项目已经具备 ROS2 多包骨架：

- `sorting_bringup`：系统启动入口。
- `sorting_control`：分拣流程、状态机、规划。
- `sorting_description`：机器人描述。
- `sorting_driver`：串口/TCP 驱动与协议。
- `sorting_interfaces`：消息定义。
- `sorting_vision`：相机、视觉识别、离线调试。

当前最需要整理的是 `sorting_vision`。问题不是包太少，而是相机探针、采样、正式节点、离线实验脚本混在一个 Python 包根目录里。

## 现有脚本归类

### 相机与 RGB-D 探针

- `camera_info_probe.py`
- `camera_input_probe.py`
- `depth_point_probe.py`

这些脚本只用于确认 RealSense / ROS2 topic / camera_info / depth 是否正常，不应参与正式识别链路。

### 样本采集

- `capture_rgbd_sample.py`

负责保存 RGB、对齐深度、metadata。它是数据采集工具，不是识别算法。

### 正式或准正式发布节点

- `real_matrix_publisher.py`
- `node.py`（当前作为 mock matrix publisher 入口）
- `grid_debug_publisher.py`（更偏 debug，但会发布图像）

这些入口会通过 ROS2 topic 对外产生数据，必须比离线脚本更谨慎。

### 离线视觉实验

- `offline_tray_debug.py`
- `tray_grid_debug.py`
- `tray_rectify_debug.py`
- `tray_matrix_overlay_debug.py`

这些脚本只用于算法验证。它们不能直接被当成最终识别能力，也不能用漂亮 overlay 掩盖失败。

### 基础模块

- `camera_model.py`
- `detector.py`

这类文件适合沉淀成可复用算法/模型代码。

## 目标结构

短期不拆 ROS2 包，只在 `sorting_vision/sorting_vision/` 内部按职责分层：

```text
sorting_vision/
  probes/        # 相机与深度探针
  capture/       # 样本采集
  nodes/         # 正式 ROS2 节点
  algorithms/    # 可复用视觉算法
  debug_tools/   # 离线调试脚本
```

迁移原则：

1. 先稳定算法，再移动文件。
2. 每次只移动一类文件，并同步修改 `setup.py` 入口。
3. 未稳定的实验脚本不注册成正式入口，最多保留离线命令。
4. debug 输出必须进入 `samples/debug_/runs/<run_name>/`，review 输出进入 `samples/debug_/_review/runs/<run_name>/`。
5. 不再把网格、矩阵、分类 overlay 和边界识别混在一个结果图里。

## 视觉算法推进顺序

当前不再继续颜色阈值重写路线。

下一条可接受的视觉主线是：

1. 从原图粗定位 3 个苗盘区域。
2. 对每个区域寻找真实外边界四条边。
3. 四边相交得到四角。
4. 透视矫正为单盘正视图。
5. 在正视图中识别穴位结构。
6. 生成每格中心点。
7. 最后才叠加 0/1/2 矩阵和深度/三维坐标。

任何一步失败，都应在 debug 图和 CSV 中显式标注失败，不画替代性的假框或假网格。

## 暂不做

- 暂不新建一堆空 ROS2 包。
- 暂不把 `sorting_vision` 文件大规模搬家。
- 暂不推进矩阵 overlay。
- 暂不推进机械臂坐标转换。
- 暂不把离线脚本包装成最终识别节点。
