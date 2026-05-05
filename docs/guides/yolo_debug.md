# YOLO 离线检测调试

这个流程用于试验“YOLO 识别 + OpenCV 可视化”的路线。

## 前提

需要安装 `ultralytics`。

模型可以是：

- 本地 `.pt` 路径。
- `yolov8n.pt` 这类 Ultralytics 预训练模型名。

没有自训练模型也可以先用 `yolov8n.pt` 试 COCO 预训练检测，但它不认识“苗盘穴位/作物状态”这类自定义目标。

## 命令

```bash
cd /home/wht/sorting_robot
python3 ros2_ws/src/sorting_vision/sorting_vision/debug_tools/yolo_object_debug.py \
  --model yolov8n.pt \
  --input-dir samples/rgbd \
  --output-dir samples/debug_/runs \
  --run-name yolo_object_test_01
```

ROS2 包构建后也可以用：

```bash
ros2 run sorting_vision yolo_object_debug \
  --model yolov8n.pt \
  --input-dir samples/rgbd \
  --output-dir samples/debug_/runs \
  --run-name yolo_object_test_01
```

## 输出

每张图片会输出：

- `*_yolo_annotated.jpg`
- `yolo_object_summary.csv`

CSV 字段包括：

- `class_id`
- `class_name`
- `confidence`
- `x1,y1,x2,y2`
- `center_u,center_v`

## 注意

这只是目标检测调试工具，不负责：

- 苗盘几何矫正
- 穴位矩阵生成
- 深度反投影
- 机械臂坐标转换

后续如果训练出苗盘/穴位/作物模型，再把 YOLO 输出接入 `algorithms/tray_geometry.py` 或新的识别模块。
