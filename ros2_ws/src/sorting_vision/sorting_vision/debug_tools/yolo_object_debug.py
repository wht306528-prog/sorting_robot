"""YOLO 目标检测离线调试工具。

这条路线用于试验“别人项目里的 YOLO 检测 + OpenCV 可视化”方案。

当前工具只做离线图片推理：
- 用 ultralytics.YOLO 加载用户指定模型。
  可以是本地 `.pt` 路径，也可以是 `yolov8n.pt` 这类 Ultralytics 预训练模型名。
- 对样本 RGB 图片做检测。
- 用 OpenCV 绘制检测框、类别、置信度、中心点。
- 输出 annotated 图片和 CSV。

没有安装 ultralytics 时明确失败，不生成伪结果。
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2

try:
    from ultralytics import YOLO
except ModuleNotFoundError:  # pragma: no cover - depends on local environment
    YOLO = None  # type: ignore[assignment]


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    if YOLO is None:
        raise ModuleNotFoundError(
            '未安装 ultralytics。请先安装 YOLO 运行依赖，例如：pip install ultralytics'
        )
    model = YOLO(args.model)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到待处理图片')

    rows: list[dict[str, object]] = []
    for image_path in image_paths:
        rows.extend(
            process_image(
                image_path=image_path,
                output_dir=output_dir,
                model=model,
                confidence_threshold=args.confidence_threshold,
            )
        )

    summary_path = output_dir / 'yolo_object_summary.csv'
    write_summary(summary_path, rows)
    print(f'处理图片数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')
    print(f'汇总文件：{summary_path}')


def process_image(
    image_path: Path,
    output_dir: Path,
    model: Any,
    confidence_threshold: float,
) -> list[dict[str, object]]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'读取图片失败：{image_path}')

    results = model(image)
    output = image.copy()
    rows: list[dict[str, object]] = []
    detection_index = 0

    for result in results:
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            continue
        for box in boxes:
            row = detection_from_box(image_path.name, detection_index, model, box)
            if float(row['confidence']) < confidence_threshold:
                continue
            draw_detection(output, row)
            rows.append(row)
            detection_index += 1

    if not rows:
        draw_status(output, 'no YOLO detections', ok=False)
        rows.append(empty_row(image_path.name, 'no_detection'))
    else:
        draw_status(output, f'detections={len(rows)}', ok=True)

    cv2.imwrite(str(output_dir / f'{image_path.stem}_yolo_annotated.jpg'), output)
    print(f'{image_path.name}: detections={max(0, len(rows) if rows[0]["status"] != "no_detection" else 0)}')
    return rows


def detection_from_box(
    image_name: str,
    detection_index: int,
    model: Any,
    box: Any,
) -> dict[str, object]:
    xyxy = box.xyxy[0].to('cpu').detach().numpy().copy()
    confidence = float(box.conf[0].to('cpu').detach().numpy()) if hasattr(box, 'conf') else 0.0
    class_id = int(box.cls[0].to('cpu').detach().numpy()) if hasattr(box, 'cls') else -1
    class_name = model.names.get(class_id, str(class_id)) if hasattr(model, 'names') else str(class_id)

    x1, y1, x2, y2 = [float(value) for value in xyxy]
    center_u = (x1 + x2) * 0.5
    center_v = (y1 + y2) * 0.5
    return {
        'image': image_name,
        'detection_index': detection_index,
        'class_id': class_id,
        'class_name': class_name,
        'confidence': f'{confidence:.6f}',
        'x1': f'{x1:.3f}',
        'y1': f'{y1:.3f}',
        'x2': f'{x2:.3f}',
        'y2': f'{y2:.3f}',
        'center_u': f'{center_u:.3f}',
        'center_v': f'{center_v:.3f}',
        'status': 'detected',
    }


def draw_detection(image: Any, row: dict[str, object]) -> None:
    x1 = int(round(float(row['x1'])))
    y1 = int(round(float(row['y1'])))
    x2 = int(round(float(row['x2'])))
    y2 = int(round(float(row['y2'])))
    center = (int(round(float(row['center_u']))), int(round(float(row['center_v']))))
    label = f'{row["class_name"]} {float(row["confidence"]):.2f}'
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)
    cv2.circle(image, center, 4, (0, 255, 0), -1, cv2.LINE_AA)
    cv2.putText(image, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)


def draw_status(image: Any, text: str, ok: bool) -> None:
    color = (0, 180, 0) if ok else (0, 0, 255)
    cv2.putText(image, text, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)


def empty_row(image_name: str, status: str) -> dict[str, object]:
    return {
        'image': image_name,
        'detection_index': '',
        'class_id': '',
        'class_name': '',
        'confidence': '',
        'x1': '',
        'y1': '',
        'x2': '',
        'y2': '',
        'center_u': '',
        'center_v': '',
        'status': status,
    }


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        'image',
        'detection_index',
        'class_id',
        'class_name',
        'confidence',
        'x1',
        'y1',
        'x2',
        'y2',
        'center_u',
        'center_v',
        'status',
    ]
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def resolve_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        return [Path(args.image)]
    return sorted(Path(args.input_dir).glob(args.pattern))


def resolve_output_dir(output_dir: str, run_name: str | None) -> Path:
    base = Path(output_dir)
    if run_name:
        return base / safe_path_name(run_name)
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_yolo_object')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(
        char if char.isalnum() or char in ('-', '_') else '_'
        for char in value.strip()
    )
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='YOLO 目标检测离线调试工具。')
    parser.add_argument('--model', required=True, help='YOLO .pt 模型路径。')
    parser.add_argument('--image', help='单张样本 RGB 图片路径。')
    parser.add_argument('--input-dir', default='samples/rgbd', help='批量样本目录。')
    parser.add_argument('--pattern', default='sample_*_color.png', help='批量图片匹配模式。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--confidence-threshold', type=float, default=0.25, help='最小置信度。')
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
