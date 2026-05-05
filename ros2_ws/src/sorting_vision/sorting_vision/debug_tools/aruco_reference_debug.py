"""ArUco 平面参考系离线调试工具。

这个工具用于验证“用 ArUco 标记建立像素到毫米参考”的路线。

它不做 YOLO，不识别苗盘，不输出抓取点。只做：
- 读取样本 RGB 图片。
- 检测 ArUco marker。
- 如果检测到指定 marker，计算 marker 边长对应的 pixel_to_mm。
- 输出带 marker 角点/中心的 debug 图和 CSV。

如果图片里没有 ArUco，会明确写 `no_marker`，不伪造坐标。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np


@dataclass(frozen=True)
class ArucoDetection:
    image_name: str
    marker_id: int
    center_u: float
    center_v: float
    side_px: float
    pixel_to_mm: float
    status: str


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到待处理图片')

    rows: list[dict[str, object]] = []
    for image_path in image_paths:
        rows.extend(process_image(image_path, output_dir, args.marker_size_mm, args.reference_id))

    summary_path = output_dir / 'aruco_reference_summary.csv'
    write_summary(summary_path, rows)
    print(f'处理图片数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')
    print(f'汇总文件：{summary_path}')


def process_image(
    image_path: Path,
    output_dir: Path,
    marker_size_mm: float,
    reference_id: int,
) -> list[dict[str, object]]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'读取图片失败：{image_path}')

    output = image.copy()
    detections = detect_aruco_markers(output, image_path.name, marker_size_mm)
    reference = next((item for item in detections if item.marker_id == reference_id), None)

    rows: list[dict[str, object]] = []
    if not detections:
        rows.append(empty_row(image_path.name, 'no_marker'))
        draw_status(output, 'no ArUco marker detected', ok=False)
    elif reference is None:
        rows.extend(detection_to_row(item, is_reference=False) for item in detections)
        draw_status(output, f'reference id {reference_id} not found', ok=False)
    else:
        rows.extend(
            detection_to_row(item, is_reference=(item.marker_id == reference_id))
            for item in detections
        )
        draw_status(
            output,
            f'ref id={reference.marker_id} scale={reference.pixel_to_mm:.3f} mm/px',
            ok=True,
        )

    cv2.imwrite(str(output_dir / f'{image_path.stem}_aruco_reference.jpg'), output)
    print(f'{image_path.name}: markers={len(detections)}')
    return rows


def detect_aruco_markers(
    image: np.ndarray,
    image_name: str,
    marker_size_mm: float,
) -> list[ArucoDetection]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners_list, ids, _rejected = aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is None or len(corners_list) == 0:
        return []

    aruco.drawDetectedMarkers(image, corners_list, ids)
    detections: list[ArucoDetection] = []
    for corners, marker_id_array in zip(corners_list, ids):
        marker_id = int(marker_id_array[0])
        points = corners.reshape(4, 2).astype(np.float32)
        center = points.mean(axis=0)
        side_lengths = [
            float(np.linalg.norm(points[(index + 1) % 4] - points[index]))
            for index in range(4)
        ]
        side_px = float(np.mean(side_lengths))
        pixel_to_mm = marker_size_mm / side_px if side_px > 0.0 else 0.0
        cv2.circle(image, tuple(np.round(center).astype(int)), 5, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.putText(
            image,
            f'id={marker_id} {pixel_to_mm:.3f}mm/px',
            tuple(np.round(center + np.asarray([8.0, -8.0])).astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        detections.append(
            ArucoDetection(
                image_name=image_name,
                marker_id=marker_id,
                center_u=float(center[0]),
                center_v=float(center[1]),
                side_px=side_px,
                pixel_to_mm=pixel_to_mm,
                status='detected',
            )
        )
    return detections


def detection_to_row(detection: ArucoDetection, is_reference: bool) -> dict[str, object]:
    return {
        'image': detection.image_name,
        'marker_id': detection.marker_id,
        'is_reference': int(is_reference),
        'center_u': f'{detection.center_u:.3f}',
        'center_v': f'{detection.center_v:.3f}',
        'side_px': f'{detection.side_px:.3f}',
        'pixel_to_mm': f'{detection.pixel_to_mm:.6f}',
        'status': detection.status,
    }


def empty_row(image_name: str, status: str) -> dict[str, object]:
    return {
        'image': image_name,
        'marker_id': '',
        'is_reference': 0,
        'center_u': '',
        'center_v': '',
        'side_px': '',
        'pixel_to_mm': '',
        'status': status,
    }


def draw_status(image: np.ndarray, text: str, ok: bool) -> None:
    color = (0, 180, 0) if ok else (0, 0, 255)
    cv2.putText(image, text, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        'image',
        'marker_id',
        'is_reference',
        'center_u',
        'center_v',
        'side_px',
        'pixel_to_mm',
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
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_aruco_reference')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(
        char if char.isalnum() or char in ('-', '_') else '_'
        for char in value.strip()
    )
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='ArUco 平面参考系离线调试工具。')
    parser.add_argument('--image', help='单张样本 RGB 图片路径。')
    parser.add_argument('--input-dir', default='samples/rgbd', help='批量样本目录。')
    parser.add_argument('--pattern', default='sample_*_color.png', help='批量图片匹配模式。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--marker-size-mm', type=float, default=34.0, help='ArUco marker 实际边长。')
    parser.add_argument('--reference-id', type=int, default=1, help='参考原点 marker id。')
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
