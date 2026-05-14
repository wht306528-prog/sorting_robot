#!/usr/bin/env python3
"""导出白球漏检/误检 ROI，诊断白球和空洞白点为什么混淆。

输入是三盘采集样本和 annotations.json。脚本复用当前几何和分类算法，
把 white 的 false negative / false positive 裁出来，保存 CSV 和总览图。
这是离线诊断工具，不参与实时节点和 F407 链路。
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'sorting_vision'
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))

from sorting_vision.algorithms.pingpong_detector import (  # noqa: E402
    CLASS_WHITE,
    PingpongDetectorConfig,
    classify_cell,
    detect_pingpong_cells,
    estimate_roi_radius,
    is_white_component_shape_reasonable,
    largest_component_stats,
)
from sorting_vision.algorithms.tray_edge_fit import TrayEdgeFitConfig, rectify  # noqa: E402
from sorting_vision.algorithms.tray_geometry import TrayGeometryConfig, detect_tray_geometry  # noqa: E402
from sorting_vision.algorithms.tray_hole_grid import TrayHoleGridConfig  # noqa: E402


ROWS = 10
COLS = 5
TRAYS = 3


def main() -> None:
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    annotation_path = Path(args.annotations).expanduser()
    if not annotation_path.is_absolute():
        annotation_path = input_dir / annotation_path
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    annotations = load_json(annotation_path)
    samples = find_samples(input_dir)
    if args.limit > 0:
        samples = samples[:args.limit]

    config = make_pingpong_config(args)
    rows: list[dict[str, object]] = []
    tiles = []
    for sample_dir in samples:
        rows.extend(
            process_sample(
                sample_dir=sample_dir,
                annotation=annotations.get(sample_dir.name),
                config=config,
                output_dir=output_dir,
                tiles=tiles,
            )
        )

    write_csv(output_dir / 'white_error_features.csv', rows)
    write_overview(output_dir / 'white_error_overview.jpg', tiles, columns=args.columns)
    print_report(output_dir, rows, tiles)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='导出白球漏检/误检 ROI。')
    parser.add_argument(
        '--input-dir',
        default='samples/pingpong_three_trays_capture/pingpong_three_trays_capture/pingpong_realtime_capture',
    )
    parser.add_argument('--annotations', default='annotations.json')
    parser.add_argument('--output-dir', default='samples/debug_/runs/pingpong_white_error_debug')
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--columns', type=int, default=6)
    parser.add_argument('--rows', type=int, default=10)
    parser.add_argument('--cols', type=int, default=5)
    parser.add_argument('--roi-radius-ratio', type=float, default=0.34)
    parser.add_argument('--min-ball-ratio', type=float, default=0.16)
    parser.add_argument('--min-white-ratio', type=float, default=0.36)
    parser.add_argument('--min-white-component-ratio', type=float, default=0.30)
    parser.add_argument('--min-white-shape-component-ratio', type=float, default=0.18)
    parser.add_argument('--min-white-component-diameter-ratio', type=float, default=0.95)
    parser.add_argument('--min-white-circularity', type=float, default=0.35)
    parser.add_argument('--max-white-center-offset-ratio', type=float, default=0.55)
    parser.add_argument('--min-yellow-component-ratio', type=float, default=0.12)
    parser.add_argument('--min-color-margin', type=float, default=0.035)
    return parser.parse_args()


def make_pingpong_config(args: argparse.Namespace) -> PingpongDetectorConfig:
    return PingpongDetectorConfig(
        rows=args.rows,
        cols=args.cols,
        roi_radius_ratio=args.roi_radius_ratio,
        min_ball_ratio=args.min_ball_ratio,
        min_white_ratio=args.min_white_ratio,
        min_white_component_ratio=args.min_white_component_ratio,
        min_white_shape_component_ratio=args.min_white_shape_component_ratio,
        min_white_component_diameter_ratio=args.min_white_component_diameter_ratio,
        min_white_circularity=args.min_white_circularity,
        max_white_center_offset_ratio=args.max_white_center_offset_ratio,
        min_yellow_component_ratio=args.min_yellow_component_ratio,
        min_color_margin=args.min_color_margin,
        hole_grid=TrayHoleGridConfig(rows=args.rows, cols=args.cols),
    )


def load_json(path: Path) -> Any:
    with path.open('r', encoding='utf-8') as file:
        return json.load(file)


def find_samples(input_dir: Path) -> list[Path]:
    return sorted(path for path in input_dir.glob('sample_*') if path.is_dir())


def process_sample(
    sample_dir: Path,
    annotation: dict[str, Any] | None,
    config: PingpongDetectorConfig,
    output_dir: Path,
    tiles: list[np.ndarray],
) -> list[dict[str, object]]:
    if annotation is None:
        return []
    image = cv2.imread(str(sample_dir / 'color.jpg'), cv2.IMREAD_COLOR)
    if image is None:
        return []
    truth = annotation_to_matrix(annotation)
    geometry_result = detect_tray_geometry(image, TrayGeometryConfig())
    rows = []
    for candidate in geometry_result.candidates:
        if candidate.corners is None:
            continue
        rectified = rectify(image, candidate.corners, TrayEdgeFitConfig())
        detection_result = detect_pingpong_cells(rectified, config)
        if detection_result.status != 'ok':
            continue
        radius = estimate_roi_radius(
            detection_result.cells,
            rectified.shape[:2],
            config,
        )
        for cell in detection_result.cells:
            true_class = truth.get((candidate.tray_id, cell.row, cell.col), 0)
            is_false_negative = true_class == 1 and cell.class_name != CLASS_WHITE
            is_false_positive = true_class != 1 and cell.class_name == CLASS_WHITE
            if not (is_false_negative or is_false_positive):
                continue
            features = white_features(rectified, cell, radius, config)
            error_type = 'white_false_negative' if is_false_negative else 'white_false_positive'
            row = {
                'sample': sample_dir.name,
                'error_type': error_type,
                'tray_id': candidate.tray_id,
                'row': cell.row,
                'col': cell.col,
                'true_class': true_class,
                'pred_class': cell.class_name,
                'confidence': f'{cell.confidence:.3f}',
                'yellow_ratio': f'{cell.yellow_ratio:.4f}',
                'white_ratio': f'{cell.white_ratio:.4f}',
                'white_component_ratio': f'{cell.white_component_ratio:.4f}',
                **features,
            }
            rows.append(row)
            tile = make_error_tile(rectified, cell, radius, row)
            tiles.append(tile)
            roi_dir = output_dir / error_type
            roi_dir.mkdir(parents=True, exist_ok=True)
            roi_name = f'{sample_dir.name}_tray{candidate.tray_id}_r{cell.row}_c{cell.col}.jpg'
            cv2.imwrite(str(roi_dir / roi_name), tile)
    return rows


def annotation_to_matrix(annotation: dict[str, Any]) -> dict[tuple[int, int, int], int]:
    matrices = annotation.get('matrices_by_tray', {})
    output: dict[tuple[int, int, int], int] = {}
    for tray_id in range(1, TRAYS + 1):
        matrix = matrices.get(str(tray_id), [])
        for row in range(1, ROWS + 1):
            for col in range(1, COLS + 1):
                value = 0
                if row - 1 < len(matrix) and col - 1 < len(matrix[row - 1]):
                    value = int(matrix[row - 1][col - 1])
                output[(tray_id, row, col)] = value if value in (0, 1, 2) else 0
    return output


def white_features(image: np.ndarray, cell, radius: int, config: PingpongDetectorConfig) -> dict[str, object]:  # noqa: ANN001
    patch, circle, local_center = roi_patch(image, cell, radius)
    if patch.size == 0:
        return empty_feature_row()
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    blue, green, red = cv2.split(patch)
    max_channel = np.maximum(np.maximum(red, green), blue)
    min_channel = np.minimum(np.minimum(red, green), blue)
    white = circle & (sat <= 70) & (val >= 135) & (min_channel >= 105) & ((max_channel - min_channel) <= 75)
    valid_count = int(np.count_nonzero(circle))
    component = largest_component_stats(white, circle, valid_count, radius)
    shape_ok = is_white_component_shape_reasonable(component, local_center, radius, config)
    offset_ratio = 999.0
    if component.centroid is not None:
        offset_ratio = float(np.hypot(component.centroid[0] - local_center[0], component.centroid[1] - local_center[1])) / max(1, radius)
    return {
        'component_area_ratio': f'{component.area_ratio:.4f}',
        'component_diameter_ratio': f'{component.diameter_ratio:.4f}',
        'component_circularity': f'{component.circularity:.4f}',
        'component_offset_ratio': f'{offset_ratio:.4f}',
        'white_shape_ok': int(shape_ok),
        'mean_v_circle': f'{float(np.mean(val[circle])):.2f}',
        'mean_s_circle': f'{float(np.mean(sat[circle])):.2f}',
    }


def empty_feature_row() -> dict[str, object]:
    return {
        'component_area_ratio': '0.0000',
        'component_diameter_ratio': '0.0000',
        'component_circularity': '0.0000',
        'component_offset_ratio': '999.0000',
        'white_shape_ok': 0,
        'mean_v_circle': '0.00',
        'mean_s_circle': '0.00',
    }


def roi_patch(image: np.ndarray, cell, radius: int):  # noqa: ANN001
    height, width = image.shape[:2]
    u = int(round(cell.u))
    v = int(round(cell.v))
    x0 = max(0, u - radius)
    y0 = max(0, v - radius)
    x1 = min(width, u + radius + 1)
    y1 = min(height, v + radius + 1)
    patch = image[y0:y1, x0:x1]
    if patch.size == 0:
        return patch, np.zeros((0, 0), dtype=bool), (0.0, 0.0)
    local_u = u - x0
    local_v = v - y0
    yy, xx = np.ogrid[:patch.shape[0], :patch.shape[1]]
    circle = (xx - local_u) ** 2 + (yy - local_v) ** 2 <= radius ** 2
    return patch, circle, (float(local_u), float(local_v))


def make_error_tile(image: np.ndarray, cell, radius: int, row: dict[str, object]) -> np.ndarray:  # noqa: ANN001
    patch, _circle, _local_center = roi_patch(image, cell, radius)
    if patch.size == 0:
        patch = np.zeros((80, 80, 3), dtype=np.uint8)
    scale = 4
    tile = cv2.resize(patch, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    label_height = 58
    canvas = np.full((tile.shape[0] + label_height, max(tile.shape[1], 260), 3), 245, dtype=np.uint8)
    canvas[label_height:label_height + tile.shape[0], :tile.shape[1]] = tile
    color = (0, 0, 220) if row['error_type'] == 'white_false_negative' else (220, 0, 160)
    text1 = f"{row['sample']} T{row['tray_id']} R{row['row']} C{row['col']}"
    text2 = f"{row['error_type'].replace('white_', '')} wr={row['white_ratio']} cr={row['component_area_ratio']}"
    cv2.putText(canvas, text1[:38], (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
    cv2.putText(canvas, text2[:46], (6, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    fieldnames = list(rows[0].keys())
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_overview(path: Path, tiles: list[np.ndarray], columns: int) -> None:
    if not tiles:
        cv2.imwrite(str(path), np.full((80, 320, 3), 245, dtype=np.uint8))
        return
    tile_width = max(tile.shape[1] for tile in tiles)
    tile_height = max(tile.shape[0] for tile in tiles)
    rows = int(np.ceil(len(tiles) / max(1, columns)))
    canvas = np.full((rows * tile_height, columns * tile_width, 3), 245, dtype=np.uint8)
    for index, tile in enumerate(tiles):
        row = index // columns
        col = index % columns
        y = row * tile_height
        x = col * tile_width
        canvas[y:y + tile.shape[0], x:x + tile.shape[1]] = tile
    cv2.imwrite(str(path), canvas)


def print_report(output_dir: Path, rows: list[dict[str, object]], tiles: list[np.ndarray]) -> None:
    false_negative = sum(1 for row in rows if row['error_type'] == 'white_false_negative')
    false_positive = sum(1 for row in rows if row['error_type'] == 'white_false_positive')
    print('')
    print('========== 白球错误 ROI 诊断 ==========')
    print(f'输出目录: {output_dir}')
    print(f'白球漏检 ROI: {false_negative}')
    print(f'白球误检 ROI: {false_positive}')
    print(f'总览图: {output_dir / "white_error_overview.jpg"}')
    print(f'特征表: {output_dir / "white_error_features.csv"}')
    print(f'ROI 数: {len(tiles)}')


if __name__ == '__main__':
    main()
