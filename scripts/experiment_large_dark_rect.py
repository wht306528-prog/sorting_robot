#!/usr/bin/env python3
"""离线评估“大面积黑色矩形”苗盘候选方法。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / 'ros2_ws/src/sorting_vision'))

from sorting_vision.algorithms.tray_geometry import (  # noqa: E402
    TrayGeometryConfig,
    locate_large_dark_rect_candidates,
)


def main() -> None:
    args = parse_args()
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    config = TrayGeometryConfig(
        expected_tray_count=args.expected_tray_count,
        dark_threshold=args.dark_threshold,
        morphology_kernel_px=args.morphology_kernel_px,
        large_dark_min_area_ratio=args.min_area_ratio,
        large_dark_min_width_ratio=args.min_width_ratio,
        large_dark_min_height_ratio=args.min_height_ratio,
        large_dark_min_extent=args.min_extent,
        large_dark_max_extent=args.max_extent,
        large_dark_min_edge_density=args.min_edge_density,
        large_dark_min_bright_components=args.min_bright_components,
    )

    image_paths = sorted(input_dir.glob('sample_*/color.jpg'))
    if not image_paths:
        raise RuntimeError(f'没有找到样本图片: {input_dir}/sample_*/color.jpg')

    tiles: list[np.ndarray] = []
    detected = 0
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'读取图片失败: {path}')
        candidates = locate_large_dark_rect_candidates(image, config)
        if candidates:
            detected += 1
        debug = draw_candidates(image, path.parent.name, candidates)
        cv2.imwrite(str(output_dir / f'{path.parent.name}_large_dark_rect.jpg'), debug)
        tiles.append(make_tile(debug, width=360))
        print(f'{path.parent.name}: candidates={len(candidates)}')

    overview = make_overview(tiles, columns=4)
    cv2.imwrite(str(output_dir / '_all_large_dark_rect.jpg'), overview)
    print(f'检出样本: {detected}/{len(image_paths)}')
    print(f'输出目录: {output_dir}')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--input-dir',
        default='samples/debug_/runs/pingpong_realtime_capture',
        help='采集样本目录，里面应包含 sample_*/color.jpg',
    )
    parser.add_argument(
        '--output-dir',
        default='samples/debug_/runs/pingpong_realtime_capture/final_effects/large_dark_rect',
        help='实验输出目录',
    )
    parser.add_argument('--expected-tray-count', type=int, default=3)
    parser.add_argument('--dark-threshold', type=int, default=88)
    parser.add_argument('--morphology-kernel-px', type=int, default=23)
    parser.add_argument('--min-area-ratio', type=float, default=0.045)
    parser.add_argument('--min-width-ratio', type=float, default=0.16)
    parser.add_argument('--min-height-ratio', type=float, default=0.30)
    parser.add_argument('--min-extent', type=float, default=0.16)
    parser.add_argument('--max-extent', type=float, default=0.82)
    parser.add_argument('--min-edge-density', type=float, default=0.08)
    parser.add_argument('--min-bright-components', type=int, default=18)
    return parser.parse_args()


def draw_candidates(image, label: str, candidates) -> np.ndarray:
    output = image.copy()
    cv2.putText(output, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(
        output,
        f'large_dark_rect candidates={len(candidates)}',
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 180, 0) if candidates else (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    for candidate in candidates:
        x, y, width, height = candidate.bbox
        cv2.rectangle(output, (x, y), (x + width, y + height), (180, 180, 180), 1, cv2.LINE_AA)
        if candidate.corners is not None:
            cv2.polylines(output, [np.round(candidate.corners).astype(np.int32)], True, (0, 255, 0), 3, cv2.LINE_AA)
        cv2.putText(
            output,
            f'tray {candidate.tray_id}',
            (x, max(18, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return output


def make_tile(image: np.ndarray, width: int) -> np.ndarray:
    height, current_width = image.shape[:2]
    scale = width / current_width
    return cv2.resize(image, (width, int(height * scale)), interpolation=cv2.INTER_AREA)


def make_overview(images: list[np.ndarray], columns: int) -> np.ndarray:
    if not images:
        return np.zeros((1, 1, 3), dtype=np.uint8)
    rows = (len(images) + columns - 1) // columns
    cell_h = max(image.shape[0] for image in images)
    cell_w = max(image.shape[1] for image in images)
    canvas = np.full((rows * cell_h, columns * cell_w, 3), 245, dtype=np.uint8)
    for index, image in enumerate(images):
        row, col = divmod(index, columns)
        y = row * cell_h
        x = col * cell_w
        canvas[y:y + image.shape[0], x:x + image.shape[1]] = image
    return canvas


if __name__ == '__main__':
    main()
