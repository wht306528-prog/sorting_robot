"""Debug hole center baseline on rectified tray images."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys

import cv2
import numpy as np

if __package__ in (None, ''):
    sys.path.append(str(Path(__file__).resolve().parents[2]))

from sorting_vision.algorithms.tray_hole_grid import (
    TrayHoleGridConfig,
    TrayHoleGridResult,
    detect_hole_centers,
    draw_hole_centers,
)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到矫正后的单盘图片')

    config = TrayHoleGridConfig(
        rows=args.rows,
        cols=args.cols,
        margin_x_ratio=args.margin_x_ratio,
        margin_y_ratio=args.margin_y_ratio,
        refine_mode=args.refine_mode,
    )

    rows: list[dict[str, object]] = []
    tiles_by_sample: dict[str, list[np.ndarray]] = {}
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'读取图片失败：{image_path}')

        result = detect_hole_centers(image, config)
        sample_name = safe_path_name(image_path.parent.parent.name)
        tray_name = safe_path_name(image_path.parent.name)
        case_dir = output_dir / sample_name / tray_name
        case_dir.mkdir(parents=True, exist_ok=True)

        overlay = draw_hole_centers(image, result)
        cv2.imwrite(str(case_dir / '01_rectified.jpg'), image)
        cv2.imwrite(str(case_dir / '02_hole_centers.jpg'), overlay)
        tiles_by_sample.setdefault(sample_name, []).append(
            make_labeled_tile(overlay, f'{tray_name}: {result.status}', 360)
        )
        rows.extend(result_rows(sample_name, tray_name, result))
        print(f'{image_path}: {result.status} centers={len(result.centers)}')

    write_summary(output_dir / 'tray_hole_centers.csv', rows)
    write_final_effects(output_dir, tiles_by_sample)
    print(f'处理单盘数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')
    print(f'CSV：{output_dir / "tray_hole_centers.csv"}')


def result_rows(sample_name: str, tray_name: str, result: TrayHoleGridResult) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    tray_id = tray_name.replace('tray_', '')
    for center in result.centers:
        rows.append(
            {
                'sample': sample_name,
                'tray': tray_id,
                'row': center.row,
                'col': center.col,
                'u_rect': f'{center.u:.3f}',
                'v_rect': f'{center.v:.3f}',
                'confidence': f'{center.confidence:.3f}',
                'method': center.method,
            }
        )
    return rows


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = ['sample', 'tray', 'row', 'col', 'u_rect', 'v_rect', 'confidence', 'method']
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_final_effects(output_dir: Path, tiles_by_sample: dict[str, list[np.ndarray]]) -> None:
    final_dir = output_dir / 'final_effects'
    final_dir.mkdir(parents=True, exist_ok=True)
    overview_tiles: list[np.ndarray] = []
    for sample_name, tiles in sorted(tiles_by_sample.items()):
        merged = make_overview(tiles, columns=3)
        cv2.imwrite(str(final_dir / f'{sample_name}_hole_grid.jpg'), merged)
        overview_tiles.append(make_labeled_tile(merged, sample_name, 900))
    cv2.imwrite(str(final_dir / '_all_samples_hole_grid.jpg'), make_overview(overview_tiles, columns=1))


def make_labeled_tile(image: np.ndarray, label: str, width: int) -> np.ndarray:
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    height, source_width = image.shape[:2]
    scale = width / max(1, source_width)
    resized = cv2.resize(image, (width, max(1, int(round(height * scale)))), interpolation=cv2.INTER_AREA)
    label_height = 30
    canvas = np.full((resized.shape[0] + label_height, width, 3), 245, dtype=np.uint8)
    canvas[label_height:, :] = resized
    cv2.putText(canvas, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def make_overview(tiles: list[np.ndarray], columns: int) -> np.ndarray:
    if not tiles:
        return np.full((80, 320, 3), 245, dtype=np.uint8)
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
    return canvas


def resolve_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        return [Path(args.image)]
    return sorted(Path(args.input_dir).glob(args.pattern))


def resolve_output_dir(output_dir: str, run_name: str | None) -> Path:
    base = Path(output_dir)
    if run_name:
        return base / safe_path_name(run_name)
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_tray_hole_grid')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(char if char.isalnum() or char in ('-', '_') else '_' for char in value.strip())
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='矫正单盘 5x10 穴位中心调试工具。')
    parser.add_argument('--image', help='单张 06_rectified.jpg 路径。')
    parser.add_argument('--input-dir', default='samples/debug_/runs/tray_edge_fit_12', help='单盘外框拟合 run 目录。')
    parser.add_argument('--pattern', default='sample_*_color/tray_*/06_rectified.jpg', help='矫正单盘匹配模式。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--rows', type=int, default=10, help='苗盘行数。')
    parser.add_argument('--cols', type=int, default=5, help='苗盘列数。')
    parser.add_argument('--margin-x-ratio', type=float, default=0.145, help='横向边距比例。')
    parser.add_argument('--margin-y-ratio', type=float, default=0.105, help='纵向边距比例。')
    parser.add_argument('--refine-mode', choices=('theory', 'local_dark'), default='theory', help='中心点模式。')
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
