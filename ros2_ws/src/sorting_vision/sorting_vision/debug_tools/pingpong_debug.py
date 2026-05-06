"""Debug ping-pong ball color classification on rectified tray images."""

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

from sorting_vision.algorithms.pingpong_detector import (
    PingpongCell,
    PingpongDetectionResult,
    PingpongDetectorConfig,
    detect_pingpong_cells,
    draw_pingpong_cells,
)
from sorting_vision.algorithms.tray_hole_grid import TrayHoleGridConfig


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到矫正后的单盘图片')

    config = PingpongDetectorConfig(
        rows=args.rows,
        cols=args.cols,
        roi_radius_ratio=args.roi_radius_ratio,
        min_ball_ratio=args.min_ball_ratio,
        min_color_margin=args.min_color_margin,
        hole_grid=TrayHoleGridConfig(rows=args.rows, cols=args.cols),
    )

    rows: list[dict[str, object]] = []
    tiles_by_sample: dict[str, list[np.ndarray]] = {}
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'读取图片失败：{image_path}')

        result = detect_pingpong_cells(image, config)
        sample_name = safe_path_name(image_path.parent.parent.name)
        tray_name = safe_path_name(image_path.parent.name)
        case_dir = output_dir / sample_name / tray_name
        case_dir.mkdir(parents=True, exist_ok=True)

        overlay = draw_pingpong_cells(image, result)
        cv2.imwrite(str(case_dir / '01_rectified.jpg'), image)
        cv2.imwrite(str(case_dir / '02_pingpong_classified.jpg'), overlay)
        write_tray_csv(case_dir / 'pingpong_cells.csv', sample_name, tray_name, result.cells)
        tiles_by_sample.setdefault(sample_name, []).append(
            make_labeled_tile(overlay, f'{tray_name}: {summary_label(result)}', 390)
        )
        rows.extend(result_rows(sample_name, tray_name, result))
        print(f'{image_path}: {summary_label(result)}')

    write_summary(output_dir / 'pingpong_cells.csv', rows)
    write_final_effects(output_dir, tiles_by_sample)
    print(f'处理单盘数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')
    print(f'CSV：{output_dir / "pingpong_cells.csv"}')


def summary_label(result: PingpongDetectionResult) -> str:
    yellow = sum(1 for cell in result.cells if cell.class_name == 'yellow_ball')
    white = sum(1 for cell in result.cells if cell.class_name == 'white_ball')
    empty = sum(1 for cell in result.cells if cell.class_name == 'empty')
    return f'{result.status} Y={yellow} W={white} E={empty}'


def result_rows(
    sample_name: str,
    tray_name: str,
    result: PingpongDetectionResult,
) -> list[dict[str, object]]:
    tray_id = tray_name.replace('tray_', '')
    return [cell_row(sample_name, tray_id, cell) for cell in result.cells]


def cell_row(sample_name: str, tray_id: str, cell: PingpongCell) -> dict[str, object]:
    return {
        'sample': sample_name,
        'tray': tray_id,
        'row': cell.row,
        'col': cell.col,
        'class_name': cell.class_name,
        'confidence': f'{cell.confidence:.3f}',
        'u_rect': f'{cell.u:.3f}',
        'v_rect': f'{cell.v:.3f}',
        'yellow_ratio': f'{cell.yellow_ratio:.4f}',
        'white_ratio': f'{cell.white_ratio:.4f}',
        'ball_ratio': f'{cell.ball_ratio:.4f}',
    }


def write_tray_csv(path: Path, sample_name: str, tray_name: str, cells: list[PingpongCell]) -> None:
    tray_id = tray_name.replace('tray_', '')
    write_summary(path, [cell_row(sample_name, tray_id, cell) for cell in cells])


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        'sample',
        'tray',
        'row',
        'col',
        'class_name',
        'confidence',
        'u_rect',
        'v_rect',
        'yellow_ratio',
        'white_ratio',
        'ball_ratio',
    ]
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
        cv2.imwrite(str(final_dir / f'{sample_name}_pingpong.jpg'), merged)
        overview_tiles.append(make_labeled_tile(merged, sample_name, 1000))
    cv2.imwrite(str(final_dir / '_all_samples_pingpong.jpg'), make_overview(overview_tiles, columns=1))


def make_labeled_tile(image: np.ndarray, label: str, width: int) -> np.ndarray:
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    height, source_width = image.shape[:2]
    scale = width / max(1, source_width)
    resized = cv2.resize(image, (width, max(1, int(round(height * scale)))), interpolation=cv2.INTER_AREA)
    label_height = 30
    canvas = np.full((resized.shape[0] + label_height, width, 3), 245, dtype=np.uint8)
    canvas[label_height:, :] = resized
    cv2.putText(canvas, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 1, cv2.LINE_AA)
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
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_pingpong_detect')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(char if char.isalnum() or char in ('-', '_') else '_' for char in value.strip())
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='矫正单盘黄/白乒乓球识别调试工具。')
    parser.add_argument('--image', help='单张矫正单盘图片路径。')
    parser.add_argument('--input-dir', default='samples/debug_/runs/tray_edge_fit_12', help='单盘外框拟合 run 目录。')
    parser.add_argument('--pattern', default='sample_*_color/tray_*/06_rectified.jpg', help='矫正单盘匹配模式。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--rows', type=int, default=10, help='苗盘行数。')
    parser.add_argument('--cols', type=int, default=5, help='苗盘列数。')
    parser.add_argument('--roi-radius-ratio', type=float, default=0.34, help='穴位检测圆半径占格距比例。')
    parser.add_argument('--min-ball-ratio', type=float, default=0.10, help='判定有球的最小颜色面积比例。')
    parser.add_argument('--min-color-margin', type=float, default=0.035, help='黄/白比例差最小值。')
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
