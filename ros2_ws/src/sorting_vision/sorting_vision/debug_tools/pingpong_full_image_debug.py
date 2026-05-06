"""Debug ping-pong classification directly from full single-tray photos."""

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
    PingpongDetectorConfig,
    detect_pingpong_cells,
    draw_pingpong_cells,
)
from sorting_vision.algorithms.tray_edge_fit import TrayEdgeFitConfig, TrayEdgeFitResult, fit_tray_edges
from sorting_vision.algorithms.tray_hole_grid import TrayHoleGridConfig


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp'}


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到乒乓球苗盘图片')

    edge_config = TrayEdgeFitConfig(
        dark_threshold=args.dark_threshold,
        close_kernel_ratio=args.close_kernel_ratio,
        min_area_ratio=args.min_area_ratio,
        side_band_ratio=args.side_band_ratio,
    )
    pingpong_config = PingpongDetectorConfig(
        roi_radius_ratio=args.roi_radius_ratio,
        min_ball_ratio=args.min_ball_ratio,
        min_white_ratio=args.min_white_ratio,
        min_color_margin=args.min_color_margin,
        hole_grid=TrayHoleGridConfig(rows=args.rows, cols=args.cols),
    )

    rows: list[dict[str, object]] = []
    tiles: list[np.ndarray] = []
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'读取图片失败：{image_path}')

        case_name = safe_path_name(image_path.stem)
        case_dir = output_dir / case_name
        case_dir.mkdir(parents=True, exist_ok=True)

        edge_result = fit_tray_edges(image, edge_config)
        cv2.imwrite(str(case_dir / '01_raw.jpg'), image)
        cv2.imwrite(str(case_dir / '02_edge_fit.jpg'), draw_edge_fit(image, edge_result))

        if edge_result.status == 'ok' and edge_result.rectified is not None:
            rectified = edge_result.rectified
            pingpong_result = detect_pingpong_cells(rectified, pingpong_config)
            classified = draw_pingpong_cells(rectified, pingpong_result)
            cv2.imwrite(str(case_dir / '03_rectified.jpg'), rectified)
            cv2.imwrite(str(case_dir / '04_pingpong_classified.jpg'), classified)
            write_case_csv(case_dir / 'pingpong_cells.csv', case_name, pingpong_result.cells)
            rows.extend(cell_rows(case_name, pingpong_result.cells))
            tile = make_labeled_tile(
                make_compare(image, edge_result, classified),
                f'{image_path.name}: {summary_label(pingpong_result.cells)}',
                900,
            )
        else:
            failed = image.copy()
            cv2.putText(failed, edge_result.message[:90], (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imwrite(str(case_dir / '03_rectified.jpg'), failed)
            cv2.imwrite(str(case_dir / '04_pingpong_classified.jpg'), failed)
            rows.append(failed_row(case_name, edge_result.message))
            tile = make_labeled_tile(failed, f'{image_path.name}: edge failed', 900)

        tiles.append(tile)
        print(f'{image_path}: {edge_result.status} {edge_result.message}')

    write_summary(output_dir / 'pingpong_cells.csv', rows)
    write_final_effects(output_dir, tiles)
    print(f'处理图片数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')
    print(f'CSV：{output_dir / "pingpong_cells.csv"}')


def draw_edge_fit(image: np.ndarray, result: TrayEdgeFitResult) -> np.ndarray:
    output = image.copy()
    if result.body_roi is not None:
        x, y, width, height = result.body_roi
        cv2.rectangle(output, (x, y), (x + width, y + height), (255, 255, 0), 2, cv2.LINE_AA)
    if result.contour is not None:
        cv2.drawContours(output, [result.contour], -1, (0, 255, 0), 2, cv2.LINE_AA)
    if result.corners is not None:
        cv2.polylines(output, [np.round(result.corners).astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
    cv2.putText(output, f'{result.status}: {result.message[:70]}', (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.78, status_color(result), 2, cv2.LINE_AA)
    return output


def make_compare(raw: np.ndarray, edge_result: TrayEdgeFitResult, classified: np.ndarray) -> np.ndarray:
    return horizontal_panels(
        [
            make_labeled_tile(draw_edge_fit(raw, edge_result), 'edge fit', 300),
            make_labeled_tile(edge_result.rectified if edge_result.rectified is not None else raw, 'rectified', 300),
            make_labeled_tile(classified, 'pingpong', 300),
        ]
    )


def summary_label(cells: list[PingpongCell]) -> str:
    yellow = sum(1 for cell in cells if cell.class_name == 'yellow_ball')
    white = sum(1 for cell in cells if cell.class_name == 'white_ball')
    empty = sum(1 for cell in cells if cell.class_name == 'empty')
    return f'Y={yellow} W={white} E={empty}'


def cell_rows(case_name: str, cells: list[PingpongCell]) -> list[dict[str, object]]:
    return [
        {
            'image': case_name,
            'status': 'ok',
            'row': cell.row,
            'col': cell.col,
            'class_name': cell.class_name,
            'confidence': f'{cell.confidence:.3f}',
            'u_rect': f'{cell.u:.3f}',
            'v_rect': f'{cell.v:.3f}',
            'yellow_ratio': f'{cell.yellow_ratio:.4f}',
            'white_ratio': f'{cell.white_ratio:.4f}',
            'ball_ratio': f'{cell.ball_ratio:.4f}',
            'yellow_component_ratio': f'{cell.yellow_component_ratio:.4f}',
            'white_component_ratio': f'{cell.white_component_ratio:.4f}',
            'message': '',
        }
        for cell in cells
    ]


def failed_row(case_name: str, message: str) -> dict[str, object]:
    return {
        'image': case_name,
        'status': 'failed',
        'row': '',
        'col': '',
        'class_name': '',
        'confidence': '',
        'u_rect': '',
        'v_rect': '',
        'yellow_ratio': '',
        'white_ratio': '',
        'ball_ratio': '',
        'yellow_component_ratio': '',
        'white_component_ratio': '',
        'message': message,
    }


def write_case_csv(path: Path, case_name: str, cells: list[PingpongCell]) -> None:
    write_summary(path, cell_rows(case_name, cells))


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        'image',
        'status',
        'row',
        'col',
        'class_name',
        'confidence',
        'u_rect',
        'v_rect',
        'yellow_ratio',
        'white_ratio',
        'ball_ratio',
        'yellow_component_ratio',
        'white_component_ratio',
        'message',
    ]
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_final_effects(output_dir: Path, tiles: list[np.ndarray]) -> None:
    final_dir = output_dir / 'final_effects'
    final_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(final_dir / '_all_pingpong_full_images.jpg'), make_overview(tiles, columns=1))


def make_labeled_tile(image: np.ndarray, label: str, width: int) -> np.ndarray:
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    height, source_width = image.shape[:2]
    scale = width / max(1, source_width)
    resized = cv2.resize(image, (width, max(1, int(round(height * scale)))), interpolation=cv2.INTER_AREA)
    label_height = 32
    canvas = np.full((resized.shape[0] + label_height, width, 3), 245, dtype=np.uint8)
    canvas[label_height:, :] = resized
    cv2.putText(canvas, label[:110], (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def horizontal_panels(panels: list[np.ndarray]) -> np.ndarray:
    height = max(panel.shape[0] for panel in panels)
    width = sum(panel.shape[1] for panel in panels)
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    x = 0
    for panel in panels:
        canvas[:panel.shape[0], x:x + panel.shape[1]] = panel
        x += panel.shape[1]
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


def status_color(result: TrayEdgeFitResult) -> tuple[int, int, int]:
    return (0, 170, 0) if result.status == 'ok' else (0, 0, 255)


def resolve_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        return [Path(args.image)]
    input_dir = Path(args.input_dir)
    paths: list[Path] = []
    for pattern in args.pattern:
        paths.extend(input_dir.glob(pattern))
    return sorted({path for path in paths if path.suffix.lower() in IMAGE_SUFFIXES})


def resolve_output_dir(output_dir: str, run_name: str | None) -> Path:
    base = Path(output_dir)
    if run_name:
        return base / safe_path_name(run_name)
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_pingpong_full_image')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(char if char.isalnum() or char in ('-', '_') else '_' for char in value.strip())
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='整张单盘照片黄/白乒乓球识别调试工具。')
    parser.add_argument('--image', help='单张整盘图片路径。')
    parser.add_argument('--input-dir', default='/home/wht/Desktop/PC-Win/PingPong', help='整盘图片目录。')
    parser.add_argument('--pattern', action='append', default=['*.jpg', '*.jpeg', '*.png'], help='图片匹配模式，可重复。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--rows', type=int, default=10, help='苗盘行数。')
    parser.add_argument('--cols', type=int, default=5, help='苗盘列数。')
    parser.add_argument('--dark-threshold', type=int, default=95, help='苗盘深色主体阈值。')
    parser.add_argument('--close-kernel-ratio', type=float, default=0.045, help='外框闭运算核比例。')
    parser.add_argument('--min-area-ratio', type=float, default=0.16, help='苗盘主体最小面积比例。')
    parser.add_argument('--side-band-ratio', type=float, default=0.075, help='边点选择带宽比例。')
    parser.add_argument('--roi-radius-ratio', type=float, default=0.34, help='穴位检测圆半径占格距比例。')
    parser.add_argument('--min-ball-ratio', type=float, default=0.16, help='判定黄球/通用有球的最小颜色面积比例。')
    parser.add_argument('--min-white-ratio', type=float, default=0.30, help='判定白球的最小白色面积比例。')
    parser.add_argument('--min-color-margin', type=float, default=0.035, help='黄/白比例差最小值。')
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
