"""Debug one-tray edge fitting from cropped tray images."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys

import cv2
import numpy as np

if __package__ in (None, ''):
    sys.path.append(str(Path(__file__).resolve().parents[2]))

from sorting_vision.algorithms.tray_edge_fit import TrayEdgeFitConfig, TrayEdgeFitResult, fit_tray_edges


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到单盘 crop 图片')

    config = TrayEdgeFitConfig(
        dark_threshold=args.dark_threshold,
        close_kernel_ratio=args.close_kernel_ratio,
        side_band_ratio=args.side_band_ratio,
        min_area_ratio=args.min_area_ratio,
        min_side_points=args.min_side_points,
    )

    overview_tiles: list[np.ndarray] = []
    final_effect_tiles_by_sample: dict[str, list[np.ndarray]] = {}
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'读取图片失败：{image_path}')

        result = fit_tray_edges(image, config)
        sample_name = safe_path_name(image_path.parent.parent.name)
        tray_name = safe_path_name(image_path.parent.name)
        case_dir = output_dir / sample_name / tray_name
        case_dir.mkdir(parents=True, exist_ok=True)
        write_debug_images(case_dir, image, result)
        final_effect = make_final_effect(image, result)
        final_effect_tiles_by_sample.setdefault(sample_name, []).append(
            make_labeled_tile(final_effect, f'{tray_name}: {result.status}', 360)
        )
        overview_tiles.append(make_labeled_tile(make_compare(image, result), f'{image_path.parent.parent.name}/{image_path.parent.name}: {result.status}', 520))
        print(f'{image_path}: {result.status} {result.message}')

    cv2.imwrite(str(output_dir / 'tray_edge_fit_overview.jpg'), make_overview(overview_tiles, columns=2))
    write_final_effects(output_dir, final_effect_tiles_by_sample)
    print(f'处理单盘数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')


def write_debug_images(output_dir: Path, image: np.ndarray, result: TrayEdgeFitResult) -> None:
    cv2.imwrite(str(output_dir / '01_crop_raw.jpg'), image)
    cv2.imwrite(str(output_dir / '02_dark_mask.jpg'), result.mask)
    cv2.imwrite(str(output_dir / '03_body_contour.jpg'), draw_contour(image, result))
    cv2.imwrite(str(output_dir / '04_side_points.jpg'), draw_side_points(image, result))
    cv2.imwrite(str(output_dir / '05_fitted_four_lines.jpg'), draw_fitted_lines(image, result))
    if result.rectified is not None:
        cv2.imwrite(str(output_dir / '06_rectified.jpg'), result.rectified)
    else:
        failed = image.copy()
        cv2.putText(failed, result.message[:70], (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(output_dir / '06_rectified.jpg'), failed)
    cv2.imwrite(str(output_dir / '07_compare.jpg'), make_compare(image, result))


def write_final_effects(output_dir: Path, tiles_by_sample: dict[str, list[np.ndarray]]) -> None:
    final_dir = output_dir / 'final_effects'
    final_dir.mkdir(parents=True, exist_ok=True)
    overview_tiles: list[np.ndarray] = []
    for sample_name, tiles in sorted(tiles_by_sample.items()):
        merged = make_overview(tiles, columns=3)
        path = final_dir / f'{sample_name}_final_effect.jpg'
        cv2.imwrite(str(path), merged)
        overview_tiles.append(make_labeled_tile(merged, sample_name, 720))
    cv2.imwrite(str(final_dir / '_all_samples_final_effects.jpg'), make_overview(overview_tiles, columns=1))


def make_final_effect(image: np.ndarray, result: TrayEdgeFitResult) -> np.ndarray:
    fitted = draw_fitted_lines(image, result)
    rectified = result.rectified if result.rectified is not None else image
    return horizontal_panels(
        [
            make_labeled_tile(fitted, 'fitted border', 260),
            make_labeled_tile(rectified, 'rectified', 260),
        ]
    )


def draw_contour(image: np.ndarray, result: TrayEdgeFitResult) -> np.ndarray:
    output = image.copy()
    draw_body_roi(output, result)
    if result.contour is not None:
        cv2.drawContours(output, [result.contour], -1, (0, 255, 0), 2, cv2.LINE_AA)
    if result.initial_box is not None:
        cv2.polylines(output, [np.round(result.initial_box).astype(np.int32)], True, (255, 180, 0), 2, cv2.LINE_AA)
    cv2.putText(output, result.message[:70], (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, status_color(result), 1, cv2.LINE_AA)
    return output


def draw_side_points(image: np.ndarray, result: TrayEdgeFitResult) -> np.ndarray:
    output = image.copy()
    draw_body_roi(output, result)
    colors = {
        'top': (0, 255, 255),
        'right': (0, 255, 0),
        'bottom': (255, 0, 255),
        'left': (255, 160, 0),
    }
    for side, points in result.side_points.items():
        color = colors.get(side, (255, 255, 255))
        for point in points:
            cv2.circle(output, (int(round(point[0])), int(round(point[1]))), 2, color, -1, cv2.LINE_AA)
        cv2.putText(output, f'{side}:{len(points)}', label_position(side, image.shape[:2]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return output


def draw_fitted_lines(image: np.ndarray, result: TrayEdgeFitResult) -> np.ndarray:
    output = image.copy()
    draw_body_roi(output, result)
    colors = {
        'top': (0, 255, 255),
        'right': (0, 255, 0),
        'bottom': (255, 0, 255),
        'left': (255, 160, 0),
    }
    for side, line in result.fitted_lines.items():
        draw_line(output, line, colors.get(side, (255, 255, 255)), 2)
    if result.corners is not None:
        cv2.polylines(output, [np.round(result.corners).astype(np.int32)], True, (0, 255, 0), 3, cv2.LINE_AA)
        for index, point in enumerate(result.corners, start=1):
            center = (int(round(point[0])), int(round(point[1])))
            cv2.circle(output, center, 5, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(output, str(index), (center[0] + 5, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(output, result.status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, status_color(result), 2, cv2.LINE_AA)
    return output


def draw_body_roi(image: np.ndarray, result: TrayEdgeFitResult) -> None:
    if result.body_roi is None:
        return
    x, y, width, height = result.body_roi
    cv2.rectangle(image, (x, y), (x + width, y + height), (255, 255, 0), 1, cv2.LINE_AA)


def draw_line(image: np.ndarray, line: tuple[float, float, float], color: tuple[int, int, int], thickness: int) -> None:
    a, b, c = line
    height, width = image.shape[:2]
    points: list[tuple[int, int]] = []
    if abs(b) > 1e-6:
        points.append((0, int(round(-c / b))))
        points.append((width - 1, int(round(-(a * (width - 1) + c) / b))))
    if abs(a) > 1e-6:
        points.append((int(round(-c / a)), 0))
        points.append((int(round(-(b * (height - 1) + c) / a)), height - 1))
    valid = [(x, y) for x, y in points if -width <= x <= width * 2 and -height <= y <= height * 2]
    if len(valid) >= 2:
        cv2.line(image, valid[0], valid[1], color, thickness, cv2.LINE_AA)


def make_compare(image: np.ndarray, result: TrayEdgeFitResult) -> np.ndarray:
    rectified = result.rectified if result.rectified is not None else image
    panels = [
        make_labeled_tile(image, '01 crop', 230),
        make_labeled_tile(result.mask, '02 mask', 230),
        make_labeled_tile(draw_side_points(image, result), '04 side points', 230),
        make_labeled_tile(draw_fitted_lines(image, result), '05 fitted lines', 230),
        make_labeled_tile(rectified, '06 rectified', 230),
    ]
    return horizontal_panels(panels)


def horizontal_panels(panels: list[np.ndarray]) -> np.ndarray:
    height = max(panel.shape[0] for panel in panels)
    width = sum(panel.shape[1] for panel in panels)
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    x = 0
    for panel in panels:
        canvas[:panel.shape[0], x:x + panel.shape[1]] = panel
        x += panel.shape[1]
    return canvas


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


def label_position(side: str, shape: tuple[int, int]) -> tuple[int, int]:
    height, width = shape
    return {
        'top': (8, 20),
        'right': (max(8, width - 90), 42),
        'bottom': (8, max(20, height - 12)),
        'left': (8, 42),
    }.get(side, (8, 20))


def status_color(result: TrayEdgeFitResult) -> tuple[int, int, int]:
    return (0, 170, 0) if result.status == 'ok' else (0, 0, 255)


def resolve_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        return [Path(args.image)]
    return sorted(Path(args.input_dir).glob(args.pattern))


def resolve_output_dir(output_dir: str, run_name: str | None) -> Path:
    base = Path(output_dir)
    if run_name:
        return base / safe_path_name(run_name)
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_tray_edge_fit')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(char if char.isalnum() or char in ('-', '_') else '_' for char in value.strip())
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='单盘 crop 外边界拟合调试工具。')
    parser.add_argument('--image', help='单张 01_crop_raw.jpg 路径。')
    parser.add_argument('--input-dir', default='samples/debug_/runs/tray_geometry_cut_rectify_02', help='单盘 crop 根目录。')
    parser.add_argument('--pattern', default='sample_*_color/tray_*/01_crop_raw.jpg', help='单盘 crop 匹配模式。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--dark-threshold', type=int, default=95, help='苗盘深色主体阈值。')
    parser.add_argument('--close-kernel-ratio', type=float, default=0.045, help='闭运算核尺寸比例。')
    parser.add_argument('--side-band-ratio', type=float, default=0.075, help='边点选择带宽比例。')
    parser.add_argument('--min-area-ratio', type=float, default=0.16, help='苗盘主体最小面积比例。')
    parser.add_argument('--min-side-points', type=int, default=18, help='每条边最少轮廓点数。')
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
