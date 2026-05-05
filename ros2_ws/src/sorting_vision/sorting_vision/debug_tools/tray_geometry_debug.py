"""苗盘几何新算法离线调试入口。"""

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

from sorting_vision.algorithms.tray_geometry import (
    TrayGeometryConfig,
    TrayGeometryResult,
    detect_tray_geometry,
)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到待处理图片')

    config = TrayGeometryConfig(
        expected_tray_count=args.expected_tray_count,
        dark_threshold=args.dark_threshold,
        projection_active_ratio=args.projection_active_ratio,
        projection_smooth_px=args.projection_smooth_px,
        min_width_ratio=args.min_width_ratio,
        min_height_ratio=args.min_height_ratio,
        min_area_ratio=args.min_area_ratio,
        x_padding_px=args.x_padding_px,
        morphology_kernel_px=args.morphology_kernel_px,
    )

    rows: list[dict[str, object]] = []
    overview_tiles: list[np.ndarray] = []
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'读取图片失败：{image_path}')

        result = detect_tray_geometry(image, config)
        debug_image = draw_result(image, result)
        output_name = f'{image_path.stem}_tray_geometry_step01.jpg'
        cv2.imwrite(str(output_dir / output_name), debug_image)
        write_tray_cut_outputs(image, image_path.stem, output_dir, result)
        overview_tiles.append(make_labeled_tile(debug_image, f'{image_path.name} {result.status}', width=420))
        rows.extend(result_rows(image_path.name, output_name, result))
        print(f'{image_path.name}: {result.status} candidates={len(result.candidates)}')

    cv2.imwrite(str(output_dir / 'tray_geometry_step01_overview.jpg'), make_overview(overview_tiles, columns=2))
    summary_path = output_dir / 'tray_geometry_step01_summary.csv'
    write_summary(summary_path, rows)
    print(f'处理图片数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')
    print(f'汇总文件：{summary_path}')


def draw_result(image: np.ndarray, result: TrayGeometryResult) -> np.ndarray:
    output = image.copy()
    status_color = (0, 180, 0) if result.status == 'ok' else (0, 165, 255)
    cv2.putText(
        output,
        f'{result.status}: {result.message}',
        (14, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        status_color,
        2,
        cv2.LINE_AA,
    )
    for candidate in result.candidates:
        x, y, width, height = candidate.bbox
        color = tray_color(candidate.tray_id)
        cv2.rectangle(output, (x, y), (x + width, y + height), (170, 170, 170), 1, cv2.LINE_AA)
        if candidate.corners is not None:
            cv2.polylines(
                output,
                [np.round(candidate.corners).astype(np.int32)],
                True,
                color,
                4,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                output,
                'edge failed',
                (x, min(output.shape[0] - 8, y + height + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.circle(
            output,
            (int(round(candidate.center[0])), int(round(candidate.center[1]))),
            4,
            color,
            -1,
            cv2.LINE_AA,
        )
        cv2.putText(
            output,
            f'tray {candidate.tray_id}',
            (x, max(18, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return output


def write_tray_cut_outputs(
    image: np.ndarray,
    image_stem: str,
    output_dir: Path,
    result: TrayGeometryResult,
) -> None:
    image_output_dir = output_dir / image_stem
    image_output_dir.mkdir(parents=True, exist_ok=True)
    for candidate in result.candidates:
        tray_output_dir = image_output_dir / f'tray_{candidate.tray_id}'
        tray_output_dir.mkdir(parents=True, exist_ok=True)
        crop = crop_candidate(image, candidate.bbox, padding=10)
        crop_path = tray_output_dir / '01_crop_raw.jpg'
        crop_edge_path = tray_output_dir / '01b_crop_edge_fit.jpg'
        rectified_path = tray_output_dir / '02_rectified.jpg'
        edges_path = tray_output_dir / '03_rectified_edges.jpg'
        compare_path = tray_output_dir / '04_pipeline_compare.jpg'
        cv2.imwrite(str(crop_path), crop)
        if candidate.corners is None:
            failed = crop.copy()
            cv2.putText(failed, 'edge fit failed; no rectification', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imwrite(str(crop_edge_path), failed)
            cv2.imwrite(str(rectified_path), failed)
            cv2.imwrite(str(edges_path), failed)
            cv2.imwrite(str(compare_path), make_pipeline_compare(crop, failed, failed, failed))
            continue

        crop_edge = draw_crop_edge_fit(crop, candidate.bbox, candidate.corners)
        rectified = rectify_from_corners(image, candidate.corners, width=500, height=1000, padding=60)
        rectified_edges = draw_rectified_edges(rectified)
        cv2.imwrite(str(crop_edge_path), crop_edge)
        cv2.imwrite(str(rectified_path), rectified)
        cv2.imwrite(str(edges_path), rectified_edges)
        cv2.imwrite(str(compare_path), make_pipeline_compare(crop, crop_edge, rectified, rectified_edges))


def crop_candidate(image: np.ndarray, bbox: tuple[int, int, int, int], padding: int) -> np.ndarray:
    x, y, width, height = bbox
    x0 = max(0, x - padding)
    y0 = max(0, y - padding)
    x1 = min(image.shape[1], x + width + padding)
    y1 = min(image.shape[0], y + height + padding)
    return image[y0:y1, x0:x1].copy()


def rectify_from_corners(
    image: np.ndarray,
    corners: np.ndarray,
    width: int,
    height: int,
    padding: int,
) -> np.ndarray:
    source = corners.astype(np.float32)
    target = np.asarray(
        [
            [float(padding), float(padding)],
            [float(padding + width - 1), float(padding)],
            [float(padding + width - 1), float(padding + height - 1)],
            [float(padding), float(padding + height - 1)],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(source, target)
    return cv2.warpPerspective(
        image,
        matrix,
        (width + padding * 2, height + padding * 2),
        borderMode=cv2.BORDER_REPLICATE,
    )


def draw_rectified_edges(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 45, 130)
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=90,
        minLineLength=max(50, min(image.shape[:2]) // 5),
        maxLineGap=16,
    )
    output = image.copy()
    if lines is None:
        cv2.putText(output, 'no rectified edge lines', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
        return output
    for line in lines[:80, 0, :]:
        x1, y1, x2, y2 = [int(value) for value in line]
        cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)
    return output


def draw_crop_edge_fit(
    crop: np.ndarray,
    bbox: tuple[int, int, int, int],
    corners: np.ndarray,
    padding: int = 10,
) -> np.ndarray:
    x, y, _width, _height = bbox
    local_corners = corners.astype(np.float32).copy()
    local_corners[:, 0] -= float(x - padding)
    local_corners[:, 1] -= float(y - padding)
    output = crop.copy()
    cv2.polylines(output, [np.round(local_corners).astype(np.int32)], True, (0, 255, 0), 3, cv2.LINE_AA)
    for index, point in enumerate(local_corners, start=1):
        center = (int(round(point[0])), int(round(point[1])))
        cv2.circle(output, center, 5, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(output, str(index), (center[0] + 5, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return output


def make_pipeline_compare(*images: np.ndarray) -> np.ndarray:
    labels = [
        '01 crop raw',
        '01b edge fit on crop',
        '02 rectified',
        '03 edges on rectified',
    ]
    panels = [
        make_labeled_tile(image, labels[index], width=260)
        for index, image in enumerate(images)
    ]
    height = max(panel.shape[0] for panel in panels)
    width = sum(panel.shape[1] for panel in panels)
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    x = 0
    for panel in panels:
        canvas[:panel.shape[0], x:x + panel.shape[1]] = panel
        x += panel.shape[1]
    return canvas


def tray_color(tray_id: int) -> tuple[int, int, int]:
    colors = {
        1: (0, 255, 255),
        2: (0, 255, 0),
        3: (255, 0, 255),
    }
    return colors.get(tray_id, (255, 255, 255))


def result_rows(image_name: str, output_name: str, result: TrayGeometryResult) -> list[dict[str, object]]:
    if not result.candidates:
        return [
            {
                'image': image_name,
                'output': output_name,
                'result_status': result.status,
                'result_message': result.message,
                'tray_id': '',
                'candidate_status': '',
                'candidate_message': '',
                'x': '',
                'y': '',
                'width': '',
                'height': '',
                'center_u': '',
                'center_v': '',
                'area': '',
                'corner_1': '',
                'corner_2': '',
                'corner_3': '',
                'corner_4': '',
            }
        ]

    rows: list[dict[str, object]] = []
    for candidate in result.candidates:
        x, y, width, height = candidate.bbox
        rows.append(
            {
                'image': image_name,
                'output': output_name,
                'result_status': result.status,
                'result_message': result.message,
                'tray_id': candidate.tray_id,
                'candidate_status': candidate.status,
                'candidate_message': candidate.message,
                'x': x,
                'y': y,
                'width': width,
                'height': height,
                'center_u': f'{candidate.center[0]:.3f}',
                'center_v': f'{candidate.center[1]:.3f}',
                'area': f'{candidate.area:.3f}',
                'corner_1': point_text(candidate.corners, 0),
                'corner_2': point_text(candidate.corners, 1),
                'corner_3': point_text(candidate.corners, 2),
                'corner_4': point_text(candidate.corners, 3),
            }
        )
    return rows


def point_text(corners: np.ndarray | None, index: int) -> str:
    if corners is None:
        return ''
    return f'{float(corners[index][0]):.3f},{float(corners[index][1]):.3f}'


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        'image',
        'output',
        'result_status',
        'result_message',
        'tray_id',
        'candidate_status',
        'candidate_message',
        'x',
        'y',
        'width',
        'height',
        'center_u',
        'center_v',
        'area',
        'corner_1',
        'corner_2',
        'corner_3',
        'corner_4',
    ]
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def make_labeled_tile(image: np.ndarray, label: str, width: int) -> np.ndarray:
    height, source_width = image.shape[:2]
    scale = width / max(1, source_width)
    resized = cv2.resize(
        image,
        (width, max(1, int(round(height * scale)))),
        interpolation=cv2.INTER_AREA,
    )
    label_height = 34
    canvas = np.full((resized.shape[0] + label_height, width, 3), 245, dtype=np.uint8)
    canvas[label_height:, :] = resized
    cv2.putText(canvas, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def make_overview(tiles: list[np.ndarray], columns: int) -> np.ndarray:
    if not tiles:
        return np.full((80, 320, 3), 245, dtype=np.uint8)
    columns = max(1, columns)
    tile_width = max(tile.shape[1] for tile in tiles)
    tile_height = max(tile.shape[0] for tile in tiles)
    rows = int(np.ceil(len(tiles) / columns))
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
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_tray_geometry_step01')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(
        char if char.isalnum() or char in ('-', '_') else '_'
        for char in value.strip()
    )
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='苗盘几何 Step 01 粗定位调试工具。')
    parser.add_argument('--image', help='单张样本 RGB 图片路径。')
    parser.add_argument('--input-dir', default='samples/rgbd', help='批量样本目录。')
    parser.add_argument('--pattern', default='sample_*_color.png', help='批量图片匹配模式。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--expected-tray-count', type=int, default=3, help='期望苗盘数量。')
    parser.add_argument('--dark-threshold', type=int, default=85, help='深色主体阈值。')
    parser.add_argument('--projection-active-ratio', type=float, default=0.10, help='列投影激活比例。')
    parser.add_argument('--projection-smooth-px', type=int, default=19, help='列投影平滑窗口。')
    parser.add_argument('--min-width-ratio', type=float, default=0.08, help='候选最小宽度比例。')
    parser.add_argument('--min-height-ratio', type=float, default=0.35, help='候选最小高度比例。')
    parser.add_argument('--min-area-ratio', type=float, default=0.025, help='候选最小面积比例。')
    parser.add_argument('--x-padding-px', type=int, default=10, help='横向候选区间 padding。')
    parser.add_argument('--morphology-kernel-px', type=int, default=17, help='mask 闭运算核大小。')
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
