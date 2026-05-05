"""苗盘内部网格结构离线调试工具。

这个工具不再把“外框四角 + 机械均分”当最终答案，而是用于批量观察：
1. 三个苗盘粗定位是否稳定。
2. 每个苗盘内部横竖格线能否被 Canny + Hough 检出。
3. 粗透视矫正后内部格线是否仍然倾斜。

输出结果用于决定下一步网格检测算法，不直接作为最终实时节点。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

if __package__ in (None, ''):
    sys.path.append(str(Path(__file__).resolve().parents[3]))

from sorting_vision.debug_tools.legacy.offline_tray_debug import (
    OfflineDebugConfig,
    detect_tray_candidates,
    detect_warped_hole_centers,
    draw_cross,
    draw_candidates,
    order_corners,
    tray_color,
    warp_tray,
)


@dataclass(frozen=True)
class TrayGridDebugConfig:
    """苗盘网格调试配置。"""

    warped_width_px: int
    warped_height_px: int
    max_auto_trays: int
    min_area_ratio: float
    dark_pixel_threshold: int
    column_active_ratio: float
    projection_smooth_px: int
    x_range_padding_px: int
    merge_kernel_px: int
    canny_low_threshold: int
    canny_high_threshold: int
    hough_threshold: int
    hough_min_line_length_px: int
    hough_max_line_gap_px: int
    vertical_angle_tolerance_deg: float
    horizontal_angle_tolerance_deg: float


@dataclass(frozen=True)
class LineStats:
    """单个苗盘内部 Hough 线统计。"""

    vertical_count: int
    horizontal_count: int
    other_count: int
    vertical_angle_median: float | None
    horizontal_angle_median: float | None


@dataclass(frozen=True)
class GridFit:
    """内部网格拟合结果。"""

    warped_to_rectified: np.ndarray
    rectified_to_warped: np.ndarray
    detected_count: int
    matched_count: int
    inlier_count: int
    inlier_ratio: float
    reprojection_error_mean_px: float
    reprojection_error_max_px: float


@dataclass(frozen=True)
class CellRecord:
    """单个穴位在原图中的调试输出。"""

    image: str
    tray_id: int
    col: int
    row: int
    class_id: int
    confidence: float
    u: float
    v: float
    z: float


def main(argv: list[str] | None = None) -> None:
    """命令入口。"""

    args = _parse_args(argv)
    config = load_config(args.config)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = _resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到待处理图片')

    rows: list[dict[str, object]] = []
    cell_records: list[CellRecord] = []
    for image_path in image_paths:
        image_rows, image_records = process_image(image_path, output_dir, config)
        rows.extend(image_rows)
        cell_records.extend(image_records)

    summary_path = output_dir / 'tray_grid_summary.csv'
    cells_path = output_dir / 'tray_grid_cells.csv'
    write_summary(summary_path, rows)
    write_cell_records(cells_path, cell_records)
    print(f'处理图片数量：{len(image_paths)}')
    print(f'已写出汇总：{summary_path}')
    print(f'已写出穴位坐标：{cells_path}')


def process_image(
    image_path: Path,
    output_dir: Path,
    config: TrayGridDebugConfig,
) -> tuple[list[dict[str, object]], list[CellRecord]]:
    """处理单张图片并输出调试图。"""

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'读取图片失败：{image_path}')

    candidates = detect_tray_candidates(image, _as_offline_config(config))
    prefix = output_dir / image_path.stem
    cv2.imwrite(str(prefix) + '_rough_candidates.jpg', draw_candidates(image, candidates))
    depth = _load_depth_for_color(image_path)
    overlay = image.copy()
    fitted_overlay = image.copy()

    rows: list[dict[str, object]] = []
    cell_records: list[CellRecord] = []
    for index, candidate in enumerate(candidates, start=1):
        corners = order_corners(candidate.corners)
        warped = warp_tray(
            image,
            corners,
            config.warped_width_px,
            config.warped_height_px,
        )
        hough_debug, stats = draw_hough_debug(warped, config)
        projection_debug = draw_projection_debug(warped)
        rectified, grid_fit = rectify_by_internal_grid(
            warped,
            config.warped_width_px,
            config.warped_height_px,
        )
        rectified_ok = grid_fit is not None

        if grid_fit is not None:
            draw_fitted_tray_pose(
                fitted_overlay,
                tray_id=index,
                tray_corners=corners,
                grid_fit=grid_fit,
                width=config.warped_width_px,
                height=config.warped_height_px,
            )
            records = build_cell_records(
                image_name=image_path.name,
                tray_id=index,
                tray_corners=corners,
                grid_fit=grid_fit,
                width=config.warped_width_px,
                height=config.warped_height_px,
                depth=depth,
            )
            cell_records.extend(records)
            draw_projected_cells(overlay, records)

        cv2.imwrite(str(prefix) + f'_tray_{index}_warped.jpg', warped)
        cv2.imwrite(str(prefix) + f'_tray_{index}_hough.jpg', hough_debug)
        cv2.imwrite(str(prefix) + f'_tray_{index}_projection.jpg', projection_debug)
        cv2.imwrite(str(prefix) + f'_tray_{index}_grid_rectified.jpg', rectified)

        rows.append(
            {
                'image': image_path.name,
                'tray_index': index,
                'candidate_count': len(candidates),
                'vertical_count': stats.vertical_count,
                'horizontal_count': stats.horizontal_count,
                'other_count': stats.other_count,
                'vertical_angle_median': _format_optional(stats.vertical_angle_median),
                'horizontal_angle_median': _format_optional(
                    stats.horizontal_angle_median
                ),
                'grid_rectified': int(rectified_ok),
                'detected_centers': grid_fit.detected_count if grid_fit else 0,
                'matched_centers': grid_fit.matched_count if grid_fit else 0,
                'inlier_count': grid_fit.inlier_count if grid_fit else 0,
                'inlier_ratio': _format_optional(
                    grid_fit.inlier_ratio if grid_fit else None
                ),
                'reprojection_error_mean_px': _format_optional(
                    grid_fit.reprojection_error_mean_px if grid_fit else None
                ),
                'reprojection_error_max_px': _format_optional(
                    grid_fit.reprojection_error_max_px if grid_fit else None
                ),
                'candidate_center_x': f'{candidate.center_x:.2f}',
                'candidate_area': f'{candidate.area:.2f}',
            }
        )

    if len(candidates) != config.max_auto_trays:
        rows.append(
            {
                'image': image_path.name,
                'tray_index': 0,
                'candidate_count': len(candidates),
                'vertical_count': 0,
                'horizontal_count': 0,
                'other_count': 0,
                'vertical_angle_median': '',
                'horizontal_angle_median': '',
                'grid_rectified': 0,
                'detected_centers': 0,
                'matched_centers': 0,
                'inlier_count': 0,
                'inlier_ratio': '',
                'reprojection_error_mean_px': '',
                'reprojection_error_max_px': '',
                'candidate_center_x': '',
                'candidate_area': '',
            }
        )

    cv2.imwrite(str(prefix) + '_grid_overlay.jpg', overlay)
    cv2.imwrite(str(prefix) + '_fitted_overlay.jpg', fitted_overlay)
    write_cell_records(prefix.with_name(prefix.name + '_cells.csv'), cell_records)
    print(f'{image_path.name}: 粗定位苗盘数量={len(candidates)}')
    return rows, cell_records


def draw_hough_debug(
    warped: np.ndarray,
    config: TrayGridDebugConfig,
) -> tuple[np.ndarray, LineStats]:
    """绘制 Hough 线检测结果。"""

    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(
        blurred,
        config.canny_low_threshold,
        config.canny_high_threshold,
    )
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=config.hough_threshold,
        minLineLength=config.hough_min_line_length_px,
        maxLineGap=config.hough_max_line_gap_px,
    )

    debug = warped.copy()
    vertical_angles: list[float] = []
    horizontal_angles: list[float] = []
    other_count = 0

    if lines is not None:
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(value) for value in line]
            angle = _line_angle_deg(x1, y1, x2, y2)
            if _is_horizontal(angle, config.horizontal_angle_tolerance_deg):
                horizontal_angles.append(angle)
                color = (0, 255, 0)
            elif _is_vertical(angle, config.vertical_angle_tolerance_deg):
                vertical_angles.append(angle)
                color = (255, 180, 0)
            else:
                other_count += 1
                color = (0, 0, 255)
            cv2.line(debug, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)

    stats = LineStats(
        vertical_count=len(vertical_angles),
        horizontal_count=len(horizontal_angles),
        other_count=other_count,
        vertical_angle_median=_median_or_none(vertical_angles),
        horizontal_angle_median=_median_or_none(horizontal_angles),
    )
    cv2.putText(
        debug,
        (
            f'V={stats.vertical_count} H={stats.horizontal_count} '
            f'O={stats.other_count}'
        ),
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return debug, stats


def draw_projection_debug(warped: np.ndarray) -> np.ndarray:
    """绘制简单灰度投影，观察行列结构是否明显。"""

    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    dark = gray < 80
    col_projection = dark.mean(axis=0)
    row_projection = dark.mean(axis=1)
    output = warped.copy()
    height, width = output.shape[:2]

    for x, value in enumerate(col_projection):
        bar = int(round(value * 120))
        cv2.line(output, (x, height - 1), (x, max(0, height - 1 - bar)), (255, 0, 255), 1)
    for y, value in enumerate(row_projection):
        bar = int(round(value * 120))
        cv2.line(output, (0, y), (min(width - 1, bar), y), (0, 255, 255), 1)
    return output


def rectify_by_internal_grid(
    warped: np.ndarray,
    width: int,
    height: int,
) -> tuple[np.ndarray, GridFit | None]:
    """用内部穴位中心拟合 homography，并输出二次矫正图。

    输入的 `warped` 只是粗矫正结果；这里再用检测到的穴位/基质中心点
    拟合到标准 5x10 网格中心坐标，尝试得到真正贴合内部结构的矫正。
    """

    detected = detect_warped_hole_centers(warped)
    if len(detected) < 24:
        debug = warped.copy()
        _draw_status(debug, f'not enough centers: {len(detected)}', False)
        return debug, None

    xs = _cluster_values([point[0] for point in detected], 5)
    ys = _cluster_values([point[1] for point in detected], 10)
    if len(xs) != 5 or len(ys) != 10:
        debug = warped.copy()
        _draw_status(debug, f'cluster failed x={len(xs)} y={len(ys)}', False)
        return debug, None

    source_points: list[list[float]] = []
    target_points: list[list[float]] = []
    for x, y in detected:
        col = int(np.argmin(np.abs(np.asarray(xs) - x)))
        row = int(np.argmin(np.abs(np.asarray(ys) - y)))
        if abs(xs[col] - x) > width * 0.12 or abs(ys[row] - y) > height * 0.06:
            continue
        source_points.append([x, y])
        target_points.append(_ideal_center(col, row, width, height))

    if len(source_points) < 12:
        debug = warped.copy()
        _draw_status(debug, f'too few matched centers: {len(source_points)}', False)
        return debug, None

    source = np.asarray(source_points, dtype=np.float32)
    target = np.asarray(target_points, dtype=np.float32)
    matrix, inliers = cv2.findHomography(source, target, cv2.RANSAC, 6.0)
    if matrix is None or inliers is None or int(inliers.sum()) < 10:
        debug = warped.copy()
        _draw_status(debug, 'homography failed', False)
        return debug, None

    rectified = cv2.warpPerspective(warped, matrix, (width, height))
    draw_ideal_grid(rectified)
    inlier_count = int(inliers.sum())
    projected = cv2.perspectiveTransform(source.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    errors = np.linalg.norm(projected - target, axis=1)
    inlier_mask = inliers.reshape(-1).astype(bool)
    inlier_errors = errors[inlier_mask]
    mean_error = float(np.mean(inlier_errors)) if inlier_errors.size else 0.0
    max_error = float(np.max(inlier_errors)) if inlier_errors.size else 0.0
    inlier_ratio = inlier_count / max(1, len(source_points))
    _draw_status(
        rectified,
        f'inliers={inlier_count}/{len(source_points)} err={mean_error:.1f}px',
        True,
    )
    inverse = np.linalg.inv(matrix)
    fit = GridFit(
        warped_to_rectified=matrix,
        rectified_to_warped=inverse,
        detected_count=len(detected),
        matched_count=len(source_points),
        inlier_count=inlier_count,
        inlier_ratio=inlier_ratio,
        reprojection_error_mean_px=mean_error,
        reprojection_error_max_px=max_error,
    )
    return rectified, fit


def build_cell_records(
    image_name: str,
    tray_id: int,
    tray_corners: np.ndarray,
    grid_fit: GridFit,
    width: int,
    height: int,
    depth: np.ndarray | None,
) -> list[CellRecord]:
    """把内部网格矫正后的 5x10 中心点反投影回原图。"""

    warped_to_original = _warped_to_original_matrix(tray_corners, width, height)
    records: list[CellRecord] = []
    for row in range(10):
        for col in range(5):
            rectified_point = np.asarray(
                [[_ideal_center(col, row, width, height)]],
                dtype=np.float32,
            )
            warped_point = cv2.perspectiveTransform(
                rectified_point,
                grid_fit.rectified_to_warped,
            )
            original_point = cv2.perspectiveTransform(
                warped_point,
                warped_to_original,
            )[0][0]
            u = float(original_point[0])
            v = float(original_point[1])
            records.append(
                CellRecord(
                    image=image_name,
                    tray_id=tray_id,
                    col=col + 1,
                    row=row + 1,
                    class_id=0,
                    confidence=grid_fit.inlier_count / max(1, grid_fit.matched_count),
                    u=u,
                    v=v,
                    z=_sample_depth_mm(depth, u, v),
                )
            )
    return records


def draw_fitted_tray_pose(
    image: np.ndarray,
    tray_id: int,
    tray_corners: np.ndarray,
    grid_fit: GridFit,
    width: int,
    height: int,
) -> None:
    """把内部网格拟合出的苗盘姿态画回原图。"""

    warped_to_original = _warped_to_original_matrix(tray_corners, width, height)
    color = tray_color(tray_id)

    rectified_border = np.asarray(
        [
            [
                [0.0, 0.0],
                [float(width - 1), 0.0],
                [float(width - 1), float(height - 1)],
                [0.0, float(height - 1)],
            ]
        ],
        dtype=np.float32,
    )
    warped_border = cv2.perspectiveTransform(
        rectified_border,
        grid_fit.rectified_to_warped,
    )
    original_border = cv2.perspectiveTransform(
        warped_border,
        warped_to_original,
    )[0]
    cv2.polylines(
        image,
        [np.round(original_border).astype(np.int32)],
        True,
        color,
        2,
        cv2.LINE_AA,
    )

    grid_points: list[list[tuple[int, int]]] = []
    for row in range(10):
        row_points: list[tuple[int, int]] = []
        for col in range(5):
            rectified_point = np.asarray(
                [[_ideal_center(col, row, width, height)]],
                dtype=np.float32,
            )
            warped_point = cv2.perspectiveTransform(
                rectified_point,
                grid_fit.rectified_to_warped,
            )
            original_point = cv2.perspectiveTransform(
                warped_point,
                warped_to_original,
            )[0][0]
            row_points.append(
                (int(round(original_point[0])), int(round(original_point[1])))
            )
        grid_points.append(row_points)

    for row_points in grid_points:
        for start, end in zip(row_points, row_points[1:]):
            cv2.line(image, start, end, color, 1, cv2.LINE_AA)
    for col in range(5):
        col_points = [grid_points[row][col] for row in range(10)]
        for start, end in zip(col_points, col_points[1:]):
            cv2.line(image, start, end, color, 1, cv2.LINE_AA)
    for row_points in grid_points:
        for point in row_points:
            draw_cross(image, point, 4, color)

    center = tuple(np.round(original_border.mean(axis=0)).astype(np.int32))
    cv2.putText(
        image,
        str(tray_id),
        center,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        color,
        2,
        cv2.LINE_AA,
    )


def draw_projected_cells(image: np.ndarray, records: list[CellRecord]) -> None:
    """在原图上画内部网格反投影得到的穴位中心。"""

    if not records:
        return
    color = tray_color(records[0].tray_id)
    by_row: dict[int, list[CellRecord]] = {}
    by_col: dict[int, list[CellRecord]] = {}
    for record in records:
        by_row.setdefault(record.row, []).append(record)
        by_col.setdefault(record.col, []).append(record)
        draw_cross(
            image,
            (int(round(record.u)), int(round(record.v))),
            4,
            color,
        )
    for row_records in by_row.values():
        row_records.sort(key=lambda item: item.col)
        points = [(int(round(item.u)), int(round(item.v))) for item in row_records]
        for start, end in zip(points, points[1:]):
            cv2.line(image, start, end, color, 1, cv2.LINE_AA)
    for col_records in by_col.values():
        col_records.sort(key=lambda item: item.row)
        points = [(int(round(item.u)), int(round(item.v))) for item in col_records]
        for start, end in zip(points, points[1:]):
            cv2.line(image, start, end, color, 1, cv2.LINE_AA)


def _warped_to_original_matrix(
    tray_corners: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    source = np.array(
        [
            [0.0, 0.0],
            [float(width - 1), 0.0],
            [float(width - 1), float(height - 1)],
            [0.0, float(height - 1)],
        ],
        dtype=np.float32,
    )
    return cv2.getPerspectiveTransform(source, tray_corners.astype(np.float32))


def _load_depth_for_color(color_path: Path) -> np.ndarray | None:
    depth_path = color_path.with_name(color_path.name.replace('_color.png', '_depth.png'))
    if not depth_path.exists():
        return None
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.ndim != 2:
        return None
    return depth


def _sample_depth_mm(depth: np.ndarray | None, u: float, v: float) -> float:
    if depth is None:
        return 0.0
    x = int(round(u))
    y = int(round(v))
    if x < 0 or y < 0 or x >= depth.shape[1] or y >= depth.shape[0]:
        return 0.0
    radius = 2
    patch = depth[
        max(0, y - radius): min(depth.shape[0], y + radius + 1),
        max(0, x - radius): min(depth.shape[1], x + radius + 1),
    ]
    valid = patch[patch > 0]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid))


def draw_ideal_grid(image: np.ndarray) -> None:
    """在二次矫正图上绘制标准 5x10 中心网格。"""

    height, width = image.shape[:2]
    centers = [
        _ideal_center(col, row, width, height)
        for row in range(10)
        for col in range(5)
    ]
    color = (0, 255, 255)
    for row in range(10):
        row_points = [
            tuple(int(round(value)) for value in centers[row * 5 + col])
            for col in range(5)
        ]
        for start, end in zip(row_points, row_points[1:]):
            cv2.line(image, start, end, color, 2, cv2.LINE_AA)
    for col in range(5):
        col_points = [
            tuple(int(round(value)) for value in centers[row * 5 + col])
            for row in range(10)
        ]
        for start, end in zip(col_points, col_points[1:]):
            cv2.line(image, start, end, color, 2, cv2.LINE_AA)
    for center in centers:
        cv2.drawMarker(
            image,
            tuple(int(round(value)) for value in center),
            color,
            markerType=cv2.MARKER_CROSS,
            markerSize=8,
            thickness=2,
            line_type=cv2.LINE_AA,
        )


def _ideal_center(col: int, row: int, width: int, height: int) -> list[float]:
    return [
        (col + 0.5) * width / 5.0,
        (row + 0.5) * height / 10.0,
    ]


def _cluster_values(values: list[float], cluster_count: int) -> list[float]:
    if len(values) < cluster_count:
        return []
    data = np.asarray(values, dtype=np.float32).reshape(-1, 1)
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        0.01,
    )
    _compactness, _labels, centers = cv2.kmeans(
        data,
        cluster_count,
        None,
        criteria,
        5,
        cv2.KMEANS_PP_CENTERS,
    )
    return sorted(float(value[0]) for value in centers)


def _draw_status(image: np.ndarray, text: str, ok: bool) -> None:
    color = (0, 255, 0) if ok else (0, 0, 255)
    cv2.putText(
        image,
        text,
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        color,
        2,
        cv2.LINE_AA,
    )


def load_config(path: str | Path) -> TrayGridDebugConfig:
    """读取 YAML 配置。"""

    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    root = data.get('tray_grid_debug', data)
    return TrayGridDebugConfig(
        warped_width_px=int(root.get('warped_width_px', 500)),
        warped_height_px=int(root.get('warped_height_px', 1000)),
        max_auto_trays=int(root.get('max_auto_trays', 3)),
        min_area_ratio=float(root.get('min_area_ratio', 0.02)),
        dark_pixel_threshold=int(root.get('dark_pixel_threshold', 75)),
        column_active_ratio=float(root.get('column_active_ratio', 0.12)),
        projection_smooth_px=int(root.get('projection_smooth_px', 11)),
        x_range_padding_px=int(root.get('x_range_padding_px', 8)),
        merge_kernel_px=int(root.get('merge_kernel_px', 23)),
        canny_low_threshold=int(root.get('canny_low_threshold', 50)),
        canny_high_threshold=int(root.get('canny_high_threshold', 150)),
        hough_threshold=int(root.get('hough_threshold', 70)),
        hough_min_line_length_px=int(root.get('hough_min_line_length_px', 60)),
        hough_max_line_gap_px=int(root.get('hough_max_line_gap_px', 12)),
        vertical_angle_tolerance_deg=float(
            root.get('vertical_angle_tolerance_deg', 18.0)
        ),
        horizontal_angle_tolerance_deg=float(
            root.get('horizontal_angle_tolerance_deg', 18.0)
        ),
    )


def _as_offline_config(config: TrayGridDebugConfig) -> OfflineDebugConfig:
    """把本工具配置转换成粗定位函数需要的配置对象。"""

    return OfflineDebugConfig(
        warped_width_px=config.warped_width_px,
        warped_height_px=config.warped_height_px,
        line_width_px=2,
        center_radius_px=4,
        grid_margin_left_ratio=0.0,
        grid_margin_right_ratio=0.0,
        grid_margin_top_ratio=0.0,
        grid_margin_bottom_ratio=0.0,
        auto_detect=True,
        max_auto_trays=config.max_auto_trays,
        min_area_ratio=config.min_area_ratio,
        dark_pixel_threshold=config.dark_pixel_threshold,
        column_active_ratio=config.column_active_ratio,
        projection_smooth_px=config.projection_smooth_px,
        x_range_padding_px=config.x_range_padding_px,
        merge_kernel_px=config.merge_kernel_px,
        trays=[],
    )


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    """写出批量调试汇总 CSV。"""

    fieldnames = [
        'image',
        'tray_index',
        'candidate_count',
        'vertical_count',
        'horizontal_count',
        'other_count',
        'vertical_angle_median',
        'horizontal_angle_median',
        'grid_rectified',
        'detected_centers',
        'matched_centers',
        'inlier_count',
        'inlier_ratio',
        'reprojection_error_mean_px',
        'reprojection_error_max_px',
        'candidate_center_x',
        'candidate_area',
    ]
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_cell_records(path: Path, records: list[CellRecord]) -> None:
    """写出每个穴位的原图坐标和深度。"""

    fieldnames = [
        'image',
        'tray_id',
        'col',
        'row',
        'class_id',
        'confidence',
        'u',
        'v',
        'z',
    ]
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    'image': record.image,
                    'tray_id': record.tray_id,
                    'col': record.col,
                    'row': record.row,
                    'class_id': record.class_id,
                    'confidence': f'{record.confidence:.3f}',
                    'u': f'{record.u:.2f}',
                    'v': f'{record.v:.2f}',
                    'z': f'{record.z:.2f}',
                }
            )


def _resolve_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        return [Path(args.image)]
    input_dir = Path(args.input_dir)
    return sorted(input_dir.glob(args.pattern))


def resolve_output_dir(output_dir: str, run_name: str | None) -> Path:
    """返回本次调试输出目录，每次运行默认新建独立 run。"""

    base = Path(output_dir)
    if run_name:
        safe_name = _safe_path_name(run_name)
        return base / safe_name
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return base / f'{timestamp}_tray_grid'


def _safe_path_name(value: str) -> str:
    cleaned = ''.join(
        char if char.isalnum() or char in ('-', '_') else '_'
        for char in value.strip()
    )
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def _line_angle_deg(x1: int, y1: int, x2: int, y2: int) -> float:
    angle = float(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
    if angle <= -90.0:
        angle += 180.0
    if angle > 90.0:
        angle -= 180.0
    return angle


def _is_horizontal(angle: float, tolerance: float) -> bool:
    return abs(angle) <= tolerance


def _is_vertical(angle: float, tolerance: float) -> bool:
    return abs(abs(angle) - 90.0) <= tolerance


def _median_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return float(np.median(np.asarray(values, dtype=np.float32)))


def _format_optional(value: float | None) -> str:
    return '' if value is None else f'{value:.2f}'


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='苗盘内部网格结构离线调试工具。')
    parser.add_argument('--image', help='单张样本 RGB 图片路径。')
    parser.add_argument(
        '--input-dir',
        default='samples/rgbd',
        help='批量样本目录。',
    )
    parser.add_argument(
        '--pattern',
        default='sample_*_color.png',
        help='批量图片匹配模式。',
    )
    parser.add_argument(
        '--config',
        default='ros2_ws/src/sorting_vision/config/legacy/tray_grid_debug.yaml',
        help='调试配置路径。',
    )
    parser.add_argument(
        '--output-dir',
        default='samples/debug_/runs',
        help='调试输出根目录。默认每次在该目录下新建一个 run 子目录。',
    )
    parser.add_argument(
        '--run-name',
        help='本次调试输出子目录名。未指定时自动使用时间戳。',
    )
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
