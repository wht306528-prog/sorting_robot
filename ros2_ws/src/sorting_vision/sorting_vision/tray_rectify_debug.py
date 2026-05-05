"""三苗盘边界识别与透视矫正离线调试工具。

当前目标只做几何调试：
- 从 RGB 样本图中分出 3 个苗盘候选。
- 为每个苗盘估计四角。
- 输出原图边界 overlay、单盘矫正图、before/after 对比图。
- 输出矫正图上的外边缘线段辅助观察。

不做穴位网格、不做矩阵、不做分类、不读取深度。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class TrayCandidate:
    corners: np.ndarray
    contour: np.ndarray
    source: str


@dataclass(frozen=True)
class RectifyResult:
    tray_id: int
    corners: np.ndarray
    contour: np.ndarray
    source: str
    before_crop: np.ndarray
    rectified: np.ndarray
    status: str


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    output_dir = resolve_output_dir(args.output_dir, args.run_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = resolve_images(args)
    if not image_paths:
        raise RuntimeError('没有找到待处理图片')

    summary_rows: list[dict[str, object]] = []
    for image_path in image_paths:
        summary_rows.extend(
            process_image(
                image_path=image_path,
                output_dir=output_dir,
                width=args.width,
                height=args.height,
                padding_ratio=args.padding_ratio,
            )
        )

    summary_path = output_dir / 'tray_rectify_summary.csv'
    write_summary(summary_path, summary_rows)
    print(f'处理图片数量：{len(image_paths)}')
    print(f'输出目录：{output_dir}')
    print(f'汇总文件：{summary_path}')


def process_image(
    image_path: Path,
    output_dir: Path,
    width: int,
    height: int,
    padding_ratio: float,
) -> list[dict[str, object]]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'读取图片失败：{image_path}')

    candidates = find_tray_candidates(image)
    overlay = image.copy()
    rows: list[dict[str, object]] = []

    for tray_id, candidate in enumerate(candidates, start=1):
        result = rectify_one_tray(image, tray_id, candidate, width, height, padding_ratio)
        draw_result_overlay(overlay, result)

        cv2.imwrite(
            str(output_dir / f'{image_path.stem}_tray_{tray_id}_rectified.jpg'),
            result.rectified,
        )
        cv2.imwrite(
            str(output_dir / f'{image_path.stem}_tray_{tray_id}_rectified_edges.jpg'),
            draw_rectified_edge_segments(result.rectified),
        )
        cv2.imwrite(
            str(output_dir / f'{image_path.stem}_tray_{tray_id}_before_after.jpg'),
            make_before_after(result),
        )
        rows.append(
            {
                'image': image_path.name,
                'tray_id': tray_id,
                'status': result.status,
                'source': result.source,
                'corner_1': point_text(result.corners[0]),
                'corner_2': point_text(result.corners[1]),
                'corner_3': point_text(result.corners[2]),
                'corner_4': point_text(result.corners[3]),
            }
        )

    cv2.imwrite(str(output_dir / f'{image_path.stem}_rectify_overlay.jpg'), overlay)
    print(f'{image_path.name}: 苗盘数量={len(candidates)}')
    return rows


def find_tray_candidates(image: np.ndarray) -> list[TrayCandidate]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dark_mask = gray < 80
    projection = dark_mask.sum(axis=0).astype(np.float32)
    projection = np.convolve(projection, np.ones(13, dtype=np.float32) / 13.0, mode='same')
    active = projection > image.shape[0] * 0.10
    ranges = active_ranges(active, min_width=max(50, image.shape[1] // 30), padding=8)

    candidates: list[TrayCandidate] = []
    for x_start, x_end in ranges:
        candidates.extend(candidates_from_range(dark_mask, x_start, x_end, image.shape))

    if len(candidates) < 3:
        candidates.extend(split_wide_ranges(dark_mask, ranges, image.shape))
    if len(candidates) < 3:
        candidates.extend(candidates_from_contours(dark_mask, image.shape))

    candidates = dedupe_candidates(candidates)
    candidates.sort(key=lambda item: float(np.mean(item.corners[:, 0])))
    return candidates[:3]


def split_wide_ranges(
    dark_mask: np.ndarray,
    ranges: list[tuple[int, int]],
    image_shape: tuple[int, ...],
) -> list[TrayCandidate]:
    if not ranges:
        ys, xs = np.where(dark_mask)
        if xs.size == 0:
            return []
        ranges = [(int(xs.min()), int(xs.max()))]

    candidates: list[TrayCandidate] = []
    image_width = image_shape[1]
    expected_width = image_width / 3.6
    for x_start, x_end in ranges:
        width = x_end - x_start + 1
        split_count = int(round(width / expected_width))
        split_count = max(1, min(3, split_count))
        if split_count <= 1 and width > image_width * 0.55:
            split_count = 3
        if split_count <= 1:
            continue
        for index in range(split_count):
            sub_start = int(round(x_start + index * width / split_count))
            sub_end = int(round(x_start + (index + 1) * width / split_count)) - 1
            candidates.extend(candidates_from_range(dark_mask, sub_start, sub_end, image_shape))
    return candidates


def active_ranges(active: np.ndarray, min_width: int, padding: int) -> list[tuple[int, int]]:
    ranges: list[tuple[int, int]] = []
    start: int | None = None
    for index, value in enumerate(active):
        if value and start is None:
            start = index
        elif not value and start is not None:
            if index - start >= min_width:
                ranges.append((max(0, start - padding), min(len(active) - 1, index + padding)))
            start = None
    if start is not None and len(active) - start >= min_width:
        ranges.append((max(0, start - padding), len(active) - 1))
    return ranges


def candidates_from_range(
    dark_mask: np.ndarray,
    x_start: int,
    x_end: int,
    image_shape: tuple[int, ...],
) -> list[TrayCandidate]:
    crop = dark_mask[:, x_start:x_end + 1].astype(np.uint8) * 255
    crop = cv2.morphologyEx(crop, cv2.MORPH_CLOSE, np.ones((19, 19), np.uint8), iterations=2)
    contours, _ = cv2.findContours(crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = float(image_shape[0] * image_shape[1])

    candidates: list[TrayCandidate] = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
        if cv2.contourArea(contour) < image_area * 0.015:
            continue
        shifted = contour.copy()
        shifted[:, :, 0] += x_start
        candidate = candidate_from_contour(shifted, dark_mask)
        if looks_like_tray(candidate.corners):
            candidates.append(candidate)
    return candidates


def candidates_from_contours(
    dark_mask: np.ndarray,
    image_shape: tuple[int, ...],
) -> list[TrayCandidate]:
    mask = dark_mask.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = float(image_shape[0] * image_shape[1])

    candidates: list[TrayCandidate] = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:8]:
        if cv2.contourArea(contour) < image_area * 0.015:
            continue
        candidate = candidate_from_contour(contour, dark_mask)
        if looks_like_tray(candidate.corners):
            candidates.append(candidate)
    return candidates


def candidate_from_contour(contour: np.ndarray, dark_mask: np.ndarray | None) -> TrayCandidate:
    contour_copy = contour.copy()
    line_quad = line_refined_corners_from_mask(contour, dark_mask)
    if line_quad is not None and looks_like_tray(line_quad):
        return TrayCandidate(corners=line_quad, contour=contour_copy, source='line_quad')

    quad = contour_quad_corners(contour)
    if quad is not None and looks_like_tray(quad):
        return TrayCandidate(corners=quad, contour=contour_copy, source='outer_quad')

    return TrayCandidate(
        corners=rect_corners_from_contour(contour),
        contour=contour_copy,
        source='fallback_rect',
    )


def line_refined_corners_from_mask(
    contour: np.ndarray,
    dark_mask: np.ndarray | None,
) -> np.ndarray | None:
    if dark_mask is None:
        return None

    points = contour.reshape(-1, 2).astype(np.float32)
    min_x = max(0, int(np.floor(float(points[:, 0].min()) - 8)))
    min_y = max(0, int(np.floor(float(points[:, 1].min()) - 8)))
    max_x = min(dark_mask.shape[1], int(np.ceil(float(points[:, 0].max()) + 8)))
    max_y = min(dark_mask.shape[0], int(np.ceil(float(points[:, 1].max()) + 8)))
    if max_x - min_x < 40 or max_y - min_y < 80:
        return None

    crop = dark_mask[min_y:max_y, min_x:max_x].astype(np.uint8) * 255
    crop = cv2.morphologyEx(crop, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1)
    edges = cv2.Canny(crop, 50, 150)
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=35,
        minLineLength=max(55, min(crop.shape[:2]) // 3),
        maxLineGap=12,
    )
    if lines is None:
        return None

    horizontal: list[tuple[float, float]] = []
    vertical: list[tuple[float, float]] = []
    for x1, y1, x2, y2 in lines[:, 0, :]:
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        length = float(np.hypot(dx, dy))
        if length < 55.0:
            continue
        angle = normalize_line_angle(np.degrees(np.arctan2(dy, dx)))
        if abs(angle) < 25.0:
            horizontal.append((angle, length))
        elif abs(angle) > 65.0:
            vertical.append((angle, length))

    if len(horizontal) < 2 or len(vertical) < 2:
        return None

    horizontal_angle = weighted_angle(horizontal)
    vertical_angle = weighted_angle(vertical)
    if abs(abs(vertical_angle - horizontal_angle) - 90.0) > 18.0:
        return None
    return oriented_bbox_from_axes(points, horizontal_angle, vertical_angle)


def contour_quad_corners(contour: np.ndarray) -> np.ndarray | None:
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    if perimeter <= 1.0:
        return None
    for epsilon_ratio in (0.012, 0.018, 0.025, 0.035, 0.050, 0.070):
        approx = cv2.approxPolyDP(hull, epsilon_ratio * perimeter, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            corners = order_corners(approx.reshape(4, 2).astype(np.float32))
            if quad_area(corners) >= cv2.contourArea(contour) * 0.65:
                return corners
    return None


def rect_corners_from_contour(contour: np.ndarray) -> np.ndarray:
    rect = cv2.minAreaRect(contour)
    return order_corners(cv2.boxPoints(rect).astype(np.float32))


def normalize_line_angle(angle: float) -> float:
    while angle <= -90.0:
        angle += 180.0
    while angle > 90.0:
        angle -= 180.0
    return angle


def weighted_angle(values: list[tuple[float, float]]) -> float:
    weights = np.asarray([item[1] for item in values], dtype=np.float32)
    angles = np.asarray([item[0] for item in values], dtype=np.float32)
    return float(np.average(angles, weights=weights))


def oriented_bbox_from_axes(
    points: np.ndarray,
    horizontal_angle: float,
    vertical_angle: float,
) -> np.ndarray | None:
    horizontal = np.asarray(
        [np.cos(np.deg2rad(horizontal_angle)), np.sin(np.deg2rad(horizontal_angle))],
        dtype=np.float32,
    )
    vertical = np.asarray(
        [np.cos(np.deg2rad(vertical_angle)), np.sin(np.deg2rad(vertical_angle))],
        dtype=np.float32,
    )
    basis = np.column_stack([horizontal, vertical])
    if abs(float(np.linalg.det(basis))) < 0.2:
        return None
    coeffs = (np.linalg.inv(basis) @ points.T).T
    mins = coeffs.min(axis=0)
    maxs = coeffs.max(axis=0)
    corners = np.asarray(
        [
            [mins[0], mins[1]],
            [maxs[0], mins[1]],
            [maxs[0], maxs[1]],
            [mins[0], maxs[1]],
        ],
        dtype=np.float32,
    )
    return order_corners((basis @ corners.T).T)


def dedupe_candidates(candidates: list[TrayCandidate]) -> list[TrayCandidate]:
    output: list[TrayCandidate] = []
    for candidate in candidates:
        cx = float(np.mean(candidate.corners[:, 0]))
        width = float(max(candidate.corners[:, 0]) - min(candidate.corners[:, 0]))
        duplicate = False
        for other in output:
            other_cx = float(np.mean(other.corners[:, 0]))
            other_width = float(max(other.corners[:, 0]) - min(other.corners[:, 0]))
            if abs(cx - other_cx) < max(width, other_width) * 0.35:
                duplicate = True
                break
        if not duplicate:
            output.append(candidate)
    return output


def looks_like_tray(corners: np.ndarray) -> bool:
    width = 0.5 * (
        np.linalg.norm(corners[1] - corners[0])
        + np.linalg.norm(corners[2] - corners[3])
    )
    height = 0.5 * (
        np.linalg.norm(corners[3] - corners[0])
        + np.linalg.norm(corners[2] - corners[1])
    )
    if width <= 1.0 or height <= 1.0:
        return False
    ratio = max(width, height) / min(width, height)
    return 1.4 <= ratio <= 3.5


def quad_area(corners: np.ndarray) -> float:
    return float(cv2.contourArea(order_corners(corners).astype(np.float32)))


def rectify_one_tray(
    image: np.ndarray,
    tray_id: int,
    candidate: TrayCandidate,
    width: int,
    height: int,
    padding_ratio: float,
) -> RectifyResult:
    rectified = warp_padded_from_corners(image, candidate.corners, width, height, padding_ratio)
    return RectifyResult(
        tray_id=tray_id,
        corners=candidate.corners,
        contour=candidate.contour,
        source=candidate.source,
        before_crop=crop_before_view(image, candidate.corners, padding_ratio),
        rectified=rectified,
        status=candidate.source,
    )


def warp_padded_from_corners(
    image: np.ndarray,
    corners: np.ndarray,
    width: int,
    height: int,
    padding_ratio: float,
) -> np.ndarray:
    pad_x = max(0, int(round(width * padding_ratio)))
    pad_y = max(0, int(round(height * padding_ratio)))
    output_width = width + pad_x * 2
    output_height = height + pad_y * 2
    source = order_corners(corners)
    target = np.asarray(
        [
            [float(pad_x), float(pad_y)],
            [float(pad_x + width - 1), float(pad_y)],
            [float(pad_x + width - 1), float(pad_y + height - 1)],
            [float(pad_x), float(pad_y + height - 1)],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(source, target)
    return cv2.warpPerspective(
        image,
        matrix,
        (output_width, output_height),
        borderMode=cv2.BORDER_REPLICATE,
    )


def crop_before_view(image: np.ndarray, corners: np.ndarray, padding_ratio: float) -> np.ndarray:
    ordered = order_corners(corners)
    min_x = float(np.min(ordered[:, 0]))
    max_x = float(np.max(ordered[:, 0]))
    min_y = float(np.min(ordered[:, 1]))
    max_y = float(np.max(ordered[:, 1]))
    pad_x = (max_x - min_x) * padding_ratio
    pad_y = (max_y - min_y) * padding_ratio
    x0 = max(0, int(np.floor(min_x - pad_x)))
    y0 = max(0, int(np.floor(min_y - pad_y)))
    x1 = min(image.shape[1], int(np.ceil(max_x + pad_x)))
    y1 = min(image.shape[0], int(np.ceil(max_y + pad_y)))
    crop = image[y0:y1, x0:x1].copy()
    shifted = ordered - np.asarray([x0, y0], dtype=np.float32)
    cv2.polylines(crop, [np.round(shifted).astype(np.int32)], True, (0, 255, 255), 2, cv2.LINE_AA)
    return crop


def make_before_after(result: RectifyResult) -> np.ndarray:
    before = resize_to_height(result.before_crop, 520)
    after = resize_to_height(result.rectified, 520)
    gap = np.full((520, 18, 3), 245, dtype=np.uint8)
    canvas = np.concatenate([before, gap, after], axis=1)
    cv2.putText(canvas, 'before', (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(
        canvas,
        f'after {result.status}',
        (before.shape[1] + gap.shape[1] + 12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas


def draw_result_overlay(image: np.ndarray, result: RectifyResult) -> None:
    cv2.polylines(
        image,
        [np.round(result.corners).astype(np.int32)],
        True,
        (0, 255, 255),
        3,
        cv2.LINE_AA,
    )


def draw_rectified_edge_segments(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    roi = dark_body_roi(gray)
    if roi is None:
        roi = (0, 0, image.shape[1], image.shape[0])
    roi_x, roi_y, roi_w, roi_h = roi
    roi_gray = gray[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
    roi_edges = cv2.Canny(cv2.GaussianBlur(roi_gray, (5, 5), 0), 45, 130)
    lines = cv2.HoughLinesP(
        roi_edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=70,
        minLineLength=max(35, min(roi_w, roi_h) // 5),
        maxLineGap=14,
    )

    output = image.copy()
    if lines is None:
        return output

    raw_lines = [
        (
            int(line[0] + roi_x),
            int(line[1] + roi_y),
            int(line[2] + roi_x),
            int(line[3] + roi_y),
        )
        for line in lines[:, 0, :]
    ]
    side_segments = select_outer_hough_segments_by_side(raw_lines, roi)
    for line in fit_outer_edge_lines(side_segments, roi):
        cv2.line(output, line[0], line[1], (0, 255, 0), 3, cv2.LINE_AA)
    return output


def select_outer_hough_segments_by_side(
    raw_lines: list[tuple[int, int, int, int]],
    roi: tuple[int, int, int, int],
) -> dict[str, list[tuple[int, int, int, int]]]:
    vertical: list[tuple[float, tuple[int, int, int, int]]] = []
    horizontal: list[tuple[float, tuple[int, int, int, int]]] = []
    for line in raw_lines:
        x1, y1, x2, y2 = line
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        length = float(np.hypot(dx, dy))
        if length < 35.0:
            continue
        angle = normalize_line_angle(np.degrees(np.arctan2(dy, dx)))
        if abs(angle) < 18.0:
            horizontal.append((length, line))
        elif abs(angle) > 72.0:
            vertical.append((length, line))

    return {
        'left': select_side_lines(vertical, side='left', roi=roi),
        'right': select_side_lines(vertical, side='right', roi=roi),
        'top': select_side_lines(horizontal, side='top', roi=roi),
        'bottom': select_side_lines(horizontal, side='bottom', roi=roi),
    }


def fit_outer_edge_lines(
    side_segments: dict[str, list[tuple[int, int, int, int]]],
    roi: tuple[int, int, int, int],
) -> list[tuple[tuple[int, int], tuple[int, int]]]:
    fitted: list[tuple[tuple[int, int], tuple[int, int]]] = []
    for side in ('left', 'right', 'top', 'bottom'):
        line = fit_side_line(side_segments.get(side, []), side, roi)
        if line is not None:
            fitted.append(line)
    return fitted


def fit_side_line(
    segments: list[tuple[int, int, int, int]],
    side: str,
    roi: tuple[int, int, int, int],
) -> tuple[tuple[int, int], tuple[int, int]] | None:
    if not segments:
        return None

    points = np.asarray(
        [(x1, y1) for x1, y1, _x2, _y2 in segments]
        + [(x2, y2) for _x1, _y1, x2, y2 in segments],
        dtype=np.float32,
    )
    if len(points) < 2:
        return None

    vx, vy, x0, y0 = cv2.fitLine(points, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(-1)
    if abs(float(vx)) < 1e-6 and abs(float(vy)) < 1e-6:
        return None

    roi_x, roi_y, roi_w, roi_h = roi
    x_min = roi_x
    x_max = roi_x + roi_w - 1
    y_min = roi_y
    y_max = roi_y + roi_h - 1

    if side in ('left', 'right'):
        if abs(float(vy)) < 1e-6:
            return None
        y1 = y_min
        y2 = y_max
        x1 = float(x0) + (y1 - float(y0)) * float(vx) / float(vy)
        x2 = float(x0) + (y2 - float(y0)) * float(vx) / float(vy)
        return clip_line_to_image((x1, y1), (x2, y2), roi)

    if abs(float(vx)) < 1e-6:
        return None
    x1 = x_min
    x2 = x_max
    y1 = float(y0) + (x1 - float(x0)) * float(vy) / float(vx)
    y2 = float(y0) + (x2 - float(x0)) * float(vy) / float(vx)
    return clip_line_to_image((x1, y1), (x2, y2), roi)


def clip_line_to_image(
    point_a: tuple[float, float],
    point_b: tuple[float, float],
    roi: tuple[int, int, int, int],
) -> tuple[tuple[int, int], tuple[int, int]]:
    roi_x, roi_y, roi_w, roi_h = roi
    x_min = roi_x
    x_max = roi_x + roi_w - 1
    y_min = roi_y
    y_max = roi_y + roi_h - 1

    x1 = int(round(np.clip(point_a[0], x_min, x_max)))
    y1 = int(round(np.clip(point_a[1], y_min, y_max)))
    x2 = int(round(np.clip(point_b[0], x_min, x_max)))
    y2 = int(round(np.clip(point_b[1], y_min, y_max)))
    return (x1, y1), (x2, y2)


def select_side_lines(
    lines: list[tuple[float, tuple[int, int, int, int]]],
    side: str,
    roi: tuple[int, int, int, int],
) -> list[tuple[int, int, int, int]]:
    if not lines:
        return []
    if side in ('left', 'right'):
        scored = [((line[0] + line[2]) * 0.5, length, line) for length, line in lines]
    else:
        scored = [((line[1] + line[3]) * 0.5, length, line) for length, line in lines]

    roi_x, roi_y, roi_w, roi_h = roi
    if side == 'left':
        target = roi_x
    elif side == 'right':
        target = roi_x + roi_w - 1
    elif side == 'top':
        target = roi_y
    else:
        target = roi_y + roi_h - 1

    scored.sort(key=lambda item: abs(item[0] - target))
    anchor = scored[0][0]
    selected = [line for position, _length, line in scored if abs(position - anchor) <= 35.0]
    selected.sort(
        key=lambda line: float(np.hypot(line[2] - line[0], line[3] - line[1])),
        reverse=True,
    )
    return selected[:8]


def dark_body_roi(gray: np.ndarray) -> tuple[int, int, int, int] | None:
    mask = gray < 95
    col_projection = mask.sum(axis=0).astype(np.float32)
    row_projection = mask.sum(axis=1).astype(np.float32)
    col_projection = smooth_projection(col_projection, kernel_size=17)
    row_projection = smooth_projection(row_projection, kernel_size=17)

    x_ranges = active_ranges(
        col_projection > gray.shape[0] * 0.08,
        min_width=max(40, gray.shape[1] // 5),
        padding=0,
    )
    y_ranges = active_ranges(
        row_projection > gray.shape[1] * 0.08,
        min_width=max(80, gray.shape[0] // 4),
        padding=0,
    )
    if not x_ranges or not y_ranges:
        return None

    x0, x1 = choose_center_range(x_ranges, gray.shape[1] // 2)
    y0, y1 = choose_center_range(y_ranges, gray.shape[0] // 2)
    x = x0
    y = y0
    w = x1 - x0 + 1
    h = y1 - y0 + 1
    pad = max(8, int(round(min(w, h) * 0.04)))
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(gray.shape[1], x + w + pad)
    y1 = min(gray.shape[0], y + h + pad)
    return x0, y0, x1 - x0, y1 - y0


def smooth_projection(projection: np.ndarray, kernel_size: int) -> np.ndarray:
    kernel = np.ones(kernel_size, dtype=np.float32) / float(kernel_size)
    return np.convolve(projection, kernel, mode='same')


def choose_center_range(ranges: list[tuple[int, int]], center: int) -> tuple[int, int]:
    for start, end in ranges:
        if start <= center <= end:
            return start, end
    return min(ranges, key=lambda item: abs(((item[0] + item[1]) // 2) - center))


def resize_to_height(image: np.ndarray, target_height: int) -> np.ndarray:
    height, width = image.shape[:2]
    scale = target_height / max(1, height)
    target_width = max(1, int(round(width * scale)))
    return cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)


def order_corners(corners: np.ndarray) -> np.ndarray:
    points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).reshape(-1)
    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(diffs)]
    ordered[3] = points[np.argmax(diffs)]
    return ordered


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = ['image', 'tray_id', 'status', 'source', 'corner_1', 'corner_2', 'corner_3', 'corner_4']
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def point_text(point: np.ndarray) -> str:
    return f'{float(point[0]):.1f},{float(point[1]):.1f}'


def resolve_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        return [Path(args.image)]
    return sorted(Path(args.input_dir).glob(args.pattern))


def resolve_output_dir(output_dir: str, run_name: str | None) -> Path:
    base = Path(output_dir)
    if run_name:
        return base / safe_path_name(run_name)
    return base / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_tray_rectify')


def safe_path_name(value: str) -> str:
    cleaned = ''.join(
        char if char.isalnum() or char in ('-', '_') else '_'
        for char in value.strip()
    )
    return cleaned or datetime.now().strftime('%Y%m%d_%H%M%S')


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='三苗盘边界识别与透视矫正调试工具。')
    parser.add_argument('--image', help='单张样本 RGB 图片路径。')
    parser.add_argument('--input-dir', default='samples/rgbd', help='批量样本目录。')
    parser.add_argument('--pattern', default='sample_*_color.png', help='图片匹配模式。')
    parser.add_argument('--output-dir', default='samples/debug_/runs', help='输出根目录。')
    parser.add_argument('--run-name', help='本次输出子目录名。')
    parser.add_argument('--width', type=int, default=500, help='拉正图宽度。')
    parser.add_argument('--height', type=int, default=1000, help='拉正图高度。')
    parser.add_argument(
        '--padding-ratio',
        type=float,
        default=0.16,
        help='拉正图四周额外保留的背景比例。',
    )
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
