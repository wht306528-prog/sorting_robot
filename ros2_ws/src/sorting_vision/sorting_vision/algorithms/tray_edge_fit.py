"""Fit one tray's outer border from an already cropped tray image."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


LineABC = tuple[float, float, float]


@dataclass(frozen=True)
class TrayEdgeFitConfig:
    dark_threshold: int = 95
    close_kernel_ratio: float = 0.045
    open_kernel_px: int = 3
    min_area_ratio: float = 0.16
    side_band_ratio: float = 0.075
    min_side_points: int = 18
    rectified_width: int = 500
    rectified_height: int = 1000
    rectified_padding: int = 50


@dataclass(frozen=True)
class TrayEdgeFitResult:
    status: str
    message: str
    mask: np.ndarray
    contour: np.ndarray | None
    initial_box: np.ndarray | None
    side_points: dict[str, np.ndarray]
    fitted_lines: dict[str, LineABC]
    corners: np.ndarray | None
    rectified: np.ndarray | None


def fit_tray_edges(
    image: np.ndarray,
    config: TrayEdgeFitConfig | None = None,
) -> TrayEdgeFitResult:
    """Fit four outer border lines from a single tray crop.

    This function assumes the input is already cropped to one tray plus a little
    surrounding background. It deliberately does not know about the original
    full image.
    """

    if config is None:
        config = TrayEdgeFitConfig()
    if image.ndim != 3 or image.shape[2] != 3:
        return failed_result('input image must be HxWx3', np.zeros(image.shape[:2], dtype=np.uint8))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = build_tray_body_mask(gray, config)
    mask = keep_center_body_component(mask, config)
    contour = find_largest_body_contour(mask, image.shape[:2], config)
    if contour is None:
        return failed_result('no large dark tray body contour', mask)

    initial_box = order_corners(cv2.boxPoints(cv2.minAreaRect(contour)).astype(np.float32))
    side_points = collect_side_points(contour, initial_box, image.shape[:2], config)
    missing = [side for side in ('top', 'right', 'bottom', 'left') if len(side_points[side]) < config.min_side_points]
    if missing:
        return TrayEdgeFitResult(
            status='failed',
            message=f'not enough contour points for sides: {",".join(missing)}',
            mask=mask,
            contour=contour,
            initial_box=initial_box,
            side_points=side_points,
            fitted_lines={},
            corners=None,
            rectified=None,
        )

    fitted_lines: dict[str, LineABC] = {}
    for side, points in side_points.items():
        line = fit_line(points)
        if line is None:
            return TrayEdgeFitResult(
                status='failed',
                message=f'line fit failed for {side}',
                mask=mask,
                contour=contour,
                initial_box=initial_box,
                side_points=side_points,
                fitted_lines=fitted_lines,
                corners=None,
                rectified=None,
            )
        fitted_lines[side] = line

    corners = intersect_fitted_sides(fitted_lines)
    if corners is None:
        return TrayEdgeFitResult(
            status='failed',
            message='line intersections failed',
            mask=mask,
            contour=contour,
            initial_box=initial_box,
            side_points=side_points,
            fitted_lines=fitted_lines,
            corners=None,
            rectified=None,
        )
    if not corners_are_valid(corners, image.shape[:2]):
        return TrayEdgeFitResult(
            status='failed',
            message='fitted corners outside valid crop geometry',
            mask=mask,
            contour=contour,
            initial_box=initial_box,
            side_points=side_points,
            fitted_lines=fitted_lines,
            corners=None,
            rectified=None,
        )

    rectified = rectify(image, corners, config)
    return TrayEdgeFitResult(
        status='ok',
        message='fitted four contour-supported tray border lines',
        mask=mask,
        contour=contour,
        initial_box=initial_box,
        side_points=side_points,
        fitted_lines=fitted_lines,
        corners=corners,
        rectified=rectified,
    )


def failed_result(message: str, mask: np.ndarray) -> TrayEdgeFitResult:
    return TrayEdgeFitResult(
        status='failed',
        message=message,
        mask=mask,
        contour=None,
        initial_box=None,
        side_points={side: np.empty((0, 2), dtype=np.float32) for side in ('top', 'right', 'bottom', 'left')},
        fitted_lines={},
        corners=None,
        rectified=None,
    )


def build_tray_body_mask(gray: np.ndarray, config: TrayEdgeFitConfig) -> np.ndarray:
    mask = (gray <= config.dark_threshold).astype(np.uint8) * 255
    mask = remove_side_border_components(mask)
    kernel_size = max(3, int(round(min(gray.shape[:2]) * config.close_kernel_ratio)))
    if kernel_size % 2 == 0:
        kernel_size += 1
    close_kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
    open_kernel_size = max(1, config.open_kernel_px)
    if open_kernel_size % 2 == 0:
        open_kernel_size += 1
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_kernel_size, open_kernel_size), np.uint8), iterations=1)
    return mask


def remove_side_border_components(mask: np.ndarray) -> np.ndarray:
    num_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask

    height, width = mask.shape[:2]
    image_area = float(height * width)
    output = mask.copy()
    for label in range(1, num_labels):
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        component_width = int(stats[label, cv2.CC_STAT_WIDTH])
        component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        area = float(stats[label, cv2.CC_STAT_AREA])
        touches_side = x <= 1 or x + component_width >= width - 1
        tall_narrow = component_height > height * 0.25 and component_width < width * 0.22
        too_small_for_target = area < image_area * 0.18
        if touches_side and (tall_narrow or too_small_for_target):
            output[labels == label] = 0
        elif touches_side and y <= 1 and component_height < height * 0.10:
            output[labels == label] = 0
    return output


def keep_center_body_component(mask: np.ndarray, config: TrayEdgeFitConfig) -> np.ndarray:
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask

    height, width = mask.shape[:2]
    center = np.asarray([width * 0.5, height * 0.5], dtype=np.float32)
    min_area = width * height * config.min_area_ratio
    best_label = 0
    best_score = float('-inf')
    for label in range(1, num_labels):
        area = float(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        component_center = np.asarray(centroids[label], dtype=np.float32)
        distance = float(np.linalg.norm((component_center - center) / np.asarray([width, height], dtype=np.float32)))
        score = area / float(width * height) - distance * 0.55
        if score > best_score:
            best_score = score
            best_label = label

    if best_label == 0:
        return mask
    return np.where(labels == best_label, 255, 0).astype(np.uint8)


def find_largest_body_contour(
    mask: np.ndarray,
    image_shape: tuple[int, int],
    config: TrayEdgeFitConfig,
) -> np.ndarray | None:
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    min_area = image_shape[0] * image_shape[1] * config.min_area_ratio
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    for contour in contours:
        if cv2.contourArea(contour) >= min_area:
            return contour
    return None


def collect_side_points(
    contour: np.ndarray,
    box: np.ndarray,
    image_shape: tuple[int, int],
    config: TrayEdgeFitConfig,
) -> dict[str, np.ndarray]:
    points = contour.reshape(-1, 2).astype(np.float32)
    top, right, bottom, left = box_side_lines(box)
    width = float(np.linalg.norm(box[1] - box[0]))
    height = float(np.linalg.norm(box[3] - box[0]))
    band = max(6.0, min(width, height) * config.side_band_ratio)

    side_defs = {
        'top': (top, box[0], box[1]),
        'right': (right, box[1], box[2]),
        'bottom': (bottom, box[3], box[2]),
        'left': (left, box[0], box[3]),
    }
    output: dict[str, np.ndarray] = {}
    for side, (line, start, end) in side_defs.items():
        distances = np.asarray([abs(signed_distance(line, point)) for point in points], dtype=np.float32)
        projections = project_fraction(points, start, end)
        selected = points[(distances <= band) & (projections >= -0.08) & (projections <= 1.08)]
        output[side] = selected.astype(np.float32)
    return output


def box_side_lines(box: np.ndarray) -> tuple[LineABC, LineABC, LineABC, LineABC]:
    return (
        line_from_points(box[0], box[1]),
        line_from_points(box[1], box[2]),
        line_from_points(box[2], box[3]),
        line_from_points(box[3], box[0]),
    )


def line_from_points(point_a: np.ndarray, point_b: np.ndarray) -> LineABC:
    x1, y1 = point_a
    x2, y2 = point_b
    a = float(y1 - y2)
    b = float(x2 - x1)
    c = float(x1 * y2 - x2 * y1)
    norm = float(np.hypot(a, b))
    if norm <= 1e-6:
        return 0.0, 0.0, 0.0
    return a / norm, b / norm, c / norm


def signed_distance(line: LineABC, point: np.ndarray) -> float:
    a, b, c = line
    return float(a * point[0] + b * point[1] + c)


def project_fraction(points: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    direction = end - start
    denom = float(np.dot(direction, direction))
    if denom <= 1e-6:
        return np.zeros((len(points),), dtype=np.float32)
    return ((points - start) @ direction) / denom


def fit_line(points: np.ndarray) -> LineABC | None:
    if len(points) < 2:
        return None
    vx, vy, x0, y0 = cv2.fitLine(points.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(-1)
    a = float(vy)
    b = -float(vx)
    c = float(vx * y0 - vy * x0)
    norm = float(np.hypot(a, b))
    if norm <= 1e-6:
        return None
    return a / norm, b / norm, c / norm


def intersect_fitted_sides(lines: dict[str, LineABC]) -> np.ndarray | None:
    pairs = [
        ('top', 'left'),
        ('top', 'right'),
        ('bottom', 'right'),
        ('bottom', 'left'),
    ]
    corners: list[tuple[float, float]] = []
    for side_a, side_b in pairs:
        point = intersect_lines(lines[side_a], lines[side_b])
        if point is None:
            return None
        corners.append(point)
    return np.asarray(corners, dtype=np.float32)


def intersect_lines(line_a: LineABC, line_b: LineABC) -> tuple[float, float] | None:
    a1, b1, c1 = line_a
    a2, b2, c2 = line_b
    determinant = a1 * b2 - a2 * b1
    if abs(determinant) < 1e-6:
        return None
    x = (b1 * c2 - b2 * c1) / determinant
    y = (c1 * a2 - c2 * a1) / determinant
    return float(x), float(y)


def corners_are_valid(corners: np.ndarray, image_shape: tuple[int, int]) -> bool:
    height, width = image_shape
    margin = max(width, height) * 0.18
    if np.any(corners[:, 0] < -margin) or np.any(corners[:, 0] > width + margin):
        return False
    if np.any(corners[:, 1] < -margin) or np.any(corners[:, 1] > height + margin):
        return False
    area = abs(float(cv2.contourArea(corners.astype(np.float32))))
    if area < width * height * 0.20:
        return False
    return is_convex_quad(corners)


def is_convex_quad(corners: np.ndarray) -> bool:
    return bool(cv2.isContourConvex(corners.astype(np.float32)))


def rectify(image: np.ndarray, corners: np.ndarray, config: TrayEdgeFitConfig) -> np.ndarray:
    padding = config.rectified_padding
    target = np.asarray(
        [
            [float(padding), float(padding)],
            [float(padding + config.rectified_width - 1), float(padding)],
            [float(padding + config.rectified_width - 1), float(padding + config.rectified_height - 1)],
            [float(padding), float(padding + config.rectified_height - 1)],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(corners.astype(np.float32), target)
    return cv2.warpPerspective(
        image,
        matrix,
        (
            config.rectified_width + padding * 2,
            config.rectified_height + padding * 2,
        ),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )


def order_corners(points: np.ndarray) -> np.ndarray:
    points = points.astype(np.float32)
    sums = points[:, 0] + points[:, 1]
    diffs = points[:, 0] - points[:, 1]
    top_left = points[int(np.argmin(sums))]
    bottom_right = points[int(np.argmax(sums))]
    top_right = points[int(np.argmax(diffs))]
    bottom_left = points[int(np.argmin(diffs))]
    return np.asarray([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
