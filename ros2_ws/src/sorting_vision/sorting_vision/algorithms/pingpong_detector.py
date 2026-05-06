"""Ping-pong ball color baseline on rectified tray images."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from sorting_vision.algorithms.tray_hole_grid import HoleCenter, TrayHoleGridConfig, detect_hole_centers


CLASS_EMPTY = 'empty'
CLASS_WHITE = 'white_ball'
CLASS_YELLOW = 'yellow_ball'


@dataclass(frozen=True)
class PingpongDetectorConfig:
    rows: int = 10
    cols: int = 5
    roi_radius_ratio: float = 0.34
    min_ball_ratio: float = 0.10
    min_color_margin: float = 0.035
    hole_grid: TrayHoleGridConfig = TrayHoleGridConfig()


@dataclass(frozen=True)
class PingpongCell:
    row: int
    col: int
    u: float
    v: float
    class_name: str
    confidence: float
    yellow_ratio: float
    white_ratio: float
    ball_ratio: float


@dataclass(frozen=True)
class PingpongDetectionResult:
    status: str
    message: str
    cells: list[PingpongCell]


def detect_pingpong_cells(
    image: np.ndarray,
    config: PingpongDetectorConfig | None = None,
) -> PingpongDetectionResult:
    if config is None:
        config = PingpongDetectorConfig()
    if image.ndim != 3 or image.shape[2] != 3:
        return PingpongDetectionResult(status='failed', message='input image must be HxWx3', cells=[])

    grid_result = detect_hole_centers(image, config.hole_grid)
    if grid_result.status != 'ok':
        return PingpongDetectionResult(status='failed', message=grid_result.message, cells=[])

    radius = estimate_roi_radius(grid_result.centers, image.shape[:2], config)
    cells = [classify_cell(image, center, radius, config) for center in grid_result.centers]
    return PingpongDetectionResult(
        status='ok',
        message=f'classified {len(cells)} ping-pong tray cells',
        cells=cells,
    )


def estimate_roi_radius(
    centers: list[HoleCenter],
    image_shape: tuple[int, int],
    config: PingpongDetectorConfig,
) -> int:
    if len(centers) >= 2:
        by_row: dict[int, list[HoleCenter]] = {}
        by_col: dict[int, list[HoleCenter]] = {}
        for center in centers:
            by_row.setdefault(center.row, []).append(center)
            by_col.setdefault(center.col, []).append(center)
        spacings: list[float] = []
        for row_centers in by_row.values():
            ordered = sorted(row_centers, key=lambda item: item.col)
            spacings.extend(
                abs(right.u - left.u)
                for left, right in zip(ordered, ordered[1:])
            )
        for col_centers in by_col.values():
            ordered = sorted(col_centers, key=lambda item: item.row)
            spacings.extend(
                abs(bottom.v - top.v)
                for top, bottom in zip(ordered, ordered[1:])
            )
        if spacings:
            return max(8, int(round(np.median(spacings) * config.roi_radius_ratio)))

    height, width = image_shape
    return max(8, int(round(min(width / config.cols, height / config.rows) * config.roi_radius_ratio)))


def classify_cell(
    image: np.ndarray,
    center: HoleCenter,
    radius: int,
    config: PingpongDetectorConfig,
) -> PingpongCell:
    height, width = image.shape[:2]
    u = int(round(center.u))
    v = int(round(center.v))
    x0 = max(0, u - radius)
    y0 = max(0, v - radius)
    x1 = min(width, u + radius + 1)
    y1 = min(height, v + radius + 1)
    patch = image[y0:y1, x0:x1]
    if patch.size == 0:
        return PingpongCell(center.row, center.col, center.u, center.v, CLASS_EMPTY, 0.20, 0.0, 0.0, 0.0)

    local_u = u - x0
    local_v = v - y0
    yy, xx = np.ogrid[:patch.shape[0], :patch.shape[1]]
    circle = (xx - local_u) ** 2 + (yy - local_v) ** 2 <= radius ** 2
    valid_count = int(np.count_nonzero(circle))
    if valid_count <= 0:
        return PingpongCell(center.row, center.col, center.u, center.v, CLASS_EMPTY, 0.20, 0.0, 0.0, 0.0)

    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    blue, green, red = cv2.split(patch)
    max_channel = np.maximum(np.maximum(red, green), blue)
    min_channel = np.minimum(np.minimum(red, green), blue)

    yellow = circle & (hue >= 10) & (hue <= 42) & (sat >= 55) & (val >= 80) & (red >= 100) & (green >= 75)
    white = circle & (sat <= 70) & (val >= 135) & (min_channel >= 105) & ((max_channel - min_channel) <= 75)
    yellow_ratio = float(np.count_nonzero(yellow)) / valid_count
    white_ratio = float(np.count_nonzero(white)) / valid_count
    ball_ratio = yellow_ratio + white_ratio

    class_name, confidence = classify_ratios(yellow_ratio, white_ratio, ball_ratio, config)
    return PingpongCell(
        row=center.row,
        col=center.col,
        u=center.u,
        v=center.v,
        class_name=class_name,
        confidence=confidence,
        yellow_ratio=yellow_ratio,
        white_ratio=white_ratio,
        ball_ratio=ball_ratio,
    )


def classify_ratios(
    yellow_ratio: float,
    white_ratio: float,
    ball_ratio: float,
    config: PingpongDetectorConfig,
) -> tuple[str, float]:
    if ball_ratio < config.min_ball_ratio:
        confidence = 0.65 + min(0.25, (config.min_ball_ratio - ball_ratio) / max(config.min_ball_ratio, 0.001) * 0.25)
        return CLASS_EMPTY, confidence

    if yellow_ratio >= white_ratio + config.min_color_margin:
        color_strength = min(1.0, yellow_ratio / max(config.min_ball_ratio, 0.001))
        margin = min(1.0, (yellow_ratio - white_ratio) / max(config.min_color_margin * 4.0, 0.001))
        return CLASS_YELLOW, 0.55 + 0.25 * color_strength + 0.15 * margin

    if white_ratio >= yellow_ratio + config.min_color_margin:
        color_strength = min(1.0, white_ratio / max(config.min_ball_ratio, 0.001))
        margin = min(1.0, (white_ratio - yellow_ratio) / max(config.min_color_margin * 4.0, 0.001))
        return CLASS_WHITE, 0.55 + 0.25 * color_strength + 0.15 * margin

    if yellow_ratio >= white_ratio:
        return CLASS_YELLOW, 0.50 + min(0.20, ball_ratio)
    return CLASS_WHITE, 0.50 + min(0.20, ball_ratio)


def draw_pingpong_cells(
    image: np.ndarray,
    result: PingpongDetectionResult,
) -> np.ndarray:
    output = image.copy()
    for cell in result.cells:
        point = (int(round(cell.u)), int(round(cell.v)))
        color = class_color(cell.class_name)
        cv2.circle(output, point, 9, color, 2, cv2.LINE_AA)
        cv2.circle(output, point, 3, color, -1, cv2.LINE_AA)
        label = class_label(cell.class_name)
        cv2.putText(
            output,
            f'{cell.row},{cell.col} {label}',
            (point[0] + 8, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            color,
            1,
            cv2.LINE_AA,
        )
    draw_counts(output, result)
    return output


def class_color(class_name: str) -> tuple[int, int, int]:
    if class_name == CLASS_YELLOW:
        return (0, 210, 255)
    if class_name == CLASS_WHITE:
        return (255, 255, 255)
    return (80, 80, 80)


def class_label(class_name: str) -> str:
    if class_name == CLASS_YELLOW:
        return 'Y'
    if class_name == CLASS_WHITE:
        return 'W'
    return 'E'


def draw_counts(image: np.ndarray, result: PingpongDetectionResult) -> None:
    yellow_count = sum(1 for cell in result.cells if cell.class_name == CLASS_YELLOW)
    white_count = sum(1 for cell in result.cells if cell.class_name == CLASS_WHITE)
    empty_count = sum(1 for cell in result.cells if cell.class_name == CLASS_EMPTY)
    text = f'{result.status} yellow={yellow_count} white={white_count} empty={empty_count}'
    cv2.putText(image, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 180, 0), 2, cv2.LINE_AA)
