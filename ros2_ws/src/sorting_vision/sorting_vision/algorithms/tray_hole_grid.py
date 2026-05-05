"""Hole center baseline on a rectified single-tray image."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class TrayHoleGridConfig:
    rows: int = 10
    cols: int = 5
    margin_x_ratio: float = 0.145
    margin_y_ratio: float = 0.105
    refine_window_ratio: float = 0.105
    min_dark_ratio: float = 0.08
    tray_dark_threshold: int = 105
    tray_body_padding_ratio: float = 0.015
    refine_mode: str = 'theory'


@dataclass(frozen=True)
class HoleCenter:
    row: int
    col: int
    u: float
    v: float
    confidence: float
    method: str


@dataclass(frozen=True)
class TrayHoleGridResult:
    status: str
    message: str
    body_bbox: tuple[int, int, int, int] | None
    centers: list[HoleCenter]


def detect_hole_centers(
    image: np.ndarray,
    config: TrayHoleGridConfig | None = None,
) -> TrayHoleGridResult:
    if config is None:
        config = TrayHoleGridConfig()
    if image.ndim != 3 or image.shape[2] != 3:
        return TrayHoleGridResult(status='failed', message='input image must be HxWx3', body_bbox=None, centers=[])
    if config.rows <= 0 or config.cols <= 0:
        return TrayHoleGridResult(status='failed', message='rows and cols must be positive', body_bbox=None, centers=[])

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    body_bbox = estimate_rectified_tray_body_bbox(gray, config)
    theoretical = theoretical_centers(image.shape[:2], config, body_bbox)
    if config.refine_mode == 'local_dark':
        centers = [refine_center(gray, row, col, u, v, config) for row, col, u, v in theoretical]
    else:
        centers = [
            HoleCenter(row=row, col=col, u=u, v=v, confidence=0.65, method='theory')
            for row, col, u, v in theoretical
        ]
    return TrayHoleGridResult(
        status='ok',
        message=f'generated {len(centers)} rectified tray hole centers',
        body_bbox=body_bbox,
        centers=centers,
    )


def theoretical_centers(
    image_shape: tuple[int, int],
    config: TrayHoleGridConfig,
    body_bbox: tuple[int, int, int, int] | None,
) -> list[tuple[int, int, float, float]]:
    height, width = image_shape
    if body_bbox is None:
        x, y, body_width, body_height = 0, 0, width, height
    else:
        x, y, body_width, body_height = body_bbox
    left = x + body_width * config.margin_x_ratio
    right = x + body_width * (1.0 - config.margin_x_ratio)
    top = y + body_height * config.margin_y_ratio
    bottom = y + body_height * (1.0 - config.margin_y_ratio)
    xs = np.linspace(left, right, config.cols, dtype=np.float32)
    ys = np.linspace(top, bottom, config.rows, dtype=np.float32)
    centers: list[tuple[int, int, float, float]] = []
    for row_index, y in enumerate(ys, start=1):
        for col_index, x in enumerate(xs, start=1):
            centers.append((row_index, col_index, float(x), float(y)))
    return centers


def estimate_rectified_tray_body_bbox(
    gray: np.ndarray,
    config: TrayHoleGridConfig,
) -> tuple[int, int, int, int] | None:
    mask = (gray <= config.tray_dark_threshold).astype(np.uint8) * 255
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    image_area = gray.shape[0] * gray.shape[1]
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < image_area * 0.08:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width < gray.shape[1] * 0.25 or height < gray.shape[0] * 0.45:
            continue
        candidates.append((area, x, y, width, height))
    if not candidates:
        return None
    _area, x, y, width, height = max(candidates, key=lambda item: item[0])
    pad = int(round(min(width, height) * config.tray_body_padding_ratio))
    x0 = max(0, x + pad)
    y0 = max(0, y + pad)
    x1 = min(gray.shape[1], x + width - pad)
    y1 = min(gray.shape[0], y + height - pad)
    if x1 <= x0 or y1 <= y0:
        return x, y, width, height
    return x0, y0, x1 - x0, y1 - y0


def refine_center(
    gray: np.ndarray,
    row: int,
    col: int,
    u: float,
    v: float,
    config: TrayHoleGridConfig,
) -> HoleCenter:
    height, width = gray.shape[:2]
    window = max(7, int(round(min(width / config.cols, height / config.rows) * config.refine_window_ratio)))
    if window % 2 == 0:
        window += 1
    x0 = max(0, int(round(u)) - window)
    y0 = max(0, int(round(v)) - window)
    x1 = min(width, int(round(u)) + window + 1)
    y1 = min(height, int(round(v)) + window + 1)
    patch = gray[y0:y1, x0:x1]
    if patch.size == 0:
        return HoleCenter(row=row, col=col, u=u, v=v, confidence=0.25, method='theory')

    threshold = min(120.0, float(np.percentile(patch, 35)))
    dark = patch <= threshold
    dark_ratio = float(np.count_nonzero(dark)) / float(patch.size)
    if dark_ratio < config.min_dark_ratio:
        return HoleCenter(row=row, col=col, u=u, v=v, confidence=0.35, method='theory')

    ys, xs = np.nonzero(dark)
    refined_u = float(x0 + np.mean(xs))
    refined_v = float(y0 + np.mean(ys))
    distance = float(np.hypot(refined_u - u, refined_v - v))
    max_distance = max(1.0, window * 0.85)
    confidence = max(0.40, min(0.95, 0.88 - distance / max_distance * 0.28 + dark_ratio * 0.18))
    return HoleCenter(row=row, col=col, u=refined_u, v=refined_v, confidence=confidence, method='local_dark')


def draw_hole_centers(
    image: np.ndarray,
    result: TrayHoleGridResult,
) -> np.ndarray:
    output = image.copy()
    if result.body_bbox is not None:
        x, y, width, height = result.body_bbox
        cv2.rectangle(output, (x, y), (x + width, y + height), (255, 255, 0), 2, cv2.LINE_AA)
    for center in result.centers:
        point = (int(round(center.u)), int(round(center.v)))
        color = confidence_color(center.confidence)
        cv2.circle(output, point, 5, color, -1, cv2.LINE_AA)
        cv2.circle(output, point, 8, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(
            output,
            f'{center.row},{center.col}',
            (point[0] + 6, point[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            color,
            1,
            cv2.LINE_AA,
        )
    cv2.putText(output, result.status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 180, 0), 2, cv2.LINE_AA)
    return output


def confidence_color(confidence: float) -> tuple[int, int, int]:
    if confidence >= 0.75:
        return (0, 255, 0)
    if confidence >= 0.50:
        return (0, 255, 255)
    return (0, 0, 255)
