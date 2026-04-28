"""
Rule-based tray grid detection helpers.

This module contains the small, ROS-independent pieces used by the first
real-camera matrix publisher. It intentionally starts with configured tray
regions of interest instead of automatic tray contour detection, because that
lets the hardware team verify camera input, grid numbering, depth sampling,
and F407 communication before the tray detector is fully tuned.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct

from sorting_interfaces.msg import TrayCell


TRAY_COLS = 5
TRAY_ROWS = 10


@dataclass(frozen=True)
class TrayRoi:
    """Configured tray rectangle in image pixel coordinates."""

    tray_id: int
    x: float
    y: float
    width: float
    height: float

    @property
    def valid(self) -> bool:
        """Whether the rectangle has usable dimensions."""
        return self.width > 0.0 and self.height > 0.0


@dataclass(frozen=True)
class ImageView:
    """Minimal view of a sensor_msgs/Image payload."""

    width: int
    height: int
    encoding: str
    step: int
    data: bytes


@dataclass(frozen=True)
class DetectionConfig:
    """Tunable thresholds for the early rule-based classifier."""

    leaf_area_ratio_threshold: float = 0.20
    weak_area_ratio_threshold: float = 0.03
    depth_window_px: int = 5
    roi_inner_scale: float = 0.65
    color_sample_step_px: int = 2


@dataclass(frozen=True)
class CellDetection:
    """Detection result for one tray cell."""

    tray_id: int
    col: int
    row: int
    class_id: int
    confidence: float
    u: float
    v: float
    z: float

    def to_message(self) -> TrayCell:
        """Convert the detection result to the ROS TrayCell message."""
        cell = TrayCell()
        cell.tray_id = self.tray_id
        cell.col = self.col
        cell.row = self.row
        cell.class_id = self.class_id
        cell.confidence = self.confidence
        cell.u = self.u
        cell.v = self.v
        cell.z = self.z
        return cell


class RuleBasedTrayDetector:
    """Generate a 5x10 tray grid and classify each cell with simple rules."""

    def __init__(self, config: DetectionConfig | None = None) -> None:
        self._config = config or DetectionConfig()

    def detect(
        self,
        color_image: ImageView,
        depth_image: ImageView | None,
        tray_rois: list[TrayRoi],
    ) -> list[CellDetection]:
        """Detect all configured tray cells."""
        detections: list[CellDetection] = []
        for roi in sorted(tray_rois, key=lambda item: item.tray_id):
            if not roi.valid:
                continue
            detections.extend(self._detect_tray(color_image, depth_image, roi))
        return detections

    def _detect_tray(
        self,
        color_image: ImageView,
        depth_image: ImageView | None,
        roi: TrayRoi,
    ) -> list[CellDetection]:
        """Generate detections for one configured tray rectangle."""
        cell_width = roi.width / TRAY_COLS
        cell_height = roi.height / TRAY_ROWS
        detections: list[CellDetection] = []

        for row in range(1, TRAY_ROWS + 1):
            for col in range(1, TRAY_COLS + 1):
                u = roi.x + (col - 0.5) * cell_width
                v = roi.y + (row - 0.5) * cell_height
                z = sample_depth_mm(
                    depth_image,
                    u,
                    v,
                    self._config.depth_window_px,
                )
                ratio = green_area_ratio(
                    color_image,
                    roi.x + (col - 1) * cell_width,
                    roi.y + (row - 1) * cell_height,
                    cell_width,
                    cell_height,
                    self._config.roi_inner_scale,
                    self._config.color_sample_step_px,
                )
                class_id, confidence = classify_by_green_ratio(
                    ratio,
                    self._config.leaf_area_ratio_threshold,
                    self._config.weak_area_ratio_threshold,
                    z,
                )
                detections.append(
                    CellDetection(
                        tray_id=roi.tray_id,
                        col=col,
                        row=row,
                        class_id=class_id,
                        confidence=confidence,
                        u=u,
                        v=v,
                        z=z,
                    )
                )
        return detections


def classify_by_green_ratio(
    green_ratio: float,
    good_threshold: float,
    weak_threshold: float,
    depth_mm: float,
) -> tuple[int, float]:
    """Classify one cell using an early green-area rule."""
    if depth_mm <= 0.0 and green_ratio <= 0.0:
        return TrayCell.CLASS_EMPTY, 0.40

    if green_ratio >= good_threshold:
        margin = min((green_ratio - good_threshold) / max(good_threshold, 0.001), 1.0)
        return TrayCell.CLASS_GOOD, 0.75 + 0.20 * margin

    if green_ratio >= weak_threshold:
        span = max(good_threshold - weak_threshold, 0.001)
        margin = min((good_threshold - green_ratio) / span, 1.0)
        return TrayCell.CLASS_WEAK, 0.60 + 0.25 * margin

    return TrayCell.CLASS_EMPTY, 0.80


def green_area_ratio(
    image: ImageView,
    x: float,
    y: float,
    width: float,
    height: float,
    inner_scale: float,
    sample_step_px: int,
) -> float:
    """Estimate green-pixel area ratio inside the inner part of a cell."""
    channels = _color_channels(image.encoding)
    if channels is None:
        return 0.0

    red_index, green_index, blue_index, bytes_per_pixel = channels
    inner_width = width * inner_scale
    inner_height = height * inner_scale
    left = math.floor(x + (width - inner_width) * 0.5)
    top = math.floor(y + (height - inner_height) * 0.5)
    right = math.ceil(left + inner_width)
    bottom = math.ceil(top + inner_height)

    left = max(0, min(left, image.width - 1))
    right = max(left + 1, min(right, image.width))
    top = max(0, min(top, image.height - 1))
    bottom = max(top + 1, min(bottom, image.height))
    step = max(1, sample_step_px)

    green_pixels = 0
    total_pixels = 0
    for pixel_y in range(top, bottom, step):
        row_offset = pixel_y * image.step
        for pixel_x in range(left, right, step):
            offset = row_offset + pixel_x * bytes_per_pixel
            red = image.data[offset + red_index]
            green = image.data[offset + green_index]
            blue = image.data[offset + blue_index]
            total_pixels += 1
            if green > 45 and green > red * 1.15 and green > blue * 1.15:
                green_pixels += 1

    if total_pixels == 0:
        return 0.0
    return green_pixels / total_pixels


def sample_depth_mm(
    image: ImageView | None,
    u: float,
    v: float,
    window_px: int,
) -> float:
    """Read a median depth value near a pixel center in millimeters."""
    if image is None:
        return 0.0

    center_x = int(round(u))
    center_y = int(round(v))
    radius = max(0, window_px // 2)
    values: list[float] = []

    for pixel_y in range(center_y - radius, center_y + radius + 1):
        if pixel_y < 0 or pixel_y >= image.height:
            continue
        for pixel_x in range(center_x - radius, center_x + radius + 1):
            if pixel_x < 0 or pixel_x >= image.width:
                continue
            value = _depth_at(image, pixel_x, pixel_y)
            if value > 0.0 and math.isfinite(value):
                values.append(value)

    if not values:
        return 0.0
    values.sort()
    return values[len(values) // 2]


def _color_channels(encoding: str) -> tuple[int, int, int, int] | None:
    """Return channel offsets for common RGB image encodings."""
    normalized = encoding.lower()
    if normalized == 'rgb8':
        return 0, 1, 2, 3
    if normalized == 'bgr8':
        return 2, 1, 0, 3
    if normalized == 'rgba8':
        return 0, 1, 2, 4
    if normalized == 'bgra8':
        return 2, 1, 0, 4
    return None


def _depth_at(image: ImageView, pixel_x: int, pixel_y: int) -> float:
    """Read one depth pixel and return millimeters."""
    encoding = image.encoding.lower()
    if encoding in ('16uc1', 'mono16'):
        offset = pixel_y * image.step + pixel_x * 2
        if offset + 2 > len(image.data):
            return 0.0
        return float(int.from_bytes(image.data[offset:offset + 2], 'little'))
    if encoding == '32fc1':
        offset = pixel_y * image.step + pixel_x * 4
        if offset + 4 > len(image.data):
            return 0.0
        return float(struct.unpack_from('<f', image.data, offset)[0] * 1000.0)
    return 0.0
