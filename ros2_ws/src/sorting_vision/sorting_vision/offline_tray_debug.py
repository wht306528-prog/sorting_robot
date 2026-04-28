"""
离线苗盘几何调试工具。

本命令从磁盘读取一张样本图片，自动检测或读取苗盘四角点，执行透视矫正，
绘制 5x10 网格，并写出调试图片。它是当前苗盘几何定位算法的主线入口，
算法稳定后再迁移到实时 RGB-D 节点中。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


TRAY_COLS = 5
TRAY_ROWS = 10


@dataclass(frozen=True)
class OfflineDebugConfig:
    """一次离线调试运行使用的配置。"""

    warped_width_px: int
    warped_height_px: int
    line_width_px: int
    center_radius_px: int
    auto_detect: bool
    max_auto_trays: int
    min_area_ratio: float
    dark_pixel_threshold: int
    column_active_ratio: float
    projection_smooth_px: int
    x_range_padding_px: int
    merge_kernel_px: int
    trays: list['ConfiguredTray']


@dataclass(frozen=True)
class ConfiguredTray:
    """由四个图像坐标角点配置的单个苗盘。"""

    tray_id: int
    corners: np.ndarray


@dataclass(frozen=True)
class TrayCandidate:
    """自动检测得到的单个苗盘候选。"""

    corners: np.ndarray
    area: float
    center_x: float


def main(argv: list[str] | None = None) -> None:
    """命令入口。"""
    args = _parse_args(argv)
    config = load_config(args.config)
    image_path = Path(args.image)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'读取图片失败：{image_path}')

    trays = config.trays
    candidates: list[TrayCandidate] = []
    if not trays and config.auto_detect:
        candidates = detect_tray_candidates(
            image,
            config,
        )
        trays = [
            ConfiguredTray(index + 1, candidate.corners)
            for index, candidate in enumerate(candidates)
        ]

    if not trays:
        raise RuntimeError(
            '没有找到苗盘角点。请在配置文件中填写手工四角点，'
            '或继续改进自动检测算法。'
        )

    prefix = output_dir / image_path.stem
    overlay = image.copy()
    candidate_debug = draw_candidates(image, candidates)

    for tray in sorted(trays, key=lambda item: item.tray_id):
        ordered = order_corners(tray.corners)
        draw_projected_grid(
            overlay,
            ordered,
            tray.tray_id,
            line_width=config.line_width_px,
            center_radius=config.center_radius_px,
        )
        warped = warp_tray(
            image,
            ordered,
            config.warped_width_px,
            config.warped_height_px,
        )
        warped_grid = warped.copy()
        draw_warped_grid(
            warped_grid,
            line_width=config.line_width_px,
            center_radius=config.center_radius_px,
        )
        cv2.imwrite(str(prefix) + f'_tray_{tray.tray_id}_warped.jpg', warped)
        cv2.imwrite(
            str(prefix) + f'_tray_{tray.tray_id}_warped_grid.jpg',
            warped_grid,
        )

    cv2.imwrite(str(prefix) + '_overlay_grid.jpg', overlay)
    cv2.imwrite(str(prefix) + '_candidates.jpg', candidate_debug)

    print(f'输入图片：{image_path}')
    print(f'输出目录：{output_dir}')
    print(f'处理苗盘数量：{len(trays)}')
    print(f'已写出：{prefix}_overlay_grid.jpg')
    print(f'已写出：{prefix}_candidates.jpg')


def load_config(path: str | Path) -> OfflineDebugConfig:
    """读取离线调试 YAML 配置。"""
    with Path(path).open('r', encoding='utf-8') as file:
        data = yaml.safe_load(file) or {}
    root = data.get('offline_tray_debug', data)

    return OfflineDebugConfig(
        warped_width_px=int(root.get('warped_width_px', 500)),
        warped_height_px=int(root.get('warped_height_px', 1000)),
        line_width_px=int(root.get('line_width_px', 2)),
        center_radius_px=int(root.get('center_radius_px', 4)),
        auto_detect=bool(root.get('auto_detect', True)),
        max_auto_trays=int(root.get('max_auto_trays', 3)),
        min_area_ratio=float(root.get('min_area_ratio', 0.02)),
        dark_pixel_threshold=int(root.get('dark_pixel_threshold', 75)),
        column_active_ratio=float(root.get('column_active_ratio', 0.12)),
        projection_smooth_px=int(root.get('projection_smooth_px', 31)),
        x_range_padding_px=int(root.get('x_range_padding_px', 8)),
        merge_kernel_px=int(root.get('merge_kernel_px', 23)),
        trays=_load_configured_trays(root.get('trays', [])),
    )


def detect_tray_candidates(
    image: np.ndarray,
    config: OfflineDebugConfig,
) -> list[TrayCandidate]:
    """自动检测苗盘矩形候选。"""
    projection_candidates = detect_trays_by_dark_projection(
        image,
        max_candidates=config.max_auto_trays,
        min_area_ratio=config.min_area_ratio,
        dark_pixel_threshold=config.dark_pixel_threshold,
        column_active_ratio=config.column_active_ratio,
        projection_smooth_px=config.projection_smooth_px,
        x_range_padding_px=config.x_range_padding_px,
        merge_kernel_px=config.merge_kernel_px,
    )
    if projection_candidates:
        return projection_candidates
    return detect_trays_by_edges(
        image,
        config.max_auto_trays,
        config.min_area_ratio,
    )


def detect_trays_by_dark_projection(
    image: np.ndarray,
    max_candidates: int,
    min_area_ratio: float,
    dark_pixel_threshold: int,
    column_active_ratio: float,
    projection_smooth_px: int,
    x_range_padding_px: int,
    merge_kernel_px: int,
) -> list[TrayCandidate]:
    """按暗像素横向投影分割深色苗盘区域。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dark_mask = gray < dark_pixel_threshold
    column_counts = dark_mask.sum(axis=0).astype(np.float32)
    smooth_width = max(3, projection_smooth_px)
    if smooth_width % 2 == 0:
        smooth_width += 1
    kernel = np.ones(smooth_width, dtype=np.float32) / float(smooth_width)
    smoothed = np.convolve(column_counts, kernel, mode='same')
    active_threshold = image.shape[0] * column_active_ratio
    x_ranges = _active_ranges(
        smoothed > active_threshold,
        min_width=max(50, image.shape[1] // 30),
        padding=x_range_padding_px,
        limit=image.shape[1] - 1,
    )

    image_area = float(image.shape[0] * image.shape[1])
    min_area = image_area * min_area_ratio
    candidates: list[TrayCandidate] = []

    for x_start, x_end in x_ranges:
        crop_mask = dark_mask[:, x_start:x_end + 1].astype(np.uint8) * 255
        kernel_size = max(3, merge_kernel_px)
        if kernel_size % 2 == 0:
            kernel_size += 1
        merge_kernel = np.ones((kernel_size, kernel_size), np.uint8)
        crop_mask = cv2.morphologyEx(
            crop_mask,
            cv2.MORPH_CLOSE,
            merge_kernel,
            iterations=2,
        )
        contours, _ = cv2.findContours(
            crop_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            continue

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        shifted = contour.copy()
        shifted[:, :, 0] += x_start
        rect = cv2.minAreaRect(shifted)
        corners = cv2.boxPoints(rect).astype(np.float32)
        ordered = order_corners(corners)
        if not _looks_like_tray(ordered):
            continue
        candidates.append(
            TrayCandidate(
                corners=ordered,
                area=float(area),
                center_x=float(np.mean(ordered[:, 0])),
            )
        )

    candidates.sort(key=lambda item: item.center_x)
    return candidates[:max_candidates]


def detect_trays_by_edges(
    image: np.ndarray,
    max_candidates: int,
    min_area_ratio: float,
) -> list[TrayCandidate]:
    """兜底的边缘矩形候选检测。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 60, 160)
    kernel = np.ones((5, 5), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = float(image.shape[0] * image.shape[1])
    min_area = image_area * min_area_ratio
    candidates: list[TrayCandidate] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            corners = approx.reshape(4, 2).astype(np.float32)
        else:
            rect = cv2.minAreaRect(contour)
            corners = cv2.boxPoints(rect).astype(np.float32)

        ordered = order_corners(corners)
        width_a = np.linalg.norm(ordered[1] - ordered[0])
        width_b = np.linalg.norm(ordered[2] - ordered[3])
        height_a = np.linalg.norm(ordered[3] - ordered[0])
        height_b = np.linalg.norm(ordered[2] - ordered[1])
        width = max(width_a, width_b)
        height = max(height_a, height_b)
        if min(width, height) <= 0.0:
            continue

        ratio = max(width, height) / min(width, height)
        if ratio < 1.2:
            continue

        candidates.append(
            TrayCandidate(
                corners=ordered,
                area=float(area),
                center_x=float(np.mean(ordered[:, 0])),
            )
        )

    candidates.sort(key=lambda item: item.area, reverse=True)
    selected = sorted(candidates[:max_candidates], key=lambda item: item.center_x)
    return selected


def _active_ranges(
    active: np.ndarray,
    min_width: int,
    padding: int,
    limit: int,
) -> list[tuple[int, int]]:
    """把活跃列布尔数组转换成带边距的横向范围。"""
    ranges: list[tuple[int, int]] = []
    start: int | None = None
    for index, value in enumerate(active):
        if value and start is None:
            start = index
        is_last = index == len(active) - 1
        if start is not None and (not value or is_last):
            end = index if is_last and value else index - 1
            if end - start + 1 >= min_width:
                ranges.append(
                    (
                        max(0, start - padding),
                        min(limit, end + padding),
                    )
                )
            start = None
    return ranges


def _looks_like_tray(corners: np.ndarray) -> bool:
    """判断四边形是否像一个苗盘外框。"""
    width_a = np.linalg.norm(corners[1] - corners[0])
    width_b = np.linalg.norm(corners[2] - corners[3])
    height_a = np.linalg.norm(corners[3] - corners[0])
    height_b = np.linalg.norm(corners[2] - corners[1])
    width = max(width_a, width_b)
    height = max(height_a, height_b)
    if min(width, height) <= 0.0:
        return False
    ratio = max(width, height) / min(width, height)
    return 1.2 <= ratio <= 4.0


def order_corners(corners: np.ndarray) -> np.ndarray:
    """按左上、右上、右下、左下顺序返回角点。"""
    points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).reshape(4)

    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(diffs)]
    ordered[3] = points[np.argmax(diffs)]
    return ordered


def warp_tray(
    image: np.ndarray,
    corners: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    """把单个苗盘透视矫正成标准正视图。"""
    destination = np.array(
        [
            [0.0, 0.0],
            [float(width - 1), 0.0],
            [float(width - 1), float(height - 1)],
            [0.0, float(height - 1)],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(corners.astype(np.float32), destination)
    return cv2.warpPerspective(image, matrix, (width, height))


def draw_candidates(image: np.ndarray, candidates: list[TrayCandidate]) -> np.ndarray:
    """在原图副本上绘制自动检测候选框。"""
    output = image.copy()
    for index, candidate in enumerate(candidates, start=1):
        points = candidate.corners.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(output, [points], True, (0, 200, 255), 2)
        center = tuple(np.mean(candidate.corners, axis=0).astype(int))
        cv2.putText(
            output,
            str(index),
            center,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 200, 255),
            2,
            cv2.LINE_AA,
        )
    return output


def draw_projected_grid(
    image: np.ndarray,
    corners: np.ndarray,
    tray_id: int,
    line_width: int,
    center_radius: int,
) -> None:
    """在原图上绘制苗盘外框和反投影的 5x10 网格。"""
    color = tray_color(tray_id)
    width = 500
    height = 1000
    source = np.array(
        [
            [0.0, 0.0],
            [float(width), 0.0],
            [float(width), float(height)],
            [0.0, float(height)],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(source, corners.astype(np.float32))

    grid_lines = build_warped_grid_lines(width, height)
    for start, end in grid_lines:
        points = np.array([[start, end]], dtype=np.float32)
        projected = cv2.perspectiveTransform(points, matrix)[0].astype(int)
        cv2.line(
            image,
            tuple(projected[0]),
            tuple(projected[1]),
            color,
            line_width,
            cv2.LINE_AA,
        )

    centers = build_warped_centers(width, height)
    for center in centers:
        point = np.array([[center]], dtype=np.float32)
        projected = cv2.perspectiveTransform(point, matrix)[0][0].astype(int)
        draw_cross(image, tuple(projected), center_radius, color)

    label_position = tuple(corners[0].astype(int))
    cv2.putText(
        image,
        str(tray_id),
        label_position,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA,
    )


def draw_warped_grid(
    image: np.ndarray,
    line_width: int,
    center_radius: int,
) -> None:
    """在透视矫正后的苗盘图上绘制 5x10 网格。"""
    height, width = image.shape[:2]
    color = (0, 255, 0)

    for start, end in build_warped_grid_lines(width, height):
        cv2.line(image, start, end, color, line_width, cv2.LINE_AA)

    for center in build_warped_centers(width, height):
        draw_cross(image, center, center_radius, color)


def build_warped_grid_lines(
    width: int,
    height: int,
) -> list[tuple[tuple[int, int], tuple[int, int]]]:
    """生成矫正图坐标系中的网格线端点。"""
    lines: list[tuple[tuple[int, int], tuple[int, int]]] = []
    for col in range(TRAY_COLS + 1):
        x = int(round(col * width / TRAY_COLS))
        lines.append(((x, 0), (x, height)))
    for row in range(TRAY_ROWS + 1):
        y = int(round(row * height / TRAY_ROWS))
        lines.append(((0, y), (width, y)))
    return lines


def build_warped_centers(width: int, height: int) -> list[tuple[int, int]]:
    """生成矫正图坐标系中的 5x10 穴位中心点。"""
    centers: list[tuple[int, int]] = []
    cell_width = width / TRAY_COLS
    cell_height = height / TRAY_ROWS
    for row in range(TRAY_ROWS):
        for col in range(TRAY_COLS):
            centers.append(
                (
                    int(round((col + 0.5) * cell_width)),
                    int(round((row + 0.5) * cell_height)),
                )
            )
    return centers


def draw_cross(
    image: np.ndarray,
    center: tuple[int, int],
    radius: int,
    color: tuple[int, int, int],
) -> None:
    """绘制一个小十字中心标记。"""
    x, y = center
    cv2.line(image, (x - radius, y), (x + radius, y), color, 1, cv2.LINE_AA)
    cv2.line(image, (x, y - radius), (x, y + radius), color, 1, cv2.LINE_AA)


def tray_color(tray_id: int) -> tuple[int, int, int]:
    """根据苗盘编号返回 BGR 颜色。"""
    colors = {
        1: (40, 40, 255),
        2: (80, 220, 40),
        3: (255, 140, 60),
    }
    return colors.get(tray_id, (0, 255, 255))


def _load_configured_trays(raw_trays: object) -> list[ConfiguredTray]:
    """从 YAML 解析手工配置的苗盘四角点。"""
    trays: list[ConfiguredTray] = []
    if raw_trays is None:
        return trays
    if not isinstance(raw_trays, list):
        raise ValueError('offline_tray_debug.trays 必须是列表')

    for index, item in enumerate(raw_trays, start=1):
        if not isinstance(item, dict):
            raise ValueError(f'第 {index} 个苗盘配置必须是字典')
        corners = item.get('corners')
        if corners is None:
            continue
        points = np.asarray(corners, dtype=np.float32)
        if points.shape != (4, 2):
            raise ValueError(
                f'苗盘 {item.get("tray_id", index)} 的 corners 必须是 4 个 [x, y] 点'
            )
        trays.append(
            ConfiguredTray(
                tray_id=int(item.get('tray_id', index)),
                corners=points,
            )
        )
    return trays


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description='离线苗盘几何调试工具。',
    )
    parser.add_argument(
        '--image',
        required=True,
        help='样本图片路径。',
    )
    parser.add_argument(
        '--config',
        default='src/sorting_vision/config/offline_tray_debug.yaml',
        help='离线苗盘调试 YAML 配置文件路径。',
    )
    parser.add_argument(
        '--output-dir',
        default='samples/debug',
        help='调试图片输出目录。',
    )
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
