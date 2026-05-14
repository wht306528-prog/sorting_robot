"""矫正后单盘图像上的乒乓球颜色识别基线。

当前阶段先服务“乒乓球吸取演示”，目标只区分 empty / white_ball / yellow_ball。
这里是 OpenCV 颜色规则基线，不是 YOLO，也不是最终作物识别算法。
"""

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
    """单穴位颜色分类参数。

    阈值保持集中配置，现场光照变化时优先调这些参数，不要直接散改分类代码。
    """

    rows: int = 10
    cols: int = 5
    roi_radius_ratio: float = 0.34
    min_ball_ratio: float = 0.16
    min_white_ratio: float = 0.36
    min_white_component_ratio: float = 0.30
    min_white_shape_component_ratio: float = 0.18
    min_white_component_diameter_ratio: float = 0.95
    min_white_circularity: float = 0.35
    max_white_center_offset_ratio: float = 0.55
    min_yellow_component_ratio: float = 0.12
    min_color_margin: float = 0.035
    hole_grid: TrayHoleGridConfig = TrayHoleGridConfig()


@dataclass(frozen=True)
class PingpongCell:
    """一个穴位的乒乓球分类结果和调试比例。"""

    row: int
    col: int
    u: float
    v: float
    class_name: str
    confidence: float
    yellow_ratio: float
    white_ratio: float
    ball_ratio: float
    yellow_component_ratio: float
    white_component_ratio: float
    debug_u: float | None = None
    debug_v: float | None = None


@dataclass(frozen=True)
class PingpongDetectionResult:
    """整盘 10x5 穴位的分类结果。"""

    status: str
    message: str
    cells: list[PingpongCell]


def detect_pingpong_cells(
    image: np.ndarray,
    config: PingpongDetectorConfig | None = None,
) -> PingpongDetectionResult:
    """在透视矫正后的单盘图里生成 10x5 穴位分类。"""

    if config is None:
        config = PingpongDetectorConfig()
    if image.ndim != 3 or image.shape[2] != 3:
        return PingpongDetectionResult(status='failed', message='input image must be HxWx3', cells=[])

    # 先复用苗盘穴位网格基线，拿到每个穴位中心，再在中心附近做颜色判断。
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
    """根据相邻穴位间距估计每个穴位的采样半径。"""

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
            # 用中位数抗单个异常穴位中心偏移，半径只覆盖穴位中心附近。
            return max(8, int(round(np.median(spacings) * config.roi_radius_ratio)))

    height, width = image_shape
    # 如果中心数量不足，就退回到图像尺寸和行列数估算。
    return max(8, int(round(min(width / config.cols, height / config.rows) * config.roi_radius_ratio)))


def classify_cell(
    image: np.ndarray,
    center: HoleCenter,
    radius: int,
    config: PingpongDetectorConfig,
) -> PingpongCell:
    """对单个穴位 ROI 做黄球/白球/空穴分类。"""

    height, width = image.shape[:2]
    u = int(round(center.u))
    v = int(round(center.v))
    x0 = max(0, u - radius)
    y0 = max(0, v - radius)
    x1 = min(width, u + radius + 1)
    y1 = min(height, v + radius + 1)
    # 只截取穴位中心附近的小块，避免相邻穴位或苗盘边框影响颜色比例。
    patch = image[y0:y1, x0:x1]
    if patch.size == 0:
        return PingpongCell(center.row, center.col, center.u, center.v, CLASS_EMPTY, 0.20, 0.0, 0.0, 0.0, 0.0, 0.0)

    local_u = u - x0
    local_v = v - y0
    yy, xx = np.ogrid[:patch.shape[0], :patch.shape[1]]
    # 圆形 mask 贴近乒乓球/穴位形状，比方形 ROI 更少吃到边缘背景。
    circle = (xx - local_u) ** 2 + (yy - local_v) ** 2 <= radius ** 2
    valid_count = int(np.count_nonzero(circle))
    if valid_count <= 0:
        return PingpongCell(center.row, center.col, center.u, center.v, CLASS_EMPTY, 0.20, 0.0, 0.0, 0.0, 0.0, 0.0)

    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    blue, green, red = cv2.split(patch)
    max_channel = np.maximum(np.maximum(red, green), blue)
    min_channel = np.minimum(np.minimum(red, green), blue)

    # 黄色主要看 HSV 色相和饱和度，同时要求 R/G 亮度，减少棕色暗区误判。
    yellow = circle & (hue >= 10) & (hue <= 42) & (sat >= 55) & (val >= 80) & (red >= 100) & (green >= 75)
    # 白色没有稳定色相，主要看低饱和、高亮度、RGB 通道差异不大。
    white = circle & (sat <= 70) & (val >= 135) & (min_channel >= 105) & ((max_channel - min_channel) <= 75)
    yellow_ratio = float(np.count_nonzero(yellow)) / valid_count
    white_ratio = float(np.count_nonzero(white)) / valid_count
    ball_ratio = yellow_ratio + white_ratio
    # 连通域比例用于过滤零散反光点；真实球面通常形成较大的连续区域。
    yellow_component_ratio = largest_component_ratio(yellow, circle, valid_count)
    white_component = largest_component_stats(white, circle, valid_count, radius)
    white_component_ratio = white_component.area_ratio
    white_shape_ok = is_white_component_shape_reasonable(
        component=white_component,
        local_center=(float(local_u), float(local_v)),
        radius=radius,
        config=config,
    )

    class_name, confidence = classify_ratios(
        yellow_ratio=yellow_ratio,
        white_ratio=white_ratio,
        ball_ratio=ball_ratio,
        yellow_component_ratio=yellow_component_ratio,
        white_component_ratio=white_component_ratio,
        white_shape_ok=white_shape_ok,
        config=config,
    )
    # debug_u/debug_v 只用于调试图画点：优先贴到颜色连通域中心，不改变矩阵输出坐标。
    debug_point = None
    if class_name == CLASS_YELLOW:
        debug_point = largest_component_centroid(yellow, circle, x0, y0)
    elif class_name == CLASS_WHITE:
        if white_component.centroid is not None:
            debug_point = (float(x0 + white_component.centroid[0]), float(y0 + white_component.centroid[1]))

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
        yellow_component_ratio=yellow_component_ratio,
        white_component_ratio=white_component_ratio,
        debug_u=debug_point[0] if debug_point is not None else None,
        debug_v=debug_point[1] if debug_point is not None else None,
    )


def largest_component_ratio(mask: np.ndarray, circle: np.ndarray, valid_count: int) -> float:
    """返回颜色 mask 中最大连通域占整个圆形 ROI 的比例。"""

    clipped = (mask & circle).astype(np.uint8)
    if int(np.count_nonzero(clipped)) <= 0:
        return 0.0
    _count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(clipped, connectivity=8)
    if len(stats) <= 1:
        return 0.0
    largest = int(stats[1:, cv2.CC_STAT_AREA].max())
    return float(largest) / float(max(1, valid_count))


@dataclass(frozen=True)
class ComponentStats:
    """颜色最大连通域的轻量形状信息。"""

    area_ratio: float
    diameter_ratio: float
    circularity: float
    centroid: tuple[float, float] | None


def largest_component_stats(
    mask: np.ndarray,
    circle: np.ndarray,
    valid_count: int,
    radius: int,
) -> ComponentStats:
    """返回颜色最大连通域面积、圆度和质心。"""

    clipped = (mask & circle).astype(np.uint8)
    if int(np.count_nonzero(clipped)) <= 0:
        return ComponentStats(area_ratio=0.0, diameter_ratio=0.0, circularity=0.0, centroid=None)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(clipped, connectivity=8)
    if count <= 1:
        return ComponentStats(area_ratio=0.0, diameter_ratio=0.0, circularity=0.0, centroid=None)

    largest_label = int(np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1)
    area = float(stats[largest_label, cv2.CC_STAT_AREA])
    component_mask = (labels == largest_label).astype(np.uint8)
    contours, _hierarchy = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    perimeter = sum(float(cv2.arcLength(contour, True)) for contour in contours)
    circularity = 0.0
    if perimeter > 1e-6:
        circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
    equivalent_diameter = float(2.0 * np.sqrt(area / np.pi))
    cx, cy = centroids[largest_label]
    return ComponentStats(
        area_ratio=area / float(max(1, valid_count)),
        diameter_ratio=equivalent_diameter / float(max(1, radius)),
        circularity=circularity,
        centroid=(float(cx), float(cy)),
    )


def is_white_component_shape_reasonable(
    component: ComponentStats,
    local_center: tuple[float, float],
    radius: int,
    config: PingpongDetectorConfig,
) -> bool:
    """保守过滤空穴白点：白色连通域应足够大、较圆、且靠近穴位中心。"""

    if component.centroid is None:
        return False
    if component.area_ratio < config.min_white_shape_component_ratio:
        return False
    if component.diameter_ratio < config.min_white_component_diameter_ratio:
        return False
    if component.circularity < config.min_white_circularity:
        return False

    dx = component.centroid[0] - local_center[0]
    dy = component.centroid[1] - local_center[1]
    offset_ratio = float(np.hypot(dx, dy)) / float(max(1, radius))
    return offset_ratio <= config.max_white_center_offset_ratio


def largest_component_centroid(
    mask: np.ndarray,
    circle: np.ndarray,
    x0: int,
    y0: int,
) -> tuple[float, float] | None:
    """返回颜色最大连通域在矫正图坐标系下的质心。

    这个点只服务 debug 图显示，避免把格子理论中心误看成真实球心。
    """

    clipped = (mask & circle).astype(np.uint8)
    if int(np.count_nonzero(clipped)) <= 0:
        return None
    _count, _labels, stats, centroids = cv2.connectedComponentsWithStats(clipped, connectivity=8)
    if len(stats) <= 1:
        return None
    largest_label = int(np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1)
    cx, cy = centroids[largest_label]
    return float(x0 + cx), float(y0 + cy)


def classify_ratios(
    yellow_ratio: float,
    white_ratio: float,
    ball_ratio: float,
    yellow_component_ratio: float,
    white_component_ratio: float,
    white_shape_ok: bool,
    config: PingpongDetectorConfig,
) -> tuple[str, float]:
    """根据颜色面积比例和连通域比例给出最终类别。

    这里的 confidence 是规则基线的相对置信度，便于下游调试，不等同于神经网络概率。
    """

    yellow_candidate = (
        yellow_ratio >= config.min_ball_ratio
        and yellow_component_ratio >= config.min_yellow_component_ratio
    )
    white_candidate = (
        white_ratio >= config.min_white_ratio
        and white_component_ratio >= config.min_white_component_ratio
        and white_shape_ok
    )
    if not yellow_candidate and not white_candidate:
        # 两类颜色都没达到阈值时按空穴输出；离阈值越远，空穴置信度越高。
        best_ratio = max(yellow_ratio, white_ratio)
        required = config.min_ball_ratio if yellow_ratio >= white_ratio else config.min_white_ratio
        confidence = 0.65 + min(0.25, (required - best_ratio) / max(required, 0.001) * 0.25)
        return CLASS_EMPTY, confidence

    if yellow_candidate and yellow_ratio >= white_ratio + config.min_color_margin:
        # 黄球要求黄色比例明显压过白色比例，避免亮黄色反光被白球分支抢走。
        color_strength = min(1.0, yellow_ratio / max(config.min_ball_ratio, 0.001))
        margin = min(1.0, (yellow_ratio - white_ratio) / max(config.min_color_margin * 4.0, 0.001))
        return CLASS_YELLOW, 0.55 + 0.25 * color_strength + 0.15 * margin

    if white_candidate and white_ratio >= yellow_ratio + config.min_color_margin:
        # 白球要求白色比例明显压过黄色比例，减少浅黄/反光区域误判。
        color_strength = min(1.0, white_ratio / max(config.min_white_ratio, 0.001))
        margin = min(1.0, (white_ratio - yellow_ratio) / max(config.min_color_margin * 4.0, 0.001))
        return CLASS_WHITE, 0.55 + 0.25 * color_strength + 0.15 * margin

    if yellow_candidate and yellow_ratio >= white_ratio:
        return CLASS_YELLOW, 0.50 + min(0.20, ball_ratio)
    if white_candidate:
        return CLASS_WHITE, 0.50 + min(0.20, ball_ratio)
    return CLASS_EMPTY, 0.60


def draw_pingpong_cells(
    image: np.ndarray,
    result: PingpongDetectionResult,
) -> np.ndarray:
    """画出每个穴位的分类标注，供 debug 图检查。"""

    output = image.copy()
    for cell in result.cells:
        # 离线 debug 图也优先把点画到颜色质心，和实时 debug_image 保持一致。
        debug_u = cell.debug_u if cell.debug_u is not None else cell.u
        debug_v = cell.debug_v if cell.debug_v is not None else cell.v
        point = (int(round(debug_u)), int(round(debug_v)))
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
    """调试图里不同类别使用的 BGR 颜色。"""

    if class_name == CLASS_YELLOW:
        return (0, 210, 255)
    if class_name == CLASS_WHITE:
        return (255, 255, 255)
    return (80, 80, 80)


def class_label(class_name: str) -> str:
    """调试图里不同类别的短标签。"""

    if class_name == CLASS_YELLOW:
        return 'Y'
    if class_name == CLASS_WHITE:
        return 'W'
    return 'E'


def draw_counts(image: np.ndarray, result: PingpongDetectionResult) -> None:
    """在调试图左上角画当前整盘统计。"""

    yellow_count = sum(1 for cell in result.cells if cell.class_name == CLASS_YELLOW)
    white_count = sum(1 for cell in result.cells if cell.class_name == CLASS_WHITE)
    empty_count = sum(1 for cell in result.cells if cell.class_name == CLASS_EMPTY)
    text = f'{result.status} yellow={yellow_count} white={white_count} empty={empty_count}'
    cv2.putText(image, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 180, 0), 2, cv2.LINE_AA)
