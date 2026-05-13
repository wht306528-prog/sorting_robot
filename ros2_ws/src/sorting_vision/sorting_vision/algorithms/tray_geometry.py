"""苗盘几何识别新主线。

这个模块是新的算法入口，从空接口开始，不继承旧的
`offline_tray_debug.py`、`tray_grid_debug.py`、`tray_rectify_debug.py` 实现。

目标顺序：
1. 粗定位 3 个苗盘区域。
2. 每个区域拟合真实外边界四条边。
3. 四边相交得到四角。
4. 透视矫正为单盘正视图。
5. 在正视图中识别穴位中心。

当前文件只定义数据结构和管线入口，避免在算法没有验证前继续画假框。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class TrayGeometryConfig:
    """苗盘几何识别配置。"""

    expected_tray_count: int = 3
    dark_threshold: int = 85
    method: str = 'large_dark_rect'
    projection_active_ratio: float = 0.10
    projection_smooth_px: int = 19
    min_width_ratio: float = 0.08
    min_height_ratio: float = 0.35
    min_area_ratio: float = 0.025
    x_padding_px: int = 10
    morphology_kernel_px: int = 17
    edge_roi_padding_px: int = 12
    hough_threshold: int = 35
    hough_min_line_length_ratio: float = 0.35
    hough_max_line_gap_px: int = 18
    split_connected_trays: bool = False
    large_dark_min_area_ratio: float = 0.035
    large_dark_min_width_ratio: float = 0.16
    large_dark_min_height_ratio: float = 0.30
    large_dark_min_extent: float = 0.35
    large_dark_max_extent: float = 0.96
    large_dark_min_edge_density: float = 0.08
    large_dark_min_bright_components: int = 18
    split_wide_large_dark_rects: bool = True
    large_dark_max_single_width_ratio: float = 0.36
    relax_split_structure: bool = True


@dataclass(frozen=True)
class TrayGeometryCandidate:
    """单个苗盘几何候选。"""

    tray_id: int
    status: str
    message: str
    bbox: tuple[int, int, int, int]
    center: tuple[float, float]
    area: float
    corners: np.ndarray | None = None


@dataclass(frozen=True)
class TrayGeometryResult:
    """一张图的苗盘几何识别结果。"""

    status: str
    message: str
    candidates: list[TrayGeometryCandidate]


def detect_tray_geometry(
    image: np.ndarray,
    config: TrayGeometryConfig | None = None,
) -> TrayGeometryResult:
    """检测一张 RGB/BGR 图中的苗盘几何。

    Step 01 只做粗定位，不做四边拟合和透视矫正。
    """

    if config is None:
        config = TrayGeometryConfig()
    if image.ndim != 3 or image.shape[2] != 3:
        return TrayGeometryResult(
            status='failed',
            message='input image must be HxWx3',
            candidates=[],
        )
    if config.expected_tray_count <= 0:
        return TrayGeometryResult(
            status='failed',
            message='expected_tray_count must be positive',
            candidates=[],
        )
    if config.method == 'large_dark_rect':
        candidates = locate_large_dark_rect_candidates(image, config)
    else:
        candidates = locate_tray_candidates(image, config)
    fitted_count = sum(1 for candidate in candidates if candidate.corners is not None)
    if not candidates:
        return TrayGeometryResult(
            status='ok',
            message=f'found 0 of up to {config.expected_tray_count} tray candidates',
            candidates=[],
        )
    if fitted_count == len(candidates):
        return TrayGeometryResult(
            status='ok',
            message=f'found {len(candidates)} of up to {config.expected_tray_count} tray candidates with fitted edges',
            candidates=candidates,
        )
    return TrayGeometryResult(
        status='partial_edges',
        message=(
            f'found {len(candidates)} of up to {config.expected_tray_count} tray candidates, '
            f'fitted edges for {fitted_count}'
        ),
        candidates=candidates,
    )


def locate_tray_candidates(
    image: np.ndarray,
    config: TrayGeometryConfig,
) -> list[TrayGeometryCandidate]:
    """按深色主体横向投影粗定位苗盘。"""

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 苗盘主体通常比背景更暗，先用灰度阈值抽出大块暗区域。
    mask = build_dark_body_mask(gray, config)
    # 沿 x 方向投影，得到每个盘大致占据的横向区间。
    ranges = find_active_x_ranges(mask, config)

    candidates: list[TrayGeometryCandidate] = []
    image_shape = image.shape
    for x_start, x_end in ranges:
        candidate = candidate_from_x_range(mask, image_shape, x_start, x_end, config)
        if candidate is not None:
            candidates.append(candidate)

    if config.split_connected_trays and len(candidates) < config.expected_tray_count:
        # 多个盘在投影上粘连时，尝试把过宽区间拆成多个候选。
        # 默认关闭：现场单个横放苗盘也会形成很宽的暗区，强行拆分会把一个盘误判成多个盘。
        split_candidates = split_wide_ranges(mask, image_shape, ranges, candidates, config)
        if len(split_candidates) >= config.expected_tray_count:
            candidates = split_candidates
        else:
            candidates.extend(split_candidates)

    candidates = dedupe_candidates(candidates)
    candidates.sort(key=lambda item: item.center[0])
    candidates = candidates[:config.expected_tray_count]
    # tray_id 按画面从左到右分配：1 左盘，2 中盘，3 右盘。
    numbered = [
        TrayGeometryCandidate(
            tray_id=index,
            status=candidate.status,
            message=candidate.message,
            bbox=candidate.bbox,
            center=candidate.center,
            area=candidate.area,
            corners=candidate.corners,
        )
        for index, candidate in enumerate(candidates, start=1)
    ]
    refined = [refine_candidate_edges(gray, candidate, config) for candidate in numbered]
    # 只有四边拟合成功的候选才进入后续识别和矩阵输出。
    # 现场画面里电线、桌腿、阴影也可能形成暗色粗框；如果继续把这些 coarse
    # 候选画成 tray，会让用户误以为系统真的识别到了多个苗盘。
    valid = [candidate for candidate in refined if candidate.corners is not None]
    valid.sort(key=lambda item: item.center[0])
    return [
        TrayGeometryCandidate(
            tray_id=index,
            status=candidate.status,
            message=candidate.message,
            bbox=candidate.bbox,
            center=candidate.center,
            area=candidate.area,
            corners=candidate.corners,
        )
        for index, candidate in enumerate(valid, start=1)
    ]


def locate_large_dark_rect_candidates(
    image: np.ndarray,
    config: TrayGeometryConfig,
) -> list[TrayGeometryCandidate]:
    """按“大面积黑色矩形主体”寻找苗盘候选。

    这个函数用于离线实验和后续可选后端。它不依赖横向投影，而是直接找
    大面积暗色连通域，再按面积、宽高和填充率过滤背景干扰。
    """

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = build_large_dark_rect_mask(gray, config)
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_height, image_width = image.shape[:2]
    min_area = image_height * image_width * config.large_dark_min_area_ratio
    min_width = image_width * config.large_dark_min_width_ratio
    min_height = image_height * config.large_dark_min_height_ratio
    max_single_width = image_width * config.large_dark_max_single_width_ratio

    candidates: list[TrayGeometryCandidate] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if (
            config.split_wide_large_dark_rects
            and width > max_single_width
            and height >= min_height
        ):
            split_boxes = split_large_dark_bbox_by_expected_width(
                bbox=(x, y, width, height),
                expected_width=estimate_large_dark_reference_width(
                    contours=contours,
                    image_shape=image.shape,
                    config=config,
                    exclude_bbox=(x, y, width, height),
                ),
                config=config,
            )
            for split_box in split_boxes:
                candidate = large_dark_candidate_from_bbox(
                    image=image,
                    mask=mask,
                    bbox=split_box,
                    config=config,
                    relax_structure=config.relax_split_structure,
                )
                if candidate is not None:
                    candidates.append(candidate)
            continue

        rect = cv2.minAreaRect(contour)
        rect_width, rect_height = rect[1]
        if rect_width <= 1.0 or rect_height <= 1.0:
            continue
        box_width = max(rect_width, rect_height)
        box_height = min(rect_width, rect_height)
        if box_width < min_width or box_height < min_height:
            continue
        rect_area = float(rect_width * rect_height)
        extent = area / max(rect_area, 1.0)
        if extent < config.large_dark_min_extent or extent > config.large_dark_max_extent:
            continue
        if not has_tray_like_internal_structure(
            image[y:y + height, x:x + width],
            config,
        ):
            continue

        box = cv2.boxPoints(rect).astype(np.float32)
        corners = order_points_clockwise(box)
        center = rect[0]
        candidates.append(
            TrayGeometryCandidate(
                tray_id=0,
                status='large_dark_rect',
                message=f'large dark rect area={area:.1f} extent={extent:.2f}',
                bbox=(int(x), int(y), int(width), int(height)),
                center=(float(center[0]), float(center[1])),
                area=area,
                corners=corners,
            )
        )

    candidates = dedupe_candidates(candidates)
    candidates.sort(key=lambda item: item.center[0])
    candidates = candidates[:config.expected_tray_count]
    return [
        TrayGeometryCandidate(
            tray_id=index,
            status=candidate.status,
            message=candidate.message,
            bbox=candidate.bbox,
            center=candidate.center,
            area=candidate.area,
            corners=candidate.corners,
        )
        for index, candidate in enumerate(candidates, start=1)
    ]


def split_large_dark_bbox_by_expected_width(
    bbox: tuple[int, int, int, int],
    expected_width: float,
    config: TrayGeometryConfig,
) -> list[tuple[int, int, int, int]]:
    """按估计单盘宽度，把粘在一起的大黑框均分成多个候选框。"""

    x, y, width, height = bbox
    split_count = int(round(width / max(1.0, expected_width)))
    split_count = max(2, min(config.expected_tray_count, split_count))
    output: list[tuple[int, int, int, int]] = []
    for index in range(split_count):
        x0 = int(round(x + index * width / split_count))
        x1 = int(round(x + (index + 1) * width / split_count)) - 1
        output.append((x0, y, x1 - x0 + 1, height))
    return output


def estimate_large_dark_reference_width(
    contours: tuple[np.ndarray, ...],
    image_shape: tuple[int, ...],
    config: TrayGeometryConfig,
    exclude_bbox: tuple[int, int, int, int],
) -> float:
    """用其他正常大黑框宽度估计单个苗盘宽度。"""

    image_height, image_width = image_shape[:2]
    min_area = image_height * image_width * config.large_dark_min_area_ratio
    min_width = image_width * config.large_dark_min_width_ratio
    min_height = image_height * config.large_dark_min_height_ratio
    widths: list[int] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if (x, y, width, height) == exclude_bbox:
            continue
        if width >= min_width and height >= min_height:
            widths.append(width)
    if widths:
        return float(np.median(np.asarray(widths, dtype=np.float32)))
    return float(max(1, exclude_bbox[2] / 2.0))


def large_dark_candidate_from_bbox(
    image: np.ndarray,
    mask: np.ndarray,
    bbox: tuple[int, int, int, int],
    config: TrayGeometryConfig,
    relax_structure: bool = False,
) -> TrayGeometryCandidate | None:
    """从拆分后的子框生成 large_dark_rect 候选。"""

    x, y, width, height = bbox
    if width <= 2 or height <= 2:
        return None
    crop = image[y:y + height, x:x + width]
    if not relax_structure and not has_tray_like_internal_structure(crop, config):
        return None

    roi_mask = mask[y:y + height, x:x + width]
    contours, _hierarchy = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(contour)
    rect_width, rect_height = rect[1]
    if rect_width <= 1.0 or rect_height <= 1.0:
        return None
    box = cv2.boxPoints(rect).astype(np.float32)
    box[:, 0] += x
    box[:, 1] += y
    corners = order_points_clockwise(box)
    area = float(cv2.contourArea(contour))
    center = (float(rect[0][0] + x), float(rect[0][1] + y))
    return TrayGeometryCandidate(
        tray_id=0,
        status='split_large_dark_rect',
        message='split wide large dark rect by expected tray width',
        bbox=(int(x), int(y), int(width), int(height)),
        center=center,
        area=area,
        corners=corners,
    )


def build_dark_body_mask(gray: np.ndarray, config: TrayGeometryConfig) -> np.ndarray:
    """生成苗盘暗色主体 mask。"""

    mask = (gray < config.dark_threshold).astype(np.uint8) * 255
    kernel_size = max(3, config.morphology_kernel_px)
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    # 闭运算连接苗盘边框和暗孔洞，开运算去掉孤立噪声点。
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)
    return mask


def build_large_dark_rect_mask(gray: np.ndarray, config: TrayGeometryConfig) -> np.ndarray:
    """生成大黑色主体候选 mask。"""

    threshold = min(config.dark_threshold, 50)
    mask = (gray < threshold).astype(np.uint8) * 255
    kernel_size = max(3, min(config.morphology_kernel_px, 5))
    if kernel_size % 2 == 0:
        kernel_size += 1
    close_kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)
    return mask


def has_tray_like_internal_structure(crop: np.ndarray, config: TrayGeometryConfig) -> bool:
    """检查候选内部是否像苗盘：边缘多、亮色小组件多。"""

    if crop.size == 0 or crop.ndim != 3:
        return False
    height, width = crop.shape[:2]
    if height <= 0 or width <= 0:
        return False

    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 45, 130)
    edge_density = float(np.count_nonzero(edges)) / float(max(1, height * width))
    if edge_density < config.large_dark_min_edge_density:
        return False

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    # 苗盘穴位里常见灰色孔底、白球、黄球等小亮块；普通黑椅子/阴影缺少这种密集结构。
    bright = ((val > 105) & (sat < 150)).astype(np.uint8) * 255
    contours, _hierarchy = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    component_count = 0
    max_component_area = max(80.0, height * width * 0.08)
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if 12.0 <= area <= max_component_area:
            component_count += 1
    return component_count >= config.large_dark_min_bright_components


def order_points_clockwise(points: np.ndarray) -> np.ndarray:
    """把四点排序为左上、右上、右下、左下。"""

    points = points.astype(np.float32)
    sums = points[:, 0] + points[:, 1]
    diffs = points[:, 0] - points[:, 1]
    top_left = points[int(np.argmin(sums))]
    bottom_right = points[int(np.argmax(sums))]
    top_right = points[int(np.argmax(diffs))]
    bottom_left = points[int(np.argmin(diffs))]
    return np.asarray([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)


def find_active_x_ranges(mask: np.ndarray, config: TrayGeometryConfig) -> list[tuple[int, int]]:
    """从横向投影中找可能包含苗盘的 x 区间。"""

    active_threshold = mask.shape[0] * config.projection_active_ratio
    projection = (mask > 0).sum(axis=0).astype(np.float32)
    # 平滑投影可以抑制单列噪声造成的断裂。
    projection = smooth_projection(projection, config.projection_smooth_px)
    active = projection > active_threshold
    min_width = max(20, int(round(mask.shape[1] * config.min_width_ratio)))
    return active_ranges(active, min_width=min_width, padding=config.x_padding_px)


def candidate_from_x_range(
    mask: np.ndarray,
    image_shape: tuple[int, ...],
    x_start: int,
    x_end: int,
    config: TrayGeometryConfig,
) -> TrayGeometryCandidate | None:
    """从一个 x 区间内提取面积和高度合格的苗盘粗候选。"""

    image_height, image_width = image_shape[:2]
    x_start = max(0, x_start)
    x_end = min(image_width - 1, x_end)
    if x_end <= x_start:
        return None

    band = mask[:, x_start:x_end + 1]
    contours, _hierarchy = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 面积和高度阈值用于过滤电线、阴影、桌面边缘等非苗盘暗块。
    min_area = image_height * image_width * config.min_area_ratio
    min_height = image_height * config.min_height_ratio
    best: tuple[np.ndarray, float] | None = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if height < min_height:
            continue
        if best is None or area > best[1]:
            best = (contour, area)

    if best is None:
        return None

    contour, area = best
    x, y, width, height = cv2.boundingRect(contour)
    x += x_start
    center = (x + width * 0.5, y + height * 0.5)
    return TrayGeometryCandidate(
        tray_id=0,
        status='coarse',
        message='dark projection candidate',
        bbox=(int(x), int(y), int(width), int(height)),
        center=(float(center[0]), float(center[1])),
        area=area,
        corners=None,
    )


def split_wide_ranges(
    mask: np.ndarray,
    image_shape: tuple[int, ...],
    ranges: list[tuple[int, int]],
    existing: list[TrayGeometryCandidate],
    config: TrayGeometryConfig,
) -> list[TrayGeometryCandidate]:
    """把粘连的宽横向区间拆成多个粗候选。"""

    if len(existing) >= config.expected_tray_count:
        return []
    if not ranges:
        return []

    image_width = image_shape[1]
    output: list[TrayGeometryCandidate] = []
    expected_width = estimate_expected_width(image_width, existing)

    for x_start, x_end in sorted(ranges, key=lambda item: item[1] - item[0], reverse=True):
        width = x_end - x_start + 1
        if width < expected_width * 1.45:
            continue
        # 根据估计盘宽把粘连区间均分，再分别提粗候选。
        split_count = int(round(width / max(1.0, expected_width)))
        split_count = max(2, min(config.expected_tray_count, split_count))
        for index in range(split_count):
            sub_start = int(round(x_start + index * width / split_count))
            sub_end = int(round(x_start + (index + 1) * width / split_count)) - 1
            candidate = candidate_from_x_range(mask, image_shape, sub_start, sub_end, config)
            if candidate is None:
                continue
            output.append(
                TrayGeometryCandidate(
                    tray_id=0,
                    status='coarse_split',
                    message='split from connected dark projection range',
                    bbox=candidate.bbox,
                    center=candidate.center,
                    area=candidate.area,
                    corners=candidate.corners,
                )
            )
    return output


def refine_candidate_edges(
    gray: np.ndarray,
    candidate: TrayGeometryCandidate,
    config: TrayGeometryConfig,
) -> TrayGeometryCandidate:
    """对粗候选继续拟合外边四角。"""

    corners = fit_outer_corners_from_roi(gray, candidate.bbox, config)
    if corners is None:
        return TrayGeometryCandidate(
            tray_id=candidate.tray_id,
            status=candidate.status,
            message=candidate.message + '; edge_fit_failed',
            bbox=candidate.bbox,
            center=candidate.center,
            area=candidate.area,
            corners=None,
        )
    return TrayGeometryCandidate(
        tray_id=candidate.tray_id,
        status='edge_fit',
        message=candidate.message + '; fitted outer edge lines',
        bbox=candidate.bbox,
        center=candidate.center,
        area=candidate.area,
        corners=corners,
    )


def fit_outer_corners_from_roi(
    gray: np.ndarray,
    bbox: tuple[int, int, int, int],
    config: TrayGeometryConfig,
) -> np.ndarray | None:
    """在候选 ROI 内用边缘线拟合外框四角。"""

    x, y, width, height = pad_bbox(bbox, gray.shape[1], gray.shape[0], config.edge_roi_padding_px)
    roi = gray[y:y + height, x:x + width]
    if roi.size == 0:
        return None

    # 先模糊再 Canny，减少纹理和小孔对外边线的干扰。
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)
    edges = cv2.Canny(blurred, 45, 130)
    min_line_length = max(30, int(round(min(width, height) * config.hough_min_line_length_ratio)))
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=config.hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=config.hough_max_line_gap_px,
    )
    if lines is None:
        return None

    raw_lines = [
        (
            float(line[0][0] + x),
            float(line[0][1] + y),
            float(line[0][2] + x),
            float(line[0][3] + y),
        )
        for line in lines
    ]
    # Hough 得到很多线段后，分别挑靠近上下左右边的线段点去拟合直线。
    sides = select_side_line_points(raw_lines, bbox)
    fitted = {side: fit_line(points) for side, points in sides.items()}
    if any(value is None for value in fitted.values()):
        return None

    top = fitted['top']
    right = fitted['right']
    bottom = fitted['bottom']
    left = fitted['left']
    assert top is not None and right is not None and bottom is not None and left is not None
    corners = [
        intersect_lines(top, left),
        intersect_lines(top, right),
        intersect_lines(bottom, right),
        intersect_lines(bottom, left),
    ]
    if any(point is None for point in corners):
        return None
    points = np.asarray(corners, dtype=np.float32)
    if not corners_are_reasonable(points, bbox):
        return None
    return points


def select_side_line_points(
    raw_lines: list[tuple[float, float, float, float]],
    bbox: tuple[int, int, int, int],
) -> dict[str, list[tuple[float, float]]]:
    """把 Hough 线段按上下左右四条边归组。"""

    x, y, width, height = bbox
    left_target = x
    right_target = x + width
    top_target = y
    bottom_target = y + height
    vertical: list[tuple[float, float, tuple[float, float, float, float]]] = []
    horizontal: list[tuple[float, float, tuple[float, float, float, float]]] = []

    for line in raw_lines:
        x1, y1, x2, y2 = line
        dx = x2 - x1
        dy = y2 - y1
        length = float(np.hypot(dx, dy))
        if length < 25.0:
            continue
        angle = normalize_angle(float(np.degrees(np.arctan2(dy, dx))))
        if abs(angle) > 65.0:
            # 近似竖直线用于左/右边。
            position = (x1 + x2) * 0.5
            vertical.append((position, length, line))
        elif abs(angle) < 25.0:
            # 近似水平线用于上/下边。
            position = (y1 + y2) * 0.5
            horizontal.append((position, length, line))

    return {
        'left': points_from_near_side(vertical, left_target, low_side=True),
        'right': points_from_near_side(vertical, right_target, low_side=False),
        'top': points_from_near_side(horizontal, top_target, low_side=True),
        'bottom': points_from_near_side(horizontal, bottom_target, low_side=False),
    }


def points_from_near_side(
    scored_lines: list[tuple[float, float, tuple[float, float, float, float]]],
    target: float,
    low_side: bool,
) -> list[tuple[float, float]]:
    """选择最靠近目标边的一组线段端点。"""

    if not scored_lines:
        return []
    scored_lines = sorted(scored_lines, key=lambda item: abs(item[0] - target))
    anchor = scored_lines[0][0]
    # 只取与最近线段位置相近的一簇，避免把内孔线段混进外边线拟合。
    selected = [
        line
        for position, _length, line in scored_lines
        if abs(position - anchor) <= 30.0
    ]
    selected = selected[:10]
    points: list[tuple[float, float]] = []
    for x1, y1, x2, y2 in selected:
        points.append((x1, y1))
        points.append((x2, y2))
    return points


def fit_line(points: list[tuple[float, float]]) -> tuple[float, float, float] | None:
    """把点集拟合成 ax + by + c = 0 形式的直线。"""

    if len(points) < 2:
        return None
    data = np.asarray(points, dtype=np.float32)
    vx, vy, x0, y0 = cv2.fitLine(data, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(-1)
    a = float(vy)
    b = -float(vx)
    c = float(vx * y0 - vy * x0)
    norm = float(np.hypot(a, b))
    if norm <= 1e-6:
        return None
    return a / norm, b / norm, c / norm


def intersect_lines(
    line_a: tuple[float, float, float],
    line_b: tuple[float, float, float],
) -> tuple[float, float] | None:
    """计算两条直线交点。"""

    a1, b1, c1 = line_a
    a2, b2, c2 = line_b
    determinant = a1 * b2 - a2 * b1
    if abs(determinant) < 1e-6:
        return None
    x = (b1 * c2 - b2 * c1) / determinant
    y = (c1 * a2 - c2 * a1) / determinant
    return float(x), float(y)


def corners_are_reasonable(points: np.ndarray, bbox: tuple[int, int, int, int]) -> bool:
    """检查四角是否还在粗框附近，并且面积没有塌缩。"""

    x, y, width, height = bbox
    margin = max(width, height) * 0.30
    if np.any(points[:, 0] < x - margin) or np.any(points[:, 0] > x + width + margin):
        return False
    if np.any(points[:, 1] < y - margin) or np.any(points[:, 1] > y + height + margin):
        return False
    area = abs(float(cv2.contourArea(points.astype(np.float32))))
    if area < width * height * 0.35:
        return False

    edge_lengths = [
        float(np.linalg.norm(points[(index + 1) % 4] - points[index]))
        for index in range(4)
    ]
    shortest = min(edge_lengths)
    longest = max(edge_lengths)
    if shortest <= 1.0 or longest / shortest > 5.0:
        return False

    rect = cv2.minAreaRect(points.astype(np.float32))
    rect_width, rect_height = rect[1]
    if rect_width <= 1.0 or rect_height <= 1.0:
        return False
    aspect = max(rect_width, rect_height) / min(rect_width, rect_height)
    # 正常俯视/斜俯视苗盘不会退化成很细长的侧面条；侧面视角不适合输出吸取矩阵。
    return aspect <= 3.2


def pad_bbox(
    bbox: tuple[int, int, int, int],
    image_width: int,
    image_height: int,
    padding: int,
) -> tuple[int, int, int, int]:
    """给候选框加边距，同时裁剪到图像范围内。"""

    x, y, width, height = bbox
    x0 = max(0, x - padding)
    y0 = max(0, y - padding)
    x1 = min(image_width, x + width + padding)
    y1 = min(image_height, y + height + padding)
    return x0, y0, x1 - x0, y1 - y0


def normalize_angle(angle: float) -> float:
    """把线段角度归一到 [-90, 90]，便于区分横线和竖线。"""

    while angle <= -90.0:
        angle += 180.0
    while angle > 90.0:
        angle -= 180.0
    return angle


def estimate_expected_width(
    image_width: int,
    existing: list[TrayGeometryCandidate],
) -> float:
    """估计单个苗盘在当前画面里的宽度。"""

    widths = [candidate.bbox[2] for candidate in existing if candidate.bbox[2] > 0]
    if widths:
        median_width = float(np.median(np.asarray(widths, dtype=np.float32)))
        if median_width < image_width * 0.55:
            return median_width
    return image_width / 3.4


def dedupe_candidates(candidates: list[TrayGeometryCandidate]) -> list[TrayGeometryCandidate]:
    """按 IoU 去掉重复候选，保留面积更大的那个。"""

    output: list[TrayGeometryCandidate] = []
    for candidate in sorted(candidates, key=lambda item: item.area, reverse=True):
        if any(bbox_iou(candidate.bbox, other.bbox) > 0.35 for other in output):
            continue
        output.append(candidate)
    return output


def bbox_iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    """计算两个候选框的交并比。"""

    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    left = max(ax, bx)
    top = max(ay, by)
    right = min(ax + aw, bx + bw)
    bottom = min(ay + ah, by + bh)
    intersection = max(0, right - left) * max(0, bottom - top)
    union = aw * ah + bw * bh - intersection
    return intersection / max(1, union)


def smooth_projection(projection: np.ndarray, kernel_size: int) -> np.ndarray:
    """滑动平均平滑一维投影曲线。"""

    kernel_size = max(1, kernel_size)
    kernel = np.ones(kernel_size, dtype=np.float32) / float(kernel_size)
    return np.convolve(projection, kernel, mode='same')


def active_ranges(active: np.ndarray, min_width: int, padding: int) -> list[tuple[int, int]]:
    """把连续 active 的列合并成 x 区间。"""

    ranges: list[tuple[int, int]] = []
    start: int | None = None
    for index, value in enumerate(active):
        if bool(value) and start is None:
            start = index
        elif not bool(value) and start is not None:
            if index - start >= min_width:
                ranges.append((max(0, start - padding), min(len(active) - 1, index + padding)))
            start = None
    if start is not None and len(active) - start >= min_width:
        ranges.append((max(0, start - padding), len(active) - 1))
    return ranges
