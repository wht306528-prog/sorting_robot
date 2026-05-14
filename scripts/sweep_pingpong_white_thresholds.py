#!/usr/bin/env python3
"""离线搜索白球识别阈值组合。

输入是已经人工标注的三盘采集样本。脚本会复用当前苗盘几何检测，
对每组白球阈值重新跑乒乓球分类，并按白球召回/误检/总体错误排序。
这个脚本只做离线调参，不修改实时节点。
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import replace
import itertools
import json
from pathlib import Path
import sys
from typing import Any

import cv2

REPO_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'sorting_vision'
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))

from sorting_vision.algorithms.pingpong_detector import (  # noqa: E402
    PingpongDetectorConfig,
    classify_cell,
    detect_pingpong_cells,
    estimate_roi_radius,
)
from sorting_vision.algorithms.tray_edge_fit import TrayEdgeFitConfig, rectify  # noqa: E402
from sorting_vision.algorithms.tray_geometry import TrayGeometryConfig, detect_tray_geometry  # noqa: E402
from sorting_vision.algorithms.tray_hole_grid import TrayHoleGridConfig  # noqa: E402


ROWS = 10
COLS = 5
TRAYS = 3
CLASS_IDS = {
    'empty': 0,
    'white_ball': 1,
    'yellow_ball': 2,
}


def main() -> None:
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    annotation_path = Path(args.annotations).expanduser()
    if not annotation_path.is_absolute():
        annotation_path = input_dir / annotation_path

    annotations = load_json(annotation_path)
    samples = find_samples(input_dir)
    if args.limit > 0:
        samples = samples[:args.limit]
    if not samples:
        raise RuntimeError(f'没有找到样本目录: {input_dir}/sample_*')

    base_pingpong_config = make_pingpong_config(args)
    prepared = prepare_samples(
        samples=samples,
        annotations=annotations,
        geometry_config=make_geometry_config(args),
        edge_config=make_edge_config(args),
    )
    if not prepared:
        raise RuntimeError('没有可评估样本：请检查 annotations.json 和 color.jpg')

    combinations = list(parameter_grid(args))
    results = []
    for index, params in enumerate(combinations, start=1):
        config = replace(base_pingpong_config, **params)
        confusion = evaluate_prepared_samples(prepared, config)
        metrics = metrics_from_confusion(confusion)
        metrics['params'] = params
        results.append(metrics)
        if args.progress and index % max(1, args.progress) == 0:
            print(f'已评估 {index}/{len(combinations)} 组参数')

    results.sort(key=ranking_key)
    print_report(input_dir, annotation_path, len(prepared), len(combinations), results, args.top_n)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='搜索乒乓球白球识别阈值。')
    parser.add_argument(
        '--input-dir',
        default='samples/pingpong_three_trays_capture/pingpong_three_trays_capture/pingpong_realtime_capture',
        help='包含 sample_* 和 annotations.json 的样本目录。',
    )
    parser.add_argument('--annotations', default='annotations.json')
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--top-n', type=int, default=12)
    parser.add_argument('--progress', type=int, default=0, help='每评估 N 组参数打印一次进度；0 表示不打印。')
    parser.add_argument('--rows', type=int, default=10)
    parser.add_argument('--cols', type=int, default=5)
    parser.add_argument('--expected-tray-count', type=int, default=3)
    parser.add_argument('--geometry-method', default='large_dark_rect')
    parser.add_argument('--geometry-dark-threshold', type=int, default=85)
    parser.add_argument('--geometry-projection-active-ratio', type=float, default=0.10)
    parser.add_argument('--geometry-projection-smooth-px', type=int, default=19)
    parser.add_argument('--geometry-min-width-ratio', type=float, default=0.08)
    parser.add_argument('--geometry-min-height-ratio', type=float, default=0.35)
    parser.add_argument('--geometry-min-area-ratio', type=float, default=0.025)
    parser.add_argument('--geometry-x-padding-px', type=int, default=10)
    parser.add_argument('--geometry-morphology-kernel-px', type=int, default=17)
    parser.add_argument('--geometry-edge-roi-padding-px', type=int, default=12)
    parser.add_argument('--split-wide-large-dark-rects', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--large-dark-max-single-width-ratio', type=float, default=0.36)
    parser.add_argument('--relax-split-structure', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--dark-threshold', type=int, default=95)
    parser.add_argument('--close-kernel-ratio', type=float, default=0.045)
    parser.add_argument('--min-area-ratio', type=float, default=0.16)
    parser.add_argument('--side-band-ratio', type=float, default=0.075)
    parser.add_argument('--roi-radius-ratio', type=float, default=0.34)
    parser.add_argument('--min-ball-ratio', type=float, default=0.16)
    parser.add_argument('--min-yellow-component-ratio', type=float, default=0.12)
    parser.add_argument('--min-color-margin', type=float, default=0.035)
    parser.add_argument('--white-ratios', type=float, nargs='+', default=[0.24, 0.28, 0.32, 0.36])
    parser.add_argument('--white-component-ratios', type=float, nargs='+', default=[0.20, 0.25, 0.30])
    parser.add_argument('--white-shape-component-ratios', type=float, nargs='+', default=[0.10, 0.14, 0.18])
    parser.add_argument('--white-diameter-ratios', type=float, nargs='+', default=[0.75, 0.85, 0.95])
    parser.add_argument('--white-circularities', type=float, nargs='+', default=[0.25, 0.30, 0.35])
    parser.add_argument('--white-center-offset-ratios', type=float, nargs='+', default=[0.55, 0.65])
    return parser.parse_args()


def load_json(path: Path) -> Any:
    with path.open('r', encoding='utf-8') as file:
        return json.load(file)


def find_samples(input_dir: Path) -> list[Path]:
    return sorted(path for path in input_dir.glob('sample_*') if path.is_dir())


def make_geometry_config(args: argparse.Namespace) -> TrayGeometryConfig:
    return TrayGeometryConfig(
        expected_tray_count=args.expected_tray_count,
        method=args.geometry_method,
        dark_threshold=args.geometry_dark_threshold,
        projection_active_ratio=args.geometry_projection_active_ratio,
        projection_smooth_px=args.geometry_projection_smooth_px,
        min_width_ratio=args.geometry_min_width_ratio,
        min_height_ratio=args.geometry_min_height_ratio,
        min_area_ratio=args.geometry_min_area_ratio,
        x_padding_px=args.geometry_x_padding_px,
        morphology_kernel_px=args.geometry_morphology_kernel_px,
        edge_roi_padding_px=args.geometry_edge_roi_padding_px,
        split_wide_large_dark_rects=args.split_wide_large_dark_rects,
        large_dark_max_single_width_ratio=args.large_dark_max_single_width_ratio,
        relax_split_structure=args.relax_split_structure,
    )


def make_edge_config(args: argparse.Namespace) -> TrayEdgeFitConfig:
    return TrayEdgeFitConfig(
        dark_threshold=args.dark_threshold,
        close_kernel_ratio=args.close_kernel_ratio,
        min_area_ratio=args.min_area_ratio,
        side_band_ratio=args.side_band_ratio,
    )


def make_pingpong_config(args: argparse.Namespace) -> PingpongDetectorConfig:
    return PingpongDetectorConfig(
        rows=args.rows,
        cols=args.cols,
        roi_radius_ratio=args.roi_radius_ratio,
        min_ball_ratio=args.min_ball_ratio,
        min_yellow_component_ratio=args.min_yellow_component_ratio,
        min_color_margin=args.min_color_margin,
        hole_grid=TrayHoleGridConfig(rows=args.rows, cols=args.cols),
    )


def parameter_grid(args: argparse.Namespace):
    names = (
        'min_white_ratio',
        'min_white_component_ratio',
        'min_white_shape_component_ratio',
        'min_white_component_diameter_ratio',
        'min_white_circularity',
        'max_white_center_offset_ratio',
    )
    value_lists = (
        args.white_ratios,
        args.white_component_ratios,
        args.white_shape_component_ratios,
        args.white_diameter_ratios,
        args.white_circularities,
        args.white_center_offset_ratios,
    )
    for values in itertools.product(*value_lists):
        yield dict(zip(names, values))


def prepare_samples(
    samples: list[Path],
    annotations: dict[str, Any],
    geometry_config: TrayGeometryConfig,
    edge_config: TrayEdgeFitConfig,
) -> list[dict[str, Any]]:
    prepared = []
    for sample_dir in samples:
        annotation = annotations.get(sample_dir.name)
        color_path = sample_dir / 'color.jpg'
        if annotation is None or not color_path.exists():
            continue
        image = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        geometry_result = detect_tray_geometry(image, geometry_config)
        rectified_by_tray = {}
        centers_by_tray = {}
        radius_by_tray = {}
        for candidate in geometry_result.candidates:
            if candidate.corners is not None and 1 <= candidate.tray_id <= TRAYS:
                rectified = rectify(image, candidate.corners, edge_config)
                rectified_by_tray[candidate.tray_id] = rectified
                base_result = detect_pingpong_cells(rectified, PingpongDetectorConfig())
                if base_result.status == 'ok':
                    centers = [
                        type('CachedCenter', (), {'row': cell.row, 'col': cell.col, 'u': cell.u, 'v': cell.v})()
                        for cell in base_result.cells
                    ]
                    centers_by_tray[candidate.tray_id] = centers
                    radius_by_tray[candidate.tray_id] = estimate_roi_radius(
                        centers,
                        rectified.shape[:2],
                        PingpongDetectorConfig(),
                    )
        prepared.append(
            {
                'sample': sample_dir.name,
                'truth': annotation_to_matrix(annotation),
                'rectified_by_tray': rectified_by_tray,
                'centers_by_tray': centers_by_tray,
                'radius_by_tray': radius_by_tray,
            }
        )
    return prepared


def annotation_to_matrix(annotation: dict[str, Any]) -> dict[tuple[int, int, int], int]:
    matrices = annotation.get('matrices_by_tray', {})
    output: dict[tuple[int, int, int], int] = {}
    for tray_id in range(1, TRAYS + 1):
        matrix = matrices.get(str(tray_id), [])
        for row in range(1, ROWS + 1):
            for col in range(1, COLS + 1):
                value = 0
                if row - 1 < len(matrix) and col - 1 < len(matrix[row - 1]):
                    value = int(matrix[row - 1][col - 1])
                output[(tray_id, row, col)] = value if value in (0, 1, 2) else 0
    return output


def evaluate_prepared_samples(
    prepared: list[dict[str, Any]],
    config: PingpongDetectorConfig,
) -> Counter[tuple[int, int]]:
    confusion: Counter[tuple[int, int]] = Counter()
    for item in prepared:
        prediction = empty_prediction()
        for tray_id, rectified in item['rectified_by_tray'].items():
            centers = item['centers_by_tray'].get(tray_id, [])
            radius = item['radius_by_tray'].get(tray_id, 0)
            if not centers or radius <= 0:
                continue
            for center in centers:
                cell = classify_cell(rectified, center, radius, config)
                prediction[(tray_id, cell.row, cell.col)] = CLASS_IDS.get(cell.class_name, 0)
        for key, true_class in item['truth'].items():
            confusion[(true_class, prediction.get(key, 0))] += 1
    return confusion


def empty_prediction() -> dict[tuple[int, int, int], int]:
    return {
        (tray_id, row, col): 0
        for tray_id in range(1, TRAYS + 1)
        for row in range(1, ROWS + 1)
        for col in range(1, COLS + 1)
    }


def metrics_from_confusion(confusion: Counter[tuple[int, int]]) -> dict[str, Any]:
    total = sum(confusion.values())
    correct = sum(count for (true_class, pred_class), count in confusion.items() if true_class == pred_class)
    white_tp = confusion[(1, 1)]
    white_truth = sum(confusion[(1, pred)] for pred in (0, 1, 2))
    white_pred = sum(confusion[(truth, 1)] for truth in (0, 1, 2))
    yellow_tp = confusion[(2, 2)]
    yellow_truth = sum(confusion[(2, pred)] for pred in (0, 1, 2))
    yellow_pred = sum(confusion[(truth, 2)] for truth in (0, 1, 2))
    return {
        'accuracy': correct / total if total else 0.0,
        'errors': total - correct,
        'white_recall': white_tp / white_truth if white_truth else 0.0,
        'white_precision': white_tp / white_pred if white_pred else 0.0,
        'white_missed': white_truth - white_tp,
        'white_false_positive': white_pred - white_tp,
        'yellow_recall': yellow_tp / yellow_truth if yellow_truth else 0.0,
        'yellow_precision': yellow_tp / yellow_pred if yellow_pred else 0.0,
        'yellow_missed': yellow_truth - yellow_tp,
        'yellow_false_positive': yellow_pred - yellow_tp,
    }


def ranking_key(metrics: dict[str, Any]) -> tuple[float, int, float, int, int]:
    # 优先少误检，其次白球召回高，再看总错误和黄球表现。
    return (
        metrics['white_false_positive'],
        -metrics['white_recall'],
        -metrics['accuracy'],
        metrics['yellow_missed'],
        metrics['errors'],
    )


def print_report(
    input_dir: Path,
    annotation_path: Path,
    sample_count: int,
    combination_count: int,
    results: list[dict[str, Any]],
    top_n: int,
) -> None:
    print('')
    print('========== 白球阈值离线搜索 ==========')
    print(f'样本目录: {input_dir}')
    print(f'标注文件: {annotation_path}')
    print(f'参与样本: {sample_count}')
    print(f'参数组合: {combination_count}')
    print('')
    print(f'推荐前 {top_n} 组:')
    for index, result in enumerate(results[:top_n], start=1):
        params = result['params']
        print(
            f'{index:02d}. '
            f'acc={result["accuracy"] * 100:5.1f}%  '
            f'W召回={result["white_recall"] * 100:5.1f}%  '
            f'W精确={result["white_precision"] * 100:5.1f}%  '
            f'W漏={result["white_missed"]:3d}  W误={result["white_false_positive"]:2d}  '
            f'Y召回={result["yellow_recall"] * 100:5.1f}%  Y误={result["yellow_false_positive"]:2d}'
        )
        print(
            '    '
            f'MIN_WHITE_RATIO={params["min_white_ratio"]:.2f}  '
            f'MIN_WHITE_COMPONENT_RATIO={params["min_white_component_ratio"]:.2f}  '
            f'MIN_WHITE_SHAPE_COMPONENT_RATIO={params["min_white_shape_component_ratio"]:.2f}  '
            f'MIN_WHITE_COMPONENT_DIAMETER_RATIO={params["min_white_component_diameter_ratio"]:.2f}  '
            f'MIN_WHITE_CIRCULARITY={params["min_white_circularity"]:.2f}  '
            f'MAX_WHITE_CENTER_OFFSET_RATIO={params["max_white_center_offset_ratio"]:.2f}'
        )


if __name__ == '__main__':
    main()
