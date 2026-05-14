#!/usr/bin/env python3
"""评估三盘乒乓球矩阵输出和人工标注的一致性。

默认读取采集目录中的 annotations.json 和每个 sample_*/tray_matrix.json。
这个脚本只做离线评估，不参与实时 ROS2 节点和 F407 通信。
"""

from __future__ import annotations

import argparse
from collections import Counter
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
    detect_pingpong_cells,
)
from sorting_vision.algorithms.tray_edge_fit import TrayEdgeFitConfig, rectify  # noqa: E402
from sorting_vision.algorithms.tray_geometry import TrayGeometryConfig, detect_tray_geometry  # noqa: E402
from sorting_vision.algorithms.tray_hole_grid import TrayHoleGridConfig  # noqa: E402


ROWS = 10
COLS = 5
TRAYS = 3
CLASS_NAMES = {
    0: 'empty',
    1: 'white',
    2: 'yellow',
}
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

    summaries = []
    total_confusion: Counter[tuple[int, int]] = Counter()
    missing_annotations = []
    missing_predictions = []

    geometry_config = make_geometry_config(args)
    edge_config = make_edge_config(args)
    pingpong_config = make_pingpong_config(args)

    for sample_dir in samples:
        sample_name = sample_dir.name
        annotation = annotations.get(sample_name)
        if annotation is None:
            missing_annotations.append(sample_name)
            continue

        truth = annotation_to_matrix(annotation)
        if args.mode == 'saved':
            matrix_path = sample_dir / 'tray_matrix.json'
            if not matrix_path.exists():
                missing_predictions.append(sample_name)
                continue
            predicted = tray_matrix_to_matrix(load_json(matrix_path))
            predict_status = 'saved'
        else:
            color_path = sample_dir / 'color.jpg'
            if not color_path.exists():
                missing_predictions.append(sample_name)
                continue
            predicted, predict_status = predict_current_matrix(
                color_path=color_path,
                geometry_config=geometry_config,
                edge_config=edge_config,
                pingpong_config=pingpong_config,
            )

        summary, confusion = evaluate_matrix(sample_name, truth, predicted)
        summary['predict_status'] = predict_status
        summaries.append(summary)
        total_confusion.update(confusion)

    print_report(
        input_dir=input_dir,
        annotation_path=annotation_path,
        summaries=summaries,
        total_confusion=total_confusion,
        missing_annotations=missing_annotations,
        missing_predictions=missing_predictions,
        top_n=args.top_n,
        mode=args.mode,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='评估乒乓球矩阵输出和人工标注。')
    parser.add_argument(
        '--input-dir',
        default='samples/pingpong_three_trays_capture/pingpong_three_trays_capture/pingpong_realtime_capture',
        help='包含 sample_* 和 annotations.json 的样本目录。',
    )
    parser.add_argument(
        '--annotations',
        default='annotations.json',
        help='人工标注 JSON。相对路径会按 input-dir 解析。',
    )
    parser.add_argument('--limit', type=int, default=0, help='只评估前 N 个样本；0 表示全部。')
    parser.add_argument('--top-n', type=int, default=8, help='打印错误最多的前 N 张样本。')
    parser.add_argument(
        '--mode',
        choices=('saved', 'current'),
        default='saved',
        help='saved=评估采集时保存的 tray_matrix.json；current=用当前代码重新识别 color.jpg。',
    )
    parser.add_argument('--rows', type=int, default=10, help='苗盘行数。')
    parser.add_argument('--cols', type=int, default=5, help='苗盘列数。')
    parser.add_argument('--expected-tray-count', type=int, default=3, help='最多检测几个苗盘。')
    parser.add_argument('--geometry-method', default='large_dark_rect', help='整帧苗盘检测方法。')
    parser.add_argument('--geometry-dark-threshold', type=int, default=85, help='整帧深色主体阈值。')
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
    parser.add_argument('--dark-threshold', type=int, default=95, help='单盘矫正深色阈值。')
    parser.add_argument('--close-kernel-ratio', type=float, default=0.045)
    parser.add_argument('--min-area-ratio', type=float, default=0.16)
    parser.add_argument('--side-band-ratio', type=float, default=0.075)
    parser.add_argument('--roi-radius-ratio', type=float, default=0.34)
    parser.add_argument('--min-ball-ratio', type=float, default=0.16)
    parser.add_argument('--min-white-ratio', type=float, default=0.36)
    parser.add_argument('--min-white-component-ratio', type=float, default=0.30)
    parser.add_argument('--min-white-shape-component-ratio', type=float, default=0.18)
    parser.add_argument('--min-white-component-diameter-ratio', type=float, default=0.95)
    parser.add_argument('--min-white-circularity', type=float, default=0.35)
    parser.add_argument('--max-white-center-offset-ratio', type=float, default=0.55)
    parser.add_argument('--min-yellow-component-ratio', type=float, default=0.12)
    parser.add_argument('--min-color-margin', type=float, default=0.035)
    return parser.parse_args()


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
        min_white_ratio=args.min_white_ratio,
        min_white_component_ratio=args.min_white_component_ratio,
        min_white_shape_component_ratio=args.min_white_shape_component_ratio,
        min_white_component_diameter_ratio=args.min_white_component_diameter_ratio,
        min_white_circularity=args.min_white_circularity,
        max_white_center_offset_ratio=args.max_white_center_offset_ratio,
        min_yellow_component_ratio=args.min_yellow_component_ratio,
        min_color_margin=args.min_color_margin,
        hole_grid=TrayHoleGridConfig(rows=args.rows, cols=args.cols),
    )


def load_json(path: Path) -> Any:
    with path.open('r', encoding='utf-8') as file:
        return json.load(file)


def find_samples(input_dir: Path) -> list[Path]:
    return sorted(path for path in input_dir.glob('sample_*') if path.is_dir())


def annotation_to_matrix(annotation: dict[str, Any]) -> dict[tuple[int, int, int], int]:
    matrices = annotation.get('matrices_by_tray', {})
    output: dict[tuple[int, int, int], int] = {}
    for tray_id in range(1, TRAYS + 1):
        matrix = matrices.get(str(tray_id), [])
        for row in range(1, ROWS + 1):
            for col in range(1, COLS + 1):
                value = 0
                if row - 1 < len(matrix) and col - 1 < len(matrix[row - 1]):
                    value = normalize_class_id(matrix[row - 1][col - 1])
                output[(tray_id, row, col)] = value
    return output


def tray_matrix_to_matrix(data: dict[str, Any]) -> dict[tuple[int, int, int], int]:
    output = {
        (tray_id, row, col): 0
        for tray_id in range(1, TRAYS + 1)
        for row in range(1, ROWS + 1)
        for col in range(1, COLS + 1)
    }
    for cell in data.get('cells', []):
        tray_id = int(cell.get('tray_id', 0))
        row = int(cell.get('row', 0))
        col = int(cell.get('col', 0))
        if 1 <= tray_id <= TRAYS and 1 <= row <= ROWS and 1 <= col <= COLS:
            output[(tray_id, row, col)] = normalize_class_id(cell.get('class_id', 0))
    return output


def predict_current_matrix(
    color_path: Path,
    geometry_config: TrayGeometryConfig,
    edge_config: TrayEdgeFitConfig,
    pingpong_config: PingpongDetectorConfig,
) -> tuple[dict[tuple[int, int, int], int], str]:
    image = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    if image is None:
        return empty_prediction(), 'read_failed'

    geometry_result = detect_tray_geometry(image, geometry_config)
    output = empty_prediction()
    detected = 0
    failed = 0
    for candidate in geometry_result.candidates:
        if candidate.corners is None:
            failed += 1
            continue
        rectified = rectify(image, candidate.corners, edge_config)
        detection_result = detect_pingpong_cells(rectified, pingpong_config)
        if detection_result.status != 'ok':
            failed += 1
            continue
        detected += 1
        for cell in detection_result.cells:
            if 1 <= candidate.tray_id <= TRAYS and 1 <= cell.row <= ROWS and 1 <= cell.col <= COLS:
                output[(candidate.tray_id, cell.row, cell.col)] = CLASS_IDS.get(cell.class_name, 0)

    return output, f'{geometry_result.status}; trays={detected}; failed={failed}; {geometry_result.message}'


def empty_prediction() -> dict[tuple[int, int, int], int]:
    return {
        (tray_id, row, col): 0
        for tray_id in range(1, TRAYS + 1)
        for row in range(1, ROWS + 1)
        for col in range(1, COLS + 1)
    }


def normalize_class_id(value: Any) -> int:
    try:
        class_id = int(value)
    except (TypeError, ValueError):
        return 0
    return class_id if class_id in CLASS_NAMES else 0


def evaluate_matrix(
    sample_name: str,
    truth: dict[tuple[int, int, int], int],
    predicted: dict[tuple[int, int, int], int],
) -> tuple[dict[str, Any], Counter[tuple[int, int]]]:
    confusion: Counter[tuple[int, int]] = Counter()
    mistakes = []
    truth_counts: Counter[int] = Counter()
    pred_counts: Counter[int] = Counter()

    for key in sorted(truth):
        true_class = truth[key]
        pred_class = predicted.get(key, 0)
        truth_counts[true_class] += 1
        pred_counts[pred_class] += 1
        confusion[(true_class, pred_class)] += 1
        if true_class != pred_class:
            mistakes.append((key, true_class, pred_class))

    correct = sum(count for (true_class, pred_class), count in confusion.items() if true_class == pred_class)
    total = len(truth)
    summary = {
        'sample': sample_name,
        'total': total,
        'correct': correct,
        'errors': total - correct,
        'accuracy': correct / total if total else 0.0,
        'truth_counts': truth_counts,
        'pred_counts': pred_counts,
        'mistakes': mistakes,
    }
    return summary, confusion


def print_report(
    input_dir: Path,
    annotation_path: Path,
    summaries: list[dict[str, Any]],
    total_confusion: Counter[tuple[int, int]],
    missing_annotations: list[str],
    missing_predictions: list[str],
    top_n: int,
    mode: str,
) -> None:
    total = sum(summary['total'] for summary in summaries)
    correct = sum(summary['correct'] for summary in summaries)
    errors = total - correct

    print('')
    print('========== 乒乓球矩阵离线评估 ==========')
    print(f'样本目录: {input_dir}')
    print(f'标注文件: {annotation_path}')
    print(f'评估模式: {mode}')
    print(f'参与评估样本: {len(summaries)}')
    if missing_annotations:
        print(f'缺少人工标注: {len(missing_annotations)}  {", ".join(missing_annotations[:6])}')
    if missing_predictions:
        print(f'缺少预测矩阵: {len(missing_predictions)}  {", ".join(missing_predictions[:6])}')
    print('')
    print('总体结果:')
    print(f'  总格数: {total}')
    print(f'  正确:   {correct}')
    print(f'  错误:   {errors}')
    print(f'  准确率: {format_percent(correct, total)}')
    print('')

    print_class_metrics(total_confusion)
    print_confusion(total_confusion)
    print_worst_samples(summaries, top_n)
    print('')


def print_class_metrics(confusion: Counter[tuple[int, int]]) -> None:
    print('按类别统计:')
    for class_id in (1, 2):
        tp = confusion[(class_id, class_id)]
        truth_total = sum(confusion[(class_id, pred)] for pred in CLASS_NAMES)
        pred_total = sum(confusion[(truth, class_id)] for truth in CLASS_NAMES)
        missed = truth_total - tp
        false_positive = pred_total - tp
        print(
            f'  {CLASS_NAMES[class_id]}: '
            f'真值={truth_total}  识别={pred_total}  '
            f'命中={tp}  漏检={missed}  误检={false_positive}  '
            f'召回={format_percent(tp, truth_total)}  精确={format_percent(tp, pred_total)}'
        )
    print('')


def print_confusion(confusion: Counter[tuple[int, int]]) -> None:
    print('混淆矩阵: 行=人工真值, 列=算法输出')
    header = 'truth\\pred'.ljust(12) + ''.join(CLASS_NAMES[class_id].rjust(10) for class_id in (0, 1, 2))
    print('  ' + header)
    for true_class in (0, 1, 2):
        row = CLASS_NAMES[true_class].ljust(12)
        row += ''.join(str(confusion[(true_class, pred_class)]).rjust(10) for pred_class in (0, 1, 2))
        print('  ' + row)
    print('')


def print_worst_samples(summaries: list[dict[str, Any]], top_n: int) -> None:
    print(f'错误最多的前 {top_n} 张:')
    worst = sorted(summaries, key=lambda item: (-item['errors'], item['sample']))[:top_n]
    for summary in worst:
        print(
            f"  {summary['sample']}: "
            f"错误={summary['errors']:3d}/{summary['total']}  "
            f"准确率={summary['accuracy'] * 100:5.1f}%  "
            f"真值 W/Y={summary['truth_counts'][1]}/{summary['truth_counts'][2]}  "
            f"识别 W/Y={summary['pred_counts'][1]}/{summary['pred_counts'][2]}"
        )
        for key, true_class, pred_class in summary['mistakes'][:8]:
            tray_id, row, col = key
            print(
                f'    tray={tray_id} row={row} col={col}: '
                f'{CLASS_NAMES[true_class]} -> {CLASS_NAMES[pred_class]}'
            )
        if len(summary['mistakes']) > 8:
            print(f"    ... 还有 {len(summary['mistakes']) - 8} 个错误")


def format_percent(numerator: int, denominator: int) -> str:
    if denominator <= 0:
        return 'n/a'
    return f'{numerator / denominator * 100:.1f}%'


if __name__ == '__main__':
    main()
