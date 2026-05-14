#!/usr/bin/env python3
"""逐张标注三盘乒乓球 10x5 矩阵。

这个脚本只做人工标注采集，不参与实时识别。默认读取采集目录里的
sample_*/color.jpg 和 debug.jpg，把标注保存成 annotations.json，供后续
参数搜索/评估脚本使用。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


ROWS = 10
COLS = 5
TRAYS = 3
VALID_DIGITS = {'0', '1', '2'}


def main() -> None:
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    output_path = Path(args.output).expanduser()
    if not output_path.is_absolute():
        output_path = input_dir / output_path

    samples = find_samples(input_dir)
    if args.limit > 0:
        samples = samples[:args.limit]
    if not samples:
        raise RuntimeError(f'没有找到样本目录: {input_dir}/sample_*')

    annotations = load_annotations(output_path)
    print_header(input_dir, output_path, len(samples))

    for index, sample_dir in enumerate(samples, start=1):
        sample_name = sample_dir.name
        if sample_name in annotations and not args.revisit:
            print(f'[{index}/{len(samples)}] {sample_name}: 已标注，跳过。')
            continue

        color_path = sample_dir / 'color.jpg'
        debug_path = sample_dir / 'debug.jpg'
        print_sample_banner(index, len(samples), sample_name, color_path, debug_path)
        if args.open_images:
            open_image(debug_path if debug_path.exists() else color_path)

        existing = annotations.get(sample_name)
        if existing:
            print('已有标注：')
            print_annotation(existing)

        annotation = prompt_sample(sample_name)
        if annotation is None:
            print('退出前保存当前标注。')
            save_annotations(output_path, annotations)
            return
        if annotation == 'skip':
            print(f'{sample_name}: 跳过。')
            continue

        annotations[sample_name] = annotation
        save_annotations(output_path, annotations)
        print(f'{sample_name}: 已保存。')

    save_annotations(output_path, annotations)
    print(f'完成。标注文件: {output_path}')


def find_samples(input_dir: Path) -> list[Path]:
    """查找 sample_* 目录。"""

    return sorted(path for path in input_dir.glob('sample_*') if path.is_dir())


def load_annotations(path: Path) -> dict[str, Any]:
    """读取已有标注，支持断点继续。"""

    if not path.exists():
        return {}
    with path.open('r', encoding='utf-8') as file:
        data = json.load(file)
    if not isinstance(data, dict):
        raise RuntimeError(f'标注文件格式错误: {path}')
    return data


def save_annotations(path: Path, annotations: dict[str, Any]) -> None:
    """保存标注 JSON。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as file:
        json.dump(annotations, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.write('\n')


def print_header(input_dir: Path, output_path: Path, count: int) -> None:
    print('')
    print('========== 乒乓球矩阵标注工具 ==========')
    print(f'样本目录: {input_dir}')
    print(f'样本数量: {count}')
    print(f'输出文件: {output_path}')
    print('')
    print('类别约定:')
    print('  0 = empty 空穴')
    print('  1 = white 白球')
    print('  2 = yellow 黄球')
    print('')
    print('输入方式:')
    print('  每个样本依次输入 tray 1、tray 2、tray 3。')
    print('  每个 tray 输入 10 行，每行 5 个数字，例如: 00120')
    print('  也可以带空格，例如: 0 0 1 2 0')
    print('  单独输入 s 跳过当前样本，q 保存并退出。')
    print('========================================')
    print('')


def print_sample_banner(
    index: int,
    total: int,
    sample_name: str,
    color_path: Path,
    debug_path: Path,
) -> None:
    print('')
    print(f'========== [{index}/{total}] {sample_name} ==========')
    print(f'原图:  {color_path}')
    print(f'debug: {debug_path}')
    print('建议对照图片，从左到右标 tray 1/2/3。')


def open_image(path: Path) -> None:
    """尽量打开图片；失败不影响终端标注。"""

    if not path.exists():
        print(f'图片不存在，无法打开: {path}')
        return
    try:
        subprocess.Popen(
            ['xdg-open', str(path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        print(f'自动打开图片失败: {exc}')


def prompt_sample(sample_name: str) -> dict[str, Any] | str | None:
    """采集一个样本的三盘矩阵。"""

    trays: dict[str, list[list[int]]] = {}
    for tray_id in range(1, TRAYS + 1):
        print(f'--- {sample_name} tray {tray_id} ---')
        matrix = prompt_tray()
        if matrix is None:
            return None
        if matrix == 'skip':
            return 'skip'
        trays[str(tray_id)] = matrix

    counts = count_classes(trays)
    print(f'本样本统计: empty={counts[0]} white={counts[1]} yellow={counts[2]}')
    while True:
        answer = input('确认保存这个样本？[y=保存 / r=重输 / s=跳过 / q=退出] ').strip().lower()
        if answer in ('', 'y', 'yes'):
            return {
                'schema': 'pingpong_tray_matrices_v1',
                'rows': ROWS,
                'cols': COLS,
                'class_ids': {
                    'empty': 0,
                    'white_ball': 1,
                    'yellow_ball': 2,
                },
                'matrices_by_tray': trays,
                'counts': {
                    'empty': counts[0],
                    'white_ball': counts[1],
                    'yellow_ball': counts[2],
                },
            }
        if answer in ('r', 'retry'):
            return prompt_sample(sample_name)
        if answer in ('s', 'skip'):
            return 'skip'
        if answer in ('q', 'quit'):
            return None
        print('请输入 y/r/s/q。')


def prompt_tray() -> list[list[int]] | str | None:
    """采集单盘 10x5 矩阵。"""

    rows: list[list[int]] = []
    row_index = 1
    while row_index <= ROWS:
        text = input(f'row {row_index:02d}/{ROWS}: ').strip().lower()
        if text in ('q', 'quit'):
            return None
        if text in ('s', 'skip'):
            return 'skip'
        if text in ('b', 'back'):
            if rows:
                rows.pop()
                row_index -= 1
            else:
                print('已经是第一行，不能后退。')
            continue

        parsed = parse_row(text)
        if parsed is None:
            print('格式错误：请输入 5 个 0/1/2，例如 00120 或 0 0 1 2 0。')
            continue
        rows.append(parsed)
        row_index += 1
    return rows


def parse_row(text: str) -> list[int] | None:
    """解析一行 5 个类别数字。"""

    compact = ''.join(text.split())
    if len(compact) != COLS:
        return None
    if any(char not in VALID_DIGITS for char in compact):
        return None
    return [int(char) for char in compact]


def count_classes(trays: dict[str, list[list[int]]]) -> dict[int, int]:
    """统计 0/1/2 数量。"""

    counts = {0: 0, 1: 0, 2: 0}
    for matrix in trays.values():
        for row in matrix:
            for value in row:
                counts[value] += 1
    return counts


def print_annotation(annotation: dict[str, Any]) -> None:
    """打印已有标注，方便复查。"""

    matrices = annotation.get('matrices_by_tray', {})
    for tray_id in sorted(matrices):
        print(f'tray {tray_id}:')
        for row in matrices[tray_id]:
            print('  ' + ''.join(str(value) for value in row))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='人工标注三盘乒乓球 10x5 矩阵。')
    parser.add_argument(
        '--input-dir',
        default='samples/pingpong_three_trays_capture/pingpong_three_trays_capture/pingpong_realtime_capture',
        help='包含 sample_*/color.jpg 的样本目录。',
    )
    parser.add_argument(
        '--output',
        default='annotations.json',
        help='输出 JSON。相对路径会写到 input-dir 下。',
    )
    parser.add_argument('--limit', type=int, default=0, help='只标前 N 个样本，0 表示全部。')
    parser.add_argument('--revisit', action='store_true', help='重新标注已有样本。')
    parser.add_argument(
        '--open-images',
        action='store_true',
        help='每个样本自动用系统图片查看器打开 debug.jpg。',
    )
    return parser.parse_args()


if __name__ == '__main__':
    main()
