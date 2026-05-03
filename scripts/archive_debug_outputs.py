#!/usr/bin/env python3
"""Build a review index for generated vision debug outputs."""

from __future__ import annotations

import argparse
import csv
import re
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png'}
CSV_SUFFIXES = {'.csv'}
PRIORITY_KINDS = [
    'grid_overlay',
    'overlay_grid',
    'hole_overlay',
    'candidates',
    'grid_rectified',
    'hole_grid',
    'warped_grid',
    'warped',
    'hough',
    'projection',
]


@dataclass(frozen=True)
class DebugFile:
    run: str
    path: Path
    rel_path: Path
    sample: str
    tray: str
    kind: str
    suffix: str


def main() -> None:
    args = parse_args()
    debug_dir = Path(args.debug_dir)
    review_dir = Path(args.review_dir)
    review_dir.mkdir(parents=True, exist_ok=True)
    (review_dir / 'contact_sheets').mkdir(exist_ok=True)

    files = collect_debug_files(debug_dir, review_dir)
    write_inventory(review_dir / 'inventory.csv', files)

    sheet_rows = []
    for run in sorted({item.run for item in files}):
        run_files = [item for item in files if item.run == run]
        for kind in PRIORITY_KINDS:
            images = [
                item.path for item in run_files
                if item.kind == kind and item.suffix in IMAGE_SUFFIXES
            ]
            if not images:
                continue
            sheet_path = review_dir / 'contact_sheets' / f'{run}__{kind}.jpg'
            make_contact_sheet(images, sheet_path, title=f'{run} / {kind}')
            sheet_rows.append((run, kind, len(images), sheet_path))

    copy_key_images(files, review_dir / 'key_images')
    write_readme(review_dir / 'README.md', debug_dir, files, sheet_rows)
    print(f'已整理 debug 归档索引：{review_dir}')
    print(f'文件清单：{review_dir / "inventory.csv"}')
    print(f'查看入口：{review_dir / "README.md"}')


def collect_debug_files(debug_dir: Path, review_dir: Path) -> list[DebugFile]:
    files: list[DebugFile] = []
    for path in sorted(debug_dir.rglob('*')):
        if not path.is_file():
            continue
        if review_dir in path.parents:
            continue
        if path.suffix.lower() not in IMAGE_SUFFIXES | CSV_SUFFIXES:
            continue
        rel_path = path.relative_to(debug_dir)
        run = rel_path.parts[0] if len(rel_path.parts) >= 2 else '_root'
        sample = parse_sample(path.name)
        tray = parse_tray(path.name)
        kind = parse_kind(path.name)
        files.append(
            DebugFile(
                run=run,
                path=path,
                rel_path=rel_path,
                sample=sample,
                tray=tray,
                kind=kind,
                suffix=path.suffix.lower(),
            )
        )
    return files


def parse_sample(name: str) -> str:
    match = re.search(r'(sample_\d+)', name)
    return match.group(1) if match else ''


def parse_tray(name: str) -> str:
    match = re.search(r'tray_(\d+)', name)
    return match.group(1) if match else ''


def parse_kind(name: str) -> str:
    stem = Path(name).stem
    stem = re.sub(r'^sample_\d+_color_', '', stem)
    stem = re.sub(r'^sample_\d+_', '', stem)
    stem = re.sub(r'tray_\d+_', '', stem)
    return stem


def write_inventory(path: Path, files: list[DebugFile]) -> None:
    with path.open('w', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(
            file,
            fieldnames=['run', 'sample', 'tray', 'kind', 'suffix', 'path'],
        )
        writer.writeheader()
        for item in files:
            writer.writerow(
                {
                    'run': item.run,
                    'sample': item.sample,
                    'tray': item.tray,
                    'kind': item.kind,
                    'suffix': item.suffix,
                    'path': str(item.rel_path),
                }
            )


def make_contact_sheet(image_paths: list[Path], output_path: Path, title: str) -> None:
    thumbs = []
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        thumbs.append((path, image))
    if not thumbs:
        return

    thumb_w = 260
    thumb_h = 190
    label_h = 38
    cols = min(4, max(1, len(thumbs)))
    rows = int(np.ceil(len(thumbs) / cols))
    header_h = 44
    canvas = np.full(
        (header_h + rows * (thumb_h + label_h), cols * thumb_w, 3),
        245,
        dtype=np.uint8,
    )
    cv2.putText(
        canvas,
        title,
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (20, 20, 20),
        2,
        cv2.LINE_AA,
    )

    for index, (path, image) in enumerate(thumbs):
        row = index // cols
        col = index % cols
        x0 = col * thumb_w
        y0 = header_h + row * (thumb_h + label_h)
        thumb = resize_to_fit(image, thumb_w, thumb_h)
        y_pad = y0 + (thumb_h - thumb.shape[0]) // 2
        x_pad = x0 + (thumb_w - thumb.shape[1]) // 2
        canvas[y_pad:y_pad + thumb.shape[0], x_pad:x_pad + thumb.shape[1]] = thumb
        label = path.name
        cv2.putText(
            canvas,
            label[:38],
            (x0 + 8, y0 + thumb_h + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (40, 40, 40),
            1,
            cv2.LINE_AA,
        )

    cv2.imwrite(str(output_path), canvas)


def resize_to_fit(image: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(max_w / width, max_h / height)
    new_size = (max(1, int(width * scale)), max(1, int(height * scale)))
    return cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)


def copy_key_images(files: list[DebugFile], output_dir: Path) -> None:
    output_dir.mkdir(exist_ok=True)
    wanted = {'grid_overlay', 'overlay_grid', 'hole_overlay', 'candidates'}
    for item in files:
        if item.suffix not in IMAGE_SUFFIXES or item.kind not in wanted:
            continue
        target = output_dir / f'{item.run}__{item.path.name}'
        shutil.copy2(item.path, target)


def write_readme(
    path: Path,
    debug_dir: Path,
    files: list[DebugFile],
    sheet_rows: list[tuple[str, str, int, Path]],
) -> None:
    runs = sorted({item.run for item in files})
    lines = [
        '# Debug Review Index',
        '',
        '这个目录是 `samples/debug` 的集中查看入口。原始 debug 输出没有移动，',
        '这里只生成清单、关键图副本和 contact sheet，方便判断哪一轮算法值得看。',
        '',
        '## 先看哪里',
        '',
        '1. `key_images/`：每轮最关键的原图 overlay、候选框图。',
        '2. `contact_sheets/`：按实验目录和图片类型拼好的总览图。',
        '3. `inventory.csv`：所有 debug 文件的索引清单。',
        '',
        '## 实验目录',
        '',
        '| run | files | images | csv | recommended focus |',
        '| --- | ---: | ---: | ---: | --- |',
    ]
    for run in runs:
        run_files = [item for item in files if item.run == run]
        image_count = sum(1 for item in run_files if item.suffix in IMAGE_SUFFIXES)
        csv_count = sum(1 for item in run_files if item.suffix in CSV_SUFFIXES)
        lines.append(
            f'| `{run}` | {len(run_files)} | {image_count} | {csv_count} | '
            f'{recommend_focus(run)} |'
        )

    lines.extend(['', '## Contact Sheets', ''])
    for run, kind, count, sheet_path in sheet_rows:
        rel = sheet_path.relative_to(path.parent)
        lines.append(f'- `{run}` / `{kind}` ({count}): [{rel}]({rel})')

    lines.extend(
        [
            '',
            '## 重新生成',
            '',
            '```bash',
            'cd /home/wht/sorting_robot',
            'python3 scripts/archive_debug_outputs.py',
            '```',
            '',
            f'原始目录：`{debug_dir}`',
            '',
        ]
    )
    path.write_text('\n'.join(lines), encoding='utf-8')


def recommend_focus(run: str) -> str:
    if run == 'tray_grid':
        return '当前主线，优先看 grid_overlay 和 grid_rectified'
    if 'hole' in run:
        return '内部穴位实验，可作对比'
    if run == 'rgbd_batch':
        return '旧的外框均分基线'
    if run == 'manual_grid':
        return '手工标定备用'
    return '历史实验/局部测试'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='整理 samples/debug 的查看索引。')
    parser.add_argument('--debug-dir', default='samples/debug')
    parser.add_argument('--review-dir', default='samples/debug/_review')
    return parser.parse_args()


if __name__ == '__main__':
    main()
