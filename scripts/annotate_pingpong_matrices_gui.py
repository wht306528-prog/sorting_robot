#!/usr/bin/env python3
"""PyQt5 三盘乒乓球矩阵标注界面。

左侧显示采集图片，右侧用 3 个 10x5 表格标注 tray 1/2/3。
标注结果保存为 annotations.json，供后续参数搜索和识别评估使用。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

from PyQt5.QtCore import QSize, Qt
from PyQt5.QtGui import QIntValidator, QKeySequence, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QShortcut,
    QSizePolicy,
    QSplitter,
    QVBoxLayout,
    QWidget,
)


ROWS = 10
COLS = 5
TRAYS = 3


class ImagePanel(QLabel):
    """有稳定尺寸边界的图片预览控件，避免 QLabel/pixmap 触发布局膨胀。"""

    def __init__(self) -> None:
        super().__init__()
        self._pixmap = QPixmap()
        self.setObjectName('imagePanel')
        self.setAlignment(Qt.AlignCenter)
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumSize(760, 570)
        self.setMaximumSize(980, 735)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setText('等待加载图片')

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(860, 645)

    def minimumSizeHint(self) -> QSize:  # noqa: N802
        return QSize(760, 570)

    def set_image(self, image_path: Path) -> None:
        pixmap = QPixmap(str(image_path))
        if pixmap.isNull():
            self._pixmap = QPixmap()
            self.clear()
            self.setText(f'无法读取图片:\n{image_path}')
            return
        self._pixmap = pixmap
        self._update_scaled_pixmap()

    def resizeEvent(self, event) -> None:  # noqa: ANN001, N802
        super().resizeEvent(event)
        self._update_scaled_pixmap()

    def _update_scaled_pixmap(self) -> None:
        if self._pixmap.isNull():
            return
        target = self.contentsRect().size()
        if target.width() <= 2 or target.height() <= 2:
            return
        scaled = self._pixmap.scaled(target, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(scaled)


class MatrixGrid(QGroupBox):
    """一个 tray 的 10x5 标注表格。"""

    def __init__(self, tray_id: int, changed_callback=None) -> None:  # noqa: ANN001
        super().__init__(f'Tray {tray_id}')
        self.tray_id = tray_id
        self.cells: list[list[QLineEdit]] = []
        self.changed_callback = changed_callback
        layout = QGridLayout()
        layout.setContentsMargins(10, 18, 10, 10)
        layout.setHorizontalSpacing(5)
        layout.setVerticalSpacing(5)

        for col in range(COLS):
            label = QLabel(f'C{col + 1}')
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label, 0, col + 1)
        for row in range(ROWS):
            label = QLabel(f'R{row + 1}')
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label, row + 1, 0)

            row_cells: list[QLineEdit] = []
            for col in range(COLS):
                edit = QLineEdit('0')
                edit.setAlignment(Qt.AlignCenter)
                edit.setMaxLength(1)
                edit.setValidator(QIntValidator(0, 2, edit))
                edit.setFixedSize(36, 26)
                edit.setToolTip('0=空穴, 1=白球, 2=黄球')
                edit.textEdited.connect(lambda _text, widget=edit: self._normalize_cell(widget))
                edit.textChanged.connect(self._notify_changed)
                layout.addWidget(edit, row + 1, col + 1)
                row_cells.append(edit)
            self.cells.append(row_cells)

        self.setLayout(layout)

    def matrix(self) -> list[list[int]]:
        """读取表格矩阵。"""

        output: list[list[int]] = []
        for row in self.cells:
            values: list[int] = []
            for edit in row:
                text = edit.text().strip()
                values.append(int(text) if text in ('0', '1', '2') else 0)
            output.append(values)
        return output

    def set_matrix(self, matrix: list[list[int]]) -> None:
        """写入已有矩阵。"""

        for row_index, row in enumerate(self.cells):
            for col_index, edit in enumerate(row):
                value = 0
                if row_index < len(matrix) and col_index < len(matrix[row_index]):
                    value = int(matrix[row_index][col_index])
                edit.setText(str(value if value in (0, 1, 2) else 0))

    def clear(self) -> None:
        """清空为全 0。"""

        for row in self.cells:
            for edit in row:
                edit.setText('0')

    @staticmethod
    def _normalize_cell(edit: QLineEdit) -> None:
        text = edit.text().strip()
        if text not in ('0', '1', '2'):
            edit.setText('')

    def _notify_changed(self) -> None:
        if self.changed_callback is not None:
            self.changed_callback()


class AnnotatorWindow(QMainWindow):
    """主标注窗口。"""

    def __init__(self, input_dir: Path, output_path: Path) -> None:
        super().__init__()
        self.input_dir = input_dir
        self.output_path = output_path
        self.samples = find_samples(input_dir)
        self.annotations = load_annotations(output_path)
        self.index = 0
        self.show_debug = True

        if not self.samples:
            raise RuntimeError(f'没有找到样本目录: {input_dir}/sample_*')

        self.setWindowTitle('Pingpong Matrix Annotator')
        self.resize(1360, 840)
        self.setMinimumSize(1180, 760)

        self.image_label = ImagePanel()

        self.info_label = QLabel()
        self.info_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.grid_widgets = [
            MatrixGrid(tray_id, changed_callback=self.update_counts)
            for tray_id in range(1, TRAYS + 1)
        ]
        self.count_label = QLabel()
        self.count_label.setObjectName('countLabel')

        self.prev_button = QPushButton('上一张')
        self.next_button = QPushButton('下一张')
        self.save_next_button = QPushButton('保存并下一张')
        self.apply_all_button = QPushButton('应用到全部样本')
        self.toggle_image_button = QPushButton('切换 color/debug')
        self.clear_button = QPushButton('清空当前')
        self.skip_button = QPushButton('跳过')

        self.prev_button.clicked.connect(self.prev_sample)
        self.next_button.clicked.connect(self.next_sample_without_save)
        self.save_next_button.clicked.connect(self.save_and_next)
        self.apply_all_button.clicked.connect(self.apply_current_to_all)
        self.toggle_image_button.clicked.connect(self.toggle_image)
        self.clear_button.clicked.connect(self.clear_current)
        self.skip_button.clicked.connect(self.next_sample_without_save)

        self._build_layout()
        self._setup_shortcuts()
        self._apply_style()
        self.load_sample(0)

    def _build_layout(self) -> None:
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(12, 12, 10, 12)
        left_layout.setSpacing(8)
        left_layout.addWidget(self.info_label)
        left_layout.addWidget(self.image_label, stretch=1)

        right_content = QWidget()
        right_content.setMinimumWidth(500)
        right_content.setMaximumWidth(540)
        right_layout = QVBoxLayout(right_content)
        right_layout.setContentsMargins(12, 12, 12, 12)
        right_layout.setSpacing(8)
        help_label = QLabel(
            '类别: 0=空穴  1=白球  2=黄球\n'
            '快捷键: Ctrl+S 保存并下一张, ←/→ 切换样本, Ctrl+D 切换图片'
        )
        right_layout.addWidget(help_label)
        for grid in self.grid_widgets:
            right_layout.addWidget(grid)
        right_layout.addWidget(self.count_label)

        nav_buttons = QHBoxLayout()
        nav_buttons.addWidget(self.prev_button)
        nav_buttons.addWidget(self.next_button)
        nav_buttons.addWidget(self.save_next_button)
        right_layout.addLayout(nav_buttons)

        tool_buttons = QHBoxLayout()
        tool_buttons.addWidget(self.apply_all_button)
        tool_buttons.addWidget(self.toggle_image_button)
        right_layout.addLayout(tool_buttons)

        edit_buttons = QHBoxLayout()
        edit_buttons.addWidget(self.clear_button)
        edit_buttons.addWidget(self.skip_button)
        right_layout.addLayout(edit_buttons)
        right_layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(right_content)
        scroll.setMinimumWidth(520)
        scroll.setMaximumWidth(560)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(scroll)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([830, 530])
        self.setCentralWidget(splitter)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow {
                background: #f4f6f8;
            }
            QLabel {
                color: #1f2933;
                font-size: 13px;
            }
            QLabel#imagePanel {
                background: #111827;
                border: 1px solid #273449;
                border-radius: 6px;
                color: #dbe4f0;
            }
            QLabel#countLabel {
                background: #eef4ff;
                border: 1px solid #c7d7fe;
                border-radius: 5px;
                padding: 6px 8px;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #d5dbe3;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QLineEdit {
                background: #ffffff;
                border: 1px solid #b9c2cf;
                border-radius: 3px;
                font-size: 13px;
            }
            QLineEdit:focus {
                border: 2px solid #2563eb;
            }
            QPushButton {
                background: #ffffff;
                border: 1px solid #b9c2cf;
                border-radius: 5px;
                padding: 6px 8px;
                min-height: 26px;
            }
            QPushButton:hover {
                background: #eef4ff;
                border-color: #7aa2f7;
            }
            """
        )

    def _setup_shortcuts(self) -> None:
        QShortcut(QKeySequence('Ctrl+S'), self, activated=self.save_and_next)
        QShortcut(QKeySequence(Qt.Key_Right), self, activated=self.next_sample_without_save)
        QShortcut(QKeySequence(Qt.Key_Left), self, activated=self.prev_sample)
        QShortcut(QKeySequence('Ctrl+D'), self, activated=self.toggle_image)

    def load_sample(self, index: int) -> None:
        """加载第 index 个样本。"""

        self.index = max(0, min(index, len(self.samples) - 1))
        sample_dir = self.samples[self.index]
        sample_name = sample_dir.name
        annotation = self.annotations.get(sample_name)
        if annotation:
            matrices = annotation.get('matrices_by_tray', {})
            for tray_index, grid in enumerate(self.grid_widgets, start=1):
                grid.set_matrix(matrices.get(str(tray_index), zero_matrix()))
        else:
            for grid in self.grid_widgets:
                grid.clear()
        self.refresh_image()
        self.update_info()
        self.update_counts()

    def refresh_image(self) -> None:
        """刷新左侧图片。"""

        sample_dir = self.samples[self.index]
        image_path = sample_dir / ('debug.jpg' if self.show_debug else 'color.jpg')
        if not image_path.exists():
            image_path = sample_dir / 'color.jpg'
        self.image_label.set_image(image_path)

    def update_info(self) -> None:
        sample_dir = self.samples[self.index]
        sample_name = sample_dir.name
        marked = '已标注' if sample_name in self.annotations else '未标注'
        image_kind = 'debug.jpg' if self.show_debug else 'color.jpg'
        self.info_label.setText(
            f'[{self.index + 1}/{len(self.samples)}] {sample_name}  {marked}  当前显示: {image_kind}\n'
            f'目录: {sample_dir}'
        )

    def update_counts(self) -> None:
        counts = {0: 0, 1: 0, 2: 0}
        for grid in self.grid_widgets:
            for row in grid.matrix():
                for value in row:
                    counts[value] += 1
        self.count_label.setText(
            f'当前统计: 空穴={counts[0]}  白球={counts[1]}  黄球={counts[2]}'
        )

    def current_annotation(self) -> dict[str, Any]:
        trays = {
            str(index): grid.matrix()
            for index, grid in enumerate(self.grid_widgets, start=1)
        }
        counts = {0: 0, 1: 0, 2: 0}
        for matrix in trays.values():
            for row in matrix:
                for value in row:
                    counts[value] += 1
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

    def save_current(self) -> None:
        sample_name = self.samples[self.index].name
        self.annotations[sample_name] = self.current_annotation()
        save_annotations(self.output_path, self.annotations)
        self.update_info()

    def save_and_next(self) -> None:
        self.save_current()
        if self.index + 1 < len(self.samples):
            self.load_sample(self.index + 1)
        else:
            QMessageBox.information(self, '完成', f'已经标到最后一张。\n保存文件: {self.output_path}')

    def apply_current_to_all(self) -> None:
        annotation = self.current_annotation()
        for sample_dir in self.samples:
            self.annotations[sample_dir.name] = annotation
        save_annotations(self.output_path, self.annotations)
        self.update_info()
        QMessageBox.information(
            self,
            '已应用到全部样本',
            f'已把当前 3 盘矩阵写入 {len(self.samples)} 个样本。\n保存文件: {self.output_path}',
        )

    def next_sample_without_save(self) -> None:
        if self.index + 1 < len(self.samples):
            self.load_sample(self.index + 1)

    def prev_sample(self) -> None:
        if self.index > 0:
            self.load_sample(self.index - 1)

    def toggle_image(self) -> None:
        self.show_debug = not self.show_debug
        self.refresh_image()
        self.update_info()

    def clear_current(self) -> None:
        for grid in self.grid_widgets:
            grid.clear()
        self.update_counts()


def zero_matrix() -> list[list[int]]:
    return [[0 for _col in range(COLS)] for _row in range(ROWS)]


def find_samples(input_dir: Path) -> list[Path]:
    return sorted(path for path in input_dir.glob('sample_*') if path.is_dir())


def load_annotations(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open('r', encoding='utf-8') as file:
        data = json.load(file)
    if not isinstance(data, dict):
        raise RuntimeError(f'标注文件格式错误: {path}')
    return data


def save_annotations(path: Path, annotations: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as file:
        json.dump(annotations, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.write('\n')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='PyQt5 三盘乒乓球矩阵标注界面。')
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    output_path = Path(args.output).expanduser()
    if not output_path.is_absolute():
        output_path = input_dir / output_path

    app = QApplication(sys.argv)
    window = AnnotatorWindow(input_dir=input_dir, output_path=output_path)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
