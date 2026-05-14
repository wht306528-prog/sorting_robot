"""鲁班猫到 F407 的文本协议工具函数。

本文件只负责“数据格式转换”和“基础校验”，不直接创建 ROS 2 节点，
也不直接操作 TCP 或串口。

当前阶段的数据流：
1. sorting_vision 发布 TrayMatrix。
2. sorting_driver 订阅 TrayMatrix。
3. 本文件将 TrayMatrix 转成文本协议帧。
4. 调试阶段先打印文本帧；后续再替换为 TCP 发送给 F407。

文本协议格式示例：
    START frame_id=12 count=150
    1,1,1,1,0.950,140.000,40.000,601.000
    ...
    END checksum=8585 tray_total=3

注意：
- 这里的 checksum 是早期调试用简单校验，不是最终工业通信校验。
- checksum 只累加 150 行 CSV 数据行的原始 UTF-8 字节。
- checksum 不包含 START 行、END 行，也不包含行尾换行符。
- 后续如果学长的 F407 程序确定了 CRC16 或其他格式，需要同步修改本文档和代码。
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from sorting_interfaces.msg import TrayMatrix


# 三个苗盘，每个苗盘 5 列 x 10 行，共 150 个穴位。
EXPECTED_CELL_COUNT = 150


def format_tray_matrix_text_frame(message: TrayMatrix) -> str:
    """把 TrayMatrix 格式化为早期调试用文本协议帧。

    返回值是一整帧字符串，包含 START 行、150 行穴位数据和 END 行。
    后续 TCP 发送时，可以直接发送这个字符串的 UTF-8 编码。
    """

    start_line = f'START frame_id={message.frame_id} count={len(message.cells)}'

    # 每个 TrayCell 转成一行 CSV 风格文本，字段顺序必须和通信文档一致。
    data_lines = [_format_cell(cell) for cell in message.cells]

    # END 行带简单 checksum，方便 F407 或调试工具判断一帧是否完整。
    end_line = f'END checksum={calculate_checksum(data_lines)} tray_total={infer_tray_total(message)}'
    return '\n'.join([start_line, *data_lines, end_line])


def validate_tray_matrix(message: TrayMatrix) -> list[str]:
    """检查 TrayMatrix 是否符合当前项目约定。

    返回错误字符串列表：
    - 空列表表示校验通过。
    - 非空表示消息存在字段范围、数量或重复穴位问题。

    这里先做软件层面的基础保护，避免明显错误的数据继续发给 F407。
    """

    errors: list[str] = []
    if len(message.cells) != EXPECTED_CELL_COUNT:
        errors.append(
            f'expected {EXPECTED_CELL_COUNT} cells, got {len(message.cells)}'
        )
    seen_positions: set[tuple[int, int, int]] = set()
    for index, cell in enumerate(message.cells):
        prefix = f'cell[{index}]'

        # 这些范围来自 docs/glossary 术语对照表.md 和 150x8 矩阵约定。
        if cell.tray_id not in (1, 2, 3):
            errors.append(f'{prefix} tray_id out of range: {cell.tray_id}')
        if not 1 <= cell.col <= 5:
            errors.append(f'{prefix} col out of range: {cell.col}')
        if not 1 <= cell.row <= 10:
            errors.append(f'{prefix} row out of range: {cell.row}')
        if cell.class_id not in (0, 1, 2):
            errors.append(f'{prefix} class_id out of range: {cell.class_id}')
        if not 0.0 <= cell.confidence <= 1.0:
            errors.append(
                f'{prefix} confidence out of range: {cell.confidence}'
            )

        # 同一帧中，一个苗盘的同一个 row/col 只能出现一次。
        position = (cell.tray_id, cell.row, cell.col)
        if position in seen_positions:
            errors.append(f'{prefix} duplicate position: {position}')
        seen_positions.add(position)

    return errors


def calculate_checksum(lines: list[str]) -> int:
    """计算文本协议的简单调试校验值。

    算法：只累加数据行本身的 UTF-8 字节，再对 65536 取余。
    不包含 START 行、END 行，也不包含各行之间的换行符。
    这个算法方便早期验证，不适合作为最终强校验方案。
    """

    total = 0
    for line in lines:
        total += sum(line.encode('utf-8'))
    return total % 65536


def infer_tray_total(message: TrayMatrix) -> int:
    """从 150 格矩阵推导本帧实际带坐标的苗盘数量。

    视觉节点对未检测到的盘会补 empty，且 u/v/z 为 0。检测到的盘即使全是空穴，
    也会写入每个穴位的真实像素坐标。因此这里按 tray_id 是否出现非零 u/v/z
    来推导 tray_total，供 F407 在 END 行做整帧有效性判断。
    """

    detected_tray_ids = {
        int(cell.tray_id)
        for cell in message.cells
        if int(cell.tray_id) in (1, 2, 3)
        and (
            abs(float(cell.u)) > 1e-6
            or abs(float(cell.v)) > 1e-6
            or abs(float(cell.z)) > 1e-6
        )
    }
    return len(detected_tray_ids)


def _format_cell(cell) -> str:
    """把单个 TrayCell 转成一行协议文本。"""

    return (
        f'{cell.tray_id},'
        f'{cell.col},'
        f'{cell.row},'
        f'{cell.class_id},'
        f'{cell.confidence:.3f},'
        f'{cell.u:.3f},'
        f'{cell.v:.3f},'
        f'{cell.z:.3f}'
    )
