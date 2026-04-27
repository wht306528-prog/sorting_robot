"""Text protocol helpers for sending tray matrix data to the F407."""

from __future__ import annotations

from sorting_interfaces.msg import TrayMatrix


EXPECTED_CELL_COUNT = 150


def format_tray_matrix_text_frame(message: TrayMatrix) -> str:
    """Format a TrayMatrix as the early text protocol frame."""

    lines = [
        f'START frame_id={message.frame_id} count={len(message.cells)}',
    ]
    lines.extend(_format_cell(cell) for cell in message.cells)
    lines.append(f'END checksum={calculate_checksum(lines)}')
    return '\n'.join(lines)


def validate_tray_matrix(message: TrayMatrix) -> list[str]:
    """Return validation errors for a TrayMatrix message."""

    errors: list[str] = []
    if len(message.cells) != EXPECTED_CELL_COUNT:
        errors.append(
            f'expected {EXPECTED_CELL_COUNT} cells, got {len(message.cells)}'
        )

    seen_positions: set[tuple[int, int, int]] = set()
    for index, cell in enumerate(message.cells):
        prefix = f'cell[{index}]'
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

        position = (cell.tray_id, cell.row, cell.col)
        if position in seen_positions:
            errors.append(f'{prefix} duplicate position: {position}')
        seen_positions.add(position)

    return errors


def calculate_checksum(lines: list[str]) -> int:
    """Calculate a simple debug checksum for text protocol lines."""

    payload = '\n'.join(lines).encode('utf-8')
    return sum(payload) % 65536


def _format_cell(cell) -> str:
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
