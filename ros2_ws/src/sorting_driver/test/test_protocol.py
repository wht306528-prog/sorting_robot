from sorting_driver.protocol import calculate_checksum, format_tray_matrix_text_frame, infer_tray_total
from sorting_driver.serial_node import format_matrix_summary


class FakeTrayMatrix:
    def __init__(self, frame_id, cells):
        self.frame_id = frame_id
        self.cells = cells


class FakeTrayCell:
    def __init__(self, tray_id, col, row, class_id, confidence, u, v, z):
        self.tray_id = tray_id
        self.col = col
        self.row = row
        self.class_id = class_id
        self.confidence = confidence
        self.u = u
        self.v = v
        self.z = z


def test_checksum_uses_only_data_line_bytes_without_newlines():
    data_lines = [
        '1,1,1,0,0.900,93.390,122.610,0.000',
        '1,2,1,2,0.800,196.695,122.610,555.000',
    ]

    expected = sum(
        sum(line.encode('utf-8'))
        for line in data_lines
    ) % 65536

    assert calculate_checksum(data_lines) == expected


def test_checksum_does_not_include_line_separators():
    data_lines = ['1,1,1,0,0.900,93.390,122.610,0.000', '2,1,1,1,0.700,10.000,20.000,30.000']
    with_newline = sum('\n'.join(data_lines).encode('utf-8')) % 65536

    assert calculate_checksum(data_lines) != with_newline


def test_text_frame_includes_tray_total_in_end_line():
    message = FakeTrayMatrix(
        frame_id=7,
        cells=[
            FakeTrayCell(1, 1, 1, 0, 1.0, 10.0, 15.0, 0.0),
            FakeTrayCell(2, 1, 1, 0, 0.9, 120.0, 20.0, 0.0),
        ],
    )

    frame = format_tray_matrix_text_frame(message)

    lines = frame.splitlines()
    assert lines[0] == 'START frame_id=7 count=2'
    assert lines[-1].endswith(' tray_total=2')


def test_matrix_summary_lists_non_empty_cells_first():
    message = FakeTrayMatrix(
        frame_id=7,
        cells=[
            FakeTrayCell(1, 1, 1, 0, 1.0, 0.0, 0.0, 0.0),
            FakeTrayCell(1, 2, 1, 1, 0.8, 11.0, 22.0, 333.0),
            FakeTrayCell(1, 3, 1, 2, 0.9, 44.0, 55.0, 666.0),
            FakeTrayCell(2, 1, 1, 0, 0.9, 77.0, 88.0, 0.0),
            FakeTrayCell(3, 1, 1, 0, 0.9, 99.0, 111.0, 0.0),
        ],
    )

    summary = format_matrix_summary(message, max_non_empty_rows=10)

    assert 'frame_id=7 tray_total=3 count=5' in summary
    assert '非空穴位: 2  白球: 1  黄球: 1  空穴: 3' in summary
    assert '1,2,1,1,0.800,11.000,22.000,333.000' in summary
    assert '1,3,1,2,0.900,44.000,55.000,666.000' in summary


def test_matrix_summary_warns_when_tray_total_is_not_three():
    message = FakeTrayMatrix(
        frame_id=8,
        cells=[
            FakeTrayCell(1, 1, 1, 0, 1.0, 0.0, 0.0, 0.0),
            FakeTrayCell(2, 1, 1, 0, 1.0, 120.0, 30.0, 0.0),
        ],
    )

    summary = format_matrix_summary(message, max_non_empty_rows=10)

    assert '警告: tray_total != 3' in summary


def test_infer_tray_total_counts_trays_with_real_coordinates():
    message = FakeTrayMatrix(
        frame_id=9,
        cells=[
            FakeTrayCell(1, 1, 1, 0, 1.0, 0.0, 0.0, 0.0),
            FakeTrayCell(1, 2, 1, 0, 0.9, 10.0, 20.0, 0.0),
            FakeTrayCell(2, 1, 1, 0, 0.9, 30.0, 40.0, 0.0),
            FakeTrayCell(3, 1, 1, 0, 1.0, 0.0, 0.0, 0.0),
        ],
    )

    assert infer_tray_total(message) == 2
