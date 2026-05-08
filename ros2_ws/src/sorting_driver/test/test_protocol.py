from sorting_driver.protocol import calculate_checksum


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
