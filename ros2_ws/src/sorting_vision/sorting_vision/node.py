"""Mock tray matrix publisher for early ROS 2 interface testing."""

from __future__ import annotations

from rclpy.node import Node
import rclpy

from sorting_interfaces.msg import TrayCell, TrayMatrix


class MockMatrixPublisher(Node):
    """Publish deterministic 3-tray, 150-cell TrayMatrix messages."""

    def __init__(self) -> None:
        super().__init__('mock_matrix_publisher')
        self._publisher = self.create_publisher(
            TrayMatrix,
            '/sorting/tray_matrix',
            10,
        )
        self._frame_id = 0
        self._timer = self.create_timer(1.0, self._publish_matrix)
        self.get_logger().info(
            'Publishing mock TrayMatrix messages on /sorting/tray_matrix'
        )

    def _publish_matrix(self) -> None:
        message = TrayMatrix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'mock_camera'
        message.frame_id = self._frame_id
        message.cells = [
            self._make_cell(tray_id, row, col)
            for tray_id in range(1, 4)
            for row in range(1, 11)
            for col in range(1, 6)
        ]

        self._publisher.publish(message)
        self.get_logger().info(
            f'Published frame_id={message.frame_id} cells={len(message.cells)}'
        )
        self._frame_id += 1

    def _make_cell(self, tray_id: int, row: int, col: int) -> TrayCell:
        cell = TrayCell()
        cell.tray_id = tray_id
        cell.row = row
        cell.col = col
        cell.class_id = self._mock_class_id(tray_id, row, col)
        cell.confidence = 0.95

        # Temporary pixel/depth values. Real vision code will replace these
        # with values from tray detection, grid mapping, and the depth image.
        cell.u = float(100 * tray_id + 40 * col)
        cell.v = float(40 * row)
        cell.z = 600.0 + float(tray_id)
        return cell

    @staticmethod
    def _mock_class_id(tray_id: int, row: int, col: int) -> int:
        if tray_id == 3:
            return TrayCell.CLASS_EMPTY
        if row == 1 and col in (2, 4):
            return TrayCell.CLASS_WEAK
        if row == 2 and col == 3:
            return TrayCell.CLASS_EMPTY
        return TrayCell.CLASS_GOOD


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockMatrixPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
