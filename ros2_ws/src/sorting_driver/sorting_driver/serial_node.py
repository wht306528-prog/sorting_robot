"""Subscribe to TrayMatrix and print the F407 text protocol frame."""

from __future__ import annotations

from rclpy.node import Node
import rclpy

from sorting_interfaces.msg import TrayMatrix

from sorting_driver.protocol import (
    format_tray_matrix_text_frame,
    validate_tray_matrix,
)


class MatrixProtocolPrinter(Node):
    """Convert TrayMatrix messages into text protocol frames for debugging."""

    def __init__(self) -> None:
        super().__init__('matrix_protocol_printer')
        self._subscription = self.create_subscription(
            TrayMatrix,
            '/sorting/tray_matrix',
            self._handle_matrix,
            10,
        )
        self.get_logger().info(
            'Listening on /sorting/tray_matrix and printing text frames'
        )

    def _handle_matrix(self, message: TrayMatrix) -> None:
        errors = validate_tray_matrix(message)
        if errors:
            self.get_logger().error(
                'Invalid TrayMatrix: ' + '; '.join(errors[:5])
            )
            if len(errors) > 5:
                self.get_logger().error(
                    f'{len(errors) - 5} additional validation errors omitted'
                )
            return

        frame = format_tray_matrix_text_frame(message)
        self.get_logger().info(
            f'Received frame_id={message.frame_id} cells={len(message.cells)}'
        )
        print(frame, flush=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MatrixProtocolPrinter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
