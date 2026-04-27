"""矩阵协议打印节点。

本节点用于早期联调阶段，暂时不连接真实 F407。

节点功能：
1. 订阅 /sorting/tray_matrix 话题。
2. 接收 sorting_vision 发布的 TrayMatrix。
3. 校验矩阵是否满足 150 个穴位、行列范围、类别范围等规则。
4. 把矩阵转成鲁班猫到 F407 的文本协议帧。
5. 将完整文本帧打印到终端。

用途：
- 检查视觉侧输出是否能被驱动侧正常接收。
- 检查协议帧字段顺序和格式是否符合文档。
- 后续开发 TCP 发送节点时，可以复用 protocol.py 中的格式化函数。

运行方式：
    ros2 run sorting_driver matrix_protocol_printer
"""

from __future__ import annotations

from rclpy.node import Node
import rclpy

from sorting_interfaces.msg import TrayMatrix

from sorting_driver.protocol import (
    format_tray_matrix_text_frame,
    validate_tray_matrix,
)


class MatrixProtocolPrinter(Node):
    """订阅 TrayMatrix 并打印 F407 文本协议帧。"""

    def __init__(self) -> None:
        super().__init__('matrix_protocol_printer')

        # 订阅视觉侧输出的三苗盘矩阵。话题名统一记录在 docs/glossary.md。
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
        """处理收到的一帧 TrayMatrix。"""

        # 先校验，再格式化。这样可以在软件层面提前发现明显异常。
        errors = validate_tray_matrix(message)
        if errors:
            self.get_logger().error(
                'Invalid TrayMatrix: ' + '; '.join(errors[:5])
            )

            # 错误太多时只打印前 5 条，避免终端被 150 个穴位错误刷满。
            if len(errors) > 5:
                self.get_logger().error(
                    f'{len(errors) - 5} additional validation errors omitted'
                )
            return

        # 当前阶段先打印协议帧。后续接 F407 时，这里会替换或扩展为 TCP 发送。
        frame = format_tray_matrix_text_frame(message)
        self.get_logger().info(
            f'Received frame_id={message.frame_id} cells={len(message.cells)}'
        )
        print(frame, flush=True)


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = MatrixProtocolPrinter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
