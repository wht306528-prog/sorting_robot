"""矩阵协议打印节点。

本节点用于早期联调阶段，暂时不连接真实 F407。

节点功能：
1. 订阅 /sorting/tray_matrix 话题。
2. 接收 sorting_vision 发布的 TrayMatrix。
3. 校验矩阵是否满足 150 个穴位、行列范围、类别范围等规则。
4. 把矩阵转成鲁班猫到 F407 的文本协议帧。
5. 先打印非空穴位摘要，再按需打印完整文本帧。

用途：
- 检查视觉侧输出是否能被驱动侧正常接收。
- 检查协议帧字段顺序和格式是否符合文档。
- 后续开发 TCP 发送节点时，可以复用 protocol.py 中的格式化函数。

运行方式：
    ros2 run sorting_driver matrix_protocol_printer
    ros2 run sorting_driver matrix_protocol_printer --ros-args -p print_full_frame:=true
"""

from __future__ import annotations

from rclpy.node import Node
import rclpy

from sorting_interfaces.msg import TrayMatrix

from sorting_driver.protocol import (
    _format_cell,
    format_tray_matrix_text_frame,
    validate_tray_matrix,
)


class MatrixProtocolPrinter(Node):
    """订阅 TrayMatrix 并打印 F407 文本协议帧。"""

    def __init__(self) -> None:
        super().__init__('matrix_protocol_printer')

        # 话题名来自 config/driver.yaml；默认值只用于没有加载配置文件的情况。
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('print_full_frame', False)
        self.declare_parameter('max_non_empty_rows', 30)
        self._topic_name = (
            self.get_parameter('tray_matrix_topic')
            .get_parameter_value()
            .string_value
        )
        self._print_full_frame = bool(self.get_parameter('print_full_frame').value)
        self._max_non_empty_rows = max(0, int(self.get_parameter('max_non_empty_rows').value))

        # 订阅视觉侧输出的三苗盘矩阵。话题名统一记录在 docs/glossary 术语对照表.md。
        self._subscription = self.create_subscription(
            TrayMatrix,
            self._topic_name,
            self._handle_matrix,
            10,
        )
        self.get_logger().info(
            f'Listening on {self._topic_name} and printing text frames'
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

        # 当前阶段先打印摘要，避免用户在 150 行空穴里找不到真实识别结果。
        self.get_logger().info(
            f'Received frame_id={message.frame_id} cells={len(message.cells)}'
        )
        print(format_matrix_summary(message, self._max_non_empty_rows), flush=True)
        if self._print_full_frame:
            frame = format_tray_matrix_text_frame(message)
            print(frame, flush=True)


def format_matrix_summary(message: TrayMatrix, max_non_empty_rows: int = 30) -> str:
    """生成便于人看的矩阵摘要，不改变发给 F407 的完整协议帧。"""

    non_empty = [cell for cell in message.cells if cell.class_id in (1, 2)]
    white_count = sum(1 for cell in non_empty if cell.class_id == 1)
    yellow_count = sum(1 for cell in non_empty if cell.class_id == 2)
    empty_count = sum(1 for cell in message.cells if cell.class_id == 0)
    lines = [
        '========== TrayMatrix 摘要 ==========',
        f'frame_id={message.frame_id} count={len(message.cells)}',
        f'非空穴位: {len(non_empty)}  白球: {white_count}  黄球: {yellow_count}  空穴: {empty_count}',
        '8列顺序: tray_id,col,row,class_id,confidence,u,v,z',
    ]
    if non_empty:
        lines.append(f'前 {min(len(non_empty), max_non_empty_rows)} 条非空穴位:')
        for cell in non_empty[:max_non_empty_rows]:
            lines.append(_format_cell(cell))
        if len(non_empty) > max_non_empty_rows:
            lines.append(f'... 还有 {len(non_empty) - max_non_empty_rows} 条非空穴位未显示')
    else:
        lines.append('本帧没有 class_id=1/2 的白球或黄球。')
    lines.append('====================================')
    return '\n'.join(lines)


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = MatrixProtocolPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
