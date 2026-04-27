"""矩阵 TCP 发送节点。

本节点是 matrix_protocol_printer 的下一步：不再只把协议帧打印到终端，
而是把协议帧通过 TCP 发送出去。

当前设计用途：
1. 订阅 /sorting/tray_matrix 话题。
2. 校验收到的 TrayMatrix 是否满足 150 个穴位等约定。
3. 使用 protocol.py 将矩阵转成文本协议帧。
4. 连接 F407/W5500 的 TCP 服务端。
5. 发送完整文本帧。

默认参数：
- tray_matrix_topic: /sorting/tray_matrix
- f407_host: 127.0.0.1
- f407_port: 9000
- send_timeout_sec: 2.0

调试方法：
    终端 1 启动一个本机 TCP 接收端：
        nc -l 9000

    终端 2 启动模拟视觉矩阵发布：
        ros2 run sorting_vision mock_matrix_publisher

    终端 3 启动本节点：
        ros2 run sorting_driver matrix_tcp_sender

后续接真实 F407 时，只需要在 config/driver.yaml 或命令行参数里改
f407_host 和 f407_port，不需要翻代码。
"""

from __future__ import annotations

import socket

from rclpy.node import Node
import rclpy

from sorting_interfaces.msg import TrayMatrix

from sorting_driver.protocol import (
    format_tray_matrix_text_frame,
    validate_tray_matrix,
)


class MatrixTcpSender(Node):
    """订阅 TrayMatrix 并通过 TCP 发送 F407 文本协议帧。"""

    def __init__(self) -> None:
        super().__init__('matrix_tcp_sender')

        # 可变参数集中在 config/driver.yaml。
        # 这里的默认值只作为兜底，现场运行时优先使用配置文件或命令行参数。
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('f407_host', '127.0.0.1')
        self.declare_parameter('f407_port', 9000)
        self.declare_parameter('send_timeout_sec', 2.0)

        self._topic_name = (
            self.get_parameter('tray_matrix_topic')
            .get_parameter_value()
            .string_value
        )
        self._host = (
            self.get_parameter('f407_host').get_parameter_value().string_value
        )
        self._port = (
            self.get_parameter('f407_port').get_parameter_value().integer_value
        )
        self._timeout = (
            self.get_parameter('send_timeout_sec')
            .get_parameter_value()
            .double_value
        )
        self._socket: socket.socket | None = None

        # 订阅视觉侧输出矩阵。收到一帧，就尝试发送一帧文本协议。
        self._subscription = self.create_subscription(
            TrayMatrix,
            self._topic_name,
            self._handle_matrix,
            10,
        )

        self.get_logger().info(
            f'Sending {self._topic_name} frames to {self._host}:{self._port}'
        )

    def _handle_matrix(self, message: TrayMatrix) -> None:
        """处理收到的一帧 TrayMatrix，并尝试通过 TCP 发送。"""

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

        # 文本帧末尾补一个换行，方便 F407 或调试工具按行读取 END。
        frame = format_tray_matrix_text_frame(message) + '\n'
        payload = frame.encode('utf-8')

        try:
            self._ensure_connected()
            assert self._socket is not None
            self._socket.sendall(payload)
        except OSError as exc:
            self.get_logger().error(
                f'Failed to send frame_id={message.frame_id}: {exc}'
            )
            self._close_socket()
            return

        self.get_logger().info(
            f'Sent frame_id={message.frame_id} bytes={len(payload)}'
        )

    def _ensure_connected(self) -> None:
        """确保 TCP socket 已连接，未连接时自动创建连接。"""

        if self._socket is not None:
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self._timeout)

        try:
            sock.connect((self._host, self._port))
        except OSError:
            sock.close()
            raise

        self._socket = sock
        self.get_logger().info(f'Connected to {self._host}:{self._port}')

    def _close_socket(self) -> None:
        """关闭当前 socket，下一帧到来时会重新连接。"""

        if self._socket is None:
            return

        try:
            self._socket.close()
        finally:
            self._socket = None

    def destroy_node(self) -> bool:
        """节点销毁时关闭 TCP 连接。"""

        self._close_socket()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = MatrixTcpSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
