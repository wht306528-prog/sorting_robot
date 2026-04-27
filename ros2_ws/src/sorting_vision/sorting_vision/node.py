"""模拟苗盘矩阵发布节点。

本文件用于项目早期联调，不连接真实相机，也不执行真实视觉算法。

节点功能：
1. 按照项目约定生成 3 个苗盘的数据。
2. 每个苗盘固定生成 5 列、10 行，共 50 个穴位。
3. 三个苗盘合计生成 150 个 TrayCell。
4. 将这些模拟数据封装为 TrayMatrix。
5. 每秒发布到 /sorting/tray_matrix 话题。

用途：
- 验证 sorting_interfaces 中的自定义 ROS 2 消息是否可用。
- 给 sorting_driver 提供稳定输入，方便调试鲁班猫到 F407 的通信格式。
- 在真实 D435iF 相机和视觉算法完成之前，先打通软件链路。

运行方式：
    ros2 run sorting_vision mock_matrix_publisher
"""

from __future__ import annotations

from rclpy.node import Node
import rclpy

from sorting_interfaces.msg import TrayCell, TrayMatrix


class MockMatrixPublisher(Node):
    """模拟发布 3 个苗盘、150 个穴位的 TrayMatrix 消息。

    这里生成的数据是确定性的，也就是每次运行时行列顺序和类别规则一致。
    这样做的好处是：调试通信协议时，输出稳定，容易和 F407 侧结果对照。
    """

    def __init__(self) -> None:
        super().__init__('mock_matrix_publisher')

        # 可变参数集中在 config/mock_vision.yaml。
        # 这里的默认值只作为兜底，现场运行时优先使用配置文件或命令行参数。
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('camera_frame_id', 'mock_camera')

        self._topic_name = (
            self.get_parameter('tray_matrix_topic')
            .get_parameter_value()
            .string_value
        )
        publish_period_sec = (
            self.get_parameter('publish_period_sec')
            .get_parameter_value()
            .double_value
        )
        self._camera_frame_id = (
            self.get_parameter('camera_frame_id')
            .get_parameter_value()
            .string_value
        )

        # 发布完整三苗盘矩阵。话题名称写入 docs/glossary.md，后续不要随意改。
        self._publisher = self.create_publisher(
            TrayMatrix,
            self._topic_name,
            10,
        )

        # frame_id 每发布一帧递增一次，方便后续排查丢帧、重复帧或乱序。
        self._frame_id = 0

        # 默认每秒发布一次模拟矩阵。真实视觉节点后续可能改为“苗盘到位后发布一次”。
        self._timer = self.create_timer(publish_period_sec, self._publish_matrix)
        self.get_logger().info(
            f'Publishing mock TrayMatrix messages on {self._topic_name}'
        )

    def _publish_matrix(self) -> None:
        """生成并发布一帧完整 TrayMatrix。"""

        message = TrayMatrix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._camera_frame_id
        message.frame_id = self._frame_id

        # 行列顺序固定为：先苗盘，再行，再列。
        # 这样生成顺序和文档中的 tray_id/row/col 编号规则一致。
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
        """生成单个穴位的模拟 TrayCell。

        参数含义遵循 docs/glossary.md：
        - tray_id：苗盘序号，1 表示左侧，2 表示中间，3 表示右侧。
        - row：穴位行坐标，1..10，从上到下递增。
        - col：穴位列坐标，1..5，从左到右递增。
        """

        cell = TrayCell()
        cell.tray_id = tray_id
        cell.row = row
        cell.col = col
        cell.class_id = self._mock_class_id(tray_id, row, col)
        cell.confidence = 0.95

        # 临时像素和深度值。
        # 后续真实视觉代码会用苗盘外框识别、网格映射和深度图结果替换这里。
        cell.u = float(100 * tray_id + 40 * col)
        cell.v = float(40 * row)
        cell.z = 600.0 + float(tray_id)
        return cell

    @staticmethod
    def _mock_class_id(tray_id: int, row: int, col: int) -> int:
        """根据固定规则生成模拟类别。

        当前规则：
        - 3 号苗盘是接收盘，默认全部为空穴，即 CLASS_EMPTY。
        - 1、2 号苗盘中少量位置模拟为 2 类苗或空穴。
        - 其余位置模拟为 1 类好苗。

        这个规则只用于软件联调，不代表最终视觉分类算法。
        """

        if tray_id == 3:
            return TrayCell.CLASS_EMPTY
        if row == 1 and col in (2, 4):
            return TrayCell.CLASS_WEAK
        if row == 2 and col == 3:
            return TrayCell.CLASS_EMPTY
        return TrayCell.CLASS_GOOD


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = MockMatrixPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
