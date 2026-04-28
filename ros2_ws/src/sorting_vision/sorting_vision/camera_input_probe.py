"""真实相机输入探测节点。

本节点用于 D435iF 接入后的第一步软件验证。

节点功能：
1. 订阅 RGB 图像话题。
2. 订阅深度图话题。
3. 记录最新图像的尺寸、编码、frame_id 和时间戳。
4. 周期性打印 RGB/Depth 的接收数量和估算频率。

重要说明：
- 本节点不做苗盘识别。
- 本节点不做 RGB-D 对齐。
- 本节点不输出 TrayMatrix。
- 它只负责确认项目自己的 ROS 2 Python 代码能收到真实相机数据。

运行方式：
    ros2 run sorting_vision camera_input_probe --ros-args \
        --params-file install/sorting_vision/share/sorting_vision/config/mock_vision.yaml
"""

from __future__ import annotations

from dataclasses import dataclass
import time

from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image


@dataclass
class ImageStats:
    """保存单路图像话题的接收统计。"""

    count: int = 0
    width: int = 0
    height: int = 0
    encoding: str = ''
    frame_id: str = ''
    stamp_sec: int = 0
    stamp_nanosec: int = 0


class CameraInputProbe(Node):
    """订阅真实 RGB/Depth 图像并周期性打印接收状态。"""

    def __init__(self) -> None:
        super().__init__('camera_input_probe')

        # 相机话题名集中放在配置文件中，避免后续不同相机命名时到处改代码。
        self.declare_parameter(
            'color_image_topic',
            '/camera/camera/color/image_raw',
        )
        self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/depth/image_rect_raw',
        )
        self.declare_parameter('report_period_sec', 2.0)

        self._color_topic = (
            self.get_parameter('color_image_topic')
            .get_parameter_value()
            .string_value
        )
        self._depth_topic = (
            self.get_parameter('depth_image_topic')
            .get_parameter_value()
            .string_value
        )
        report_period_sec = (
            self.get_parameter('report_period_sec')
            .get_parameter_value()
            .double_value
        )

        self._start_time = time.monotonic()
        self._color = ImageStats()
        self._depth = ImageStats()

        self._color_subscription = self.create_subscription(
            Image,
            self._color_topic,
            self._handle_color,
            10,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            self._depth_topic,
            self._handle_depth,
            10,
        )
        self._timer = self.create_timer(report_period_sec, self._report)

        self.get_logger().info(f'Listening color image: {self._color_topic}')
        self.get_logger().info(f'Listening depth image: {self._depth_topic}')

    def _handle_color(self, message: Image) -> None:
        """记录最新 RGB 图像信息。"""

        self._update_stats(self._color, message)

    def _handle_depth(self, message: Image) -> None:
        """记录最新深度图信息。"""

        self._update_stats(self._depth, message)

    @staticmethod
    def _update_stats(stats: ImageStats, message: Image) -> None:
        """用最新 Image 消息更新统计信息。"""

        stats.count += 1
        stats.width = message.width
        stats.height = message.height
        stats.encoding = message.encoding
        stats.frame_id = message.header.frame_id
        stats.stamp_sec = message.header.stamp.sec
        stats.stamp_nanosec = message.header.stamp.nanosec

    def _report(self) -> None:
        """周期性打印 RGB/Depth 接收状态。"""

        elapsed = max(time.monotonic() - self._start_time, 0.001)
        color_hz = self._color.count / elapsed
        depth_hz = self._depth.count / elapsed

        self.get_logger().info(
            'color '
            f'count={self._color.count} hz={color_hz:.2f} '
            f'size={self._color.width}x{self._color.height} '
            f'encoding={self._color.encoding} frame={self._color.frame_id}'
        )
        self.get_logger().info(
            'depth '
            f'count={self._depth.count} hz={depth_hz:.2f} '
            f'size={self._depth.width}x{self._depth.height} '
            f'encoding={self._depth.encoding} frame={self._depth.frame_id}'
        )


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = CameraInputProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
