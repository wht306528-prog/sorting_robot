"""单点 RGB-D 深度和相机坐标探测节点。

这个节点用于验证 D435iF 这类 RGB-D 相机是否能在指定像素点读取到稳定深度。
它不做苗盘识别，只做一件事：读取一个像素点附近的深度，并用 CameraInfo
里的内参换算成相机坐标系下的 X/Y/Z。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from sorting_vision.algorithms.camera_model import CameraIntrinsics, intrinsics_from_camera_info
from sorting_vision.algorithms.detector import ImageView, sample_depth_mm


class DepthPointProbe(Node):
    """订阅对齐深度图并打印指定像素点的深度和相机坐标。"""

    def __init__(self) -> None:
        super().__init__('depth_point_probe')

        self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('u', 320.0)
        self.declare_parameter('v', 240.0)
        self.declare_parameter('depth_window_px', 5)
        self.declare_parameter('report_period_sec', 1.0)

        self._depth_topic = str(self.get_parameter('depth_image_topic').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self._u = float(self.get_parameter('u').value)
        self._v = float(self.get_parameter('v').value)
        self._depth_window_px = int(self.get_parameter('depth_window_px').value)

        self._latest_depth: Image | None = None
        self._intrinsics: CameraIntrinsics | None = None
        self._warned_waiting_depth = False
        self._warned_waiting_info = False

        self.create_subscription(Image, self._depth_topic, self._handle_depth, 10)
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._handle_camera_info,
            10,
        )
        self.create_timer(
            float(self.get_parameter('report_period_sec').value),
            self._report,
        )

        self.get_logger().info(f'正在订阅深度图：{self._depth_topic}')
        self.get_logger().info(f'正在订阅相机内参：{self._camera_info_topic}')
        self.get_logger().info(
            f'探测像素点：u={self._u:.1f}, v={self._v:.1f}, '
            f'窗口={self._depth_window_px}px'
        )

    def _handle_depth(self, message: Image) -> None:
        """保存最新深度图。"""
        self._latest_depth = message

    def _handle_camera_info(self, message: CameraInfo) -> None:
        """保存最新相机内参。"""
        self._intrinsics = intrinsics_from_camera_info(message)

    def _report(self) -> None:
        """定时打印单点深度和相机坐标。"""
        if self._latest_depth is None:
            if not self._warned_waiting_depth:
                self.get_logger().warning('正在等待深度图，暂时无法测距')
                self._warned_waiting_depth = True
            return
        if self._intrinsics is None:
            if not self._warned_waiting_info:
                self.get_logger().warning('正在等待相机内参，暂时无法换算坐标')
                self._warned_waiting_info = True
            return

        image = ImageView(
            width=self._latest_depth.width,
            height=self._latest_depth.height,
            encoding=self._latest_depth.encoding,
            step=self._latest_depth.step,
            data=bytes(self._latest_depth.data),
        )
        z_mm = sample_depth_mm(image, self._u, self._v, self._depth_window_px)
        if z_mm <= 0.0:
            self.get_logger().warning(
                f'pixel=({self._u:.1f}, {self._v:.1f}) depth=0.0mm，'
                '该点深度无效或超出有效范围'
            )
            return

        x_mm, y_mm, z_camera_mm = self._intrinsics.pixel_to_camera(
            self._u,
            self._v,
            z_mm,
        )
        self.get_logger().info(
            f'pixel=({self._u:.1f}, {self._v:.1f}) '
            f'depth={z_mm:.1f}mm '
            f'camera_xyz=({x_mm:.1f}, {y_mm:.1f}, {z_camera_mm:.1f})mm '
            f'encoding={self._latest_depth.encoding}'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthPointProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
