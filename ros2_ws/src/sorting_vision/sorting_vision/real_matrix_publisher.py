"""
Real-camera tray matrix publisher.

This node is the first runnable bridge from D435iF image topics to the project
TrayMatrix message. It uses configured tray ROIs and a simple green-area rule
as a temporary classifier, so the team can verify camera placement, 5x10 grid
numbering, depth sampling, and downstream F407 communication before automatic
tray contour detection is tuned.
"""

from __future__ import annotations

from rclpy.node import Node
import rclpy
from sensor_msgs.msg import CameraInfo, Image

from sorting_interfaces.msg import TrayMatrix
from sorting_vision.camera_model import CameraIntrinsicsCache
from sorting_vision.detector import (
    DetectionConfig,
    ImageView,
    RuleBasedTrayDetector,
    TrayRoi,
)


class RealMatrixPublisher(Node):
    """Publish a full 150-cell TrayMatrix from real RGB-D image topics."""

    def __init__(self) -> None:
        super().__init__('real_matrix_publisher')

        self._declare_parameters()
        self._topic_name = self._string_parameter(
            'tray_matrix_topic',
            '/sorting/tray_matrix',
        )
        self._color_topic = self._string_parameter(
            'color_image_topic',
            '/camera/camera/color/image_raw',
        )
        self._depth_topic = self._string_parameter(
            'depth_image_topic',
            '/camera/camera/depth/image_rect_raw',
        )
        self._color_info_topic = self._string_parameter(
            'color_camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self._depth_info_topic = self._string_parameter(
            'depth_camera_info_topic',
            '/camera/camera/depth/camera_info',
        )
        self._camera_frame_id = self._string_parameter('camera_frame_id', '')
        publish_period_sec = self._float_parameter('publish_period_sec', 1.0)

        self._intrinsics = CameraIntrinsicsCache()
        self._detector = RuleBasedTrayDetector(self._detection_config())
        self._tray_rois = self._load_tray_rois()
        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None
        self._frame_id = 0
        self._warned_waiting = False
        self._warned_image_size = False

        self._publisher = self.create_publisher(TrayMatrix, self._topic_name, 10)
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
        self._color_info_subscription = self.create_subscription(
            CameraInfo,
            self._color_info_topic,
            self._handle_color_info,
            10,
        )
        self._depth_info_subscription = self.create_subscription(
            CameraInfo,
            self._depth_info_topic,
            self._handle_depth_info,
            10,
        )
        self._timer = self.create_timer(publish_period_sec, self._publish_matrix)

        self.get_logger().info(f'Publishing real TrayMatrix on {self._topic_name}')
        self.get_logger().info(f'Listening color image: {self._color_topic}')
        self.get_logger().info(f'Listening depth image: {self._depth_topic}')
        self.get_logger().info(f'Configured tray ROIs: {self._tray_rois}')

    def _declare_parameters(self) -> None:
        """Declare ROS parameters used by the node."""
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('camera_frame_id', '')
        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/depth/image_rect_raw',
        )
        self.declare_parameter(
            'color_camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self.declare_parameter(
            'depth_camera_info_topic',
            '/camera/camera/depth/camera_info',
        )
        self.declare_parameter('tray_1_roi', [20.0, 50.0, 185.0, 380.0])
        self.declare_parameter('tray_2_roi', [227.5, 50.0, 185.0, 380.0])
        self.declare_parameter('tray_3_roi', [435.0, 50.0, 185.0, 380.0])
        self.declare_parameter('leaf_area_ratio_threshold', 0.20)
        self.declare_parameter('weak_area_ratio_threshold', 0.03)
        self.declare_parameter('depth_window_px', 5)
        self.declare_parameter('roi_inner_scale', 0.65)
        self.declare_parameter('color_sample_step_px', 2)

    def _detection_config(self) -> DetectionConfig:
        """Load detector thresholds from ROS parameters."""
        return DetectionConfig(
            leaf_area_ratio_threshold=self._float_parameter(
                'leaf_area_ratio_threshold',
                0.20,
            ),
            weak_area_ratio_threshold=self._float_parameter(
                'weak_area_ratio_threshold',
                0.03,
            ),
            depth_window_px=self._int_parameter('depth_window_px', 5),
            roi_inner_scale=self._float_parameter('roi_inner_scale', 0.65),
            color_sample_step_px=self._int_parameter('color_sample_step_px', 2),
        )

    def _load_tray_rois(self) -> list[TrayRoi]:
        """Load three configured tray rectangles."""
        return [
            self._roi_parameter(1),
            self._roi_parameter(2),
            self._roi_parameter(3),
        ]

    def _roi_parameter(self, tray_id: int) -> TrayRoi:
        """Read one tray ROI parameter in x, y, width, height order."""
        values = list(self.get_parameter(f'tray_{tray_id}_roi').value)
        if len(values) != 4:
            raise ValueError(f'tray_{tray_id}_roi must contain 4 values')
        return TrayRoi(
            tray_id=tray_id,
            x=float(values[0]),
            y=float(values[1]),
            width=float(values[2]),
            height=float(values[3]),
        )

    def _handle_color(self, message: Image) -> None:
        """Store the latest color image."""
        self._latest_color = message

    def _handle_depth(self, message: Image) -> None:
        """Store the latest depth image."""
        self._latest_depth = message

    def _handle_color_info(self, message: CameraInfo) -> None:
        """Cache color camera intrinsics."""
        self._intrinsics.update_color(message)

    def _handle_depth_info(self, message: CameraInfo) -> None:
        """Cache depth camera intrinsics."""
        self._intrinsics.update_depth(message)

    def _publish_matrix(self) -> None:
        """Build and publish a TrayMatrix from the latest RGB-D frame."""
        if self._latest_color is None:
            if not self._warned_waiting:
                self.get_logger().warning('Waiting for color image before publish')
                self._warned_waiting = True
            return

        depth_image = self._latest_depth
        self._warn_if_geometry_differs(self._latest_color, depth_image)
        detections = self._detector.detect(
            self._image_view(self._latest_color),
            self._image_view(depth_image) if depth_image is not None else None,
            self._tray_rois,
        )

        message = TrayMatrix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._resolve_frame_id()
        message.frame_id = self._frame_id
        message.cells = [detection.to_message() for detection in detections]

        self._publisher.publish(message)
        self.get_logger().info(
            f'Published frame_id={message.frame_id} cells={len(message.cells)}'
        )
        self._frame_id += 1

    def _resolve_frame_id(self) -> str:
        """Choose the frame id for the outgoing TrayMatrix header."""
        if self._camera_frame_id:
            return self._camera_frame_id
        if self._latest_color is not None and self._latest_color.header.frame_id:
            return self._latest_color.header.frame_id
        if self._intrinsics.color is not None:
            return self._intrinsics.color.frame_id
        return 'camera_color_optical_frame'

    def _warn_if_geometry_differs(
        self,
        color_image: Image,
        depth_image: Image | None,
    ) -> None:
        """Warn once when depth pixels may not align with color pixels."""
        if depth_image is None or self._warned_image_size:
            return
        if (
            color_image.width == depth_image.width
            and color_image.height == depth_image.height
        ):
            return
        self.get_logger().warning(
            'Color and depth image sizes differ; z sampling assumes aligned '
            'RGB-D streams. Enable RealSense depth alignment or adjust topics.'
        )
        self._warned_image_size = True

    @staticmethod
    def _image_view(message: Image) -> ImageView:
        """Convert a ROS Image message to the detector's minimal view."""
        return ImageView(
            width=message.width,
            height=message.height,
            encoding=message.encoding,
            step=message.step,
            data=bytes(message.data),
        )

    def _string_parameter(self, name: str, default: str) -> str:
        """Read a string parameter."""
        value = self.get_parameter(name).value
        return str(value) if value is not None else default

    def _float_parameter(self, name: str, default: float) -> float:
        """Read a float parameter."""
        value = self.get_parameter(name).value
        return float(value) if value is not None else default

    def _int_parameter(self, name: str, default: int) -> int:
        """Read an integer parameter."""
        value = self.get_parameter(name).value
        return int(value) if value is not None else default


def main(args: list[str] | None = None) -> None:
    """ROS 2 node entry point."""
    rclpy.init(args=args)
    node = RealMatrixPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
