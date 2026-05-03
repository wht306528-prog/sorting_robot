"""
真实相机苗盘矩阵发布节点。

本节点是从 D435iF 或普通相机图像到 TrayMatrix 消息的第一版桥接节点。
它当前使用配置好的苗盘 ROI 和简单绿色面积规则，方便先验证相机输入、
5x10 网格编号、深度采样和后续 F407 通信。它是早期联调工具，不是最终
苗盘外框检测方案。
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
    """从真实 RGB 或 RGB-D 图像话题发布完整 150 格 TrayMatrix。"""

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
        self._use_depth = self._bool_parameter('use_depth', True)
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
        self._depth_subscription = None
        self._depth_info_subscription = None
        if self._use_depth:
            self._depth_subscription = self.create_subscription(
                Image,
                self._depth_topic,
                self._handle_depth,
                10,
            )
            self._depth_info_subscription = self.create_subscription(
                CameraInfo,
                self._depth_info_topic,
                self._handle_depth_info,
                10,
            )
        self._color_info_subscription = self.create_subscription(
            CameraInfo,
            self._color_info_topic,
            self._handle_color_info,
            10,
        )
        self._timer = self.create_timer(publish_period_sec, self._publish_matrix)

        self.get_logger().info(f'正在发布真实 TrayMatrix：{self._topic_name}')
        self.get_logger().info(f'正在订阅 RGB 图像：{self._color_topic}')
        if self._use_depth:
            self.get_logger().info(f'正在订阅深度图像：{self._depth_topic}')
        else:
            self.get_logger().info('已关闭深度输入：将以 RGB-only 模式发布，z=0')
        self.get_logger().info(f'当前配置的苗盘 ROI：{self._tray_rois}')

    def _declare_parameters(self) -> None:
        """声明本节点使用的 ROS 参数。"""
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('camera_frame_id', '')
        self.declare_parameter('use_depth', True)
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
        """从 ROS 参数读取检测阈值。"""
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
        """读取三个配置好的苗盘矩形。"""
        return [
            self._roi_parameter(1),
            self._roi_parameter(2),
            self._roi_parameter(3),
        ]

    def _roi_parameter(self, tray_id: int) -> TrayRoi:
        """按 x、y、width、height 顺序读取一个苗盘 ROI 参数。"""
        values = list(self.get_parameter(f'tray_{tray_id}_roi').value)
        if len(values) != 4:
            raise ValueError(f'tray_{tray_id}_roi 必须包含 4 个数值')
        return TrayRoi(
            tray_id=tray_id,
            x=float(values[0]),
            y=float(values[1]),
            width=float(values[2]),
            height=float(values[3]),
        )

    def _handle_color(self, message: Image) -> None:
        """保存最新 RGB 图像。"""
        self._latest_color = message

    def _handle_depth(self, message: Image) -> None:
        """保存最新深度图像。"""
        self._latest_depth = message

    def _handle_color_info(self, message: CameraInfo) -> None:
        """缓存 RGB 相机内参。"""
        self._intrinsics.update_color(message)

    def _handle_depth_info(self, message: CameraInfo) -> None:
        """缓存深度相机内参。"""
        self._intrinsics.update_depth(message)

    def _publish_matrix(self) -> None:
        """根据最新 RGB 或 RGB-D 帧构建并发布 TrayMatrix。"""
        if self._latest_color is None:
            if not self._warned_waiting:
                self.get_logger().warning('正在等待 RGB 图像，暂不发布矩阵')
                self._warned_waiting = True
            return

        depth_image = self._latest_depth if self._use_depth else None
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
            f'已发布 frame_id={message.frame_id} cells={len(message.cells)}'
        )
        self._frame_id += 1

    def _resolve_frame_id(self) -> str:
        """选择输出 TrayMatrix header 使用的 frame_id。"""
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
        """当深度图和 RGB 图尺寸不一致时只警告一次。"""
        if depth_image is None or self._warned_image_size:
            return
        if (
            color_image.width == depth_image.width
            and color_image.height == depth_image.height
        ):
            return
        self.get_logger().warning(
            'RGB 图和深度图尺寸不一致；当前 z 采样默认两路图像已经对齐。'
            '请启用深度到 RGB 的对齐，或调整订阅话题。'
        )
        self._warned_image_size = True

    @staticmethod
    def _image_view(message: Image) -> ImageView:
        """把 ROS Image 消息转换成检测器使用的轻量图像视图。"""
        return ImageView(
            width=message.width,
            height=message.height,
            encoding=message.encoding,
            step=message.step,
            data=bytes(message.data),
        )

    def _string_parameter(self, name: str, default: str) -> str:
        """读取字符串参数。"""
        value = self.get_parameter(name).value
        return str(value) if value is not None else default

    def _float_parameter(self, name: str, default: float) -> float:
        """读取浮点数参数。"""
        value = self.get_parameter(name).value
        return float(value) if value is not None else default

    def _int_parameter(self, name: str, default: int) -> int:
        """读取整数参数。"""
        value = self.get_parameter(name).value
        return int(value) if value is not None else default

    def _bool_parameter(self, name: str, default: bool) -> bool:
        """读取布尔参数。"""
        value = self.get_parameter(name).value
        return bool(value) if value is not None else default


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口。"""
    rclpy.init(args=args)
    node = RealMatrixPublisher()
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
