"""OpenCV 风格的 ROS 图像预览窗口。

这个工具只负责把已有 Image topic 显示成一个简洁窗口，方便现场演示时观察
相机画面或识别标注图。它不参与识别、不发布矩阵，也不影响 F407 通信链路。
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class OpenCvImageViewer(Node):
    """订阅一个 ROS 图像话题，并用 cv2.imshow 显示。"""

    def __init__(self) -> None:
        super().__init__('opencv_image_viewer')

        self.declare_parameter('image_topic', '/sorting/pingpong/debug_image')
        self.declare_parameter('window_name', 'sorting_robot_view')
        self.declare_parameter('max_width', 1280)
        self.declare_parameter('max_height', 720)
        self.declare_parameter('show_status', True)

        self._image_topic = str(self.get_parameter('image_topic').value)
        self._window_name = str(self.get_parameter('window_name').value)
        self._max_width = int(self.get_parameter('max_width').value)
        self._max_height = int(self.get_parameter('max_height').value)
        self._show_status = bool(self.get_parameter('show_status').value)

        self._frame_count = 0
        self._start_time = time.monotonic()
        self._last_report_time = self._start_time

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        self.create_subscription(Image, self._image_topic, self._handle_image, 10)

        self.get_logger().info(f'OpenCV 预览窗口正在订阅：{self._image_topic}')
        self.get_logger().info('窗口内按 q 或 ESC 退出')

    def _handle_image(self, message: Image) -> None:
        """收到图像后刷新窗口。"""

        try:
            image = image_message_to_bgr(message)
        except ValueError as exc:
            now = time.monotonic()
            if now - self._last_report_time > 2.0:
                self.get_logger().error(str(exc))
                self._last_report_time = now
            return

        self._frame_count += 1
        image = self._resize_for_screen(image)
        if self._show_status:
            self._draw_status(image, message)

        cv2.imshow(self._window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.get_logger().info('收到退出按键，关闭 OpenCV 预览窗口')
            rclpy.shutdown()

    def _resize_for_screen(self, image: np.ndarray) -> np.ndarray:
        """按最大宽高等比例缩小，避免 X11 窗口过大。"""

        height, width = image.shape[:2]
        scale = min(self._max_width / width, self._max_height / height, 1.0)
        if scale >= 1.0:
            return image

        new_size = (int(width * scale), int(height * scale))
        return cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)

    def _draw_status(self, image: np.ndarray, message: Image) -> None:
        """在左上角写入 topic、尺寸和估算 FPS，便于现场确认看的是哪路图。"""

        elapsed = max(time.monotonic() - self._start_time, 0.001)
        fps = self._frame_count / elapsed
        text = (
            f'{self._image_topic} | '
            f'{message.width}x{message.height} {message.encoding} | '
            f'{fps:.1f} FPS'
        )
        cv2.rectangle(image, (8, 8), (min(image.shape[1] - 8, 760), 40), (0, 0, 0), -1)
        cv2.putText(
            image,
            text,
            (16, 31),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def image_message_to_bgr(message: Image) -> np.ndarray:
    """把常见 ROS Image 编码转换成 OpenCV BGR 图像。"""

    encoding = message.encoding.lower()
    if encoding in ('rgb8', 'bgr8'):
        image = _reshape_u8_image(message, channels=3)
        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()

    if encoding in ('rgba8', 'bgra8'):
        image = _reshape_u8_image(message, channels=4)
        if encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    if encoding in ('mono8', '8uc1'):
        gray = _reshape_u8_image(message, channels=1)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if encoding in ('16uc1', 'mono16'):
        depth = _reshape_u16_image(message)
        return colorize_depth_for_view(depth)

    raise ValueError(f'OpenCV 预览暂不支持图像编码：{message.encoding}')


def _reshape_u8_image(message: Image, channels: int) -> np.ndarray:
    """按 ROS step 处理 uint8 图像，避免行对齐字节导致 reshape 错误。"""

    row_bytes = message.width * channels
    raw = np.frombuffer(message.data, dtype=np.uint8)
    image = raw.reshape((message.height, message.step))[:, :row_bytes]
    if channels == 1:
        return image.reshape((message.height, message.width))
    return image.reshape((message.height, message.width, channels))


def _reshape_u16_image(message: Image) -> np.ndarray:
    """按 ROS step 处理 16 位单通道图像。"""

    row_values = message.step // 2
    raw = np.frombuffer(message.data, dtype='<u2')
    image = raw.reshape((message.height, row_values))[:, : message.width]
    return image.copy()


def colorize_depth_for_view(depth_mm: np.ndarray) -> np.ndarray:
    """把 16UC1 深度图转成伪彩色，仅用于肉眼观察。"""

    valid = depth_mm > 0
    if not np.any(valid):
        scaled = np.zeros(depth_mm.shape, dtype=np.uint8)
    else:
        low = float(np.percentile(depth_mm[valid], 2))
        high = float(np.percentile(depth_mm[valid], 98))
        if high <= low:
            high = low + 1.0
        scaled = np.clip((depth_mm.astype(np.float32) - low) * 255.0 / (high - low), 0, 255)
        scaled = scaled.astype(np.uint8)
        scaled[~valid] = 0
    return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = OpenCvImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
