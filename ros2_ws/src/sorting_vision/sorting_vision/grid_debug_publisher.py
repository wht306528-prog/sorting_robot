"""
发布带有苗盘 ROI 网格的调试图像。

本节点是一个可视化调试工具，用于临时查看 tray_1_roi、tray_2_roi、
tray_3_roi 的位置。它不做苗盘分类，也不发布 TrayMatrix，只负责把固定
ROI 网格画到图像上，方便现场观察。
"""

from __future__ import annotations

from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from sorting_vision.detector import TRAY_COLS, TRAY_ROWS, TrayRoi


class GridDebugPublisher(Node):
    """在输入图像上绘制配置好的苗盘矩形和 5x10 网格。"""

    def __init__(self) -> None:
        super().__init__('grid_debug_publisher')

        self._declare_parameters()
        self._color_topic = self._string_parameter(
            'color_image_topic',
            '/camera/camera/color/image_raw',
        )
        self._debug_topic = self._string_parameter(
            'debug_image_topic',
            '/sorting/debug/grid_image',
        )
        self._tray_rois = [
            self._roi_parameter(1),
            self._roi_parameter(2),
            self._roi_parameter(3),
        ]
        self._line_width = max(1, self._int_parameter('debug_line_width_px', 2))
        self._center_radius = max(1, self._int_parameter('debug_center_radius_px', 3))
        self._warned_encoding = False

        self._publisher = self.create_publisher(Image, self._debug_topic, 10)
        self._subscription = self.create_subscription(
            Image,
            self._color_topic,
            self._handle_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'正在订阅 RGB 图像：{self._color_topic}')
        self.get_logger().info(f'正在发布网格调试图像：{self._debug_topic}')
        self.get_logger().info(f'当前配置的苗盘 ROI：{self._tray_rois}')

    def _declare_parameters(self) -> None:
        """声明本节点使用的 ROS 参数。"""
        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('debug_image_topic', '/sorting/debug/grid_image')
        self.declare_parameter('tray_1_roi', [20.0, 50.0, 185.0, 380.0])
        self.declare_parameter('tray_2_roi', [227.5, 50.0, 185.0, 380.0])
        self.declare_parameter('tray_3_roi', [435.0, 50.0, 185.0, 380.0])
        self.declare_parameter('debug_line_width_px', 2)
        self.declare_parameter('debug_center_radius_px', 3)

    def _handle_image(self, message: Image) -> None:
        """绘制配置好的苗盘网格并发布标注图像。"""
        channels = _channel_offsets(message.encoding)
        if channels is None:
            if not self._warned_encoding:
                self.get_logger().warning(
                    f'网格调试图像暂不支持该编码：{message.encoding}'
                )
                self._warned_encoding = True
            return

        output = Image()
        output.header = message.header
        output.height = message.height
        output.width = message.width
        output.encoding = message.encoding
        output.is_bigendian = message.is_bigendian
        output.step = message.step

        pixels = bytearray(message.data)
        for roi, color in zip(self._tray_rois, _tray_colors()):
            _draw_tray_grid(
                pixels,
                message.width,
                message.height,
                message.step,
                channels,
                roi,
                color,
                self._line_width,
                self._center_radius,
            )

        output.data = bytes(pixels)
        self._publisher.publish(output)

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

    def _string_parameter(self, name: str, default: str) -> str:
        """读取字符串参数。"""
        value = self.get_parameter(name).value
        return str(value) if value is not None else default

    def _int_parameter(self, name: str, default: int) -> int:
        """读取整数参数。"""
        value = self.get_parameter(name).value
        return int(value) if value is not None else default


def _draw_tray_grid(
    pixels: bytearray,
    image_width: int,
    image_height: int,
    step: int,
    channels: tuple[int, int, int, int],
    roi: TrayRoi,
    color: tuple[int, int, int],
    line_width: int,
    center_radius: int,
) -> None:
    """绘制一个苗盘 ROI 外框、网格线和穴位中心点。"""
    left = int(round(roi.x))
    top = int(round(roi.y))
    right = int(round(roi.x + roi.width))
    bottom = int(round(roi.y + roi.height))
    cell_width = roi.width / TRAY_COLS
    cell_height = roi.height / TRAY_ROWS

    _draw_rectangle(
        pixels,
        image_width,
        image_height,
        step,
        channels,
        left,
        top,
        right,
        bottom,
        color,
        line_width,
    )

    for col in range(1, TRAY_COLS):
        x = int(round(roi.x + col * cell_width))
        _draw_line(
            pixels, image_width, image_height, step, channels,
            x, top, x, bottom, color, max(1, line_width - 1),
        )
    for row in range(1, TRAY_ROWS):
        y = int(round(roi.y + row * cell_height))
        _draw_line(
            pixels, image_width, image_height, step, channels,
            left, y, right, y, color, max(1, line_width - 1),
        )

    for row in range(1, TRAY_ROWS + 1):
        for col in range(1, TRAY_COLS + 1):
            center_x = int(round(roi.x + (col - 0.5) * cell_width))
            center_y = int(round(roi.y + (row - 0.5) * cell_height))
            _draw_cross(
                pixels,
                image_width,
                image_height,
                step,
                channels,
                center_x,
                center_y,
                center_radius,
                color,
            )


def _draw_rectangle(
    pixels: bytearray,
    image_width: int,
    image_height: int,
    step: int,
    channels: tuple[int, int, int, int],
    left: int,
    top: int,
    right: int,
    bottom: int,
    color: tuple[int, int, int],
    line_width: int,
) -> None:
    """绘制矩形边框。"""
    _draw_line(
        pixels, image_width, image_height, step, channels,
        left, top, right, top, color, line_width,
    )
    _draw_line(
        pixels, image_width, image_height, step, channels,
        left, bottom, right, bottom, color, line_width,
    )
    _draw_line(
        pixels, image_width, image_height, step, channels,
        left, top, left, bottom, color, line_width,
    )
    _draw_line(
        pixels, image_width, image_height, step, channels,
        right, top, right, bottom, color, line_width,
    )


def _draw_cross(
    pixels: bytearray,
    image_width: int,
    image_height: int,
    step: int,
    channels: tuple[int, int, int, int],
    center_x: int,
    center_y: int,
    radius: int,
    color: tuple[int, int, int],
) -> None:
    """在一个穴位中心绘制小十字。"""
    _draw_line(
        pixels, image_width, image_height, step, channels,
        center_x - radius, center_y, center_x + radius, center_y, color, 1,
    )
    _draw_line(
        pixels, image_width, image_height, step, channels,
        center_x, center_y - radius, center_x, center_y + radius, color, 1,
    )


def _draw_line(
    pixels: bytearray,
    image_width: int,
    image_height: int,
    step: int,
    channels: tuple[int, int, int, int],
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    color: tuple[int, int, int],
    line_width: int,
) -> None:
    """绘制水平线或垂直线。"""
    if x1 == x2:
        start_y, end_y = sorted((y1, y2))
        for delta in range(-(line_width // 2), line_width // 2 + 1):
            x = x1 + delta
            for y in range(start_y, end_y + 1):
                _set_pixel(
                    pixels, image_width, image_height, step, channels, x, y, color,
                )
        return

    if y1 == y2:
        start_x, end_x = sorted((x1, x2))
        for delta in range(-(line_width // 2), line_width // 2 + 1):
            y = y1 + delta
            for x in range(start_x, end_x + 1):
                _set_pixel(
                    pixels, image_width, image_height, step, channels, x, y, color,
                )


def _set_pixel(
    pixels: bytearray,
    image_width: int,
    image_height: int,
    step: int,
    channels: tuple[int, int, int, int],
    x: int,
    y: int,
    color: tuple[int, int, int],
) -> None:
    """在检查边界和通道顺序后设置一个像素。"""
    if x < 0 or x >= image_width or y < 0 or y >= image_height:
        return
    red_index, green_index, blue_index, bytes_per_pixel = channels
    offset = y * step + x * bytes_per_pixel
    if offset + bytes_per_pixel > len(pixels):
        return
    red, green, blue = color
    pixels[offset + red_index] = red
    pixels[offset + green_index] = green
    pixels[offset + blue_index] = blue


def _channel_offsets(encoding: str) -> tuple[int, int, int, int] | None:
    """返回常见彩色图像编码中的 RGB 通道偏移。"""
    normalized = encoding.lower()
    if normalized == 'rgb8':
        return 0, 1, 2, 3
    if normalized == 'bgr8':
        return 2, 1, 0, 3
    if normalized == 'rgba8':
        return 0, 1, 2, 4
    if normalized == 'bgra8':
        return 2, 1, 0, 4
    return None


def _tray_colors() -> list[tuple[int, int, int]]:
    """返回 1、2、3 号苗盘使用的不同 RGB 颜色。"""
    return [
        (255, 40, 40),
        (40, 220, 80),
        (60, 140, 255),
    ]


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口。"""
    rclpy.init(args=args)
    node = GridDebugPublisher()
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
