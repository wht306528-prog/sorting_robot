"""RGB-D 样本采集节点。

运行一次保存一组样本：
- RGB 彩色图。
- 已对齐到 RGB 的深度图。
- RGB 和深度 CameraInfo。

这些文件用于离线调试苗盘定位、穴位切分和深度采样，不要求实时运行。
"""

from __future__ import annotations

from pathlib import Path
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RgbdSampleCapture(Node):
    """等待一帧 RGB-D 数据并保存到样本目录。"""

    def __init__(self) -> None:
        super().__init__('capture_rgbd_sample')

        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'color_camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self.declare_parameter(
            'depth_camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('output_dir', 'samples/rgbd')
        self.declare_parameter('prefix', 'sample')
        self.declare_parameter('capture_timeout_sec', 10.0)

        self._color_topic = str(self.get_parameter('color_image_topic').value)
        self._depth_topic = str(self.get_parameter('depth_image_topic').value)
        self._color_info_topic = str(
            self.get_parameter('color_camera_info_topic').value
        )
        self._depth_info_topic = str(
            self.get_parameter('depth_camera_info_topic').value
        )
        self._output_dir = Path(str(self.get_parameter('output_dir').value))
        self._prefix = str(self.get_parameter('prefix').value)
        self._timeout_sec = float(self.get_parameter('capture_timeout_sec').value)

        self._color: Image | None = None
        self._depth: Image | None = None
        self._color_info: CameraInfo | None = None
        self._depth_info: CameraInfo | None = None
        self._saved = False
        self._start_time = self.get_clock().now()

        self.create_subscription(Image, self._color_topic, self._handle_color, 10)
        self.create_subscription(Image, self._depth_topic, self._handle_depth, 10)
        self.create_subscription(
            CameraInfo,
            self._color_info_topic,
            self._handle_color_info,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self._depth_info_topic,
            self._handle_depth_info,
            10,
        )
        self.create_timer(0.2, self._maybe_save)

        self.get_logger().info(f'等待 RGB 图像：{self._color_topic}')
        self.get_logger().info(f'等待对齐深度图：{self._depth_topic}')
        self.get_logger().info(f'等待 RGB 内参：{self._color_info_topic}')
        self.get_logger().info(f'等待深度内参：{self._depth_info_topic}')
        self.get_logger().info(f'样本输出目录：{self._output_dir}')

    def _handle_color(self, message: Image) -> None:
        self._color = message

    def _handle_depth(self, message: Image) -> None:
        self._depth = message

    def _handle_color_info(self, message: CameraInfo) -> None:
        self._color_info = message

    def _handle_depth_info(self, message: CameraInfo) -> None:
        self._depth_info = message

    def _maybe_save(self) -> None:
        """数据齐全后保存样本；超时则提示缺少哪一路。"""
        if self._saved:
            return

        if self._ready:
            self._save()
            self._saved = True
            rclpy.shutdown()
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed > self._timeout_sec:
            missing = []
            if self._color is None:
                missing.append('RGB 图像')
            if self._depth is None:
                missing.append('深度图像')
            if self._color_info is None:
                missing.append('RGB 内参')
            if self._depth_info is None:
                missing.append('深度内参')
            self.get_logger().error('采集超时，缺少：' + '、'.join(missing))
            self._saved = True
            rclpy.shutdown()

    @property
    def _ready(self) -> bool:
        return (
            self._color is not None
            and self._depth is not None
            and self._color_info is not None
            and self._depth_info is not None
        )

    def _save(self) -> None:
        assert self._color is not None
        assert self._depth is not None
        assert self._color_info is not None
        assert self._depth_info is not None

        self._output_dir.mkdir(parents=True, exist_ok=True)
        sample_id = self._next_sample_id()
        stem = f'{self._prefix}_{sample_id:03d}'

        color_path = self._output_dir / f'{stem}_color.png'
        depth_path = self._output_dir / f'{stem}_depth.png'
        metadata_path = self._output_dir / f'{stem}_metadata.json'

        color = _image_to_color_array(self._color)
        depth = _image_to_depth_array(self._depth)
        cv2.imwrite(str(color_path), color)
        cv2.imwrite(str(depth_path), depth)

        metadata = {
            'sample_id': sample_id,
            'color_image': color_path.name,
            'depth_image': depth_path.name,
            'color_topic': self._color_topic,
            'depth_topic': self._depth_topic,
            'color_header': _header_to_dict(self._color),
            'depth_header': _header_to_dict(self._depth),
            'color_camera_info': _camera_info_to_dict(self._color_info),
            'depth_camera_info': _camera_info_to_dict(self._depth_info),
        }
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )

        self.get_logger().info(f'已保存 RGB：{color_path}')
        self.get_logger().info(f'已保存深度：{depth_path}')
        self.get_logger().info(f'已保存元数据：{metadata_path}')

    def _next_sample_id(self) -> int:
        existing = sorted(self._output_dir.glob(f'{self._prefix}_*_color.png'))
        if not existing:
            return 1

        max_id = 0
        for path in existing:
            parts = path.stem.split('_')
            if len(parts) < 2:
                continue
            try:
                max_id = max(max_id, int(parts[-2]))
            except ValueError:
                continue
        return max_id + 1


def _image_to_color_array(message: Image) -> np.ndarray:
    """把 ROS 彩色图像消息转换为 OpenCV 可保存的 BGR/RGBA 数组。"""
    encoding = message.encoding.lower()
    channels = {
        'rgb8': 3,
        'bgr8': 3,
        'rgba8': 4,
        'bgra8': 4,
    }.get(encoding)
    if channels is None:
        raise ValueError(f'暂不支持彩色图编码：{message.encoding}')

    row_bytes = message.width * channels
    raw = np.frombuffer(message.data, dtype=np.uint8)
    image = raw.reshape((message.height, message.step))[:, :row_bytes]
    image = image.reshape((message.height, message.width, channels))
    if encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
    return image.copy()


def _image_to_depth_array(message: Image) -> np.ndarray:
    """把 ROS 深度图像消息转换为可保存的 16 位 PNG 数组，单位 mm。"""
    encoding = message.encoding.lower()
    if encoding in ('16uc1', 'mono16'):
        row_values = message.step // 2
        raw = np.frombuffer(message.data, dtype='<u2')
        image = raw.reshape((message.height, row_values))[:, :message.width]
        return image.copy()
    if encoding == '32fc1':
        row_values = message.step // 4
        raw = np.frombuffer(message.data, dtype='<f4')
        image_m = raw.reshape((message.height, row_values))[:, :message.width]
        image_mm = np.nan_to_num(image_m * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
        return np.clip(image_mm, 0, np.iinfo(np.uint16).max).astype(np.uint16)
    raise ValueError(f'暂不支持深度图编码：{message.encoding}')


def _header_to_dict(message: Image) -> dict[str, object]:
    return {
        'stamp': {
            'sec': int(message.header.stamp.sec),
            'nanosec': int(message.header.stamp.nanosec),
        },
        'frame_id': message.header.frame_id,
        'width': int(message.width),
        'height': int(message.height),
        'encoding': message.encoding,
        'step': int(message.step),
    }


def _camera_info_to_dict(message: CameraInfo) -> dict[str, object]:
    return {
        'stamp': {
            'sec': int(message.header.stamp.sec),
            'nanosec': int(message.header.stamp.nanosec),
        },
        'frame_id': message.header.frame_id,
        'width': int(message.width),
        'height': int(message.height),
        'distortion_model': message.distortion_model,
        'd': [float(value) for value in message.d],
        'k': [float(value) for value in message.k],
        'r': [float(value) for value in message.r],
        'p': [float(value) for value in message.p],
    }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RgbdSampleCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
