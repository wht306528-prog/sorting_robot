"""采集实时乒乓球识别调试样本。

这个工具用于鲁班猫现场测试时保存一组可复盘数据：
- 原始 RGB 图。
- 识别 debug 图。
- cells_json。
- TrayMatrix。

保存下来的样本可以在笔记本上离线分析，避免只看 X11 窗口凭感觉调算法。
"""

from __future__ import annotations

from pathlib import Path
import json
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sorting_interfaces.msg import TrayMatrix
from sorting_vision.debug_tools.opencv_image_viewer import image_message_to_bgr


class PingpongDebugSampleCapture(Node):
    """订阅实时识别 topic，并按间隔保存多组调试样本。"""

    def __init__(self) -> None:
        super().__init__('capture_pingpong_debug_sample')

        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('debug_image_topic', '/sorting/pingpong/debug_image')
        self.declare_parameter('cells_json_topic', '/sorting/pingpong/cells_json')
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('output_dir', 'samples/debug_/runs/pingpong_realtime_capture')
        self.declare_parameter('sample_count', 20)
        self.declare_parameter('interval_sec', 0.5)
        self.declare_parameter('timeout_sec', 20.0)

        self._color_topic = str(self.get_parameter('color_image_topic').value)
        self._debug_topic = str(self.get_parameter('debug_image_topic').value)
        self._cells_topic = str(self.get_parameter('cells_json_topic').value)
        self._matrix_topic = str(self.get_parameter('tray_matrix_topic').value)
        self._output_dir = Path(str(self.get_parameter('output_dir').value))
        self._sample_count = max(1, int(self.get_parameter('sample_count').value))
        self._interval_sec = max(0.1, float(self.get_parameter('interval_sec').value))
        self._timeout_sec = max(1.0, float(self.get_parameter('timeout_sec').value))

        self._latest_color: Image | None = None
        self._latest_debug: Image | None = None
        self._latest_cells: String | None = None
        self._latest_matrix: TrayMatrix | None = None
        self._saved_count = 0
        self._last_save_time = 0.0
        self._start_time = time.monotonic()

        self.create_subscription(Image, self._color_topic, self._handle_color, 10)
        self.create_subscription(Image, self._debug_topic, self._handle_debug, 10)
        self.create_subscription(String, self._cells_topic, self._handle_cells, 10)
        self.create_subscription(TrayMatrix, self._matrix_topic, self._handle_matrix, 10)
        self.create_timer(0.1, self._maybe_save)

        self.get_logger().info(f'等待原始 RGB 图：{self._color_topic}')
        self.get_logger().info(f'等待 debug 图：{self._debug_topic}')
        self.get_logger().info(f'等待 cells_json：{self._cells_topic}')
        self.get_logger().info(f'等待 TrayMatrix：{self._matrix_topic}')
        self.get_logger().info(f'输出目录：{self._output_dir}')
        self.get_logger().info(f'计划采集 {self._sample_count} 组，间隔 {self._interval_sec:.2f}s')

    def _handle_color(self, message: Image) -> None:
        self._latest_color = message

    def _handle_debug(self, message: Image) -> None:
        self._latest_debug = message

    def _handle_cells(self, message: String) -> None:
        self._latest_cells = message

    def _handle_matrix(self, message: TrayMatrix) -> None:
        self._latest_matrix = message

    def _maybe_save(self) -> None:
        """数据齐全且达到间隔后保存一组样本。"""

        if self._saved_count >= self._sample_count:
            self.get_logger().info(f'采集完成：{self._output_dir}')
            rclpy.shutdown()
            return

        elapsed = time.monotonic() - self._start_time
        if elapsed > self._timeout_sec and not self._ready:
            missing = self._missing_names()
            self.get_logger().error('采集超时，缺少：' + '、'.join(missing))
            rclpy.shutdown()
            return

        now = time.monotonic()
        if not self._ready or now - self._last_save_time < self._interval_sec:
            return

        self._saved_count += 1
        self._last_save_time = now
        self._save_sample(self._saved_count)

    @property
    def _ready(self) -> bool:
        return (
            self._latest_color is not None
            and self._latest_debug is not None
            and self._latest_cells is not None
            and self._latest_matrix is not None
        )

    def _missing_names(self) -> list[str]:
        missing = []
        if self._latest_color is None:
            missing.append('原始 RGB 图')
        if self._latest_debug is None:
            missing.append('debug 图')
        if self._latest_cells is None:
            missing.append('cells_json')
        if self._latest_matrix is None:
            missing.append('TrayMatrix')
        return missing

    def _save_sample(self, sample_index: int) -> None:
        """保存一组图像和结构化结果。"""

        assert self._latest_color is not None
        assert self._latest_debug is not None
        assert self._latest_cells is not None
        assert self._latest_matrix is not None

        sample_dir = self._output_dir / f'sample_{sample_index:03d}'
        sample_dir.mkdir(parents=True, exist_ok=True)

        color_path = sample_dir / 'color.jpg'
        debug_path = sample_dir / 'debug.jpg'
        cells_path = sample_dir / 'cells_json.json'
        matrix_path = sample_dir / 'tray_matrix.json'
        metadata_path = sample_dir / 'metadata.json'

        cv2.imwrite(str(color_path), image_message_to_bgr(self._latest_color))
        cv2.imwrite(str(debug_path), image_message_to_bgr(self._latest_debug))
        cells_data = _parse_cells_json(self._latest_cells.data)
        cells_path.write_text(
            json.dumps(cells_data, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        matrix_path.write_text(
            json.dumps(_tray_matrix_to_dict(self._latest_matrix), ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        metadata = {
            'sample_index': sample_index,
            'color_topic': self._color_topic,
            'debug_topic': self._debug_topic,
            'cells_topic': self._cells_topic,
            'matrix_topic': self._matrix_topic,
            'color_header': _image_header_to_dict(self._latest_color),
            'debug_header': _image_header_to_dict(self._latest_debug),
            'frame_id': int(self._latest_matrix.frame_id),
        }
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        self.get_logger().info(f'已保存样本 {sample_index}/{self._sample_count}: {sample_dir}')


def _parse_cells_json(data: str) -> object:
    """把 cells_json 字符串解析成 JSON；解析失败时保留原文。"""

    try:
        return json.loads(data)
    except json.JSONDecodeError:
        return {'raw': data}


def _image_header_to_dict(message: Image) -> dict[str, object]:
    """提取 Image 头信息，便于核对样本时间和尺寸。"""

    return {
        'stamp': {
            'sec': int(message.header.stamp.sec),
            'nanosec': int(message.header.stamp.nanosec),
        },
        'frame_id': message.header.frame_id,
        'width': int(message.width),
        'height': int(message.height),
        'encoding': message.encoding,
    }


def _tray_matrix_to_dict(message: TrayMatrix) -> dict[str, object]:
    """把 TrayMatrix 转成可保存 JSON。"""

    return {
        'header': {
            'stamp': {
                'sec': int(message.header.stamp.sec),
                'nanosec': int(message.header.stamp.nanosec),
            },
            'frame_id': message.header.frame_id,
        },
        'frame_id': int(message.frame_id),
        'cell_count': len(message.cells),
        'cells': [
            {
                'tray_id': int(cell.tray_id),
                'col': int(cell.col),
                'row': int(cell.row),
                'class_id': int(cell.class_id),
                'confidence': float(cell.confidence),
                'u': float(cell.u),
                'v': float(cell.v),
                'z': float(cell.z),
            }
            for cell in message.cells
        ],
    }


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = PingpongDebugSampleCapture()
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
