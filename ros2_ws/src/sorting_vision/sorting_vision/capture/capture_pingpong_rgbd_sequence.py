"""交互式 RGB-D 连续采样节点。

启动一次后持续订阅 RGB、对齐深度和 CameraInfo。终端每按一次 Enter，
保存一组样本；输入 q 后退出。用于现场采集乒乓球 RGBD 调试数据，
避免反复启动 capture_rgbd_sample 和手工改目录/文件名。
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import json
import queue
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from sorting_vision.capture.capture_rgbd_sample import (
    _camera_info_to_dict,
    _header_to_dict,
    _image_to_color_array,
    _image_to_depth_array,
)


class PingpongRgbdSequenceCapture(Node):
    """按 Enter 连续保存 RGB-D 样本。"""

    def __init__(self) -> None:
        super().__init__('capture_pingpong_rgbd_sequence')

        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_image_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter('color_camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter(
            'depth_camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('output_dir', '')
        self.declare_parameter('prefix', 'sample')
        self.declare_parameter('target_count', 20)
        self.declare_parameter('capture_timeout_sec', 10.0)

        self._color_topic = str(self.get_parameter('color_image_topic').value)
        self._depth_topic = str(self.get_parameter('depth_image_topic').value)
        self._color_info_topic = str(self.get_parameter('color_camera_info_topic').value)
        self._depth_info_topic = str(self.get_parameter('depth_camera_info_topic').value)
        self._prefix = str(self.get_parameter('prefix').value)
        self._target_count = max(1, int(self.get_parameter('target_count').value))
        self._timeout_sec = max(1.0, float(self.get_parameter('capture_timeout_sec').value))

        output_dir = str(self.get_parameter('output_dir').value).strip()
        if output_dir:
            self._output_dir = Path(output_dir).expanduser()
        else:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._output_dir = Path.home() / 'sorting_robot' / 'samples' / 'debug_' / 'runs' / f'pingpong_rgbd_{stamp}'

        self._lock = threading.Lock()
        self._commands: queue.Queue[str] = queue.Queue()
        self._color: Image | None = None
        self._depth: Image | None = None
        self._color_info: CameraInfo | None = None
        self._depth_info: CameraInfo | None = None
        self._saved_count = 0
        self._last_ready_log = 0.0

        self.create_subscription(Image, self._color_topic, self._handle_color, 10)
        self.create_subscription(Image, self._depth_topic, self._handle_depth, 10)
        self.create_subscription(CameraInfo, self._color_info_topic, self._handle_color_info, 10)
        self.create_subscription(CameraInfo, self._depth_info_topic, self._handle_depth_info, 10)
        self.create_timer(0.1, self._process_commands)
        self.create_timer(1.0, self._log_waiting_status)

        self._input_thread = threading.Thread(target=self._read_commands, daemon=True)
        self._input_thread.start()

        self.get_logger().info(f'输出目录：{self._output_dir}')
        self.get_logger().info(f'目标采样数：{self._target_count}')
        self.get_logger().info(f'RGB 图像：{self._color_topic}')
        self.get_logger().info(f'对齐深度图：{self._depth_topic}')
        self.get_logger().info(f'RGB 内参：{self._color_info_topic}')
        self.get_logger().info(f'深度内参：{self._depth_info_topic}')
        self.get_logger().info('摆好球后按 Enter 保存一组；输入 q 后回车退出。')

    def _handle_color(self, message: Image) -> None:
        with self._lock:
            self._color = message

    def _handle_depth(self, message: Image) -> None:
        with self._lock:
            self._depth = message

    def _handle_color_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._color_info = message

    def _handle_depth_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._depth_info = message

    def _read_commands(self) -> None:
        while rclpy.ok():
            try:
                command = input('[采样] Enter=保存下一组，q=退出 > ').strip().lower()
            except EOFError:
                command = 'q'
            self._commands.put(command)
            if command in ('q', 'quit', 'exit'):
                return

    def _process_commands(self) -> None:
        while not self._commands.empty():
            command = self._commands.get()
            if command in ('q', 'quit', 'exit'):
                self.get_logger().info('收到退出命令。')
                rclpy.shutdown()
                return
            self._capture_once()

    def _capture_once(self) -> None:
        started = time.monotonic()
        while rclpy.ok():
            snapshot = self._snapshot()
            missing = self._missing(snapshot)
            if not missing:
                break
            if time.monotonic() - started > self._timeout_sec:
                self.get_logger().error('采集超时，缺少：' + '、'.join(missing))
                return
            time.sleep(0.05)

        color, depth, color_info, depth_info = snapshot
        assert color is not None
        assert depth is not None
        assert color_info is not None
        assert depth_info is not None

        self._output_dir.mkdir(parents=True, exist_ok=True)
        sample_id = self._next_sample_id()
        stem = f'{self._prefix}_{sample_id:03d}'
        color_path = self._output_dir / f'{stem}_color.png'
        depth_path = self._output_dir / f'{stem}_depth.png'
        metadata_path = self._output_dir / f'{stem}_metadata.json'

        cv2.imwrite(str(color_path), _image_to_color_array(color))
        cv2.imwrite(str(depth_path), _image_to_depth_array(depth))
        metadata = {
            'sample_id': sample_id,
            'color_image': color_path.name,
            'depth_image': depth_path.name,
            'color_topic': self._color_topic,
            'depth_topic': self._depth_topic,
            'color_header': _header_to_dict(color),
            'depth_header': _header_to_dict(depth),
            'color_camera_info': _camera_info_to_dict(color_info),
            'depth_camera_info': _camera_info_to_dict(depth_info),
        }
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )

        self._saved_count += 1
        self.get_logger().info(f'[{self._saved_count}/{self._target_count}] 已保存：{stem}')
        self.get_logger().info(f'  RGB ：{color_path}')
        self.get_logger().info(f'  深度：{depth_path}')
        if self._saved_count >= self._target_count:
            self.get_logger().info('已达到目标采样数，自动退出。')
            rclpy.shutdown()

    def _snapshot(self) -> tuple[Image | None, Image | None, CameraInfo | None, CameraInfo | None]:
        with self._lock:
            return self._color, self._depth, self._color_info, self._depth_info

    @staticmethod
    def _missing(
        snapshot: tuple[Image | None, Image | None, CameraInfo | None, CameraInfo | None],
    ) -> list[str]:
        color, depth, color_info, depth_info = snapshot
        missing = []
        if color is None:
            missing.append('RGB 图像')
        if depth is None:
            missing.append('深度图像')
        if color_info is None:
            missing.append('RGB 内参')
        if depth_info is None:
            missing.append('深度内参')
        return missing

    def _log_waiting_status(self) -> None:
        snapshot = self._snapshot()
        missing = self._missing(snapshot)
        now = time.monotonic()
        if missing and now - self._last_ready_log > 5.0:
            self.get_logger().warning('仍在等待：' + '、'.join(missing))
            self._last_ready_log = now

    def _next_sample_id(self) -> int:
        existing = sorted(self._output_dir.glob(f'{self._prefix}_*_color.png'))
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


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PingpongRgbdSequenceCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
