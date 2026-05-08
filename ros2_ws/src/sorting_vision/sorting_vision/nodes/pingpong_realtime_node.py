"""Realtime ping-pong tray classifier node."""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sorting_interfaces.msg import TrayCell, TrayMatrix
from sorting_vision.algorithms.detector import ImageView as DetectorImageView
from sorting_vision.algorithms.detector import sample_depth_mm
from sorting_vision.algorithms.pingpong_detector import (
    PingpongCell,
    PingpongDetectorConfig,
    detect_pingpong_cells,
    draw_pingpong_cells,
)
from sorting_vision.algorithms.tray_edge_fit import TrayEdgeFitConfig, fit_tray_edges
from sorting_vision.algorithms.tray_hole_grid import TrayHoleGridConfig


CLASS_IDS = {
    'empty': 0,
    'white_ball': 1,
    'yellow_ball': 2,
}


class PingpongRealtimeNode(Node):
    """Subscribe to RGB images, classify one tray, and publish debug output."""

    def __init__(self) -> None:
        super().__init__('pingpong_realtime_node')

        self._declare_parameters()
        self._color_topic = self._string_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self._depth_topic = self._string_parameter(
            'depth_image_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self._debug_topic = self._string_parameter('debug_image_topic', '/sorting/pingpong/debug_image')
        self._cells_topic = self._string_parameter('cells_json_topic', '/sorting/pingpong/cells_json')
        self._tray_matrix_topic = self._string_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self._use_depth = self._bool_parameter('use_depth', True)
        self._depth_window_px = max(1, self._int_parameter('depth_window_px', 5))
        self._active_tray_id = max(1, min(3, self._int_parameter('active_tray_id', 1)))
        self._process_every_n = max(1, self._int_parameter('process_every_n_frames', 3))
        self._log_every_sec = max(0.5, self._float_parameter('log_every_sec', 2.0))
        self._frame_index = 0
        self._last_log_time = 0.0
        self._edge_config = self._edge_config_from_parameters()
        self._pingpong_config = self._pingpong_config_from_parameters()
        self._latest_depth: Image | None = None
        self._warned_encoding = False
        self._warned_depth_waiting = False
        self._warned_depth_size = False

        self._debug_publisher = self.create_publisher(Image, self._debug_topic, 10)
        self._cells_publisher = self.create_publisher(String, self._cells_topic, 10)
        self._matrix_publisher = self.create_publisher(TrayMatrix, self._tray_matrix_topic, 10)
        self._subscription = self.create_subscription(
            Image,
            self._color_topic,
            self._handle_image,
            qos_profile_sensor_data,
        )
        self._depth_subscription = None
        if self._use_depth:
            self._depth_subscription = self.create_subscription(
                Image,
                self._depth_topic,
                self._handle_depth,
                qos_profile_sensor_data,
            )

        self.get_logger().info(f'正在订阅 RGB 图像：{self._color_topic}')
        if self._use_depth:
            self.get_logger().info(f'正在订阅对齐深度图像：{self._depth_topic}')
        else:
            self.get_logger().info('已关闭深度输入：TrayCell.z 将保持 0')
        self.get_logger().info(f'正在发布乒乓球标注图：{self._debug_topic}')
        self.get_logger().info(f'正在发布乒乓球 JSON：{self._cells_topic}')
        self.get_logger().info(f'正在发布标准 TrayMatrix：{self._tray_matrix_topic}')
        self.get_logger().info(f'当前实时识别结果写入 tray_id={self._active_tray_id}，其他苗盘补 empty')
        self.get_logger().info(f'每 {self._process_every_n} 帧处理一次')

    def _declare_parameters(self) -> None:
        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_window_px', 5)
        self.declare_parameter('debug_image_topic', '/sorting/pingpong/debug_image')
        self.declare_parameter('cells_json_topic', '/sorting/pingpong/cells_json')
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('active_tray_id', 1)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('log_every_sec', 2.0)
        self.declare_parameter('rows', 10)
        self.declare_parameter('cols', 5)
        self.declare_parameter('dark_threshold', 95)
        self.declare_parameter('close_kernel_ratio', 0.045)
        self.declare_parameter('min_area_ratio', 0.16)
        self.declare_parameter('side_band_ratio', 0.075)
        self.declare_parameter('roi_radius_ratio', 0.34)
        self.declare_parameter('min_ball_ratio', 0.16)
        self.declare_parameter('min_white_ratio', 0.30)
        self.declare_parameter('min_color_margin', 0.035)

    def _handle_depth(self, message: Image) -> None:
        self._latest_depth = message

    def _edge_config_from_parameters(self) -> TrayEdgeFitConfig:
        return TrayEdgeFitConfig(
            dark_threshold=self._int_parameter('dark_threshold', 95),
            close_kernel_ratio=self._float_parameter('close_kernel_ratio', 0.045),
            min_area_ratio=self._float_parameter('min_area_ratio', 0.16),
            side_band_ratio=self._float_parameter('side_band_ratio', 0.075),
        )

    def _pingpong_config_from_parameters(self) -> PingpongDetectorConfig:
        return PingpongDetectorConfig(
            roi_radius_ratio=self._float_parameter('roi_radius_ratio', 0.34),
            min_ball_ratio=self._float_parameter('min_ball_ratio', 0.16),
            min_white_ratio=self._float_parameter('min_white_ratio', 0.30),
            min_color_margin=self._float_parameter('min_color_margin', 0.035),
            hole_grid=TrayHoleGridConfig(
                rows=self._int_parameter('rows', 10),
                cols=self._int_parameter('cols', 5),
            ),
        )

    def _handle_image(self, message: Image) -> None:
        self._frame_index += 1
        if self._frame_index % self._process_every_n != 0:
            return

        image = ros_image_to_bgr(message)
        if image is None:
            if not self._warned_encoding:
                self.get_logger().warning(f'暂不支持图像编码：{message.encoding}')
                self._warned_encoding = True
            return

        edge_result = fit_tray_edges(image, self._edge_config)
        if edge_result.status == 'ok' and edge_result.rectified is not None:
            detection_result = detect_pingpong_cells(edge_result.rectified, self._pingpong_config)
            debug_image = draw_pingpong_cells(edge_result.rectified, detection_result)
            depth_by_position, source_by_position = self._sample_cell_depths(
                source_message=message,
                cells=detection_result.cells,
                corners=edge_result.corners,
            )
            payload = self._payload(
                message,
                edge_result.status,
                edge_result.message,
                detection_result.cells,
                depth_by_position,
                source_by_position,
            )
        else:
            debug_image = image.copy()
            self._draw_failure(debug_image, edge_result.message)
            payload = self._payload(message, edge_result.status, edge_result.message, [])

        self._publish_debug_image(debug_image, message)
        self._publish_cells(payload)
        self._publish_tray_matrix(message, payload)
        self._log_summary(payload)

    def _publish_debug_image(self, image, source_message: Image) -> None:
        debug_message = bgr_to_ros_image(image, source_message)
        debug_message.header = source_message.header
        self._debug_publisher.publish(debug_message)

    def _publish_cells(self, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._cells_publisher.publish(message)

    def _publish_tray_matrix(self, source_message: Image, payload: dict[str, object]) -> None:
        matrix = TrayMatrix()
        matrix.header = source_message.header
        matrix.frame_id = int(payload['frame_index'])
        matrix.cells = tray_matrix_cells(
            payload_cells=payload['cells'],
            active_tray_id=self._active_tray_id,
        )
        self._matrix_publisher.publish(matrix)

    def _payload(
        self,
        message: Image,
        status: str,
        status_message: str,
        cells,
        depth_by_position: dict[tuple[int, int], float] | None = None,
        source_by_position: dict[tuple[int, int], tuple[float, float]] | None = None,
    ) -> dict[str, object]:
        depth_by_position = depth_by_position or {}
        source_by_position = source_by_position or {}
        counts = {
            'yellow_ball': sum(1 for cell in cells if cell.class_name == 'yellow_ball'),
            'white_ball': sum(1 for cell in cells if cell.class_name == 'white_ball'),
            'empty': sum(1 for cell in cells if cell.class_name == 'empty'),
        }
        rows = self._int_parameter('rows', 10)
        cols = self._int_parameter('cols', 5)
        return {
            'frame_index': self._frame_index,
            'stamp': {
                'sec': int(message.header.stamp.sec),
                'nanosec': int(message.header.stamp.nanosec),
            },
            'status': status,
            'message': status_message,
            'counts': counts,
            'class_ids': CLASS_IDS,
            'matrix': class_matrix(cells, rows, cols),
            'matrix_ids': class_id_matrix(cells, rows, cols),
            'cells': [
                {
                    'row': cell.row,
                    'col': cell.col,
                    'class_name': cell.class_name,
                    'class_id': CLASS_IDS.get(cell.class_name, -1),
                    'confidence': round(cell.confidence, 3),
                    'u_rect': round(cell.u, 3),
                    'v_rect': round(cell.v, 3),
                    'u_source': round(source_by_position.get((cell.row, cell.col), (0.0, 0.0))[0], 3),
                    'v_source': round(source_by_position.get((cell.row, cell.col), (0.0, 0.0))[1], 3),
                    'z_mm': round(depth_by_position.get((cell.row, cell.col), 0.0), 3),
                }
                for cell in cells
            ],
        }

    def _sample_cell_depths(
        self,
        source_message: Image,
        cells: list[PingpongCell],
        corners,
    ) -> tuple[dict[tuple[int, int], float], dict[tuple[int, int], tuple[float, float]]]:
        if not self._use_depth:
            return {}, {}
        if self._latest_depth is None:
            if not self._warned_depth_waiting:
                self.get_logger().warning('尚未收到深度图像，本帧 TrayCell.z=0')
                self._warned_depth_waiting = True
            return {}, {}
        if corners is None:
            return {}, {}
        if (
            self._latest_depth.width != source_message.width
            or self._latest_depth.height != source_message.height
        ):
            if not self._warned_depth_size:
                self.get_logger().warning(
                    '深度图尺寸与 RGB 图不一致，本帧不采样 z。'
                    '请使用对齐到 RGB 的深度图 topic。'
                )
                self._warned_depth_size = True
            return {}, {}

        source_points = rectified_cells_to_source_points(cells, corners, self._edge_config)
        depth_view = ros_depth_to_image_view(self._latest_depth)
        depth_by_position = {
            position: sample_depth_mm(depth_view, point[0], point[1], self._depth_window_px)
            for position, point in source_points.items()
        }
        return depth_by_position, source_points

    def _log_summary(self, payload: dict[str, object]) -> None:
        now = time.monotonic()
        if now - self._last_log_time < self._log_every_sec:
            return
        self._last_log_time = now
        counts = payload['counts']
        self.get_logger().info(
            'frame={frame} status={status} yellow={yellow} white={white} empty={empty}'.format(
                frame=payload['frame_index'],
                status=payload['status'],
                yellow=counts['yellow_ball'],
                white=counts['white_ball'],
                empty=counts['empty'],
            )
        )

    @staticmethod
    def _draw_failure(image, message: str) -> None:
        import cv2

        cv2.putText(
            image,
            f'edge failed: {message[:80]}',
            (18, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _string_parameter(self, name: str, default: str) -> str:
        value = self.get_parameter(name).value
        return str(value) if value is not None else default

    def _float_parameter(self, name: str, default: float) -> float:
        value = self.get_parameter(name).value
        return float(value) if value is not None else default

    def _int_parameter(self, name: str, default: int) -> int:
        value = self.get_parameter(name).value
        return int(value) if value is not None else default

    def _bool_parameter(self, name: str, default: bool) -> bool:
        value = self.get_parameter(name).value
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ('1', 'true', 'yes', 'on'):
                return True
            if normalized in ('0', 'false', 'no', 'off'):
                return False
        return bool(value)


def ros_image_to_bgr(message: Image) -> np.ndarray | None:
    if message.encoding not in ('bgr8', 'rgb8'):
        return None
    expected_step = message.width * 3
    array = np.frombuffer(message.data, dtype=np.uint8)
    if message.step < expected_step or len(array) < message.step * message.height:
        return None
    rows = array.reshape((message.height, message.step))
    compact = rows[:, :expected_step].reshape((message.height, message.width, 3))
    image = compact.copy()
    if message.encoding == 'rgb8':
        image = image[:, :, ::-1].copy()
    return image


def bgr_to_ros_image(image: np.ndarray, source_message: Image) -> Image:
    output = Image()
    output.header = source_message.header
    output.height = int(image.shape[0])
    output.width = int(image.shape[1])
    output.encoding = 'bgr8'
    output.is_bigendian = 0
    output.step = int(image.shape[1] * 3)
    output.data = image.astype(np.uint8).tobytes()
    return output


def ros_depth_to_image_view(message: Image) -> DetectorImageView:
    return DetectorImageView(
        width=message.width,
        height=message.height,
        encoding=message.encoding,
        step=message.step,
        data=bytes(message.data),
    )


def rectified_cells_to_source_points(
    cells: list[PingpongCell],
    corners: np.ndarray,
    config: TrayEdgeFitConfig,
) -> dict[tuple[int, int], tuple[float, float]]:
    import cv2

    padding = config.rectified_padding
    target = np.asarray(
        [
            [float(padding), float(padding)],
            [float(padding + config.rectified_width - 1), float(padding)],
            [float(padding + config.rectified_width - 1), float(padding + config.rectified_height - 1)],
            [float(padding), float(padding + config.rectified_height - 1)],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(target, corners.astype(np.float32))
    rectified_points = np.asarray([[[float(cell.u), float(cell.v)]] for cell in cells], dtype=np.float32)
    if len(rectified_points) == 0:
        return {}
    source_points = cv2.perspectiveTransform(rectified_points, matrix).reshape(-1, 2)
    return {
        (cell.row, cell.col): (float(point[0]), float(point[1]))
        for cell, point in zip(cells, source_points)
    }


def class_matrix(cells, rows: int, cols: int) -> list[list[str]]:
    matrix = [['unknown' for _col in range(cols)] for _row in range(rows)]
    for cell in cells:
        if 1 <= cell.row <= rows and 1 <= cell.col <= cols:
            matrix[cell.row - 1][cell.col - 1] = cell.class_name
    return matrix


def class_id_matrix(cells, rows: int, cols: int) -> list[list[int]]:
    matrix = [[-1 for _col in range(cols)] for _row in range(rows)]
    for cell in cells:
        if 1 <= cell.row <= rows and 1 <= cell.col <= cols:
            matrix[cell.row - 1][cell.col - 1] = CLASS_IDS.get(cell.class_name, -1)
    return matrix


def tray_matrix_cells(payload_cells: list[dict[str, object]], active_tray_id: int) -> list[TrayCell]:
    by_position = {
        (int(cell['row']), int(cell['col'])): cell
        for cell in payload_cells
    }
    cells: list[TrayCell] = []
    for tray_id in range(1, 4):
        for row in range(1, 11):
            for col in range(1, 6):
                source = by_position.get((row, col)) if tray_id == active_tray_id else None
                cell = TrayCell()
                cell.tray_id = tray_id
                cell.col = col
                cell.row = row
                if source is None:
                    cell.class_id = CLASS_IDS['empty']
                    cell.confidence = 1.0 if tray_id != active_tray_id else 0.0
                    cell.u = 0.0
                    cell.v = 0.0
                    cell.z = 0.0
                else:
                    cell.class_id = int(source['class_id'])
                    cell.confidence = float(source['confidence'])
                    cell.u = float(source['u_rect'])
                    cell.v = float(source['v_rect'])
                    cell.z = float(source.get('z_mm', 0.0))
                cells.append(cell)
    return cells


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PingpongRealtimeNode()
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
