"""实时乒乓球苗盘识别节点。"""

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
from sorting_vision.algorithms.tray_edge_fit import TrayEdgeFitConfig, rectify
from sorting_vision.algorithms.tray_geometry import TrayGeometryConfig, detect_tray_geometry
from sorting_vision.algorithms.tray_hole_grid import TrayHoleGridConfig


CLASS_IDS = {
    'empty': 0,
    'white_ball': 1,
    'yellow_ball': 2,
}


class PingpongRealtimeNode(Node):
    """订阅 RGB/可选深度图，识别画面中的多个乒乓球苗盘并发布矩阵。"""

    def __init__(self) -> None:
        super().__init__('pingpong_realtime_node')

        self._declare_parameters()
        # 这些 topic/开关由 launch 或现场 env 注入，避免换相机时改 Python 代码。
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
        self._expected_tray_count = max(1, min(3, self._int_parameter('expected_tray_count', 3)))
        self._process_every_n = max(1, self._int_parameter('process_every_n_frames', 3))
        self._log_every_sec = max(0.5, self._float_parameter('log_every_sec', 2.0))
        self._frame_index = 0
        self._last_log_time = 0.0
        # 三组配置分别对应：整帧找多个盘、单盘透视矫正、单穴位颜色分类。
        self._geometry_config = self._geometry_config_from_parameters()
        self._edge_config = self._edge_config_from_parameters()
        self._pingpong_config = self._pingpong_config_from_parameters()
        # 深度是可选输入；普通 USB/RGB 相机没有深度时 z 保持 0。
        self._latest_depth: Image | None = None
        self._warned_encoding = False
        self._warned_depth_waiting = False
        self._warned_depth_size = False

        # debug_image 给人看，cells_json 给终端/日志看，TrayMatrix 给 F407/TCP 链路用。
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
            # 这里要求深度图已经对齐到 RGB，否则同一个像素位置不能直接采样 z。
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
        self.get_logger().info(f'当前按整帧检测最多 {self._expected_tray_count} 个苗盘，左到右写入 tray_id')
        self.get_logger().info(f'每 {self._process_every_n} 帧处理一次')

    def _declare_parameters(self) -> None:
        # ROS 参数分三类：输入输出 topic、实时性能开关、视觉阈值。
        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_window_px', 5)
        self.declare_parameter('debug_image_topic', '/sorting/pingpong/debug_image')
        self.declare_parameter('cells_json_topic', '/sorting/pingpong/cells_json')
        self.declare_parameter('tray_matrix_topic', '/sorting/tray_matrix')
        self.declare_parameter('expected_tray_count', 3)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('log_every_sec', 2.0)
        self.declare_parameter('rows', 10)
        self.declare_parameter('cols', 5)
        self.declare_parameter('geometry_dark_threshold', 85)
        self.declare_parameter('geometry_projection_active_ratio', 0.10)
        self.declare_parameter('geometry_projection_smooth_px', 19)
        self.declare_parameter('geometry_min_width_ratio', 0.08)
        self.declare_parameter('geometry_min_height_ratio', 0.35)
        self.declare_parameter('geometry_min_area_ratio', 0.025)
        self.declare_parameter('geometry_x_padding_px', 10)
        self.declare_parameter('geometry_morphology_kernel_px', 17)
        self.declare_parameter('geometry_edge_roi_padding_px', 12)
        self.declare_parameter('dark_threshold', 95)
        self.declare_parameter('close_kernel_ratio', 0.045)
        self.declare_parameter('min_area_ratio', 0.16)
        self.declare_parameter('side_band_ratio', 0.075)
        self.declare_parameter('roi_radius_ratio', 0.34)
        self.declare_parameter('min_ball_ratio', 0.16)
        self.declare_parameter('min_white_ratio', 0.30)
        self.declare_parameter('min_color_margin', 0.035)

    def _handle_depth(self, message: Image) -> None:
        """保存最新一帧对齐深度图。"""
        self._latest_depth = message

    def _geometry_config_from_parameters(self) -> TrayGeometryConfig:
        """从 ROS 参数生成整帧多盘几何检测配置。"""
        return TrayGeometryConfig(
            expected_tray_count=self._expected_tray_count,
            dark_threshold=self._int_parameter('geometry_dark_threshold', 85),
            projection_active_ratio=self._float_parameter('geometry_projection_active_ratio', 0.10),
            projection_smooth_px=self._int_parameter('geometry_projection_smooth_px', 19),
            min_width_ratio=self._float_parameter('geometry_min_width_ratio', 0.08),
            min_height_ratio=self._float_parameter('geometry_min_height_ratio', 0.35),
            min_area_ratio=self._float_parameter('geometry_min_area_ratio', 0.025),
            x_padding_px=self._int_parameter('geometry_x_padding_px', 10),
            morphology_kernel_px=self._int_parameter('geometry_morphology_kernel_px', 17),
            edge_roi_padding_px=self._int_parameter('geometry_edge_roi_padding_px', 12),
        )

    def _edge_config_from_parameters(self) -> TrayEdgeFitConfig:
        """从 ROS 参数生成单盘透视矫正配置。"""
        return TrayEdgeFitConfig(
            dark_threshold=self._int_parameter('dark_threshold', 95),
            close_kernel_ratio=self._float_parameter('close_kernel_ratio', 0.045),
            min_area_ratio=self._float_parameter('min_area_ratio', 0.16),
            side_band_ratio=self._float_parameter('side_band_ratio', 0.075),
        )

    def _pingpong_config_from_parameters(self) -> PingpongDetectorConfig:
        """从 ROS 参数生成乒乓球颜色分类配置。"""
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
        """处理一帧 RGB 图像：找盘、矫正、识别、发布调试图和矩阵。"""
        self._frame_index += 1
        # 降频处理，避免低算力板子上每帧都跑视觉造成延迟堆积。
        if self._frame_index % self._process_every_n != 0:
            return

        image = ros_image_to_bgr(message)
        if image is None:
            if not self._warned_encoding:
                self.get_logger().warning(f'暂不支持图像编码：{message.encoding}')
                self._warned_encoding = True
            return

        # 第一步在整帧里找最多 3 个苗盘候选，并按从左到右分配 tray_id。
        geometry_result = detect_tray_geometry(image, self._geometry_config)
        debug_image = image.copy()
        payload_cells: list[dict[str, object]] = []
        detected_tray_ids: set[int] = set()

        for candidate in geometry_result.candidates:
            if candidate.corners is None:
                # 粗定位到了候选但没拟合出四角时，只画失败框，不写该盘真实识别。
                self._draw_candidate(debug_image, candidate.tray_id, candidate.bbox, None, candidate.status)
                continue

            # 每个盘单独透视矫正，再复用单盘 10x5 穴位分类逻辑。
            rectified = rectify(image, candidate.corners, self._edge_config)
            detection_result = detect_pingpong_cells(rectified, self._pingpong_config)
            # 深度采样必须从“矫正图穴位中心”反算回“原始 RGB 坐标”后再取 z。
            depth_by_position, source_by_position = self._sample_cell_depths(
                source_message=message,
                cells=detection_result.cells,
                corners=candidate.corners,
            )
            payload_cells.extend(
                self._payload_cells_for_tray(
                    tray_id=candidate.tray_id,
                    cells=detection_result.cells,
                    depth_by_position=depth_by_position,
                    source_by_position=source_by_position,
                )
            )
            detected_tray_ids.add(candidate.tray_id)
            # 调试图保留整帧上的盘框，不把透视矫正图直接发布出去。
            self._draw_candidate(debug_image, candidate.tray_id, candidate.bbox, candidate.corners, detection_result.status)

        if not payload_cells:
            # 完全没有可发布的真实穴位时，debug 图上直接写失败原因。
            self._draw_failure(debug_image, geometry_result.message)

        payload = self._payload(
            message=message,
            status=geometry_result.status,
            status_message=geometry_result.message,
            cells=payload_cells,
            detected_tray_ids=detected_tray_ids,
        )

        self._publish_debug_image(debug_image, message)
        self._publish_cells(payload)
        self._publish_tray_matrix(message, payload)
        self._log_summary(payload)

    def _publish_debug_image(self, image, source_message: Image) -> None:
        """发布带有盘位置和识别状态的调试图。"""
        debug_message = bgr_to_ros_image(image, source_message)
        debug_message.header = source_message.header
        self._debug_publisher.publish(debug_message)

    def _publish_cells(self, payload: dict[str, object]) -> None:
        """发布便于终端查看的 JSON 识别结果。"""
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._cells_publisher.publish(message)

    def _publish_tray_matrix(self, source_message: Image, payload: dict[str, object]) -> None:
        """发布给 F407/TCP 链路使用的标准 150 格 TrayMatrix。"""
        matrix = TrayMatrix()
        matrix.header = source_message.header
        matrix.frame_id = int(payload['frame_index'])
        # 下游协议固定 3*10*5=150 格；没检测到的盘也要补齐，不能变长。
        matrix.cells = tray_matrix_cells(
            payload_cells=payload['cells'],
            detected_tray_ids=set(payload['detected_tray_ids']),
        )
        self._matrix_publisher.publish(matrix)

    def _payload(
        self,
        message: Image,
        status: str,
        status_message: str,
        cells: list[dict[str, object]],
        detected_tray_ids: set[int] | None = None,
    ) -> dict[str, object]:
        """组织 JSON 载荷，按 tray_id 保留每个盘的识别矩阵。"""
        detected_tray_ids = detected_tray_ids or set()
        # counts 是全画面总数，matrices_by_tray 才是按 1/2/3 号盘展开的结果。
        counts = {
            'yellow_ball': sum(1 for cell in cells if cell['class_name'] == 'yellow_ball'),
            'white_ball': sum(1 for cell in cells if cell['class_name'] == 'white_ball'),
            'empty': sum(1 for cell in cells if cell['class_name'] == 'empty'),
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
            'detected_tray_ids': sorted(detected_tray_ids),
            'matrices_by_tray': class_matrices_by_tray(cells, rows, cols),
            'matrix_ids_by_tray': class_id_matrices_by_tray(cells, rows, cols),
            'cells': cells,
        }

    @staticmethod
    def _payload_cells_for_tray(
        tray_id: int,
        cells: list[PingpongCell],
        depth_by_position: dict[tuple[int, int], float],
        source_by_position: dict[tuple[int, int], tuple[float, float]],
    ) -> list[dict[str, object]]:
        """把单盘识别结果转换成带 tray_id 的字典列表。"""
        output: list[dict[str, object]] = []
        for cell in cells:
            # u_rect/v_rect 是矫正图坐标；u_source/v_source 是原始相机图坐标。
            source_u, source_v = source_by_position.get((cell.row, cell.col), (0.0, 0.0))
            output.append(
                {
                    'tray_id': tray_id,
                    'row': cell.row,
                    'col': cell.col,
                    'class_name': cell.class_name,
                    'class_id': CLASS_IDS.get(cell.class_name, -1),
                    'confidence': round(cell.confidence, 3),
                    'u_rect': round(cell.u, 3),
                    'v_rect': round(cell.v, 3),
                    'u_source': round(source_u, 3),
                    'v_source': round(source_v, 3),
                    'z_mm': round(depth_by_position.get((cell.row, cell.col), 0.0), 3),
                }
            )
        return output

    def _sample_cell_depths(
        self,
        source_message: Image,
        cells: list[PingpongCell],
        corners,
    ) -> tuple[dict[tuple[int, int], float], dict[tuple[int, int], tuple[float, float]]]:
        """把矫正图穴位中心反算到原图坐标，并在对齐深度图中采样 z。"""
        if not self._use_depth:
            # 普通 RGB/USB 相机走这里，z 明确保持 0。
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

        # 透视矫正改变了坐标系，深度图还在原始相机坐标系，所以必须反变换。
        source_points = rectified_cells_to_source_points(cells, corners, self._edge_config)
        depth_view = ros_depth_to_image_view(self._latest_depth)
        depth_by_position = {
            position: sample_depth_mm(depth_view, point[0], point[1], self._depth_window_px)
            for position, point in source_points.items()
        }
        return depth_by_position, source_points

    def _log_summary(self, payload: dict[str, object]) -> None:
        """按固定间隔输出一行简要统计，避免刷屏。"""
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
        """在调试图上标出整帧检测失败原因。"""
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

    @staticmethod
    def _draw_candidate(image, tray_id: int, bbox, corners, status: str) -> None:
        """在调试图上标出单个盘候选框、角点和状态。"""
        import cv2

        x, y, width, height = bbox
        color = (0, 220, 255) if corners is not None else (0, 0, 255)
        cv2.rectangle(image, (x, y), (x + width, y + height), color, 2, cv2.LINE_AA)
        if corners is not None:
            cv2.polylines(image, [np.round(corners).astype(np.int32)], True, (0, 255, 0), 3, cv2.LINE_AA)
        cv2.putText(
            image,
            f'tray {tray_id}: {status}',
            (max(0, x), max(24, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
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
    """把 ROS RGB/BGR 图像消息转换为 OpenCV BGR 图像。"""
    if message.encoding not in ('bgr8', 'rgb8'):
        return None
    expected_step = message.width * 3
    array = np.frombuffer(message.data, dtype=np.uint8)
    if message.step < expected_step or len(array) < message.step * message.height:
        return None
    # ROS Image 的 step 可能带行填充，先按 step reshape，再截取真实像素宽度。
    rows = array.reshape((message.height, message.step))
    compact = rows[:, :expected_step].reshape((message.height, message.width, 3))
    image = compact.copy()
    if message.encoding == 'rgb8':
        # OpenCV 默认 BGR，ROS 相机常见 rgb8，需要翻转通道。
        image = image[:, :, ::-1].copy()
    return image


def bgr_to_ros_image(image: np.ndarray, source_message: Image) -> Image:
    """把 OpenCV BGR 图像转换回 ROS Image 消息。"""
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
    """把 ROS 深度图消息包装成复用深度采样函数需要的轻量视图。"""
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
    """把矫正图中的穴位中心反变换回原始 RGB 图像坐标。"""
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
    # target 是矫正图中的四角，corners 是原始 RGB 图中的四角。
    matrix = cv2.getPerspectiveTransform(target, corners.astype(np.float32))
    rectified_points = np.asarray([[[float(cell.u), float(cell.v)]] for cell in cells], dtype=np.float32)
    if len(rectified_points) == 0:
        return {}
    source_points = cv2.perspectiveTransform(rectified_points, matrix).reshape(-1, 2)
    return {
        (cell.row, cell.col): (float(point[0]), float(point[1]))
        for cell, point in zip(cells, source_points)
    }


def class_matrices_by_tray(cells: list[dict[str, object]], rows: int, cols: int) -> dict[str, list[list[str]]]:
    """按 tray_id 生成便于 JSON 查看的人类可读分类矩阵。"""
    matrices = {
        str(tray_id): [['unknown' for _col in range(cols)] for _row in range(rows)]
        for tray_id in range(1, 4)
    }
    for cell in cells:
        tray_id = int(cell['tray_id'])
        row = int(cell['row'])
        col = int(cell['col'])
        if 1 <= tray_id <= 3 and 1 <= row <= rows and 1 <= col <= cols:
            matrices[str(tray_id)][row - 1][col - 1] = str(cell['class_name'])
    return matrices


def class_id_matrices_by_tray(cells: list[dict[str, object]], rows: int, cols: int) -> dict[str, list[list[int]]]:
    """按 tray_id 生成便于 JSON 查看和调试的数字分类矩阵。"""
    matrices = {
        str(tray_id): [[-1 for _col in range(cols)] for _row in range(rows)]
        for tray_id in range(1, 4)
    }
    for cell in cells:
        tray_id = int(cell['tray_id'])
        row = int(cell['row'])
        col = int(cell['col'])
        if 1 <= tray_id <= 3 and 1 <= row <= rows and 1 <= col <= cols:
            matrices[str(tray_id)][row - 1][col - 1] = int(cell['class_id'])
    return matrices


def class_matrix(cells, rows: int, cols: int) -> list[list[str]]:
    """兼容旧调试代码的单盘字符串矩阵生成函数。"""
    matrix = [['unknown' for _col in range(cols)] for _row in range(rows)]
    for cell in cells:
        if 1 <= cell.row <= rows and 1 <= cell.col <= cols:
            matrix[cell.row - 1][cell.col - 1] = cell.class_name
    return matrix


def class_id_matrix(cells, rows: int, cols: int) -> list[list[int]]:
    """兼容旧调试代码的单盘数字矩阵生成函数。"""
    matrix = [[-1 for _col in range(cols)] for _row in range(rows)]
    for cell in cells:
        if 1 <= cell.row <= rows and 1 <= cell.col <= cols:
            matrix[cell.row - 1][cell.col - 1] = CLASS_IDS.get(cell.class_name, -1)
    return matrix


def tray_matrix_cells(payload_cells: list[dict[str, object]], detected_tray_ids: set[int]) -> list[TrayCell]:
    """生成固定 150 格输出；检测到的盘填真实结果，未检测到的盘补 empty。"""
    by_position = {
        (int(cell['tray_id']), int(cell['row']), int(cell['col'])): cell
        for cell in payload_cells
    }
    cells: list[TrayCell] = []
    for tray_id in range(1, 4):
        for row in range(1, 11):
            for col in range(1, 6):
                source = by_position.get((tray_id, row, col))
                cell = TrayCell()
                cell.tray_id = tray_id
                cell.col = col
                cell.row = row
                if source is None:
                    # 检测到该盘但某格没有结果：confidence=0；整盘没检测到：补空且 confidence=1。
                    cell.class_id = CLASS_IDS['empty']
                    cell.confidence = 0.0 if tray_id in detected_tray_ids else 1.0
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
