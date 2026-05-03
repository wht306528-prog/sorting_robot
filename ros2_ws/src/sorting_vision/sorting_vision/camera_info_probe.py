"""相机内参探测节点。

本节点用于读取普通 RGB 相机或 RGB-D 相机驱动发布的 CameraInfo。

节点功能：
1. 订阅 RGB 相机内参话题。
2. 可选订阅 Depth 相机内参话题。
3. 打印图像尺寸、畸变模型、D 参数、K 矩阵和 P 矩阵。
4. 从 K 矩阵中提取 fx、fy、cx、cy。

重要说明：
- 这里读取的是相机驱动提供的 CameraInfo，不绑定具体相机品牌。
- 这些内参用于像素坐标 u/v 和深度 z 转相机坐标 Xc/Yc/Zc。
- 相机到机械臂基座的 R/T 不是相机内参，后续仍然需要现场手眼标定。

运行方式：
    ros2 run sorting_vision camera_info_probe --ros-args \
        --params-file install/sorting_vision/share/sorting_vision/config/mock_vision.yaml
"""

from __future__ import annotations

from rclpy.node import Node
import rclpy
from sensor_msgs.msg import CameraInfo

from sorting_vision.camera_model import CameraIntrinsics, CameraIntrinsicsCache


class CameraInfoProbe(Node):
    """订阅 RGB/Depth CameraInfo 并打印关键内参。"""

    def __init__(self) -> None:
        super().__init__('camera_info_probe')

        # CameraInfo 话题名集中放在配置文件里，避免不同相机命名时改代码。
        self.declare_parameter(
            'color_camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self.declare_parameter(
            'depth_camera_info_topic',
            '/camera/camera/depth/camera_info',
        )
        self.declare_parameter('use_depth', True)

        self._color_topic = (
            self.get_parameter('color_camera_info_topic')
            .get_parameter_value()
            .string_value
        )
        self._depth_topic = (
            self.get_parameter('depth_camera_info_topic')
            .get_parameter_value()
            .string_value
        )
        self._use_depth = (
            self.get_parameter('use_depth')
            .get_parameter_value()
            .bool_value
        )
        self._printed_color = False
        self._printed_depth = False
        self._intrinsics = CameraIntrinsicsCache()

        self._color_subscription = self.create_subscription(
            CameraInfo,
            self._color_topic,
            self._handle_color_info,
            10,
        )
        self._depth_subscription = None
        if self._use_depth:
            self._depth_subscription = self.create_subscription(
                CameraInfo,
                self._depth_topic,
                self._handle_depth_info,
                10,
            )

        self.get_logger().info(f'正在订阅 RGB 相机内参：{self._color_topic}')
        if self._use_depth:
            self.get_logger().info(f'正在订阅深度相机内参：{self._depth_topic}')
        else:
            self.get_logger().info('已关闭深度内参探测：当前只检查 RGB CameraInfo')

    def _handle_color_info(self, message: CameraInfo) -> None:
        """打印 RGB 相机内参。"""

        if self._printed_color:
            return
        intrinsics = self._intrinsics.update_color(message)
        self._print_camera_info('RGB', intrinsics)
        self._printed_color = True
        self._try_shutdown_after_print()

    def _handle_depth_info(self, message: CameraInfo) -> None:
        """打印 Depth 相机内参。"""

        if self._printed_depth:
            return
        intrinsics = self._intrinsics.update_depth(message)
        self._print_camera_info('深度', intrinsics)
        self._printed_depth = True
        self._try_shutdown_after_print()

    def _print_camera_info(self, name: str, intrinsics: CameraIntrinsics) -> None:
        """格式化打印一组相机内参。"""

        self.get_logger().info(f'[{name}] frame_id={intrinsics.frame_id}')
        self.get_logger().info(
            f'[{name}] size={intrinsics.width}x{intrinsics.height} '
            f'distortion_model={intrinsics.distortion_model}'
        )
        self.get_logger().info(
            f'[{name}] fx={intrinsics.fx:.6f} '
            f'fy={intrinsics.fy:.6f} '
            f'cx={intrinsics.cx:.6f} '
            f'cy={intrinsics.cy:.6f}'
        )
        self.get_logger().info(f'[{name}] D={list(intrinsics.d)}')
        self.get_logger().info(f'[{name}] K={list(intrinsics.k)}')
        self.get_logger().info(f'[{name}] P={list(intrinsics.p)}')

    def _try_shutdown_after_print(self) -> None:
        """两路内参都打印后，提示用户可以退出节点。"""

        if self._printed_color and (self._printed_depth or not self._use_depth):
            self.get_logger().info(
                '需要的相机内参都已收到。需要结束时请按 Ctrl+C。'
            )


def main(args: list[str] | None = None) -> None:
    """ROS 2 节点入口函数。"""

    rclpy.init(args=args)
    node = CameraInfoProbe()
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
