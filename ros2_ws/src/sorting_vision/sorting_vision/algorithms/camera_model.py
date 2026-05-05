"""相机模型和内参工具。

这个文件用于把 RealSense 发布的 CameraInfo 转成项目内部统一使用的
相机内参对象。

为什么要单独放一个文件：
1. 后续真实视觉节点都需要 `fx/fy/cx/cy`。
2. 这些参数应该从 CameraInfo 自动读取，不应该手写散落在代码里。
3. 如果相机分辨率或话题配置变化，节点只要收到新的 CameraInfo，
   就能使用当前图像流对应的内参。

本文件解决的是“相机内参”问题，不解决“手眼标定”问题。
相机到机械臂基座的 R/T 仍然需要现场标定。
"""

from __future__ import annotations

from dataclasses import dataclass

from sensor_msgs.msg import CameraInfo


@dataclass(frozen=True)
class CameraIntrinsics:
    """项目内部统一使用的相机内参结构。"""

    width: int
    height: int
    frame_id: str
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    p: tuple[float, ...]
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def is_calibrated(self) -> bool:
        """判断 CameraInfo 是否包含有效内参。"""

        return self.fx != 0.0 and self.fy != 0.0

    def pixel_to_camera(self, u: float, v: float, z_mm: float) -> tuple[float, float, float]:
        """把像素坐标和深度转换为相机坐标。

        参数：
        - u：像素横坐标。
        - v：像素纵坐标。
        - z_mm：深度值，单位 mm。

        返回：
        - Xc, Yc, Zc，相机坐标系下的三维坐标，单位 mm。

        注意：
        这里使用的是 pinhole camera model。输入的 u/v/z 必须来自同一个相机流
        或者已经完成 RGB-D 对齐，否则结果会有偏差。
        """

        if not self.is_calibrated:
            raise ValueError('Camera intrinsics are not calibrated')

        x_c = (u - self.cx) * z_mm / self.fx
        y_c = (v - self.cy) * z_mm / self.fy
        return x_c, y_c, z_mm


def intrinsics_from_camera_info(message: CameraInfo) -> CameraIntrinsics:
    """从 ROS 2 CameraInfo 消息提取项目内参结构。"""

    return CameraIntrinsics(
        width=message.width,
        height=message.height,
        frame_id=message.header.frame_id,
        distortion_model=message.distortion_model,
        d=tuple(float(value) for value in message.d),
        k=tuple(float(value) for value in message.k),
        p=tuple(float(value) for value in message.p),
        fx=float(message.k[0]),
        fy=float(message.k[4]),
        cx=float(message.k[2]),
        cy=float(message.k[5]),
    )


class CameraIntrinsicsCache:
    """缓存 RGB 和 Depth 两路相机内参。

    后续真实视觉节点可以持有这个对象：
    - 收到 color CameraInfo 时调用 update_color。
    - 收到 depth CameraInfo 时调用 update_depth。
    - 处理图像时读取 color/depth 属性。

    这样节点不需要手动维护一堆分散的 fx/fy/cx/cy 变量。
    """

    def __init__(self) -> None:
        self.color: CameraIntrinsics | None = None
        self.depth: CameraIntrinsics | None = None

    def update_color(self, message: CameraInfo) -> CameraIntrinsics:
        """更新 RGB 相机内参并返回当前内参。"""

        self.color = intrinsics_from_camera_info(message)
        return self.color

    def update_depth(self, message: CameraInfo) -> CameraIntrinsics:
        """更新 Depth 相机内参并返回当前内参。"""

        self.depth = intrinsics_from_camera_info(message)
        return self.depth

    @property
    def ready(self) -> bool:
        """RGB 和 Depth 内参是否都已经收到。"""

        return self.color is not None and self.depth is not None
