"""苗盘几何识别新主线。

这个模块是新的算法入口，从空接口开始，不继承旧的
`offline_tray_debug.py`、`tray_grid_debug.py`、`tray_rectify_debug.py` 实现。

目标顺序：
1. 粗定位 3 个苗盘区域。
2. 每个区域拟合真实外边界四条边。
3. 四边相交得到四角。
4. 透视矫正为单盘正视图。
5. 在正视图中识别穴位中心。

当前文件只定义数据结构和管线入口，避免在算法没有验证前继续画假框。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrayGeometryConfig:
    """苗盘几何识别配置。"""

    expected_tray_count: int = 3


@dataclass(frozen=True)
class TrayGeometryCandidate:
    """单个苗盘几何候选。"""

    tray_id: int
    status: str
    message: str
    corners: np.ndarray | None = None


@dataclass(frozen=True)
class TrayGeometryResult:
    """一张图的苗盘几何识别结果。"""

    status: str
    message: str
    candidates: list[TrayGeometryCandidate]


def detect_tray_geometry(
    image: np.ndarray,
    config: TrayGeometryConfig | None = None,
) -> TrayGeometryResult:
    """检测一张 RGB/BGR 图中的苗盘几何。

    这是新算法入口。当前只做输入校验，不输出任何伪造候选。
    后续每加一步，都必须配套 debug 输出和失败状态。
    """

    if config is None:
        config = TrayGeometryConfig()
    if image.ndim != 3 or image.shape[2] != 3:
        return TrayGeometryResult(
            status='failed',
            message='input image must be HxWx3',
            candidates=[],
        )
    if config.expected_tray_count <= 0:
        return TrayGeometryResult(
            status='failed',
            message='expected_tray_count must be positive',
            candidates=[],
        )
    return TrayGeometryResult(
        status='not_implemented',
        message='tray geometry algorithm skeleton is ready; detection is not implemented yet',
        candidates=[],
    )
