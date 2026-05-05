# 未提交内容分诊

更新时间：2026-05-05

当前工作区存在多类未提交内容，不能混成一个“算法进展”提交。

## 可作为结构整理提交的内容

- `.gitignore`
  - 增加 `.claude/`，避免本机工具状态进入项目。
- `docs/README.md`
  - 增加 `current/vision_structure_plan.md` 入口。
- `docs/current/vision_structure_plan.md`
  - 记录 `sorting_vision` 职责划分与后续整理原则。
- `docs/current/uncommitted_triage.md`
  - 当前文件，记录未提交内容如何处理。

这些内容不改变运行行为，可以单独提交。

## 暂不提交的旧视觉实验

- `ros2_ws/src/sorting_vision/sorting_vision/offline_tray_debug.py`
  - 大量离线实验代码，包括穴位亮斑拟合、额外 CSV、run 目录支持等。
  - 内容过多，且与当前“先稳定边界和透视矫正”的主线不完全一致。
- `ros2_ws/src/sorting_vision/config/offline_tray_debug.yaml`
  - 只改了 `projection_smooth_px: 31 -> 11`。
  - 属于旧实验参数，不应单独混入结构提交。
- `ros2_ws/src/sorting_vision/sorting_vision/tray_grid_debug.py`
  - 增加内部网格拟合统计、重投影误差等字段。
  - 有价值，但应等网格主线恢复后单独评审。

## 暂不提交的新实验脚本

- `ros2_ws/src/sorting_vision/sorting_vision/tray_rectify_debug.py`
  - 新几何实验脚本，目标是三苗盘边界识别与透视矫正。
  - 当前仍是离线实验，效果未稳定。
- `ros2_ws/src/sorting_vision/sorting_vision/tray_matrix_overlay_debug.py`
  - 依赖 `tray_rectify_debug.py` 的矩阵覆盖调试脚本。
  - 当前不推进矩阵 overlay，暂不提交。
- `ros2_ws/src/sorting_vision/setup.py`
  - 目前只增加了 `tray_rectify_debug` 入口。
  - 如果 `tray_rectify_debug.py` 暂不提交，则这个入口也不应提交。

## 建议处理顺序

1. 先提交结构整理文档和 `.gitignore`。
2. 暂停 `tray_matrix_overlay_debug.py`，不要注册入口。
3. 单独评审 `tray_rectify_debug.py`，决定是继续打磨、改名归档，还是删除。
4. 单独评审 `offline_tray_debug.py` 和 `tray_grid_debug.py`，避免旧网格实验影响当前边界主线。

## 当前主线

当前视觉主线只推进：

1. 三个苗盘的真实外边界识别。
2. 四边相交得到四角。
3. 透视矫正单盘正视图。

网格、矩阵、分类、三维坐标在边界稳定前都暂缓。
