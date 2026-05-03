# Project Docs

从这里进入文档。当前项目以“少看、看对”为原则，优先阅读 `current/`。

## 必看

1. [`current/project_state.md`](current/project_state.md)
   当前真实状态、已验证能力、样本/debug 位置、下一步主线。
2. [`current/todo_and_decisions.md`](current/todo_and_decisions.md)
   当前决策、近期待办、暂缓事项和不建议继续投入的方向。
3. [`current/session_handoff.md`](current/session_handoff.md)
   新窗口 AI 或换人接手时的项目交接说明。

## 操作指南

`guides/` 放“怎么做”的文档：

- [`guides/development.md`](guides/development.md)：开发、构建、Git 使用。
- [`guides/hardware_bringup.md`](guides/hardware_bringup.md)：硬件和相机启动。
- [`guides/runtime_configuration.md`](guides/runtime_configuration.md)：运行参数。
- [`guides/sample_collection.md`](guides/sample_collection.md)：样本采集。

## 参考资料

`reference/` 放“背景和约定”的文档：

- [`reference/system_design.md`](reference/system_design.md)：系统设计。
- [`reference/vision_algorithm.md`](reference/vision_algorithm.md)：视觉算法背景。
- [`reference/communication_protocol.md`](reference/communication_protocol.md)：通信协议。
- [`reference/calibration.md`](reference/calibration.md)：标定说明。
- [`reference/glossary.md`](reference/glossary.md)：术语表。
- [`reference/project_directory_explanation.txt`](reference/project_directory_explanation.txt)：目录说明。

## Debug 查看入口

算法效果不要在 `samples/debug` 里到处翻，先看：

```text
samples/debug/_review/README.md
```

重建 debug 索引：

```bash
cd /home/wht/sorting_robot
python3 scripts/archive_debug_outputs.py
```

