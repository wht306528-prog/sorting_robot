# 开发说明

## 1. 开发环境

当前开发环境：

- 虚拟机：VMware 中的 Ubuntu。
- 编辑器：VSCode。
- ROS 版本：ROS 2 Humble。
- 工作区目录：`/home/wht/sorting_robot/ros2_ws`。
- 项目根目录：`/home/wht/sorting_robot`。
- Git 仓库目录：`/home/wht/sorting_robot`。

进入项目目录：

```bash
cd /home/wht/sorting_robot
```

进入 ROS 2 工作区：

```bash
cd /home/wht/sorting_robot/ros2_ws
```

## 2. 项目目录结构

```text
sorting_robot/
├── docs/                    # 项目说明文档
├── README/                  # 预留说明目录
├── scripts/                 # 辅助脚本
└── ros2_ws/
    ├── src/
    │   ├── sorting_bringup/
    │   ├── sorting_control/
    │   ├── sorting_description/
    │   ├── sorting_driver/
    │   ├── sorting_interfaces/
    │   └── sorting_vision/
    ├── build/               # colcon 生成物，不提交 Git
    ├── install/             # colcon 生成物，不提交 Git
    └── log/                 # colcon 日志，不提交 Git
```

## 3. Git 使用规范

每完成一个清楚的小目标，提交一次 Git：

```bash
git status
git diff
git add .
git commit -m "Add system design document"
```

常用命令：

```bash
git status          # 查看当前状态
git diff            # 查看未暂存的具体改动
git add <path>      # 添加指定文件或目录
git add .           # 添加当前目录下所有改动
git commit -m "..." # 提交一个版本
git log --oneline   # 查看提交历史
```

建议提交粒度：

- 一个提交只做一类事情。
- 提交信息写清楚“做了什么”。
- 不提交 `build/`、`install/`、`log/`。
- 不提交临时文件、缓存文件、大体积测试数据。

示例提交顺序：

```text
commit 1: 初始化 ROS 2 项目结构
commit 2: 编写系统设计文档
commit 3: 定义 ROS 2 消息接口
commit 4: 编写视觉矩阵模拟发布节点
commit 5: 编写鲁班猫到 F407 的 TCP 发送程序
commit 6: 接入 RealSense 相机
commit 7: 实现苗盘外框识别和透视矫正
commit 8: 实现 5x10 网格生成
commit 9: 实现 ROI 分类
commit 10: 输出完整 150x8 矩阵
```

## 4. ROS 2 编译方法

加载 ROS 2 环境后，在工作区根目录编译：

```bash
cd /home/wht/sorting_robot/ros2_ws
colcon build
```

编译完成后加载本工作区：

```bash
source install/setup.sh
```

如果只编译某一个包：

```bash
colcon build --packages-select sorting_interfaces
```

## 5. 运行方式

后续建议统一通过 `sorting_bringup` 启动：

```bash
ros2 launch sorting_bringup bringup.launch.py
```

在正式节点完成前，可以先分别运行单个节点进行调试。例如：

```bash
ros2 run sorting_vision mock_matrix_publisher
ros2 run sorting_driver f407_sender
```

具体节点名称以后以 `setup.py` 中的 `entry_points` 为准。

查看模拟矩阵发布：

```bash
ros2 topic echo /sorting/tray_matrix
```

## 6. 文档维护规范

写代码时同步更新文档：

- 改英文变量、代号、中文名称或字段含义时，更新 `docs/glossary.md`。
- 改通信字段时，更新 `docs/communication_protocol.md`。
- 改识别流程时，更新 `docs/vision_algorithm.md`。
- 改坐标定义或标定参数时，更新 `docs/calibration.md`。
- 改系统分工或流程时，更新 `docs/system_design.md`。

文档不是最后补的内容，而是开发过程中的设计记录。

## 7. 回退方法

查看历史：

```bash
git log --oneline
```

放弃某个未提交文件的修改：

```bash
git restore <file>
```

取消已经 `git add` 的文件：

```bash
git restore --staged <file>
```

撤销一个已经提交的版本，推荐使用：

```bash
git revert <commit_id>
```

`git reset --hard` 会丢弃改动，除非明确知道后果，否则不要随便使用。

## 8. 当前开发重点

当前阶段建议优先完成：

1. 固定系统设计文档。
2. 固定 150 行 8 列矩阵格式。
3. 定义 ROS 2 消息接口。
4. 编写模拟矩阵发布和发送程序。
5. 再接入真实相机和视觉算法。
