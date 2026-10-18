# ROS2 自动泊车规划与控制原型

[![CI](https://github.com/gj-465930/ros2-parking-planner/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/gj-465930/ros2-parking-planner/actions/workflows/ci.yml)

基于 ROS2 Humble 与 C++17 的自动泊车规划与控制学习项目。
目标是在非结构化泊车场景中，逐步实现路径搜索、路径平滑、速度规划和闭环跟踪，
并通过单元测试、RViz 可视化和量化评测验证各模块。

**当前处于工程骨架阶段**：已完成 core/runtime 分层、最小功能测试和静止车辆框显示。
Hybrid A*、轨迹优化与车辆控制属于后续计划。

## 当前进度

- `parking_core`：纯 C++ 静态库，当前包含用于验证构建与链接的占位函数。
- `parking_runtime`：ROS 节点与可视化适配，节点负责定时调度，`Visualizer` 负责消息构造与发布。
- `parking_node`：ROS2 可执行入口，可通过 launch 与 RViz 一起启动。
- `test_core_smoke`：最小 gtest，验证 core 函数可被独立链接和调用。
- RViz 演示：以约 1 Hz 发布一个长 4.5 m、宽 1.8 m 的静止矩形车辆示意框。

CI 工作流已编写，首次远程构建和 PR 验证待完成；顶部徽章用于显示 `main` 分支的实际运行状态。

## 架构与目录

| 构建目标 | 职责 | 主要依赖 |
|---|---|---|
| `parking_core` | 几何、规划、控制等纯 C++ 算法的承载层，当前为占位实现 | C++ 标准库 |
| `parking_runtime` | ROS 节点、消息适配与可视化发布 | core、rclcpp、消息包 |
| `parking_node` | 初始化 ROS、创建节点、执行 spin 与关闭流程 | runtime |
| `test_core_smoke` | core 的最小功能测试 | core、gtest |

头文件、实现和测试按层与领域对应归类；目录负责组织代码，CMake target 负责管理编译与链接。

```text
.
├── .github/workflows/ci.yml
├── README.md
└── src/parking_planner/
    ├── CMakeLists.txt
    ├── package.xml
    ├── LICENSE
    ├── include/parking_planner/
    │   ├── core/
    │   ├── runtime/
    │   └── visualization/
    ├── src/
    │   ├── core/
    │   ├── runtime/
    │   ├── visualization/
    │   └── main.cpp
    ├── test/core/
    ├── launch/parking_demo.launch.py
    └── rviz/parking_demo.rviz
```

`build/`、`install/`、`log/` 由 colcon 生成，不纳入版本控制。

## 环境与依赖

- 本地开发环境：macOS arm64，使用 RoboStack/Pixi 提供的 ROS2 Humble。
- CI 配置环境：Ubuntu 22.04 runner，`ros:humble-ros-base` 容器。
- 构建工具：支持 C++17 的编译器、CMake、colcon、ament_cmake。
- ROS 依赖：`rclcpp`、`geometry_msgs`、`visualization_msgs`、`launch`、`launch_ros`、
  `ament_index_python`、`rviz2`。
- 测试依赖：`ament_cmake_gtest`，以及包清单中声明的 ament lint 依赖。

完整依赖以 [package.xml](src/parking_planner/package.xml) 为准。
本仓库复用已有 ROS2 环境，未提供 Pixi 环境清单。
macOS 用户应先激活自己的 RoboStack/Pixi Humble 环境，再进入工作空间。
Ubuntu 用户可按 [ROS2 Humble 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
准备 ROS 环境，并安装 colcon、rosdep 与编译工具。

以下构建、测试与启动命令均在工作空间根目录执行，终端需已激活 ROS2 Humble 环境。

## 获取与构建

获取仓库；如果已有本地工作空间，直接进入其根目录即可：

```bash
git clone https://github.com/gj-465930/ros2-parking-planner.git ros2_parking_ws
cd ros2_parking_ws
```

Ubuntu 用户准备好 ROS2、colcon、rosdep 与编译工具后，在工作空间根目录安装本包依赖：

```bash
source /opt/ros/humble/setup.bash

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi

rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

构建包并开启测试、生成编译数据库：

```bash
colcon build --symlink-install --packages-select parking_planner \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

编译数据库位于 `build/parking_planner/compile_commands.json`，仓库中的 `.clangd` 已指向该目录。

## 功能测试

```bash
colcon test --packages-select parking_planner \
  --return-code-on-test-failure \
  --event-handlers console_direct+ \
  --ctest-args -R '^test_' --no-tests=error --output-on-failure

colcon test-result --verbose
```

当前应实际运行 `test_core_smoke` 中的 1 个 gtest 用例，并报告零失败。
该测试验证基础构建与链接链路；算法测试随后续模块逐步增加。
`--return-code-on-test-failure` 使测试失败返回非零退出码，`--no-tests=error` 使未匹配到测试时报告错误。
目前只筛选名称以 `test_` 开头的功能测试，ament lint 尚未纳入 CI 门槛。

## RViz 演示

构建成功后，在当前终端加载工作空间安装环境。zsh 使用：

```zsh
source install/setup.zsh
```

bash 使用：

```bash
source install/setup.bash
```

然后启动节点与 RViz：

```bash
ros2 launch parking_planner parking_demo.launch.py
```

预期可见位于 `map` 原点、朝向为零的绿色闭合矩形。它是固定尺寸的车辆示意框，
车辆运动学模型与车辆位姿更新将在后续阶段实现。按 `Ctrl-C` 结束 launch 及其启动的进程。

| 项目 | 配置 |
|---|---|
| 话题 | `/parking/markers` |
| 消息类型 | `visualization_msgs/msg/MarkerArray` |
| Marker 类型 | `LINE_STRIP` |
| Fixed Frame | `map` |
| 发布频率 | 约 1 Hz |

在另一个已激活 ROS 环境并加载工作空间的终端，可观察消息与频率：

```bash
ros2 topic echo /parking/markers --once
ros2 topic hz /parking/markers
```

launch 从安装后的包资源目录加载 RViz 配置。
修改显示设置后，可通过 RViz 的 **File → Save Config As** 保存到
`src/parking_planner/rviz/parking_demo.rviz`。
使用 `--symlink-install` 构建时，已有配置文件的修改可在重新加载配置或重启 RViz 后生效。

## 持续集成

[CI 工作流](.github/workflows/ci.yml) 在推送到 `main`、以及创建或更新目标为 `main` 的 PR 时触发。
其步骤为：检出代码、安装工具和包依赖、构建、运行功能测试、汇总测试结果。
构建成功后，即使测试失败也会输出测试汇总，失败状态会保留。
CI 执行构建与测试，RViz 显示通过本地演示验证。

## 后续路线

| 阶段 | 目标 | 状态 |
|---|---|---|
| 0 | 工程分层、gtest、RViz、CI 与项目说明 | 进行中 |
| 1–2 | 几何工具、碰撞检测、车辆模型与场景表示 | 计划中 |
| 3–5 | 栅格 A*、Reeds-Shepp、Hybrid A* 与规划评测 | 计划中 |
| 6–7 | 基于 QP 的路径平滑、速度规划与轨迹合成 | 计划中 |
| 8 | Pure Pursuit、LQR、MPC 与含倒车轨迹的闭环跟踪 | 计划中 |
| 9 | 系统集成、场景回归、指标分析与演示 | 计划中 |

## 许可

采用 [Apache-2.0](src/parking_planner/LICENSE) 许可。
