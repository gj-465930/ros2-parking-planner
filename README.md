# ROS2 自动泊车规划与控制原型

[![CI](https://github.com/gj-465930/ros2-parking-planner/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/gj-465930/ros2-parking-planner/actions/workflows/ci.yml)

基于 ROS2 Humble 与 C++17 的自动泊车规划与控制学习项目。目标是在非结构化泊车场景中，
逐步实现路径搜索、路径平滑、速度规划和闭环跟踪，并用测试、可视化与量化评测验证各模块。

**当前已实现几何工具、车辆运动学、静态环境、YAML 场景加载与 RViz 场景显示。**
三个场景已通过加载测试和本地显示验证；目前演示为静态场景，尚未实现路径规划、车辆运动演示或闭环控制。
场景加载成功不代表已证明起终点之间存在可行路径。

## 已实现能力

| 模块 | 当前能力 |
|---|---|
| 几何 | 二维向量、角度归一化与插值、局部/世界坐标变换、线段投影与相交检测 |
| 碰撞几何 | 有向矩形、严格凸多边形、基于 SAT 的矩形/矩形与凸多边形/矩形重叠判断 |
| 车辆 | 尺寸与转角参数、后轴中心到车框中心的换算、完整车辆轮廓 |
| 运动学 | 固定等效前轮转角下的圆弧精确传播、一步欧拉近似，支持前进与倒车 |
| 环境 | 占据栅格索引转换、凸障碍物栅格化、完整车框与原始障碍物及地图边界的碰撞检查 |
| 场景 | 三份 YAML 输入、结构与数值校验、起终车辆碰撞检查、目标车辆是否完整位于车位内 |
| ROS/RViz | 场景选择参数、安装资源定位、地图/障碍物/车位/起终车辆与后轴朝向显示 |

最近一次本地验证（2026-11-03，macOS arm64）：构建与安装成功，**138 个 gtest 用例、12 个 CTest 全部通过**，
其中包含 6 个场景消息适配测试。三个场景显示及晚打开 RViz 接收场景已在本地人工验证。
本阶段改动的远程 CI 尚未核验；顶部徽章展示的是 `main` 分支状态。

## 架构与目录

| 构建目标 | 职责 | 依赖 |
|---|---|---|
| `parking_core` | 几何、车辆模型、环境与场景加载，不依赖 ROS | C++17、yaml-cpp |
| `parking_runtime` | 节点参数与场景持有、ROS 消息适配、可视化发布 | core、rclcpp、消息包 |
| `parking_node` | ROS 初始化、节点运行、异常日志与退出 | runtime |
| 功能测试 | core 行为测试与独立的可视化消息测试 | 对应库、gtest |

`ParkingNode` 加载并持有场景；`Visualizer` 借用节点创建发布器，不保存场景副本。
`buildSceneMarkers()` 只构造消息，测试时无需启动节点或 RViz。

```text
src/parking_planner/
├── CMakeLists.txt
├── package.xml
├── include/parking_planner/
│   ├── core/
│   │   ├── geometry/
│   │   ├── vehicle/
│   │   ├── map/
│   │   └── scenario/
│   ├── runtime/
│   └── visualization/
├── src/                       # 与头文件按层和领域对应
│   ├── core/
│   ├── runtime/
│   ├── visualization/
│   └── main.cpp
├── test/
│   ├── core/
│   └── visualization/
├── scenarios/                 # 三份场景 YAML
├── launch/parking_demo.launch.py
└── rviz/parking_demo.rviz
```

目录管理文件归类，CMake target 管理编译与链接；新增领域目录不会自动新增库。
`build/`、`install/`、`log/` 为工作空间生成产物，不纳入版本控制。

## 环境与依赖

- 本地：macOS arm64，RoboStack/Pixi 提供的 ROS2 Humble。
- CI 配置：Ubuntu 22.04 runner，`ros:humble-ros-base` 容器。
- 构建：C++17 编译器、CMake、colcon、ament_cmake。
- 场景解析：yaml-cpp；CMake 兼容 0.7 与 0.8 的导出 target 名称差异。
- ROS：`rclcpp`、`geometry_msgs`、`visualization_msgs`、`launch`、`launch_ros`、`ament_index_python`、`rviz2`。
- 测试：`ament_cmake_gtest`，以及包清单中声明的 ament lint 依赖。

完整依赖见 [package.xml](src/parking_planner/package.xml)。本仓库复用已有 ROS 环境，未提供 Pixi 环境清单。
macOS 用户先激活自己的 RoboStack/Pixi Humble 环境，并确保其中安装了 yaml-cpp。
Ubuntu 用户可按 [ROS2 Humble 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
准备 ROS 环境及 colcon、rosdep、编译工具。

以下命令在工作空间根目录执行，终端需已激活 ROS2 Humble 环境。

## 获取与构建

```bash
git clone https://github.com/gj-465930/ros2-parking-planner.git ros2_parking_ws
cd ros2_parking_ws
```

Ubuntu 用户安装包依赖：

```bash
source /opt/ros/humble/setup.bash

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi

rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

构建并开启测试、生成编译数据库：

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

当前注册 12 个功能测试 target，涵盖几何边界与非法输入、车辆轮廓与运动传播、栅格与碰撞、
YAML 校验和三份真实场景，以及可视化消息的坐标、闭合轮廓、箭头、标记身份与清理指令。
消息单元测试不验证 DDS 历史交付或 RViz 渲染，这两项通过本地运行检查。

`--return-code-on-test-failure` 使测试失败返回非零退出码，`--no-tests=error` 使没有匹配测试时报告错误。
ament lint 尚未纳入 CI 门槛。复用旧构建目录时，`test-result` 可能汇总已移除 target 的旧 XML；
本次实际执行情况应结合 CTest 输出核对，汇总数不等于 gtest 用例数。

## 三场景演示

构建后加载当前工作空间；zsh 使用 `source install/setup.zsh`，bash 使用 `source install/setup.bash`。
每次启动一个场景，切换前在原 launch 终端按 `Ctrl-C` 停止上一次运行。

```bash
# 垂直泊车，亦为默认场景
ros2 launch parking_planner parking_demo.launch.py

# 平行泊车
ros2 launch parking_planner parking_demo.launch.py \
  scenario:=parallel_parking.yaml

# 窄通道调头场景
ros2 launch parking_planner parking_demo.launch.py \
  scenario:=narrow_passage.yaml
```

| 颜色/图形 | 含义 |
|---|---|
| 灰色闭合轮廓 | 地图范围 |
| 红色闭合轮廓 | 原始障碍物多边形 |
| 黄色闭合轮廓 | 目标车位 |
| 绿色车框与箭头 | 起点车辆轮廓、起点后轴位置及朝向 |
| 蓝色车框与箭头 | 目标车辆轮廓、目标后轴位置及朝向 |

话题为 `/parking/markers`，类型为 `visualization_msgs/msg/MarkerArray`，RViz Fixed Frame 为 `map`。
轮廓使用 `LINE_STRIP`，后轴朝向使用两点式 `ARROW`；车框顶点已是世界坐标，Marker pose 使用单位变换。
场景范围为 20 m × 16 m，看不全时可拉远视角或切换俯视视角。

### 静态发布与晚加入 RViz

节点启动时只发布一次完整场景，随后保持运行。发布器使用 `KeepLast(1)`、`Reliable`、`Transient Local`；
RViz 也使用 `Reliable` 与 `Transient Local`。发布器存活期间，晚加入的订阅者可以取得最新场景。
这不是磁盘持久化，节点退出后不再提供缓存消息。

验证时只关闭 RViz 窗口，保持原 launch 和节点运行，在另一个已激活 ROS 并加载工作空间的终端执行：

```bash
rviz2 -d "$(ros2 pkg prefix --share parking_planner)/rviz/parking_demo.rviz"
```

新 RViz 无需重启节点即可显示场景。也可读取缓存消息、核对实际端点 QoS：

```bash
ros2 topic echo /parking/markers visualization_msgs/msg/MarkerArray \
  --qos-reliability reliable --qos-durability transient_local --once

ros2 topic info /parking/markers --verbose
```

场景是一次性快照，不以周期频率作为验收指标。每份快照先 `DELETEALL` 再添加当前标记，
防止旧 namespace/id 的图形残留；该可视化话题由本 Visualizer 独占。当前未实现运行中热切换场景。

launch 从安装后的包资源目录加载 RViz 配置。修改显示设置后，通过 **File → Save Config As** 保存到
`src/parking_planner/rviz/parking_demo.rviz`。使用 `--symlink-install` 构建时，已有配置文件的修改
可在重新加载配置或重启 RViz 后生效。

## 场景输入与约定

完整示例见 [垂直泊车](src/parking_planner/scenarios/perpendicular_parking.yaml)、
[平行泊车](src/parking_planner/scenarios/parallel_parking.yaml)、
[窄通道](src/parking_planner/scenarios/narrow_passage.yaml)。

| YAML 字段 | 内容与单位 |
|---|---|
| `name` | 非空、非纯空白的场景名称 |
| `vehicle` | 轴距、车宽、前后悬（米）、最大等效前轮转角（弧度） |
| `map` | `origin: [x, y]` 为地图左下角世界坐标；`resolution` 为米/格；`width/height` 为格数 |
| `obstacles` | 障碍物列表，允许 `[]`；每项含 `vertices: [[x, y], ...]` |
| `start` / `goal` | 后轴中心的世界位姿 `x, y, theta`，米、弧度 |
| `parking_space` | 几何中心 `center`、朝向 `heading`、长 `length`、宽 `width` |

所有几何数据使用同一个世界坐标系，角度逆时针为正。车辆局部 x 朝前、y 朝左。
地图左下角、后轴中心与车框几何中心是不同概念，修改地图范围不会改变世界坐标原点。

障碍物需要沿边界排列至少三个严格凸顶点，不重复首点；接受顺时针或逆时针输入，内部统一为逆时针。
重复点、冗余共线、退化、凹形和自交输入会被拒绝。地图内部无障碍区域默认允许行驶，当前不表达道路语义。

障碍物栅格化采用格子正方形与多边形的重叠检测，接触也标为占用，因此可能偏保守。
**车辆碰撞检测使用完整车框与原始障碍物做 SAT，同时检查地图边界，不能用后轴所在格是否空闲代替。**
接触障碍物、地图边界或处于相应容差范围内均按碰撞处理。

加载器检查字段结构、数值与几何合法性，要求起终车辆均不碰撞或越界，并要求目标完整车框位于车位内；
接触车位边界允许，因为车位是目标区域，不是障碍物。不要求车位中心与后轴中心重合。
这些检查不证明路径可达，也不检查一段运动过程中的连续碰撞。

launch 参数 `scenario` 选择内置文件名；节点参数 `scenario_file` 接收完整路径。
自定义文件可直接传给节点（另开 RViz 查看）：

```bash
ros2 run parking_planner parking_node --ros-args \
  -p scenario_file:="$(pwd)/src/parking_planner/scenarios/perpendicular_parking.yaml"
```

缺少路径、文件不存在或场景校验失败时，节点打印错误并返回退出码 `1`；加载器错误包含文件和解析上下文。

## 源码阅读笔记

- [车辆参考点、运动传播与栅格约定：Autoware 源码阅读笔记](docs/blog/vehicle_model_and_grid_conventions.md)：固定源码版本，比较后轴参考点、圆弧传播、小角度处理与栅格索引约定。

## 持续集成与后续路线

[CI 工作流](.github/workflows/ci.yml) 监听 `main` push 和面向 `main` 的 PR 创建/更新，
执行依赖安装、构建、功能测试与结果汇总。单独推送没有对应 PR 的开发分支不会触发当前工作流。
CI 不运行 RViz 图形演示。

| 内容 | 状态 |
|---|---|
| 工程分层、gtest、CI 配置 | 已实现 |
| 几何、车辆模型、静态环境、场景加载与显示 | 本地实现与验证已完成，阶段收尾中 |
| 栅格 A*、Dijkstra 启发代价场 | 后续计划 |
| Dubins、Reeds-Shepp、Hybrid A* 与规划评测 | 后续计划 |
| QP 路径平滑、速度规划、轨迹合成 | 后续计划 |
| Pure Pursuit、LQR、MPC 与闭环仿真 | 后续计划 |
| 系统集成、批量评测、指标与演示 | 后续计划 |

## 许可

采用 [Apache-2.0](src/parking_planner/LICENSE) 许可。
