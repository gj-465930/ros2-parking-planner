# 车辆参考点、运动传播与栅格约定：Autoware 源码阅读笔记

本文对照 Autoware Universe 的车辆运动传播与栅格转换片段，说明本项目采用的接口和取舍。
阅读版本固定为 `2a9e2b7905bb4031f7e6f619e9a7716d079d8f9f`，不以持续变化的 `main` 作为引用依据。
未编译或运行 Autoware；以下结论限定在列出的文件和函数。

## 阅读范围

| 文件 | 本文涉及的范围 |
|---|---|
| [kinematic_bicycle_model.hpp](https://github.com/autowarefoundation/autoware_universe/blob/2a9e2b7905bb4031f7e6f619e9a7716d079d8f9f/planning/autoware_freespace_planning_algorithms/include/autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp) | 完整读取；重点为 `getTurningRadius`、`getPose` |
| [abstract_algorithm.hpp](https://github.com/autowarefoundation/autoware_universe/blob/2a9e2b7905bb4031f7e6f619e9a7716d079d8f9f/planning/autoware_freespace_planning_algorithms/include/autoware/freespace_planning_algorithms/abstract_algorithm.hpp) | 选择性读取 `VehicleShape`、坐标转换声明、`isOutOfRange`、`isObs`、`indexToId` |
| [abstract_algorithm.cpp](https://github.com/autowarefoundation/autoware_universe/blob/2a9e2b7905bb4031f7e6f619e9a7716d079d8f9f/planning/autoware_freespace_planning_algorithms/src/abstract_algorithm.cpp) | 选择性读取 `pose2index`、`index2pose`、`global2local`、`local2global`、`setMap` 的占用表初始化片段 |

这里没有展开 Autoware 的距离场、碰撞缓存或完整规划流程，不能据此概括其全部碰撞检测策略。

## 后轴参考点与车框中心

运动状态和几何形状可以采用不同参考点，但两者必须通过明确的变换关联。
本项目 `Pose2d` 的车辆位姿表示后轴中心，局部 x 朝前、y 朝左。
设轴距为 L，前悬为 f，后悬为 r，车宽为 w，则相对后轴的车身范围是：

- 纵向：从 `-r` 到 `L + f`。
- 横向：从 `-w/2` 到 `w/2`。

所以总长和几何中心纵向偏移分别为：

$$
\ell=L+f+r,\qquad d=\frac{L+f-r}{2}.
$$

后轴世界位姿为 `(x, y, theta)` 时，车框中心为：

$$
c_x=x+d\cos\theta,\qquad c_y=y+d\sin\theta.
$$

Autoware 的 `VehicleShape` 保存 `length`、`width`、`base_length`、`max_steering` 和 `base2back`。
在没有额外 margin 时，`base_length` 对应轴距，`base2back` 对应后悬。
`length - base2back` 是参考点到车头的距离，包含轴距与前悬，不能直接当作前悬。

本项目 [VehicleParams](../../src/parking_planner/include/parking_planner/core/vehicle/vehicle_params.h)
以五个基础参数构造，缓存总长、中心偏移和最小转弯半径，只提供查询接口。
`footprint()` 先将局部中心 `(d, 0)` 变换到世界坐标，再构造有向矩形。
RViz 车框使用这一轮廓，箭头则仍从后轴出发；两者没有必要重合。

## 固定转角的圆弧传播

低速、平面、无侧滑的自行车模型，以后轴中心为参考：

$$
\dot x=v\cos\theta,\qquad
\dot y=v\sin\theta,\qquad
\dot\theta=\frac{v\tan\delta}{L}.
$$

这里 delta 是等效前轮转角，不是方向盘角。给定固定 delta 和后轴有符号弧长 s：

$$
\kappa=\frac{\tan\delta}{L},\qquad \beta=s\kappa.
$$

s 前进为正、倒车为负；theta 始终表示车头朝向，倒车不需要把 theta 加 pi。
非零曲率下，`R = 1/kappa` 为带符号的后轴轨迹半径，圆弧终点为：

$$
x_1=x+R[\sin(\theta+\beta)-\sin\theta],
$$

$$
y_1=y+R[\cos\theta-\cos(\theta+\beta)],\qquad
\theta_1=\theta+\beta.
$$

Autoware 的 `getPose` 从 ROS Pose 四元数提取 yaw，用上述圆弧公式更新位置，再生成终点四元数。
当 `abs(steering_angle) < 0.001` 时，该版本改为按初始 yaw 直行，保持原朝向。
这是小转角近似；对于非零转角，不能脱离行驶距离将它理解为完全精确的直线运动。

本项目 [propagateExact](../../src/parking_planner/src/core/vehicle/kinematic_bicycle_model.cpp)
保持纯 C++ 接口，输入 `Pose2d`、`VehicleParams`、转角和有符号距离，返回新的 `Pose2d`。
为避免零转角除零，以及小角度下相近三角函数相减的精度损失，使用等价表达：

$$
u=\frac{\beta}{2},\qquad \operatorname{sinc}(u)=\frac{\sin u}{u},
$$

$$
x_1=x+s\operatorname{sinc}(u)\cos(\theta+u),\qquad
y_1=y+s\operatorname{sinc}(u)\sin(\theta+u).
$$

定义 `sinc(0)=1`；实现对 `abs(u) < 1e-4` 使用 `1-u²/6+u⁴/120` 展开。
这个局部多项式存在截断误差，因此“精确传播”指固定输入下的圆弧解析模型，计算仍受浮点误差影响。

另一接口 `propagateEuler()` 用步首航向计算位置增量，供数值近似对照。
调用方可以拆分距离反复调用；固定输入下多步欧拉趋近圆弧解，不需要在精确传播后再调用欧拉。
两个接口都检查输入和转角限幅，并将输出航向归一化到 `[-pi, pi)`；它们不负责碰撞检测。

## 栅格转换先明确语义

Autoware 此处的 `pose2index` 使用 `round(local_position / resolution)`，
`index2pose` 则用 `index * resolution` 作为代表位置，没有半格偏移。
`global2local/local2global` 使用地图原点的刚体变换，能表达旋转的地图原点。

本项目 [OccupancyGrid](../../src/parking_planner/src/core/map/occupancy_grid.cpp)
只表达轴对齐地图，查询的是世界点所处的半开单元。对左下角原点 `(ox, oy)`、分辨率 h：

$$
i=\left\lfloor\frac{x-o_x}{h}\right\rfloor,\qquad
j=\left\lfloor\frac{y-o_y}{h}\right\rfloor.
$$

反向返回格子中心 `(ox + (i+0.5)h, oy + (j+0.5)h)`，不是还原原始查询点。
世界点到格子是多对一映射，往返测试应验证格子归属一致，不能要求任意点精确往返。

| 项目 | 所读 Autoware 片段 | 本项目 |
|---|---|---|
| 位置离散化 | `round`，选择离散代表位置 | `floor`，查询所属半开单元 |
| 反向位置 | `index * resolution` | 格子中心，含半格偏移 |
| 地图坐标变换 | 地图原点刚体变换 | 轴对齐，仅减原点 |
| 二维展平 | `y * width + x` | 同一行优先布局 |
| 占用输入 | `setMap` 将未知值及达到阈值的成本标为障碍 | 静态凸障碍与格子正方形重叠时标占用 |

两种索引转换服务于不同约定，不应只替换 `round/floor` 或半格偏移而保留其余假设。
本项目世界点越界查询返回 `nullopt`，无效整数索引访问抛出异常。

## 环境碰撞与显示验证的边界

本项目环境同时保留原始凸障碍物和占据栅格。
格子与障碍物接触也算占用，栅格表示会偏保守；完整车辆碰撞检查则对车框与原始多边形做 SAT，
并检查车框是否接触地图边界或越界，使用各自的数值容差。
这是本项目的几何选择，不表示与 Autoware 的完整碰撞策略相同。

场景加载器要求起终完整车框合法，并检查目标车框是否位于车位内。
静态显示可以帮助核对参考点、坐标与尺寸，却不能证明存在满足转弯约束的路径。
后续搜索仍需检查候选运动途中多个位姿的完整车框；离散采样本身也不能自动保证采样间隔内绝无碰撞。

当前本地行为测试覆盖参考点偏移、旋转车框、固定转角前进/倒车、欧拉分步对照、栅格边界、
原始障碍碰撞、场景输入与可视化消息。验证入口与三场景启动方法见 [README](../../README.md)。
测试通过不代表已经与 Autoware 做过数值基准对比，当前没有这种对比结果或性能结论。
