#pragma once

#include <string>

#include "parking_planner/core/scenario/scenario.h"

namespace parking_planner::core::scenario {

/**
 * @brief 从 YAML 文件加载并校验一个静态泊车场景。
 *
 * 场景统一使用世界坐标，长度单位为米，角度单位为弧度。
 * start 和 goal 均以后轴中心为参考点。
 *
 * 起终点的完整车辆不得接触障碍物、接触地图边界或越界。
 * 目标车辆轮廓必须位于车位矩形内部或边界上，
 * 包含判断沿用几何模块的浮点容差。
 * 不要求车辆几何中心与车位中心精确重合。
 *
 * 加载成功只表示静态场景有效，不保证存在可行路径。
 *
 * @param file_path 场景文件路径；相对路径基于进程工作目录解析。
 * @return 通过校验的场景对象，独立拥有加载的数据。
 * @throws std::runtime_error 文件无法读取、YAML 解析失败，
 *         或字段、几何、场景约束不合法；错误信息包含文件路径与原因。
 * @throws std::bad_alloc 内存分配失败。
 */
Scenario loadScenarioFromFile(const std::string &file_path);

} // namespace parking_planner::core::scenario