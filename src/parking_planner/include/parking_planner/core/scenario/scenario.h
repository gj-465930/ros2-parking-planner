#pragma once

#include <string>

#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/map/environment.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

namespace parking_planner::core::scenario {

/**
 * @brief 保存一个静态泊车场景的领域数据。
 *
 * 所有位置均使用同一世界坐标系，长度单位为米，
 * 角度单位为弧度，逆时针为正。
 * 起点与目标位姿均以后轴中心为参考点；
 * 车位矩形则以自身的几何中心为参考点。
 *
 * 本类型只组织数据，加载器负责输入与场景合法性校验。
 * 修改公开成员后，需要重新校验相关约束。
 */
struct Scenario {
  std::string name;               ///< 场景名称
  vehicle::VehicleParams vehicle; ///< 车辆尺寸与转向限制
  map::Environment environment;   ///< 地图、原始障碍物及占据栅格
  geometry::Pose2d start;         ///< 起点后轴中心的世界位姿，米、弧度
  geometry::Pose2d goal;          ///< 目标后轴中心的世界位姿，米、弧度
  geometry::OrientedRectangle2d parking_space; ///< 目标车位矩形
};

} // namespace parking_planner::core::scenario
