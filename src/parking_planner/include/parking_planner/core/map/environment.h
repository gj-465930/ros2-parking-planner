#pragma once

#include <vector>

#include "parking_planner/core/geometry/convex_polygon2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"
#include "parking_planner/core/map/occupancy_grid.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

namespace parking_planner::core::map {

/**
 * @brief 保存静态障碍物、占据栅格并提供车辆碰撞查询。
 *
 * 地图为轴对齐的矩形区域，内部无障碍区域默认允许行驶。
 * 栅格由障碍物生成，不单独修改。
 * 不表示道路语义，不检查两个位姿之间的连续运动碰撞。
 */
class Environment {
public:
  /**
   * @brief 创建环境，并根据障碍物生成占据栅格。
   *
   * 格子正方形与任一障碍物重叠时标为占用，
   * 边接触、角接触及既有 SAT 容差范围内的接近均算重叠。
   *
   * @param origin 地图左下角世界坐标，单位米。
   * @param resolution 格子边长，单位米。
   * @param width 水平方向格子数量。
   * @param height 竖直方向格子数量。
   * @param obstacles 同一世界坐标系下的凸障碍物。
   *        允许障碍物部分或全部位于地图外。
   * @throws std::invalid_argument 地图参数或几何计算非法。
   * @throws std::length_error 格子数量超过容器容量上限。
   * @throws std::bad_alloc 内存分配失败。
   */
  Environment(const geometry::Vec2d &origin, double resolution, int width,
              int height,
              const std::vector<geometry::ConvexPolygon2d> &obstacles);

  /**
   * @brief 只读访问根据障碍物生成的占据栅格。
   * @return 内部地图的引用，生命周期依附于当前环境。
   */
  const OccupancyGrid &grid() const { return grid_; }

  /**
   * @brief 只读访问原始障碍物轮廓。
   * @return 内部障碍物容器的引用，生命周期依附于当前环境。
   */
  const std::vector<geometry::ConvexPolygon2d> &obstacles() const {
    return obstacles_;
  }

  /**
   * @brief 查询给定位姿下的完整车辆是否碰撞。
   *
   * 使用车辆矩形与原始障碍物做 SAT，不以占用格子替代障碍物。
   * 接触障碍物、接触地图边界或越界均返回 true。
   * 距地图边界不超过 1e-9 米时也按接触处理。
   *
   * @param rear_axle_pose 后轴中心世界位姿，单位米、弧度。
   * @param params 已通过构造校验的车辆参数，不额外膨胀车框。
   * @return 车框完全处于地图内部且不接触障碍物时返回 false。
   * @throws std::invalid_argument 位姿非法或几何计算产生非有限值。
   */
  bool hasCollision(const geometry::Pose2d &rear_axle_pose,
                    const vehicle::VehicleParams &params) const;

private:
  void rasterizeObstacles();

  OccupancyGrid grid_; ///< 保存地图范围及生成的占用状态

  std::vector<geometry::ConvexPolygon2d> obstacles_; ///< 原始障碍物轮廓
};

} // namespace parking_planner::core::map
