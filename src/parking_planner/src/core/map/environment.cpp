#include <vector>

#include "parking_planner/core/geometry/convex_polygon2d.h"
#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"
#include "parking_planner/core/map/environment.h"
#include "parking_planner/core/map/occupancy_grid.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

namespace parking_planner::core::map {
namespace {

constexpr double kBoundaryTolerance = 1e-9;

}

Environment::Environment(
    const geometry::Vec2d &origin, double resolution, int width, int height,
    const std::vector<geometry::ConvexPolygon2d> &obstacles)
    : grid_(origin, resolution, width, height), obstacles_(obstacles) {
  rasterizeObstacles();
}

void Environment::rasterizeObstacles() {
  for (int y = 0; y < grid_.height(); ++y) {
    for (int x = 0; x < grid_.width(); ++x) {
      const GridIndex index{x, y};

      const geometry::OrientedRectangle2d cell_rectangle(
          grid_.gridToWorld(index), 0.0, grid_.resolution(),
          grid_.resolution());

      for (const auto &obstacle : obstacles_) {
        if (obstacle.hasOverlap(cell_rectangle)) {
          grid_.setOccupied(index, true);

          // 只要有障碍物占用就不需要判断其他障碍物是否占用这个格子了
          break;
        }
      }
    }
  }
}

bool Environment::hasCollision(const geometry::Pose2d &rear_axle_pose,
                               const vehicle::VehicleParams &params) const {
  const geometry::OrientedRectangle2d footprint =
      params.footprint(rear_axle_pose);

  const auto origin = grid_.origin();
  const double max_x = grid_.maxX();
  const double max_y = grid_.maxY();

  // 四个角不能超处地图边界
  for (const auto &corner : footprint.corners()) {
    if (corner.x() - origin.x() <= kBoundaryTolerance ||
        max_x - corner.x() <= kBoundaryTolerance ||
        corner.y() - origin.y() <= kBoundaryTolerance ||
        max_y - corner.y() <= kBoundaryTolerance) {
      return true;
    }
  }

  for (const auto &obstacle : obstacles_) {
    if (obstacle.hasOverlap(footprint)) {
      return true;
    }
  }
  return false;
}

} // namespace parking_planner::core::map
