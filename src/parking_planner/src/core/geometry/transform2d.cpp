#include "parking_planner/core/geometry/transform2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"

#include <cmath>
#include <stdexcept>

namespace parking_planner::core::geometry {

namespace {

void ValidateReferencePose(const Pose2d &pose) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.theta)) {
    throw std::invalid_argument("reference pose must be finite");
  }
}

} // namespace

Vec2d transformToGlobal(const Vec2d &local_point,
                        const Pose2d &reference_pose) {
  ValidateReferencePose(reference_pose);

  const double cos_theta = std::cos(reference_pose.theta);
  const double sin_theta = std::sin(reference_pose.theta);

  const double global_x = reference_pose.x + local_point.x() * cos_theta -
                          local_point.y() * sin_theta;
  const double global_y = reference_pose.y + local_point.x() * sin_theta +
                          local_point.y() * cos_theta;

  return Vec2d(global_x, global_y);
}

Vec2d transformToLocal(const Vec2d &global_point,
                       const Pose2d &reference_pose) {
  ValidateReferencePose(reference_pose);

  const double cos_theta = std::cos(reference_pose.theta);
  const double sin_theta = std::sin(reference_pose.theta);

  const double dx = global_point.x() - reference_pose.x;
  const double dy = global_point.y() - reference_pose.y;

  const double local_x = dy * sin_theta + dx * cos_theta;
  const double local_y = dy * cos_theta - dx * sin_theta;
  return Vec2d(local_x, local_y);
}

} // namespace parking_planner::core::geometry