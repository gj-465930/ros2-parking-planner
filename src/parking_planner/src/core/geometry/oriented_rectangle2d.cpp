#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/transform2d.h"
#include "parking_planner/core/geometry/vec2d.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace parking_planner::core::geometry {

namespace {
constexpr double kGeometryEpsilon = 1e-9;

double projectionRadius(double half_length, double half_width,
                        const Vec2d &longitudinal_axis,
                        const Vec2d &lateral_axis,
                        const Vec2d &projection_axis) {
  return half_length * std::abs(longitudinal_axis.dot(projection_axis)) +
         half_width * std::abs(lateral_axis.dot(projection_axis));
}
} // namespace

OrientedRectangle2d::OrientedRectangle2d(const Vec2d &center, double heading,
                                         double length, double width)
    : center_(center), heading_(heading), length_(length), width_(width) {
  if (!std::isfinite(heading)) {
    throw std::invalid_argument("heading must be finite");
  }

  if (!std::isfinite(length) || length <= 0.0) {
    throw std::invalid_argument("length must be finite and positive");
  }

  if (!std::isfinite(width) || width <= 0.0) {
    throw std::invalid_argument("width must be finite and positive");
  }
}

std::array<Vec2d, 4> OrientedRectangle2d::corners() const {
  const double half_length = length_ / 2.0;
  const double half_width = width_ / 2.0;

  std::array<Vec2d, 4> local_corners{
      Vec2d(half_length, half_width), Vec2d(-half_length, half_width),
      Vec2d(-half_length, -half_width), Vec2d(half_length, -half_width)};

  std::array<Vec2d, 4> world_corners;

  Pose2d global_coordinate_pose;
  global_coordinate_pose.x = center_.x();
  global_coordinate_pose.y = center_.y();
  global_coordinate_pose.theta = heading_;

  for (std::size_t i = 0; i < local_corners.size(); ++i) {

    world_corners[i] =
        transformToGlobal(local_corners[i], global_coordinate_pose);
  }

  return world_corners;
}

bool OrientedRectangle2d::isPointIn(const Vec2d &point) const {
  // 将矩形与点的位姿转换到局部坐标系中判断
  const double half_length = length_ / 2.0;
  const double half_width = width_ / 2.0;

  Pose2d global_coordinate_pose;
  global_coordinate_pose.x = center_.x();
  global_coordinate_pose.y = center_.y();
  global_coordinate_pose.theta = heading_;

  Vec2d local_point = transformToLocal(point, global_coordinate_pose);

  return std::abs(local_point.x()) <= half_length + kGeometryEpsilon &&
         std::abs(local_point.y()) <= half_width + kGeometryEpsilon;
}

bool OrientedRectangle2d::hasOverlap(const OrientedRectangle2d &other) const {
  const double this_cos_heading = std::cos(heading_);
  const double this_sin_heading = std::sin(heading_);

  const Vec2d this_longitudinal_axis(this_cos_heading, this_sin_heading);
  const Vec2d this_lateral_axis(-this_sin_heading, this_cos_heading);

  const double other_cos_heading = std::cos(other.heading_);
  const double other_sin_heading = std::sin(other.heading_);

  const Vec2d other_longitudinal_axis(other_cos_heading, other_sin_heading);
  const Vec2d other_lateral_axis(-other_sin_heading, other_cos_heading);

  const std::array<Vec2d, 4> candidate_axes{
      this_longitudinal_axis, this_lateral_axis, other_longitudinal_axis,
      other_lateral_axis};

  const Vec2d center_offset = other.center_ - center_;

  const double half_length = length_ / 2.0;
  const double half_width = width_ / 2.0;
  const double other_half_length = other.length_ / 2.0;
  const double other_half_width = other.width_ / 2.0;

  for (const Vec2d &axis : candidate_axes) {
    const double center_projection_distance = std::abs(center_offset.dot(axis));

    const double this_projection_radius =
        projectionRadius(half_length, half_width, this_longitudinal_axis,
                         this_lateral_axis, axis);
    const double other_projection_radius =
        projectionRadius(other_half_length, other_half_width,
                         other_longitudinal_axis, other_lateral_axis, axis);

    const double maximum_overlap_distance =
        this_projection_radius + other_projection_radius;

    if (center_projection_distance >
        maximum_overlap_distance + kGeometryEpsilon) {
      return false;
    }
  }
  return true;
}

} // namespace parking_planner::core::geometry