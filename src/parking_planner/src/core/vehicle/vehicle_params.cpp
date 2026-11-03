#include "parking_planner/core/vehicle/vehicle_params.h"
#include "parking_planner/core/geometry/angle.h"
#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/transform2d.h"
#include "parking_planner/core/geometry/vec2d.h"

#include <cmath>
#include <stdexcept>

namespace parking_planner::core::vehicle {

VehicleParams::VehicleParams(double wheelbase, double width,
                             double front_overhang, double rear_overhang,
                             double max_steering_angle)
    : wheelbase_(wheelbase), width_(width), front_overhang_(front_overhang),
      rear_overhang_(rear_overhang), max_steering_angle_(max_steering_angle) {
  if (!std::isfinite(wheelbase_) || !std::isfinite(width_) ||
      !std::isfinite(front_overhang_) || !std::isfinite(rear_overhang_) ||
      !std::isfinite(max_steering_angle_)) {
    throw std::invalid_argument("vegucke parameters must be finite");
  }

  if (wheelbase_ <= 0.0 || width_ <= 0.0 || front_overhang_ < 0.0 ||
      rear_overhang_ < 0.0) {
    throw std::invalid_argument(
        "wheelbase and width must be positive; overhangs must be nonnegative");
  }

  if (max_steering_angle_ <= 0.0 ||
      max_steering_angle_ >= geometry::kPi / 2.0) {
    throw std::invalid_argument("maximum steering angle must be in (0, pi/2)");
  }

  const double front_extent = wheelbase_ + front_overhang_;

  length_ = front_extent + rear_overhang_;
  center_offset_ = (front_extent - rear_overhang_) / 2.0;
  min_turning_radius_ = wheelbase_ / std::tan(max_steering_angle_);

  if (!std::isfinite(length_) || !std::isfinite(center_offset_) ||
      !std::isfinite(min_turning_radius_) || length_ <= 0.0 ||
      min_turning_radius_ <= 0.0) {
    throw std::invalid_argument("derived vehicle geometry must be finite with "
                                "positive length and radius");
  }
}

geometry::OrientedRectangle2d
VehicleParams::footprint(const geometry::Pose2d &rear_axle_pose) const {
  const geometry::Vec2d local_center(center_offset_, 0.0);

  const geometry::Vec2d world_center =
      geometry::transformToGlobal(local_center, rear_axle_pose);

  return geometry::OrientedRectangle2d(world_center, rear_axle_pose.theta,
                                       length_, width_);
}

} // namespace parking_planner::core::vehicle