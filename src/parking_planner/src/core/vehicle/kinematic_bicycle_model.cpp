#include "parking_planner/core/vehicle/kinematic_bicycle_model.h"
#include "parking_planner/core/geometry/angle.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

#include <cmath>
#include <stdexcept>

namespace parking_planner::core::vehicle {
namespace {

void validateInputs(const geometry::Pose2d &pose, const VehicleParams &params,
                    double steering_angle, double distance) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.theta) || !std::isfinite(steering_angle) ||
      !std::isfinite(distance)) {
    throw std::invalid_argument("propagate  inputs must be finite");
  }

  if (std::abs(steering_angle) > params.maxSteeringAngle()) {
    throw std::invalid_argument("steering angle exceeds vehicle limit");
  }
}

double headingChange(const VehicleParams &param, double steering_angle,
                     double distance) {
  const double curvature = std::tan(steering_angle) / param.wheelbase();
  const double beta = distance * curvature;

  if (!std::isfinite(curvature) || !std::isfinite(beta)) {
    throw std::invalid_argument("heading change must be finite");
  }

  return beta;
}

double sinc(double value) {
  if (std::abs(value) < 1e-4) {
    const double squared = value * value;
    // 泰勒展开
    return 1.0 - squared / 6.0 + squared * squared / 120.0;
  }

  return std::sin(value) / value;
}

geometry::Pose2d makeResult(double x, double y, double theta) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta)) {
    throw std::invalid_argument("propagated pose must be finite");
  }

  return {x, y, geometry::normalizeAngle(theta)};
}

} // namespace
geometry::Pose2d propagateExact(const geometry::Pose2d &pose,
                                const VehicleParams &params,
                                double steering_angle, double distance) {
  validateInputs(pose, params, steering_angle, distance);

  const double theta = geometry::normalizeAngle(pose.theta);
  const double beta = headingChange(params, steering_angle, distance);
  const double half_beta = beta / 2.0;

  const double x =
      distance * sinc(half_beta) * std::cos(theta + half_beta) + pose.x;
  const double y =
      distance * sinc(half_beta) * std::sin(theta + half_beta) + pose.y;
  return makeResult(x, y, theta + beta);
}

geometry::Pose2d propagateEuler(const geometry::Pose2d &pose,
                                const VehicleParams &params,
                                double steering_angle, double distance) {
  validateInputs(pose, params, steering_angle, distance);

  const double theta = geometry::normalizeAngle(pose.theta);
  const double beta = headingChange(params, steering_angle, distance);

  const double x = pose.x + distance * std::cos(theta);
  const double y = pose.y + distance * std::sin(theta);

  return makeResult(x, y, theta + beta);
}

} // namespace parking_planner::core::vehicle