#include "parking_planner/core/geometry/line_segment2d.h"

#include "parking_planner/core/geometry/vec2d.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace parking_planner::core::geometry {

namespace {
constexpr double kGeometryEpsilon = 1e-9;
constexpr double kGeometryEpsilonSquared = kGeometryEpsilon * kGeometryEpsilon;

bool isDegenerate(const Vec2d &direction) {
  return direction.lengthSquared() <= kGeometryEpsilonSquared;
}

int sigWithTolerance(double value, double tolerance) {
  if (value > tolerance) {
    return 1;
  }

  if (value < -tolerance) {
    return -1;
  }

  return 0;
}

} // namespace

LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end), direction_((end - start)) {
  length_ = direction_.length();
}

std::optional<double>
LineSegment2d::projectionParameter(const Vec2d &point) const {
  if (isDegenerate(direction_)) {
    return std::nullopt;
  }

  const Vec2d offset = point - start_;
  const double denominator = direction_.lengthSquared();

  return offset.dot(direction_) / denominator;
}

Vec2d LineSegment2d::nearestPoint(const Vec2d &point) const {
  const std::optional<double> parameter = projectionParameter(point);

  if (!parameter.has_value()) {
    return start_;
  }

  const double clamped_parameter = std::clamp(parameter.value(), 0.0, 1.0);

  return start_ + direction_ * clamped_parameter;
}

double LineSegment2d::distanceTo(const Vec2d &point) const {
  Vec2d nearestpoint = nearestPoint(point);
  return (point - nearestpoint).length();
}

bool LineSegment2d::isPointIn(const Vec2d &point) const {
  if (isDegenerate(direction_)) {
    return (point - start_).length() <= kGeometryEpsilon;
  }

  Vec2d offset = point - start_;
  const double cross_tolerance = kGeometryEpsilon * length_;

  if (std::abs(direction_.cross(offset)) > cross_tolerance) {
    return false;
  }

  const double projected_distance = offset.dot(direction_) / length_;

  return projected_distance >= -kGeometryEpsilon &&
         projected_distance <= length_ + kGeometryEpsilon;
}

bool LineSegment2d::hasIntersect(const LineSegment2d &other) const {
  if (isDegenerate(direction_)) {
    return other.isPointIn(start_);
  }

  if (isDegenerate(other.direction_)) {
    return isPointIn(other.start_);
  }

  if (isPointIn(other.start_) || isPointIn(other.end_) ||
      other.isPointIn(start_) || other.isPointIn(end_)) {
    return true;
  }

  const double this_cross_tolerance = kGeometryEpsilon * length_;
  const double other_cross_torlerance = kGeometryEpsilon * other.length_;

  const double orientation_1 = direction_.cross(other.start_ - start_);
  const double orientation_2 = direction_.cross(other.end_ - start_);

  const double orientation_3 = other.direction_.cross(start_ - other.start_);
  const double orientation_4 = other.direction_.cross(end_ - other.start_);

  const int side_1 = sigWithTolerance(orientation_1, this_cross_tolerance);
  const int side_2 = sigWithTolerance(orientation_2, this_cross_tolerance);
  const int side_3 = sigWithTolerance(orientation_3, other_cross_torlerance);
  const int side_4 = sigWithTolerance(orientation_4, other_cross_torlerance);

  return side_1 != 0 && side_2 != 0 && side_3 != 0 && side_4 != 0 &&
         side_1 != side_2 && side_3 != side_4;
}

} // namespace parking_planner::core::geometry