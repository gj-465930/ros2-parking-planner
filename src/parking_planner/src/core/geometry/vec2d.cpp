#include "parking_planner/core/geometry/vec2d.h"

#include <cmath>
#include <stdexcept>

namespace parking_planner::core::geometry {

Vec2d::Vec2d(double x, double y) : x_(x), y_(y) {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    throw std::invalid_argument("coordinates must be finite");
  }
}

double Vec2d::length() const { return std::hypot(x_, y_); }

double Vec2d::lengthSquared() const { return x_ * x_ + y_ * y_; }

double Vec2d::dot(const Vec2d &other) const {
  return x_ * other.x_ + y_ * other.y_;
}

double Vec2d::cross(const Vec2d &other) const {
  return x_ * other.y_ - other.x_ * y_;
}

Vec2d Vec2d::rotate(double angle) const {
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("angle must be finite");
  }

  const double cos_theta = std::cos(angle);
  const double sin_theta = std::sin(angle);

  return Vec2d(x_ * cos_theta - y_ * sin_theta,
               x_ * sin_theta + y_ * cos_theta);
}

Vec2d Vec2d::operator+(const Vec2d &other) const {
  return Vec2d(x_ + other.x_, y_ + other.y_);
}

Vec2d Vec2d::operator-(const Vec2d &other) const {
  return Vec2d(x_ - other.x_, y_ - other.y_);
}

Vec2d Vec2d::operator*(const double scalar) const {
  if (!std::isfinite(scalar)) {
    throw std::invalid_argument("scalar must be finite");
  }

  return Vec2d(x_ * scalar, y_ * scalar);
}

} // namespace parking_planner::core::geometry