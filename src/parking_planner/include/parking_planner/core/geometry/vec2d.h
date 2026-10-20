#pragma once

namespace parking_planner::core::geometry {

class Vec2d {
public:
  Vec2d() = default;

  Vec2d(double x, double y);

  double x() const { return x_; }
  double y() const { return y_; }

  double length() const;
  double lengthSquared() const;

  double dot(const Vec2d &other) const;
  double cross(const Vec2d &other) const;

  // 逆时针旋转
  Vec2d rotate(double angle) const;

  Vec2d operator+(const Vec2d &other) const;
  Vec2d operator-(const Vec2d &other) const;
  Vec2d operator*(double scalar) const;

private:
  double x_ = 0.0;
  double y_ = 0.0;
};

} // namespace parking_planner::core::geometry