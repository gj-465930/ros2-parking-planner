#include "parking_planner/core/geometry/angle.h"

#include <cmath>
#include <stdexcept>

namespace parking_planner::core::geometry {

double normalizeAngle(double angle) {
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("angle must be finite");
  }

  double a = std::fmod(angle + kPi, kTwoPi);
  if (a < 0) {
    a += kTwoPi;
  }

  const double result = a - kPi;

  // 去除浮点数影响
  if (result >= kPi) {
    return -kPi;
  }

  return result;
}

double angleDiff(double from, double to) {
  const double normalized_from = normalizeAngle(from);
  const double normalized_to = normalizeAngle(to);

  return normalizeAngle(normalized_to - normalized_from);
}

double interpolateAngle(double from, double to, double t) {
  if (!std::isfinite(t) || t < 0.0 || t > 1.0) {
    throw std::invalid_argument("t must be finite and within [0, 1]");
  }

  const double start = normalizeAngle(from);
  const double delta = angleDiff(from, to);

  return normalizeAngle(start + t * delta);
}

} // namespace parking_planner::core::geometry