#include "parking_planner/core/geometry/vec2d.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace parking_planner::core::geometry {
namespace {

constexpr double kTolerance = 1e-12;
constexpr double kPi = 3.14159265358979323846;

TEST(Vec2dTest, ConstructsVectorsAndComputesLength) {
  const Vec2d zero;

  EXPECT_NEAR(0.0, zero.x(), kTolerance);
  EXPECT_NEAR(0.0, zero.y(), kTolerance);
  EXPECT_NEAR(0.0, zero.length(), kTolerance);
  EXPECT_NEAR(0.0, zero.lengthSquared(), kTolerance);

  const Vec2d v(3.0, 4.0);
  EXPECT_NEAR(3.0, v.x(), kTolerance);
  EXPECT_NEAR(4.0, v.y(), kTolerance);
  EXPECT_NEAR(5.0, v.length(), kTolerance);
  EXPECT_NEAR(25.0, v.lengthSquared(), kTolerance);
}

TEST(Vec2dTest, AddsSubtractsAndScales) {
  const Vec2d a(3.0, 4.0);
  const Vec2d b(1.0, 2.0);

  const Vec2d sum = a + b;
  EXPECT_NEAR(4.0, sum.x(), kTolerance);
  EXPECT_NEAR(6.0, sum.y(), kTolerance);

  const Vec2d difference = a - b;
  EXPECT_NEAR(2.0, difference.x(), kTolerance);
  EXPECT_NEAR(2.0, difference.y(), kTolerance);

  const Vec2d scaled = a * 2.0;
  EXPECT_NEAR(6.0, scaled.x(), kTolerance);
  EXPECT_NEAR(8.0, scaled.y(), kTolerance);

  const Vec2d reversed = a * -1.0;
  EXPECT_NEAR(-3.0, reversed.x(), kTolerance);
  EXPECT_NEAR(-4.0, reversed.y(), kTolerance);

  const Vec2d zero = a * 0.0;
  EXPECT_NEAR(0.0, zero.x(), kTolerance);
  EXPECT_NEAR(0.0, zero.y(), kTolerance);

  // 有没有修改原向量
  EXPECT_NEAR(3.0, a.x(), kTolerance);
  EXPECT_NEAR(4.0, a.y(), kTolerance);
}

TEST(Vec2dTest, ComputeDotAndCrossProducts) {
  const Vec2d x_axis(1.0, 0.0);
  const Vec2d y_axis(0.0, 1.0);
  const Vec2d parallel(2.0, 0.0);
  const Vec2d opposite(-1.0, 0.0);

  EXPECT_NEAR(0.0, x_axis.dot(y_axis), kTolerance);
  EXPECT_NEAR(2.0, x_axis.dot(parallel), kTolerance);
  EXPECT_NEAR(-1.0, x_axis.dot(opposite), kTolerance);

  EXPECT_NEAR(1.0, x_axis.cross(y_axis), kTolerance);
  EXPECT_NEAR(0.0, x_axis.cross(parallel), kTolerance);
  EXPECT_NEAR(0.0, x_axis.cross(opposite), kTolerance);
  EXPECT_NEAR(-1.0, y_axis.cross(x_axis), kTolerance);

  const Vec2d a(3.0, 4.0);
  const Vec2d b(1.0, 2.0);

  EXPECT_NEAR(11.0, a.dot(b), kTolerance);
  EXPECT_NEAR(2.0, a.cross(b), kTolerance);
}

TEST(Vec2dTest, RotatesAxisAndZeroVector) {
  const Vec2d v(1.0, 0.0);

  struct Case {
    double angle;
    double expected_x;
    double expected_y;
  };

  const Case cases[] = {
      {0.0, 1.0, 0.0},
      {kPi / 2.0, 0.0, 1.0},
      {-kPi / 2.0, 0.0, -1.0},
      {kPi, -1.0, 0.0},
  };

  for (const auto &item : cases) {
    const Vec2d result = v.rotate(item.angle);

    EXPECT_NEAR(item.expected_x, result.x(), kTolerance);
    EXPECT_NEAR(item.expected_y, result.y(), kTolerance);
  }

  EXPECT_NEAR(1.0, v.x(), kTolerance);
  EXPECT_NEAR(0.0, v.y(), kTolerance);

  const Vec2d zero;
  const Vec2d rotated_zero = zero.rotate(kPi / 3.0);

  EXPECT_NEAR(0.0, rotated_zero.x(), kTolerance);
  EXPECT_NEAR(0.0, rotated_zero.y(), kTolerance);
}

TEST(Vec2dTest, RotationPreservesLengthAndCanBeReversed) {
  const Vec2d original(3.0, 4.0);
  constexpr double angle = 0.37;

  const Vec2d rotated = original.rotate(angle);
  const Vec2d restored = rotated.rotate(-angle);

  EXPECT_NEAR(original.length(), rotated.length(), kTolerance);
  EXPECT_NEAR(original.x(), restored.x(), kTolerance);
  EXPECT_NEAR(original.y(), restored.y(), kTolerance);
}

TEST(Vec2dTest, RejectsNonFiniteInputs) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double invalid_values[] = {infinity, -infinity, nan};

  const Vec2d v(3.0, 4.0);

  for (double value : invalid_values) {
    EXPECT_THROW(Vec2d(value, 0.0), std::invalid_argument);
    EXPECT_THROW(Vec2d(0.0, value), std::invalid_argument);
    EXPECT_THROW(v * value, std::invalid_argument);
    EXPECT_THROW(v.rotate(value), std::invalid_argument);
  }
}

} // namespace
} // namespace parking_planner::core::geometry