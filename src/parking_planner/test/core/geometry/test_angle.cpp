#include "parking_planner/core/geometry/angle.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace parking_planner::core::geometry {
namespace {

constexpr double kTolerance = 1e-12;

TEST(AngleTest, NormalizesKnownAngles) {
  struct Case {
    double input;
    double expected;
  };

  const Case cases[] = {
      {0.0, 0.0},         {kPi, -kPi},
      {-kPi, -kPi},       {kTwoPi, 0.0},
      {-kTwoPi, 0.0},     {3.0 * kPi, -kPi},
      {-3.0 * kPi, -kPi}, {-1000.0 * kPi / 180.0, 80.0 * kPi / 180.0},
  };

  for (const auto &item : cases) {
    EXPECT_NEAR(item.expected, normalizeAngle(item.input), kTolerance)
        << "input = " << item.input;
  }
}

TEST(AngleTest, HandlesBothSidesOfBoundary) {
  constexpr double epsilon = 1e-6;

  EXPECT_NEAR(kPi - epsilon, normalizeAngle(kPi - epsilon), kTolerance);
  EXPECT_NEAR(-kPi + epsilon, normalizeAngle(kPi + epsilon), kTolerance);
  EXPECT_NEAR(kPi - epsilon, normalizeAngle(-kPi - epsilon), kTolerance);
  EXPECT_NEAR(-kPi + epsilon, normalizeAngle(-kPi + epsilon), kTolerance);

  const double input = std::nextafter(-kPi, -kTwoPi);
  const double result = normalizeAngle(input);

  EXPECT_GE(result, -kPi);
  EXPECT_LT(result, kPi);
  EXPECT_NEAR(std::sin(input), std::sin(result), kTolerance);
}

TEST(AngleTest, ComputesShortestSignedDifference) {
  const double positive = 179.0 * kPi / 180.0;
  const double negative = -179.0 * kPi / 180.0;
  const double two_degrees = 2.0 * kPi / 180.0;

  EXPECT_NEAR(two_degrees, angleDiff(positive, negative), kTolerance);
  EXPECT_NEAR(-two_degrees, angleDiff(negative, positive), kTolerance);

  EXPECT_NEAR(-kPi, angleDiff(0.0, kPi), kTolerance);
}

TEST(AngleTest, InterpolatesAcrossBoundary) {
  const double from = 179.0 * kPi / 180.0;
  const double to = -179.0 * kPi / 180.0;

  EXPECT_NEAR(from, interpolateAngle(from, to, 0.0), kTolerance);
  EXPECT_NEAR(to, interpolateAngle(from, to, 1.0), kTolerance);

  EXPECT_NEAR(179.5 * kPi / 180.0, interpolateAngle(from, to, 0.25),
              kTolerance);
  EXPECT_NEAR(-kPi, interpolateAngle(from, to, 0.5), kTolerance);
}

TEST(AngleTest, RejectsInvalidInputs) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double invalid_values[] = {infinity, -infinity, nan};

  for (double value : invalid_values) {
    EXPECT_THROW(normalizeAngle(value), std::invalid_argument);

    EXPECT_THROW(angleDiff(value, 0.0), std::invalid_argument);
    EXPECT_THROW(angleDiff(0.0, value), std::invalid_argument);

    EXPECT_THROW(interpolateAngle(value, 0.0, 0.5), std::invalid_argument);
    EXPECT_THROW(interpolateAngle(0.0, value, 0.5), std::invalid_argument);
    EXPECT_THROW(interpolateAngle(0.0, 1.0, value), std::invalid_argument);
  }

  EXPECT_THROW(interpolateAngle(0.0, 1.0, -0.1), std::invalid_argument);
  EXPECT_THROW(interpolateAngle(0.0, 1.0, 1.1), std::invalid_argument);
}

} // namespace
} // namespace parking_planner::core::geometry