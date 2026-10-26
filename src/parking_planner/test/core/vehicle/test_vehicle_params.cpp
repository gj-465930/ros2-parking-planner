#include "parking_planner/core/vehicle/vehicle_params.h"

#include "parking_planner/core/geometry/angle.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace parking_planner::core::vehicle {
namespace {

constexpr double kTolerance = 1e-9;

using geometry::kPi;
using geometry::Pose2d;
using geometry::Vec2d;

TEST(VehicleParamsTest, StoresParametersAndComputesDerivedGeometry) {
  const VehicleParams params(2.8, 2.0, 1.0, 0.8, kPi / 4.0);

  EXPECT_NEAR(2.8, params.wheelbase(), kTolerance);
  EXPECT_NEAR(2.0, params.width(), kTolerance);
  EXPECT_NEAR(1.0, params.frontOverhang(), kTolerance);
  EXPECT_NEAR(0.8, params.rearOverhang(), kTolerance);
  EXPECT_NEAR(kPi / 4.0, params.maxSteeringAngle(), kTolerance);

  EXPECT_NEAR(4.6, params.length(), kTolerance);
  EXPECT_NEAR(1.5, params.centerOffset(), kTolerance);

  // tan(pi/4) = 1，所以最小半径等于轴距。
  EXPECT_NEAR(2.8, params.minTurningRadius(), kTolerance);
}

TEST(VehicleParamsTest, PlacesCenterUsingRearAxlePose) {
  const VehicleParams params(2.8, 2.0, 1.0, 0.8, kPi / 4.0);

  struct Case {
    Pose2d pose;
    double center_x;
    double center_y;
  };

  const Case cases[] = {
      {{0.0, 0.0, 0.0}, 1.5, 0.0},
      {{2.0, 3.0, 0.0}, 3.5, 3.0},
      {{2.0, 3.0, kPi / 2.0}, 2.0, 4.5},
      {{2.0, 3.0, -kPi / 2.0}, 2.0, 1.5},
      {{2.0, 3.0, kPi}, 0.5, 3.0},
      {{2.0, 3.0, kPi / 2.0 + 2.0 * kPi}, 2.0, 4.5},
  };

  for (const auto &item : cases) {
    const auto box = params.footprint(item.pose);

    EXPECT_NEAR(item.center_x, box.center().x(), kTolerance);
    EXPECT_NEAR(item.center_y, box.center().y(), kTolerance);
    EXPECT_NEAR(item.pose.theta, box.heading(), kTolerance);
    EXPECT_NEAR(4.6, box.length(), kTolerance);
    EXPECT_NEAR(2.0, box.width(), kTolerance);
  }
}

TEST(VehicleParamsTest, ProducesCorrectCornersAtNinetyDegrees) {
  const VehicleParams params(2.8, 2.0, 1.0, 0.8, kPi / 4.0);
  const auto box = params.footprint(Pose2d{2.0, 3.0, kPi / 2.0});

  // 局部前边界 x=3.8，后边界 x=-0.8，左右边界 y=±1。
  // 朝世界 +y 时，车身左侧对应世界 -x。
  const std::array<Vec2d, 4> expected = {
      Vec2d(1.0, 6.8),
      Vec2d(1.0, 2.2),
      Vec2d(3.0, 2.2),
      Vec2d(3.0, 6.8),
  };

  const auto corners = box.corners();

  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(expected[i].x(), corners[i].x(), kTolerance);
    EXPECT_NEAR(expected[i].y(), corners[i].y(), kTolerance);
  }
}

TEST(VehicleParamsTest, ProducesCorrectGeometryAtFortyFiveDegrees) {
  const VehicleParams params(2.8, 2.0, 1.0, 0.8, kPi / 4.0);
  const auto box = params.footprint(Pose2d{2.0, 3.0, kPi / 4.0});

  const double q = std::sqrt(0.5);

  EXPECT_NEAR(2.0 + 1.5 * q, box.center().x(), kTolerance);
  EXPECT_NEAR(3.0 + 1.5 * q, box.center().y(), kTolerance);

  const std::array<Vec2d, 4> expected = {
      Vec2d(2.0 + 2.8 * q, 3.0 + 4.8 * q),
      Vec2d(2.0 - 1.8 * q, 3.0 + 0.2 * q),
      Vec2d(2.0 + 0.2 * q, 3.0 - 1.8 * q),
      Vec2d(2.0 + 4.8 * q, 3.0 + 2.8 * q),
  };

  const auto corners = box.corners();

  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(expected[i].x(), corners[i].x(), kTolerance);
    EXPECT_NEAR(expected[i].y(), corners[i].y(), kTolerance);
  }
}

TEST(VehicleParamsTest, AcceptsZeroOverhangs) {
  const VehicleParams params(2.8, 2.0, 0.0, 0.0, kPi / 4.0);
  const auto box = params.footprint(Pose2d{});

  EXPECT_NEAR(2.8, params.length(), kTolerance);
  EXPECT_NEAR(1.4, params.centerOffset(), kTolerance);

  const auto corners = box.corners();

  EXPECT_NEAR(2.8, corners[0].x(), kTolerance);
  EXPECT_NEAR(0.0, corners[1].x(), kTolerance);
}

TEST(VehicleParamsTest, AcceptsZeroAndNegativeCenterOffsets) {
  // 后轴到车头为 3 米，后悬也是 3 米，中心恰好在后轴上。
  const VehicleParams centered(2.0, 2.0, 1.0, 3.0, kPi / 4.0);

  EXPECT_NEAR(0.0, centered.centerOffset(), kTolerance);

  const auto centered_box = centered.footprint(Pose2d{2.0, 3.0, kPi / 2.0});

  EXPECT_NEAR(2.0, centered_box.center().x(), kTolerance);
  EXPECT_NEAR(3.0, centered_box.center().y(), kTolerance);

  // 后悬增至 5 米，几何中心位于后轴后方 1 米。
  const VehicleParams rear_heavy(2.0, 2.0, 1.0, 5.0, kPi / 4.0);

  EXPECT_NEAR(8.0, rear_heavy.length(), kTolerance);
  EXPECT_NEAR(-1.0, rear_heavy.centerOffset(), kTolerance);

  const auto rear_box = rear_heavy.footprint(Pose2d{2.0, 3.0, kPi / 2.0});

  EXPECT_NEAR(2.0, rear_box.center().x(), kTolerance);
  EXPECT_NEAR(2.0, rear_box.center().y(), kTolerance);
}

TEST(VehicleParamsTest, RejectsInvalidDimensionsAndSteeringLimits) {
  // 每行依次为：轴距、车宽、前悬、后悬、最大前轮转角。
  const std::array<double, 5> cases[] = {
      {0.0, 2.0, 1.0, 0.8, 0.5},       {-2.8, 2.0, 1.0, 0.8, 0.5},
      {2.8, 0.0, 1.0, 0.8, 0.5},       {2.8, -2.0, 1.0, 0.8, 0.5},
      {2.8, 2.0, -1.0, 0.8, 0.5},      {2.8, 2.0, 1.0, -0.8, 0.5},
      {2.8, 2.0, 1.0, 0.8, 0.0},       {2.8, 2.0, 1.0, 0.8, -0.5},
      {2.8, 2.0, 1.0, 0.8, kPi / 2.0}, {2.8, 2.0, 1.0, 0.8, kPi},
  };

  for (const auto &values : cases) {
    EXPECT_THROW(
        (VehicleParams{values[0], values[1], values[2], values[3], values[4]}),
        std::invalid_argument);
  }
}

TEST(VehicleParamsTest, RejectsNonFiniteParameters) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (double value : {infinity, -infinity, nan}) {
    for (std::size_t i = 0; i < 5; ++i) {
      std::array<double, 5> values = {2.8, 2.0, 1.0, 0.8, 0.5};
      values[i] = value;

      EXPECT_THROW((VehicleParams{values[0], values[1], values[2], values[3],
                                  values[4]}),
                   std::invalid_argument);
    }
  }
}

TEST(VehicleParamsTest, RejectsOverflowInDerivedGeometry) {
  const double maximum = std::numeric_limits<double>::max();

  // 输入分别有限，但总长相加溢出。
  EXPECT_THROW((VehicleParams{maximum, 2.0, maximum, 0.8, 0.5}),
               std::invalid_argument);

  // 总长仍有限，但过大的轴距除以较小 tan(delta) 后半径溢出。
  EXPECT_THROW((VehicleParams{maximum, 2.0, 0.0, 0.0, 0.1}),
               std::invalid_argument);
}

TEST(VehicleParamsTest, RejectsNonFiniteRearAxlePose) {
  const VehicleParams params(2.8, 2.0, 1.0, 0.8, kPi / 4.0);

  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (double value : {infinity, -infinity, nan}) {
    const Pose2d poses[] = {
        {value, 0.0, 0.0},
        {0.0, value, 0.0},
        {0.0, 0.0, value},
    };

    for (const auto &pose : poses) {
      EXPECT_THROW(params.footprint(pose), std::invalid_argument);
    }
  }
}

} // namespace
} // namespace parking_planner::core::vehicle