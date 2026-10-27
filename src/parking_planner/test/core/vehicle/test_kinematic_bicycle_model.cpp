#include "parking_planner/core/vehicle/kinematic_bicycle_model.h"

#include "parking_planner/core/geometry/angle.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>

namespace parking_planner::core::vehicle {
namespace {

using geometry::kPi;
using geometry::Pose2d;

constexpr double kTolerance = 1e-9;

void expectPoseNear(const Pose2d &actual, const Pose2d &expected,
                    double tolerance = kTolerance) {
  EXPECT_NEAR(expected.x, actual.x, tolerance);
  EXPECT_NEAR(expected.y, actual.y, tolerance);
  EXPECT_NEAR(0.0, geometry::angleDiff(expected.theta, actual.theta),
              tolerance);
}

TEST(KinematicBicycleModelTest, SupportsStraightForwardAndReverse) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);
  const Pose2d start{2.0, 3.0, kPi / 2.0};

  // method 依次指向精确传播和欧拉传播，检查共同的直行行为
  for (const auto method : {propagateExact, propagateEuler}) {
    expectPoseNear(method(start, params, 0.0, 2.0),
                   Pose2d{2.0, 5.0, kPi / 2.0});

    expectPoseNear(method(start, params, 0.0, -2.0),
                   Pose2d{2.0, 1.0, kPi / 2.0});
  }
}

TEST(KinematicBicycleModelTest, ZeroDistanceNormalizesHeading) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);
  const Pose2d start{2.0, 3.0, 5.0 * kPi / 2.0};

  for (const auto method : {propagateExact, propagateEuler}) {
    const Pose2d result = method(start, params, 0.3, 0.0);

    expectPoseNear(result, Pose2d{2.0, 3.0, kPi / 2.0});
    EXPECT_GE(result.theta, -kPi);
    EXPECT_LT(result.theta, kPi);
  }

  // 输入位姿不被修改
  EXPECT_DOUBLE_EQ(5.0 * kPi / 2.0, start.theta);
}

TEST(KinematicBicycleModelTest, HandlesLeftRightForwardAndReverse) {
  // L=1、|delta|=pi/4，因此转弯半径的大小为1米
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);

  struct Case {
    double steering;
    double distance;
    Pose2d expected;
  };

  const Case cases[] = {
      {kPi / 4.0, kPi / 2.0, {1.0, 1.0, kPi / 2.0}},
      {-kPi / 4.0, kPi / 2.0, {1.0, -1.0, -kPi / 2.0}},
      {kPi / 4.0, -kPi / 2.0, {-1.0, 1.0, -kPi / 2.0}},
      {-kPi / 4.0, -kPi / 2.0, {-1.0, -1.0, kPi / 2.0}},
  };

  for (const auto &item : cases) {
    expectPoseNear(
        propagateExact(Pose2d{}, params, item.steering, item.distance),
        item.expected);
  }
}

TEST(KinematicBicycleModelTest, HandlesTranslatedAndRotatedStart) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);
  const Pose2d start{2.0, 3.0, 5.0 * kPi / 2.0};

  const Pose2d result = propagateExact(start, params, kPi / 4.0, kPi / 2.0);

  // 初始朝+y，左转四分之一圆，位移为(-1,+1)，最终朝-x
  expectPoseNear(result, Pose2d{1.0, 4.0, -kPi});
  EXPECT_GE(result.theta, -kPi);
  EXPECT_LT(result.theta, kPi);
}

TEST(KinematicBicycleModelTest, MatchesRadiusFormulaForOrdinaryTurn) {
  const VehicleParams params(2.8, 2.0, 1.0, 0.8, 0.6);
  const Pose2d start{2.0, -1.0, 0.37};
  const double steering = 0.3;
  const double distance = 1.7;

  // 使用原始半径公式，交叉检查稳定表达式
  const double radius = 2.8 / std::tan(steering);
  const double beta = distance / radius;

  const Pose2d expected{
      start.x + radius * (std::sin(start.theta + beta) - std::sin(start.theta)),
      start.y + radius * (std::cos(start.theta) - std::cos(start.theta + beta)),
      start.theta + beta,
  };

  expectPoseNear(propagateExact(start, params, steering, distance), expected);
}

TEST(KinematicBicycleModelTest, ReturnsToStartAfterFullCircle) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);
  const Pose2d start{2.0, -1.0, 0.37};

  expectPoseNear(propagateExact(start, params, kPi / 4.0, 2.0 * kPi), start);
}

TEST(KinematicBicycleModelTest, ReversesAlongSameArc) {
  const VehicleParams params(2.8, 2.0, 1.0, 0.8, 0.6);
  const Pose2d start{2.0, -1.0, 0.7};

  const Pose2d forward = propagateExact(start, params, -0.3, 2.4);
  const Pose2d restored = propagateExact(forward, params, -0.3, -2.4);

  expectPoseNear(restored, start);
}

TEST(KinematicBicycleModelTest, PreservesTinyNonzeroSteering) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);

  const Pose2d result = propagateExact(Pose2d{}, params, 1e-10, 1000.0);

  // beta约1e-7，横向位移约s*beta/2=5e-5米
  EXPECT_NEAR(1000.0, result.x, 1e-9);
  EXPECT_NEAR(5e-5, result.y, 1e-12);
  EXPECT_NEAR(1e-7, result.theta, 1e-12);

  const Pose2d right = propagateExact(Pose2d{}, params, -1e-10, 1000.0);

  EXPECT_NEAR(result.x, right.x, kTolerance);
  EXPECT_NEAR(-result.y, right.y, 1e-12);
}

TEST(KinematicBicycleModelTest, EulerUsesStartHeading) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);

  const Pose2d result = propagateEuler(Pose2d{}, params, kPi / 4.0, kPi / 2.0);

  // 起点朝+x，单步欧拉把全部位移投到+x，航向仍增加pi/2
  expectPoseNear(result, Pose2d{kPi / 2.0, 0.0, kPi / 2.0});
}

TEST(KinematicBicycleModelTest, SmallerEulerStepsReducePositionError) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);
  const double total_distance = kPi / 2.0;
  const Pose2d expected{1.0, 1.0, kPi / 2.0};

  double previous_error = std::numeric_limits<double>::infinity();

  for (int steps : {4, 20, 100}) {
    Pose2d pose;
    const double step_distance = total_distance / steps;

    for (int i = 0; i < steps; ++i) {
      pose = propagateEuler(pose, params, kPi / 4.0, step_distance);
    }

    const double error = std::hypot(pose.x - expected.x, pose.y - expected.y);

    EXPECT_LT(error, previous_error);
    EXPECT_NEAR(0.0, geometry::angleDiff(expected.theta, pose.theta),
                kTolerance);
    previous_error = error;
  }

  EXPECT_LT(previous_error, 0.02);
}

TEST(KinematicBicycleModelTest, EnforcesSteeringLimits) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, 0.5);

  for (const auto method : {propagateExact, propagateEuler}) {
    EXPECT_NO_THROW(method(Pose2d{}, params, 0.5, 1.0));
    EXPECT_NO_THROW(method(Pose2d{}, params, -0.5, 1.0));

    EXPECT_THROW(method(Pose2d{}, params, 0.51, 1.0), std::invalid_argument);
    EXPECT_THROW(method(Pose2d{}, params, -0.51, 1.0), std::invalid_argument);

    // 即使不移动，也不能传非法转角
    EXPECT_THROW(method(Pose2d{}, params, 0.51, 0.0), std::invalid_argument);
  }
}

TEST(KinematicBicycleModelTest, RejectsNonFiniteInputs) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, 0.5);
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (const auto method : {propagateExact, propagateEuler}) {
    for (double value : {infinity, -infinity, nan}) {
      const Pose2d poses[] = {
          {value, 0.0, 0.0},
          {0.0, value, 0.0},
          {0.0, 0.0, value},
      };

      for (const auto &pose : poses) {
        EXPECT_THROW(method(pose, params, 0.0, 1.0), std::invalid_argument);
      }

      EXPECT_THROW(method(Pose2d{}, params, value, 1.0), std::invalid_argument);
      EXPECT_THROW(method(Pose2d{}, params, 0.0, value), std::invalid_argument);
    }
  }
}

TEST(KinematicBicycleModelTest, RejectsNonFiniteCalculations) {
  const VehicleParams params(1.0, 1.0, 0.2, 0.2, kPi / 4.0);
  const VehicleParams short_wheelbase(0.01, 1.0, 0.2, 0.2, kPi / 4.0);
  const double maximum = std::numeric_limits<double>::max();
  const Pose2d large_position{maximum, 0.0, 0.0};

  for (const auto method : {propagateExact, propagateEuler}) {
    // x + distance 溢出
    EXPECT_THROW(method(large_position, params, 0.0, maximum),
                 std::invalid_argument);

    // distance * curvature 溢出
    EXPECT_THROW(method(Pose2d{}, short_wheelbase, kPi / 4.0, maximum),
                 std::invalid_argument);
  }
}

} // namespace
} // namespace parking_planner::core::vehicle