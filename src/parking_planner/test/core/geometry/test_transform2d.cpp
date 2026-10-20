#include <gtest/gtest.h>

#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/transform2d.h"
#include "parking_planner/core/geometry/vec2d.h"

#include <limits>
#include <stdexcept>

namespace parking_planner::core::geometry {
namespace {

constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2 * kPi;

TEST(Transform2dTest, DefaultPoseActsAsIdentity) {
  const Pose2d reference_pose;
  const Vec2d point(2.0, -1.0);

  const Vec2d global = transformToGlobal(point, reference_pose);
  const Vec2d local = transformToLocal(point, reference_pose);

  EXPECT_NEAR(2.0, global.x(), kTolerance);
  EXPECT_NEAR(-1.0, global.y(), kTolerance);

  EXPECT_NEAR(2.0, local.x(), kTolerance);
  EXPECT_NEAR(-1.0, local.y(), kTolerance);
}

TEST(Transform2dTest, TranslatesWithoutRotation) {
  const Pose2d reference_pose{3.0, -2.0, 0.0};

  const Vec2d global = transformToGlobal(Vec2d(2.0, 1.0), reference_pose);

  EXPECT_NEAR(5.0, global.x(), kTolerance);
  EXPECT_NEAR(-1.0, global.y(), kTolerance);

  const Vec2d local = transformToLocal(Vec2d(5.0, -1.0), reference_pose);

  EXPECT_NEAR(2.0, local.x(), kTolerance);
  EXPECT_NEAR(1.0, local.y(), kTolerance);
}

TEST(Transform2dTest, RotatesPositiveAndNegativeNinetyDegrees) {
  struct Case {
    double theta;
    double global_x;
    double global_y;
  };

  const Case cases[] = {
      {kPi / 2.0, -1.0, 2.0},
      {-kPi / 2.0, 1.0, -2.0},
  };

  for (const auto &item : cases) {
    const Pose2d reference_pose{0.0, 0.0, item.theta};

    const Vec2d global = transformToGlobal(Vec2d(2.0, 1.0), reference_pose);

    EXPECT_NEAR(item.global_x, global.x(), kTolerance);
    EXPECT_NEAR(item.global_y, global.y(), kTolerance);

    const Vec2d local =
        transformToLocal(Vec2d(item.global_x, item.global_y), reference_pose);

    EXPECT_NEAR(2.0, local.x(), kTolerance);
    EXPECT_NEAR(1.0, local.y(), kTolerance);
  }
}

TEST(Transform2dTest, RotatesThenTranslates) {
  const Pose2d reference_pose{3.0, 2.0, kPi / 2.0};

  // (2,1) 旋转后为 (-1,2)，加上 (3,2) 得到 (2,4)。
  const Vec2d global = transformToGlobal(Vec2d(2.0, 1.0), reference_pose);

  EXPECT_NEAR(2.0, global.x(), kTolerance);
  EXPECT_NEAR(4.0, global.y(), kTolerance);

  // 逆变换。
  const Vec2d local = transformToLocal(Vec2d(2.0, 4.0), reference_pose);

  EXPECT_NEAR(2.0, local.x(), kTolerance);
  EXPECT_NEAR(1.0, local.y(), kTolerance);
}

TEST(Transform2dTest, MapsVehicleOriginAndFixedOffset) {
  const Pose2d rear_axle_pose{3.0, 2.0, kPi / 2.0};

  const Vec2d world_origin = transformToGlobal(Vec2d(), rear_axle_pose);

  EXPECT_NEAR(3.0, world_origin.x(), kTolerance);
  EXPECT_NEAR(2.0, world_origin.y(), kTolerance);

  const Vec2d local_origin = transformToLocal(Vec2d(3.0, 2.0), rear_axle_pose);

  EXPECT_NEAR(0.0, local_origin.x(), kTolerance);
  EXPECT_NEAR(0.0, local_origin.y(), kTolerance);

  // 几何中心在后轴中心前方 1.5 米
  const Vec2d center_local(1.5, 0.0);
  const Vec2d center_global = transformToGlobal(center_local, rear_axle_pose);

  // 车头朝世界 +y，因此偏移加在世界 y 上
  EXPECT_NEAR(3.0, center_global.x(), kTolerance);
  EXPECT_NEAR(3.5, center_global.y(), kTolerance);
}

TEST(Transform2dTest, RoundTripsInBothDirections) {
  const Pose2d reference_poses[] = {
      {0.0, 0.0, 0.0},
      {3.0, 2.0, 0.37},
      {-8.0, 4.0, -1.2},
      {10.0, -5.0, kPi},
      {1.0, -2.0, 3.0 * kPi / 2.0},
  };

  const Vec2d points[] = {
      {0.0, 0.0},
      {2.0, 1.0},
      {-4.0, 3.0},
      {100.0, -80.0},
  };

  for (const auto &reference_pose : reference_poses) {
    for (const auto &point : points) {
      // 将 point 当作局部坐标
      const Vec2d global = transformToGlobal(point, reference_pose);
      const Vec2d restored_local = transformToLocal(global, reference_pose);

      EXPECT_LT((restored_local - point).length(), kTolerance);

      // 将 point 当作全局坐标
      const Vec2d local = transformToLocal(point, reference_pose);
      const Vec2d restored_global = transformToGlobal(local, reference_pose);

      EXPECT_LT((restored_global - point).length(), kTolerance);
    }
  }
}

TEST(Transform2dTest, AcceptsUnnormalizedFiniteHeadings) {
  const double headings[] = {
      kPi / 2.0 + kTwoPi,
      kPi / 2.0 - kTwoPi,
  };

  for (double theta : headings) {
    const Pose2d reference_pose{3.0, 2.0, theta};

    const Vec2d global = transformToGlobal(Vec2d(2.0, 1.0), reference_pose);

    EXPECT_NEAR(2.0, global.x(), kTolerance);
    EXPECT_NEAR(4.0, global.y(), kTolerance);

    const Vec2d local = transformToLocal(Vec2d(2.0, 4.0), reference_pose);

    EXPECT_NEAR(2.0, local.x(), kTolerance);
    EXPECT_NEAR(1.0, local.y(), kTolerance);
  }
}

TEST(Transform2dTest, RejectsNonFiniteReferencePose) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double invalid_values[] = {infinity, -infinity, nan};

  const Vec2d point(2.0, 1.0);

  for (double value : invalid_values) {
    const Pose2d invalid_poses[] = {
        {value, 0.0, 0.0},
        {0.0, value, 0.0},
        {0.0, 0.0, value},
    };

    for (const auto &reference_pose : invalid_poses) {
      EXPECT_THROW(transformToGlobal(point, reference_pose),
                   std::invalid_argument);

      EXPECT_THROW(transformToLocal(point, reference_pose),
                   std::invalid_argument);
    }
  }
}

} // namespace
} // namespace parking_planner::core::geometry