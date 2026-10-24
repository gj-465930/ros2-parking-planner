#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/transform2d.h"

namespace parking_planner::core::geometry {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

void expectPointNear(const Vec2d &actual, const Vec2d &expected) {
  EXPECT_NEAR(expected.x(), actual.x(), kTolerance);
  EXPECT_NEAR(expected.y(), actual.y(), kTolerance);
}

// 相交关系不应因调用顺序改变。
void expectOverlap(const OrientedRectangle2d &first,
                   const OrientedRectangle2d &second, bool expected) {
  EXPECT_EQ(expected, first.hasOverlap(second));
  EXPECT_EQ(expected, second.hasOverlap(first));
}

TEST(OrientedRectangle2dTest, ReportsGeometry) {
  const OrientedRectangle2d rectangle(Vec2d(3.0, 2.0), 0.5, 4.0, 2.0);

  expectPointNear(rectangle.center(), Vec2d(3.0, 2.0));
  EXPECT_DOUBLE_EQ(0.5, rectangle.heading());
  EXPECT_DOUBLE_EQ(4.0, rectangle.length());
  EXPECT_DOUBLE_EQ(2.0, rectangle.width());
}

TEST(OrientedRectangle2dTest, ComputesAxisAlignedCorners) {
  const OrientedRectangle2d rectangle(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  const auto actual = rectangle.corners();
  const std::array<Vec2d, 4> expected{Vec2d(2.0, 1.0), Vec2d(-2.0, 1.0),
                                      Vec2d(-2.0, -1.0), Vec2d(2.0, -1.0)};

  for (std::size_t i = 0; i < expected.size(); ++i) {
    expectPointNear(actual[i], expected[i]);
  }
}

TEST(OrientedRectangle2dTest, ComputesQuarterTurnCorners) {
  const OrientedRectangle2d rectangle(Vec2d(3.0, 2.0), kPi / 2.0, 4.0, 2.0);

  const auto actual = rectangle.corners();
  const std::array<Vec2d, 4> expected{Vec2d(2.0, 4.0), Vec2d(2.0, 0.0),
                                      Vec2d(4.0, 0.0), Vec2d(4.0, 4.0)};

  for (std::size_t i = 0; i < expected.size(); ++i) {
    expectPointNear(actual[i], expected[i]);
  }
}

TEST(OrientedRectangle2dTest, ComputesDiagonalCorners) {
  const OrientedRectangle2d rectangle(Vec2d(0.0, 0.0), kPi / 4.0, 4.0, 2.0);

  const double q = std::sqrt(0.5);
  const auto actual = rectangle.corners();

  expectPointNear(actual[0], Vec2d(q, 3.0 * q));
  expectPointNear(actual[1], Vec2d(-3.0 * q, -q));
  expectPointNear(actual[2], Vec2d(-q, -3.0 * q));
  expectPointNear(actual[3], Vec2d(3.0 * q, q));
}

TEST(OrientedRectangle2dTest, IncludesInteriorEdgesAndCorners) {
  const OrientedRectangle2d rectangle(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  EXPECT_TRUE(rectangle.isPointIn(Vec2d(0.0, 0.0)));
  EXPECT_TRUE(rectangle.isPointIn(Vec2d(2.0, 0.0)));
  EXPECT_TRUE(rectangle.isPointIn(Vec2d(2.0, 1.0)));

  EXPECT_FALSE(rectangle.isPointIn(Vec2d(2.1, 0.0)));
  EXPECT_FALSE(rectangle.isPointIn(Vec2d(0.0, -1.1)));
}

TEST(OrientedRectangle2dTest, HandlesPointsInRotatedRectangles) {
  const OrientedRectangle2d quarter_turn(Vec2d(3.0, 2.0), kPi / 2.0, 4.0, 2.0);

  EXPECT_TRUE(quarter_turn.isPointIn(Vec2d(3.0, 3.0)));
  EXPECT_TRUE(quarter_turn.isPointIn(Vec2d(2.0, 4.0)));
  EXPECT_FALSE(quarter_turn.isPointIn(Vec2d(4.1, 2.0)));

  const OrientedRectangle2d diagonal(Vec2d(0.0, 0.0), kPi / 4.0, 4.0, 2.0);

  EXPECT_TRUE(diagonal.isPointIn(Vec2d(1.0, 1.0)));
  EXPECT_FALSE(diagonal.isPointIn(Vec2d(2.0, 2.0)));
}

TEST(OrientedRectangle2dTest, DetectsParallelSeparation) {
  const OrientedRectangle2d base(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  const OrientedRectangle2d separated_x(Vec2d(3.5, 0.0), 0.0, 2.0, 2.0);
  const OrientedRectangle2d separated_y(Vec2d(0.0, 2.5), 0.0, 2.0, 2.0);

  expectOverlap(base, separated_x, false);
  expectOverlap(base, separated_y, false);
}

TEST(OrientedRectangle2dTest, DetectsPartialAndRotatedOverlap) {
  const OrientedRectangle2d base(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  // x 投影有长度为 0.5 米的交集，不只是边界接触。
  const OrientedRectangle2d partial(Vec2d(2.5, 0.0), 0.0, 2.0, 2.0);
  const OrientedRectangle2d rotated(Vec2d(1.0, 0.0), kPi / 4.0, 4.0, 2.0);

  expectOverlap(base, partial, true);
  expectOverlap(base, rotated, true);
}

TEST(OrientedRectangle2dTest, IncludesContainmentAndIdenticalRectangles) {
  const OrientedRectangle2d outer(Vec2d(0.0, 0.0), 0.0, 6.0, 4.0);
  const OrientedRectangle2d inner(Vec2d(0.5, 0.5), kPi / 4.0, 1.0, 1.0);

  expectOverlap(outer, inner, true);
  expectOverlap(outer, outer, true);
}

TEST(OrientedRectangle2dTest, CountsEdgeAndCornerContactAsOverlap) {
  const OrientedRectangle2d base(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  const OrientedRectangle2d edge_contact(Vec2d(3.0, 0.0), 0.0, 2.0, 2.0);
  const OrientedRectangle2d corner_contact(Vec2d(3.0, 2.0), 0.0, 2.0, 2.0);

  expectOverlap(base, edge_contact, true);
  expectOverlap(base, corner_contact, true);
}

TEST(OrientedRectangle2dTest, NeedsSecondRectanglesAxisForSeparation) {
  const OrientedRectangle2d first(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);
  const OrientedRectangle2d second(Vec2d(3.5, 2.8), kPi / 4.0, 4.0, 2.0);

  // 世界 x、y 方向投影都有交集；
  // second 的长度方向才能发现分离。
  expectOverlap(first, second, false);
}

TEST(OrientedRectangle2dTest, AppliesBoundaryTolerance) {
  const OrientedRectangle2d base(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  EXPECT_TRUE(base.isPointIn(Vec2d(2.0 + 1e-10, 0.0)));
  EXPECT_FALSE(base.isPointIn(Vec2d(2.0 + 1e-6, 0.0)));

  const OrientedRectangle2d tiny_gap(Vec2d(3.0 + 1e-10, 0.0), 0.0, 2.0, 2.0);
  const OrientedRectangle2d clear_gap(Vec2d(3.0 + 1e-6, 0.0), 0.0, 2.0, 2.0);

  expectOverlap(base, tiny_gap, true);
  expectOverlap(base, clear_gap, false);
}

TEST(OrientedRectangle2dTest, PreservesOverlapUnderRigidTransform) {
  const Pose2d frame{7.0, -3.0, 0.7};

  const OrientedRectangle2d first(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);
  const OrientedRectangle2d overlapping(Vec2d(1.0, 0.0), kPi / 4.0, 4.0, 2.0);
  const OrientedRectangle2d separated(Vec2d(3.5, 2.8), kPi / 4.0, 4.0, 2.0);

  // 对所有矩形施加相同的旋转和平移，碰撞关系应保持不变。
  const auto transformed = [&frame](const OrientedRectangle2d &rectangle) {
    return OrientedRectangle2d(transformToGlobal(rectangle.center(), frame),
                               rectangle.heading() + frame.theta,
                               rectangle.length(), rectangle.width());
  };

  expectOverlap(first, overlapping, true);
  expectOverlap(first, separated, false);

  const auto moved_first = transformed(first);
  const auto moved_overlapping = transformed(overlapping);
  const auto moved_separated = transformed(separated);

  expectOverlap(moved_first, moved_overlapping, true);
  expectOverlap(moved_first, moved_separated, false);
}

TEST(OrientedRectangle2dTest, RejectsInvalidDimensionsAndHeading) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const std::array<double, 4> invalid_dimensions{0.0, -1.0, infinity, nan};

  for (const double value : invalid_dimensions) {
    EXPECT_THROW(OrientedRectangle2d(Vec2d(0.0, 0.0), 0.0, value, 2.0),
                 std::invalid_argument);

    EXPECT_THROW(OrientedRectangle2d(Vec2d(0.0, 0.0), 0.0, 4.0, value),
                 std::invalid_argument);
  }

  EXPECT_THROW(OrientedRectangle2d(Vec2d(0.0, 0.0), infinity, 4.0, 2.0),
               std::invalid_argument);

  EXPECT_THROW(OrientedRectangle2d(Vec2d(0.0, 0.0), nan, 4.0, 2.0),
               std::invalid_argument);
}

} // namespace
} // namespace parking_planner::core::geometry