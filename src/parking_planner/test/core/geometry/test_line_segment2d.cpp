#include <gtest/gtest.h>

#include <cmath>

#include "parking_planner/core/geometry/line_segment2d.h"

namespace parking_planner::core::geometry {
namespace {

constexpr double kTolerance = 1e-9;

TEST(LineSegment2dTest, ReportsBasicGeometry) {
  const LineSegment2d segment(Vec2d(1.0, 2.0), Vec2d(4.0, 6.0));

  EXPECT_NEAR(1.0, segment.start().x(), kTolerance);
  EXPECT_NEAR(2.0, segment.start().y(), kTolerance);
  EXPECT_NEAR(4.0, segment.end().x(), kTolerance);
  EXPECT_NEAR(6.0, segment.end().y(), kTolerance);

  EXPECT_NEAR(3.0, segment.direction().x(), kTolerance);
  EXPECT_NEAR(4.0, segment.direction().y(), kTolerance);
  EXPECT_NEAR(5.0, segment.length(), kTolerance);
}

TEST(LineSegment2dTest, ComputesProjectionInsideSegment) {
  const LineSegment2d segment(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));

  const auto parameter = segment.projectionParameter(Vec2d(2.0, 3.0));

  ASSERT_TRUE(parameter.has_value());
  EXPECT_NEAR(0.5, parameter.value(), kTolerance);

  const Vec2d nearest = segment.nearestPoint(Vec2d(2.0, 3.0));

  EXPECT_NEAR(2.0, nearest.x(), kTolerance);
  EXPECT_NEAR(0.0, nearest.y(), kTolerance);
  EXPECT_NEAR(3.0, segment.distanceTo(Vec2d(2.0, 3.0)), kTolerance);
}

TEST(LineSegment2dTest, ClampsProjectionBeforeStart) {
  const LineSegment2d segment(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const Vec2d point(-2.0, 3.0);

  const auto parameter = segment.projectionParameter(point);

  ASSERT_TRUE(parameter.has_value());
  EXPECT_NEAR(-0.5, parameter.value(), kTolerance);

  const Vec2d nearest = segment.nearestPoint(point);

  EXPECT_NEAR(0.0, nearest.x(), kTolerance);
  EXPECT_NEAR(0.0, nearest.y(), kTolerance);
  EXPECT_NEAR(std::sqrt(13.0), segment.distanceTo(point), kTolerance);
}

TEST(LineSegment2dTest, ClampsProjectionAfterEnd) {
  const LineSegment2d segment(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const Vec2d point(6.0, 3.0);

  const auto parameter = segment.projectionParameter(point);

  ASSERT_TRUE(parameter.has_value());
  EXPECT_NEAR(1.5, parameter.value(), kTolerance);

  const Vec2d nearest = segment.nearestPoint(point);

  EXPECT_NEAR(4.0, nearest.x(), kTolerance);
  EXPECT_NEAR(0.0, nearest.y(), kTolerance);
  EXPECT_NEAR(std::sqrt(13.0), segment.distanceTo(point), kTolerance);
}

TEST(LineSegment2dTest, DetectsPointsOnAndOffSegment) {
  const LineSegment2d segment(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));

  EXPECT_TRUE(segment.isPointIn(Vec2d(0.0, 0.0)));
  EXPECT_TRUE(segment.isPointIn(Vec2d(2.0, 0.0)));
  EXPECT_TRUE(segment.isPointIn(Vec2d(4.0, 0.0)));

  EXPECT_FALSE(segment.isPointIn(Vec2d(5.0, 0.0)));
  EXPECT_FALSE(segment.isPointIn(Vec2d(2.0, 1.0)));

  EXPECT_TRUE(segment.isPointIn(Vec2d(2.0, 1e-10)));
  EXPECT_FALSE(segment.isPointIn(Vec2d(2.0, 1e-6)));
}

TEST(LineSegment2dTest, HandlesDegenerateSegment) {
  const LineSegment2d segment(Vec2d(2.0, 3.0), Vec2d(2.0, 3.0));

  EXPECT_NEAR(0.0, segment.length(), kTolerance);
  EXPECT_NEAR(0.0, segment.direction().length(), kTolerance);
  EXPECT_FALSE(segment.projectionParameter(Vec2d(5.0, 8.0)).has_value());

  const Vec2d nearest = segment.nearestPoint(Vec2d(5.0, 8.0));

  EXPECT_NEAR(2.0, nearest.x(), kTolerance);
  EXPECT_NEAR(3.0, nearest.y(), kTolerance);
  EXPECT_NEAR(std::sqrt(34.0), segment.distanceTo(Vec2d(5.0, 8.0)), kTolerance);

  EXPECT_TRUE(segment.isPointIn(Vec2d(2.0, 3.0)));
  EXPECT_FALSE(segment.isPointIn(Vec2d(2.0, 3.1)));
}

TEST(LineSegment2dTest, DetectsCrossingAndParallelSegments) {
  const LineSegment2d horizontal(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const LineSegment2d vertical(Vec2d(2.0, -1.0), Vec2d(2.0, 1.0));
  const LineSegment2d parallel(Vec2d(0.0, 1.0), Vec2d(4.0, 1.0));

  EXPECT_TRUE(horizontal.hasIntersect(vertical));
  EXPECT_FALSE(horizontal.hasIntersect(parallel));
}

TEST(LineSegment2dTest, DetectsTouchingAndCollinearSegments) {
  const LineSegment2d base(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));

  const LineSegment2d touching(Vec2d(4.0, 0.0), Vec2d(4.0, 2.0));
  const LineSegment2d overlapping(Vec2d(2.0, 0.0), Vec2d(6.0, 0.0));
  const LineSegment2d separated(Vec2d(5.0, 0.0), Vec2d(6.0, 0.0));

  EXPECT_TRUE(base.hasIntersect(touching));
  EXPECT_TRUE(base.hasIntersect(overlapping));
  EXPECT_FALSE(base.hasIntersect(separated));
}

TEST(LineSegment2dTest, HandlesDegenerateIntersections) {
  const LineSegment2d base(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const LineSegment2d point_on_segment(Vec2d(2.0, 0.0), Vec2d(2.0, 0.0));
  const LineSegment2d point_off_segment(Vec2d(2.0, 1.0), Vec2d(2.0, 1.0));

  EXPECT_TRUE(base.hasIntersect(point_on_segment));
  EXPECT_FALSE(base.hasIntersect(point_off_segment));

  const LineSegment2d same_point(Vec2d(2.0, 0.0), Vec2d(2.0, 0.0));
  EXPECT_TRUE(point_on_segment.hasIntersect(same_point));
}

TEST(LineSegment2dTest, RejectsIntersectionOutsideFiniteSegments) {
  const LineSegment2d horizontal(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const LineSegment2d vertical(Vec2d(6.0, -1.0), Vec2d(6.0, 1.0));

  EXPECT_FALSE(horizontal.hasIntersect(vertical));
  EXPECT_FALSE(vertical.hasIntersect(horizontal));
}

} // namespace
} // namespace parking_planner::core::geometry