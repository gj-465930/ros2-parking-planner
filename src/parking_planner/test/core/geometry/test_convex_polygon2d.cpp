#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "parking_planner/core/geometry/convex_polygon2d.h"
#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/transform2d.h"

namespace parking_planner::core::geometry {
namespace {

constexpr double kPi = 3.14159265358979323846;

ConvexPolygon2d makeTriangle() {
  return ConvexPolygon2d({Vec2d(0.0, 0.0), Vec2d(4.0, 0.0), Vec2d(0.0, 3.0)});
}

TEST(ConvexPolygon2dTest, AcceptsDifferentVertexCounts) {
  const std::vector<std::vector<Vec2d>> inputs{
      {Vec2d(0.0, 0.0), Vec2d(4.0, 0.0), Vec2d(0.0, 3.0)},
      {Vec2d(0.0, 0.0), Vec2d(3.0, 0.0), Vec2d(3.0, 2.0), Vec2d(0.0, 2.0)},
      {Vec2d(0.0, 0.0), Vec2d(3.0, 0.0), Vec2d(4.0, 2.0), Vec2d(2.0, 4.0),
       Vec2d(-1.0, 2.0)}};

  for (const auto &points : inputs) {
    const ConvexPolygon2d polygon(points);
    EXPECT_EQ(points.size(), polygon.vertices().size());
  }
}

TEST(ConvexPolygon2dTest, NormalizesClockwiseInput) {
  const ConvexPolygon2d clockwise(
      {Vec2d(0.0, 0.0), Vec2d(0.0, 3.0), Vec2d(4.0, 0.0)});

  const auto &points = clockwise.vertices();

  ASSERT_EQ(3u, points.size());
  EXPECT_GT((points[1] - points[0]).cross(points[2] - points[0]), 0.0);

  const OrientedRectangle2d inside(Vec2d(0.75, 0.75), 0.0, 0.5, 0.5);
  const OrientedRectangle2d outside(Vec2d(3.0, 3.0), 0.0, 1.0, 1.0);

  EXPECT_TRUE(clockwise.hasOverlap(inside));
  EXPECT_FALSE(clockwise.hasOverlap(outside));
}

TEST(ConvexPolygon2dTest, RejectsTooFewVertices) {
  const std::vector<std::vector<Vec2d>> inputs{
      {}, {Vec2d(0.0, 0.0)}, {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0)}};

  for (const auto &points : inputs) {
    EXPECT_THROW((void)ConvexPolygon2d(points), std::invalid_argument);
  }
}

TEST(ConvexPolygon2dTest, RejectsDuplicateVertices) {
  const std::vector<std::vector<Vec2d>> inputs{
      // 相邻重复
      {Vec2d(0.0, 0.0), Vec2d(0.0, 0.0), Vec2d(4.0, 0.0), Vec2d(0.0, 3.0)},
      // 重复提供首点
      {Vec2d(0.0, 0.0), Vec2d(4.0, 0.0), Vec2d(0.0, 3.0), Vec2d(0.0, 0.0)},
      // 非相邻重复
      {Vec2d(0.0, 0.0), Vec2d(4.0, 0.0), Vec2d(0.0, 3.0), Vec2d(4.0, 0.0)},
      // 容差内视为重复
      {Vec2d(0.0, 0.0), Vec2d(1e-10, 0.0), Vec2d(4.0, 0.0), Vec2d(0.0, 3.0)}};

  for (const auto &points : inputs) {
    EXPECT_THROW((void)ConvexPolygon2d(points), std::invalid_argument);
  }
}

TEST(ConvexPolygon2dTest, RejectsCollinearAndNearlyDegenerateInput) {
  const std::vector<std::vector<Vec2d>> inputs{
      // 所有点共线
      {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0), Vec2d(2.0, 0.0)},
      // 下边界存在冗余共线点
      {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0), Vec2d(2.0, 0.0), Vec2d(2.0, 2.0),
       Vec2d(0.0, 2.0)},
      // 三角形高度低于长度容差
      {Vec2d(0.0, 0.0), Vec2d(2.0, 0.0), Vec2d(1.0, 1e-10)}};

  for (const auto &points : inputs) {
    EXPECT_THROW((void)ConvexPolygon2d(points), std::invalid_argument);
  }
}

TEST(ConvexPolygon2dTest, RejectsConcaveAndSelfIntersectingInput) {
  const std::vector<std::vector<Vec2d>> inputs{
      // 凹多边形
      {Vec2d(0.0, 0.0), Vec2d(3.0, 0.0), Vec2d(1.0, 1.0), Vec2d(3.0, 3.0),
       Vec2d(0.0, 3.0)},
      // 蝴蝶结自交
      {Vec2d(0.0, 0.0), Vec2d(3.0, 3.0), Vec2d(0.0, 3.0), Vec2d(3.0, 0.0)},
      // 局部转向一致的自交星形
      {Vec2d(0.0, 0.0), Vec2d(4.0, 2.0), Vec2d(-1.0, 2.0), Vec2d(3.0, 0.0),
       Vec2d(2.0, 4.0)}};

  for (const auto &points : inputs) {
    EXPECT_THROW((void)ConvexPolygon2d(points), std::invalid_argument);
  }
}

TEST(ConvexPolygon2dTest, DetectsOverlapAndBothContainmentDirections) {
  const auto triangle = makeTriangle();

  const OrientedRectangle2d partial(Vec2d(2.0, 1.0), 0.0, 2.0, 1.0);
  const OrientedRectangle2d inside(Vec2d(0.75, 0.75), 0.4, 0.5, 0.5);
  const OrientedRectangle2d outside_container(Vec2d(2.0, 1.5), 0.0, 10.0, 10.0);

  EXPECT_TRUE(triangle.hasOverlap(partial));
  EXPECT_TRUE(triangle.hasOverlap(inside));
  EXPECT_TRUE(triangle.hasOverlap(outside_container));
}

TEST(ConvexPolygon2dTest, DetectsSeparationOnEitherSide) {
  const auto triangle = makeTriangle();

  const std::array<Vec2d, 4> centers{Vec2d(10.0, 0.0), Vec2d(-10.0, 0.0),
                                     Vec2d(0.0, 10.0), Vec2d(0.0, -10.0)};

  for (const Vec2d &center : centers) {
    EXPECT_FALSE(
        triangle.hasOverlap(OrientedRectangle2d(center, 0.0, 1.0, 1.0)));
  }
}

TEST(ConvexPolygon2dTest, CountsEdgeAndVertexContactAsOverlap) {
  const auto triangle = makeTriangle();

  const OrientedRectangle2d edge_contact(Vec2d(1.0, -0.5), 0.0, 1.0, 1.0);
  const OrientedRectangle2d vertex_contact(Vec2d(4.5, -0.5), 0.0, 1.0, 1.0);

  EXPECT_TRUE(triangle.hasOverlap(edge_contact));
  EXPECT_TRUE(triangle.hasOverlap(vertex_contact));
}

TEST(ConvexPolygon2dTest, NeedsPolygonEdgeNormalForSeparation) {
  const auto triangle = makeTriangle();

  // 世界 x、y 投影均重叠，斜边法向才能发现分离
  const OrientedRectangle2d rectangle(Vec2d(3.0, 3.0), 0.0, 1.0, 1.0);

  EXPECT_FALSE(triangle.hasOverlap(rectangle));
}

TEST(ConvexPolygon2dTest, NeedsRectangleAxisForSeparation) {
  const OrientedRectangle2d base(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  const auto corners = base.corners();
  const ConvexPolygon2d polygon(
      std::vector<Vec2d>(corners.begin(), corners.end()));

  const OrientedRectangle2d rectangle(Vec2d(3.5, 2.8), kPi / 4.0, 4.0, 2.0);

  EXPECT_FALSE(polygon.hasOverlap(rectangle));
}

TEST(ConvexPolygon2dTest, AppliesToleranceOnAnObliqueUnitAxis) {
  const auto triangle = makeTriangle();

  // 斜边中点为 (2,1.5)，外法向为 (0.6,0.8)
  const Vec2d midpoint(2.0, 1.5);
  const Vec2d normal(0.6, 0.8);
  const double heading = std::atan2(0.8, 0.6);

  const OrientedRectangle2d tiny_gap(midpoint + normal * (0.5 + 1e-10), heading,
                                     1.0, 1.0);

  const OrientedRectangle2d clear_gap(midpoint + normal * (0.5 + 1e-6), heading,
                                      1.0, 1.0);

  EXPECT_TRUE(triangle.hasOverlap(tiny_gap));
  EXPECT_FALSE(triangle.hasOverlap(clear_gap));
}

TEST(ConvexPolygon2dTest, MatchesRectangleSATForRectangularPolygon) {
  const OrientedRectangle2d base(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  const auto corners = base.corners();
  const ConvexPolygon2d polygon(
      std::vector<Vec2d>(corners.begin(), corners.end()));

  const std::array<OrientedRectangle2d, 4> rectangles{
      OrientedRectangle2d(Vec2d(0.0, 0.0), 0.5, 1.0, 1.0),
      OrientedRectangle2d(Vec2d(1.0, 0.0), 0.7, 4.0, 2.0),
      OrientedRectangle2d(Vec2d(3.0, 0.0), 0.0, 2.0, 2.0),
      OrientedRectangle2d(Vec2d(3.5, 2.8), kPi / 4.0, 4.0, 2.0)};

  for (const auto &rectangle : rectangles) {
    EXPECT_EQ(base.hasOverlap(rectangle), polygon.hasOverlap(rectangle));
  }
}

TEST(ConvexPolygon2dTest, PreservesOverlapUnderRigidTransform) {
  const auto triangle = makeTriangle();
  const Pose2d frame{7.0, -3.0, 0.7};

  std::vector<Vec2d> moved_vertices;

  for (const Vec2d &vertex : triangle.vertices()) {
    moved_vertices.push_back(transformToGlobal(vertex, frame));
  }

  const ConvexPolygon2d moved_triangle(moved_vertices);

  const std::array<OrientedRectangle2d, 2> rectangles{
      OrientedRectangle2d(Vec2d(0.75, 0.75), 0.0, 0.5, 0.5),
      OrientedRectangle2d(Vec2d(3.0, 3.0), 0.0, 1.0, 1.0)};

  for (const auto &rectangle : rectangles) {
    const OrientedRectangle2d moved_rectangle(
        transformToGlobal(rectangle.center(), frame),
        rectangle.heading() + frame.theta, rectangle.length(),
        rectangle.width());

    EXPECT_EQ(triangle.hasOverlap(rectangle),
              moved_triangle.hasOverlap(moved_rectangle));
  }
}

} // namespace
} // namespace parking_planner::core::geometry