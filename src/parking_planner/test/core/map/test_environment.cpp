#include "parking_planner/core/map/environment.h"

#include "parking_planner/core/geometry/angle.h"
#include "parking_planner/core/geometry/convex_polygon2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace parking_planner::core::map {
namespace {

using geometry::ConvexPolygon2d;
using geometry::kPi;
using geometry::Pose2d;
using geometry::Vec2d;
using vehicle::VehicleParams;

ConvexPolygon2d rectangleObstacle(double min_x, double min_y, double max_x,
                                  double max_y) {
  return ConvexPolygon2d(std::vector<Vec2d>{
      Vec2d(min_x, min_y),
      Vec2d(max_x, min_y),
      Vec2d(max_x, max_y),
      Vec2d(min_x, max_y),
  });
}

TEST(EnvironmentTest, CreatesFreeEnvironmentWithoutObstacles) {
  const Environment environment(Vec2d(-2.0, -2.0), 0.5, 20, 20, {});
  const VehicleParams params(1.0, 0.4, 0.2, 0.2, 0.5);

  EXPECT_TRUE(environment.obstacles().empty());

  for (int y = 0; y < environment.grid().height(); ++y) {
    for (int x = 0; x < environment.grid().width(); ++x) {
      EXPECT_FALSE(environment.grid().isOccupied(GridIndex{x, y}));
    }
  }

  EXPECT_FALSE(environment.hasCollision(Pose2d{2.0, 2.0, 0.0}, params));
}

TEST(EnvironmentTest, MarksThinObstacleWithoutCenterSampling) {
  // 障碍位于第(0,0)格，但不经过格子中心(0.5,0.5)。
  const auto obstacle = rectangleObstacle(0.1, 0.1, 0.2, 0.9);
  const Environment environment(Vec2d(0.0, 0.0), 1.0, 4, 3, {obstacle});

  EXPECT_EQ(1u, environment.obstacles().size());

  for (int y = 0; y < environment.grid().height(); ++y) {
    for (int x = 0; x < environment.grid().width(); ++x) {
      const bool expected = x == 0 && y == 0;

      EXPECT_EQ(expected, environment.grid().isOccupied(GridIndex{x, y}));
    }
  }
}

TEST(EnvironmentTest, MarksCellsTouchingObstacleEdgesAndCorners) {
  const auto obstacle = rectangleObstacle(0.8, 0.8, 1.0, 1.0);
  const Environment environment(Vec2d(0.0, 0.0), 1.0, 3, 3, {obstacle});

  // 面积重叠。
  EXPECT_TRUE(environment.grid().isOccupied(GridIndex{0, 0}));

  // 共享边接触。
  EXPECT_TRUE(environment.grid().isOccupied(GridIndex{1, 0}));
  EXPECT_TRUE(environment.grid().isOccupied(GridIndex{0, 1}));

  // 仅角接触。
  EXPECT_TRUE(environment.grid().isOccupied(GridIndex{1, 1}));

  EXPECT_FALSE(environment.grid().isOccupied(GridIndex{2, 2}));
}

TEST(EnvironmentTest, HandlesObstaclesOutsideMap) {
  const auto crossing = rectangleObstacle(-1.0, 0.2, 0.2, 0.8);
  const auto outside = rectangleObstacle(5.0, 5.0, 6.0, 6.0);

  const Environment environment(Vec2d(0.0, 0.0), 1.0, 3, 3,
                                {crossing, outside});

  EXPECT_EQ(2u, environment.obstacles().size());

  for (int y = 0; y < environment.grid().height(); ++y) {
    for (int x = 0; x < environment.grid().width(); ++x) {
      const bool expected = x == 0 && y == 0;

      EXPECT_EQ(expected, environment.grid().isOccupied(GridIndex{x, y}));
    }
  }
}

TEST(EnvironmentTest, UsesFullFootprintFromRearAxlePose) {
  const VehicleParams params(1.0, 0.4, 0.2, 0.2, 0.5);

  // 后轴(2,2)，车框范围：
  // x∈[1.8,3.2]，y∈[1.8,2.2]。
  const auto obstacle = rectangleObstacle(3.2, 1.9, 3.5, 2.1);
  const Environment environment(Vec2d(0.0, 0.0), 0.5, 20, 20, {obstacle});

  // 车头接触障碍，后轴中心本身未进入障碍。
  EXPECT_TRUE(environment.hasCollision(Pose2d{2.0, 2.0, 0.0}, params));

  // 向上平移后分离。
  EXPECT_FALSE(environment.hasCollision(Pose2d{2.0, 3.0, 0.0}, params));
}

TEST(EnvironmentTest, DetectsObstacleEntirelyInsideVehicle) {
  const VehicleParams params(1.0, 0.4, 0.2, 0.2, 0.5);
  const Pose2d pose{2.0, 2.0, 0.0};

  const auto obstacle = rectangleObstacle(2.4, 1.95, 2.5, 2.05);
  const Environment environment(Vec2d(0.0, 0.0), 0.5, 20, 20, {obstacle});

  // 四个车角都不在这块小障碍内，但障碍位于车身内部。
  for (const auto &corner : params.footprint(pose).corners()) {
    const bool inside_obstacle = corner.x() >= 2.4 && corner.x() <= 2.5 &&
                                 corner.y() >= 1.95 && corner.y() <= 2.05;

    EXPECT_FALSE(inside_obstacle);
  }

  EXPECT_TRUE(environment.hasCollision(pose, params));
}

TEST(EnvironmentTest, DetectsVehicleEntirelyInsideObstacle) {
  const VehicleParams params(1.0, 0.4, 0.2, 0.2, 0.5);
  const auto obstacle = rectangleObstacle(1.0, 1.0, 5.0, 5.0);

  const Environment environment(Vec2d(0.0, 0.0), 0.5, 20, 20, {obstacle});

  EXPECT_TRUE(environment.hasCollision(Pose2d{3.0, 3.0, 0.0}, params));
}

TEST(EnvironmentTest, UsesOriginalGeometryForVehicleCollision) {
  const VehicleParams params(0.4, 0.2, 0.1, 0.1, 0.5);
  const auto obstacle = rectangleObstacle(3.1, 3.1, 3.2, 3.2);

  const Environment environment(Vec2d(0.0, 0.0), 2.0, 5, 5, {obstacle});

  // 第(1,1)格覆盖[2,4]×[2,4]，其中有小障碍。
  EXPECT_TRUE(environment.grid().isOccupied(GridIndex{1, 1}));

  // 车辆也在这个格子内，但实际车框与障碍分离。
  EXPECT_FALSE(environment.hasCollision(Pose2d{2.3, 2.3, 0.0}, params));

  // 车框移到实际障碍处后发生碰撞。
  EXPECT_TRUE(environment.hasCollision(Pose2d{3.0, 3.15, 0.0}, params));
}

TEST(EnvironmentTest, RejectsBoundaryContactAndFootprintExit) {
  // 后轴位于车尾，车长1米、宽1米。
  const VehicleParams params(1.0, 1.0, 0.0, 0.0, 0.5);
  const Environment environment(Vec2d(0.0, 0.0), 1.0, 10, 10, {});

  EXPECT_FALSE(environment.hasCollision(Pose2d{5.0, 5.0, 0.0}, params));

  const Pose2d contacts[] = {
      {0.0, 5.0, 0.0}, // 车尾接触左边界
      {9.0, 5.0, 0.0}, // 车头接触右边界
      {5.0, 0.5, 0.0}, // 车身接触下边界
      {5.0, 9.5, 0.0}, // 车身接触上边界
  };

  for (const auto &pose : contacts) {
    EXPECT_TRUE(environment.hasCollision(pose, params));
  }

  // 后轴在地图内，车头却已经越界。
  const Pose2d outside{9.5, 5.0, 0.0};

  EXPECT_TRUE(
      environment.grid().worldToGrid(Vec2d(outside.x, outside.y)).has_value());

  EXPECT_TRUE(environment.hasCollision(outside, params));
}

TEST(EnvironmentTest, ChecksRotatedVehicleCorners) {
  const VehicleParams params(1.0, 0.4, 0.2, 0.2, 0.5);
  const Environment environment(Vec2d(-2.0, -2.0), 0.5, 10, 10, {});

  EXPECT_FALSE(environment.hasCollision(Pose2d{0.0, 0.0, kPi / 4.0}, params));

  // 地图上界y=3，后轴在内，但朝+y的车头越过上界。
  EXPECT_TRUE(environment.hasCollision(Pose2d{0.0, 2.0, kPi / 2.0}, params));
}

TEST(EnvironmentTest, RejectsInvalidMapAndPoseInputs) {
  EXPECT_THROW((Environment{Vec2d(0.0, 0.0), 0.0, 10, 10, {}}),
               std::invalid_argument);

  const Environment environment(Vec2d(0.0, 0.0), 1.0, 10, 10, {});
  const VehicleParams params(1.0, 0.4, 0.2, 0.2, 0.5);

  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const Pose2d invalid_poses[] = {
      {infinity, 2.0, 0.0},
      {2.0, nan, 0.0},
      {2.0, 2.0, infinity},
  };

  for (const auto &pose : invalid_poses) {
    EXPECT_THROW(environment.hasCollision(pose, params), std::invalid_argument);
  }
}

TEST(EnvironmentTest, SupportsTriangularObstacles) {
  // 三角形内部满足x>=4、y>=4、x+y<=10。
  const ConvexPolygon2d triangle(std::vector<Vec2d>{
      Vec2d(4.0, 4.0),
      Vec2d(6.0, 4.0),
      Vec2d(4.0, 6.0),
  });

  const Environment environment(Vec2d(0.0, 0.0), 1.0, 10, 10, {triangle});
  const VehicleParams params(0.4, 0.2, 0.1, 0.1, 0.5);

  EXPECT_TRUE(environment.hasCollision(Pose2d{4.3, 4.5, 0.0}, params));

  // 车框完全位于斜边x+y=10的外侧。
  EXPECT_FALSE(environment.hasCollision(Pose2d{5.3, 5.5, 0.0}, params));
}

} // namespace
} // namespace parking_planner::core::map