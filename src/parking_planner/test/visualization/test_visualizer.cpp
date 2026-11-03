#include <array>
#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <gtest/gtest.h>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "parking_planner/core/geometry/angle.h"
#include "parking_planner/core/geometry/convex_polygon2d.h"
#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"
#include "parking_planner/core/map/environment.h"
#include "parking_planner/core/scenario/scenario.h"
#include "parking_planner/core/vehicle/vehicle_params.h"
#include "parking_planner/visualization/visualizer.h"

namespace parking_planner::visualization {
namespace {

using core::geometry::ConvexPolygon2d;
using core::geometry::kPi;
using core::geometry::OrientedRectangle2d;
using core::geometry::Pose2d;
using core::geometry::Vec2d;
using core::map::Environment;
using core::scenario::Scenario;
using core::vehicle::VehicleParams;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kTolerance = 1e-9;

// 仅用于测试：创建一个坐标便于手算的合法场景
Scenario makeScene(bool include_obstacles = true) {
  std::vector<ConvexPolygon2d> obstacles;

  if (include_obstacles) {
    obstacles.emplace_back(std::vector<Vec2d>{
        Vec2d(12.0, 0.0),
        Vec2d(14.0, 0.0),
        Vec2d(13.0, 2.0),
    });

    obstacles.emplace_back(std::vector<Vec2d>{
        Vec2d(14.0, 9.0),
        Vec2d(16.0, 9.0),
        Vec2d(16.0, 11.0),
        Vec2d(14.0, 11.0),
    });
  }

  return Scenario{
      "visualizer_test",
      VehicleParams(2.0, 2.0, 1.0, 1.0, 0.5),
      Environment(Vec2d(-2.0, -3.0), 0.5, 40, 32, obstacles),
      Pose2d{2.0, 2.0, kPi / 2.0},
      Pose2d{10.0, 6.0, kPi},
      OrientedRectangle2d(Vec2d(9.0, 6.0), kPi, 6.0, 4.0),
  };
}

// 按身份查找，避免测试依赖各个 ADD 标记的排列顺序
const Marker &findMarker(const MarkerArray &message, const std::string &ns,
                         std::int32_t id = 0) {
  for (const auto &marker : message.markers) {
    if (marker.action == Marker::ADD && marker.ns == ns && marker.id == id) {
      return marker;
    }
  }

  throw std::runtime_error("missing test marker: " + ns);
}

void expectPoint(const geometry_msgs::msg::Point &point, double x, double y,
                 double z = 0.0) {
  EXPECT_NEAR(x, point.x, kTolerance);
  EXPECT_NEAR(y, point.y, kTolerance);
  EXPECT_NEAR(z, point.z, kTolerance);
}

TEST(VisualizerTest, BuildsCompleteSnapshotWithValidMarkerProperties) {
  const auto scene = makeScene();
  const rclcpp::Time stamp(12, 345);
  const auto message = Visualizer::buildSceneMarkers(scene, stamp);

  // 清理指令 + 地图 + 两障碍 + 车位 + 两车框 + 两箭头
  ASSERT_EQ(9u, message.markers.size());
  EXPECT_EQ(Marker::DELETEALL, message.markers.front().action);

  using MarkerKey = std::pair<std::string, std::int32_t>;

  const std::set<MarkerKey> expected_keys{
      {"map_boundary", 0},  {"obstacles", 0},     {"obstacles", 1},
      {"parking_space", 0}, {"start_vehicle", 0}, {"goal_vehicle", 0},
      {"start_pose", 0},    {"goal_pose", 0},
  };

  std::set<MarkerKey> actual_keys;

  for (std::size_t i = 0; i < message.markers.size(); ++i) {
    const auto &marker = message.markers[i];
    SCOPED_TRACE("marker index: " + std::to_string(i));

    EXPECT_EQ("map", marker.header.frame_id);
    EXPECT_EQ(12, marker.header.stamp.sec);
    EXPECT_EQ(345u, marker.header.stamp.nanosec);

    if (i == 0) {
      continue;
    }

    EXPECT_EQ(Marker::ADD, marker.action);

    const auto insertion = actual_keys.emplace(marker.ns, marker.id);
    EXPECT_TRUE(insertion.second) << "duplicate namespace/id";

    // points 已经是世界坐标，Marker pose 必须保持单位变换
    expectPoint(marker.pose.position, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.x);
    EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.y);
    EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.z);
    EXPECT_DOUBLE_EQ(1.0, marker.pose.orientation.w);

    EXPECT_GT(marker.color.a, 0.0F);
    EXPECT_LE(marker.color.a, 1.0F);
    EXPECT_EQ(0, marker.lifetime.sec);
    EXPECT_EQ(0u, marker.lifetime.nanosec);
    EXPECT_GT(marker.scale.x, 0.0);

    if (marker.ns == "start_pose" || marker.ns == "goal_pose") {
      EXPECT_EQ(Marker::ARROW, marker.type);
      ASSERT_EQ(2u, marker.points.size());
      EXPECT_GT(marker.scale.y, 0.0);
      EXPECT_GT(marker.scale.z, 0.0);
    } else {
      EXPECT_EQ(Marker::LINE_STRIP, marker.type);
      ASSERT_GE(marker.points.size(), 4u);

      const auto &first = marker.points.front();
      const auto &last = marker.points.back();
      expectPoint(last, first.x, first.y, first.z);
    }
  }

  EXPECT_EQ(expected_keys, actual_keys);
}

TEST(VisualizerTest, UsesWorldMapBoundsWithNonzeroOrigin) {
  const auto message =
      Visualizer::buildSceneMarkers(makeScene(), rclcpp::Time(1, 0));
  const auto &boundary = findMarker(message, "map_boundary");

  ASSERT_EQ(5u, boundary.points.size());

  // x 上界 = -2 + 40 * 0.5 = 18
  // y 上界 = -3 + 32 * 0.5 = 13
  expectPoint(boundary.points[0], -2.0, -3.0);
  expectPoint(boundary.points[1], 18.0, -3.0);
  expectPoint(boundary.points[2], 18.0, 13.0);
  expectPoint(boundary.points[3], -2.0, 13.0);
  expectPoint(boundary.points[4], -2.0, -3.0);
}

TEST(VisualizerTest, PreservesObstacleVerticesAndRotatedParkingSpace) {
  const auto message =
      Visualizer::buildSceneMarkers(makeScene(), rclcpp::Time(1, 0));

  const auto &triangle = findMarker(message, "obstacles", 0);
  ASSERT_EQ(4u, triangle.points.size());

  expectPoint(triangle.points[0], 12.0, 0.0);
  expectPoint(triangle.points[1], 14.0, 0.0);
  expectPoint(triangle.points[2], 13.0, 2.0);
  expectPoint(triangle.points[3], 12.0, 0.0);

  const auto &rectangle = findMarker(message, "obstacles", 1);
  ASSERT_EQ(5u, rectangle.points.size());

  expectPoint(rectangle.points[0], 14.0, 9.0);
  expectPoint(rectangle.points[1], 16.0, 9.0);
  expectPoint(rectangle.points[2], 16.0, 11.0);
  expectPoint(rectangle.points[3], 14.0, 11.0);
  expectPoint(rectangle.points[4], 14.0, 9.0);

  const auto &slot = findMarker(message, "parking_space");
  ASSERT_EQ(5u, slot.points.size());

  // 中心(9,6)，朝向 pi，长6宽4
  expectPoint(slot.points[0], 6.0, 4.0);
  expectPoint(slot.points[1], 12.0, 4.0);
  expectPoint(slot.points[2], 12.0, 8.0);
  expectPoint(slot.points[3], 6.0, 8.0);
  expectPoint(slot.points[4], 6.0, 4.0);
}

TEST(VisualizerTest, PlacesVehicleOutlinesUsingRearAxlePoses) {
  const auto message =
      Visualizer::buildSceneMarkers(makeScene(), rclcpp::Time(1, 0));

  const auto &start = findMarker(message, "start_vehicle");
  ASSERT_EQ(5u, start.points.size());

  // 后轴(2,2)，朝向90度；前边界y=5，后边界y=1
  const std::array<Vec2d, 4> expected_start{
      Vec2d(1.0, 5.0),
      Vec2d(1.0, 1.0),
      Vec2d(3.0, 1.0),
      Vec2d(3.0, 5.0),
  };

  const auto &goal = findMarker(message, "goal_vehicle");
  ASSERT_EQ(5u, goal.points.size());

  // 后轴(10,6)，朝向180度；前边界x=7，后边界x=11
  const std::array<Vec2d, 4> expected_goal{
      Vec2d(7.0, 5.0),
      Vec2d(11.0, 5.0),
      Vec2d(11.0, 7.0),
      Vec2d(7.0, 7.0),
  };

  for (std::size_t i = 0; i < expected_start.size(); ++i) {
    SCOPED_TRACE("corner index: " + std::to_string(i));

    expectPoint(start.points[i], expected_start[i].x(), expected_start[i].y());
    expectPoint(goal.points[i], expected_goal[i].x(), expected_goal[i].y());
  }
}

TEST(VisualizerTest, DrawsHeadingArrowsFromRearAxleCenters) {
  const auto message =
      Visualizer::buildSceneMarkers(makeScene(), rclcpp::Time(1, 0));

  const auto &start = findMarker(message, "start_pose");
  ASSERT_EQ(2u, start.points.size());

  // 从后轴出发，沿世界+y延伸1.5米
  expectPoint(start.points[0], 2.0, 2.0, 0.10);
  expectPoint(start.points[1], 2.0, 3.5, 0.10);

  const auto &goal = findMarker(message, "goal_pose");
  ASSERT_EQ(2u, goal.points.size());

  // 从后轴出发，沿世界-x延伸1.5米
  expectPoint(goal.points[0], 10.0, 6.0, 0.10);
  expectPoint(goal.points[1], 8.5, 6.0, 0.10);
}

TEST(VisualizerTest, StartsEachSnapshotWithClearAndUsesCurrentSceneOnly) {
  const auto first =
      Visualizer::buildSceneMarkers(makeScene(), rclcpp::Time(1, 0));
  const auto second =
      Visualizer::buildSceneMarkers(makeScene(false), rclcpp::Time(2, 0));

  ASSERT_EQ(9u, first.markers.size());
  ASSERT_EQ(7u, second.markers.size());

  EXPECT_EQ(Marker::DELETEALL, first.markers.front().action);
  EXPECT_EQ(Marker::DELETEALL, second.markers.front().action);

  for (const auto &marker : second.markers) {
    EXPECT_EQ(2, marker.header.stamp.sec);
    EXPECT_NE("obstacles", marker.ns);
  }
}

} // namespace
} // namespace parking_planner::visualization