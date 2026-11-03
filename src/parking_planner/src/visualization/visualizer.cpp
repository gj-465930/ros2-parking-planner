#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"
#include "parking_planner/core/scenario/scenario.h"
#include "parking_planner/visualization/visualizer.h"

namespace parking_planner::visualization {

Visualizer::Visualizer(rclcpp::Node &node) : node_(&node) {
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/parking/markers", qos);
}

visualization_msgs::msg::MarkerArray
Visualizer::buildSceneMarkers(const core::scenario::Scenario &scene,
                              const rclcpp::Time &stamp) {
  using core::geometry::Pose2d;
  using core::geometry::Vec2d;
  using visualization_msgs::msg::Marker;

  visualization_msgs::msg::MarkerArray message;

  Marker clear_marker;
  clear_marker.header.frame_id = "map";
  clear_marker.header.stamp = stamp;
  clear_marker.action = Marker::DELETEALL;

  message.markers.push_back(std::move(clear_marker));

  // 添加闭合轮廓
  const auto addOutline = [&message, &stamp](const std::string &ns,
                                             std::int32_t id,
                                             const auto &vertices, float red,
                                             float green, float blue) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;

    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;

    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;

    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 0;

    for (const auto &vertice : vertices) {
      geometry_msgs::msg::Point point;
      point.x = vertice.x();
      point.y = vertice.y();
      point.z = 0.0;

      marker.points.push_back(point);
    }

    // 重复首点闭合
    marker.points.push_back(marker.points.front());

    message.markers.push_back(std::move(marker));
  };

  // 添加后轴位置与车头朝向箭头
  const auto addArrow = [&message, &stamp](const std::string ns,
                                           const Pose2d &pose, float red,
                                           float green, float blue) {
    constexpr double kArrowLength = 1.5;
    Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = stamp;

    marker.ns = ns;
    marker.id = 0;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.12;
    marker.scale.y = 0.24;
    marker.scale.z = 0.35;

    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;

    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 0;

    geometry_msgs::msg::Point tail;
    tail.x = pose.x;
    tail.y = pose.y;
    tail.z = 0.10;

    geometry_msgs::msg::Point tip;
    tip.x = tail.x + kArrowLength * std::cos(pose.theta);
    tip.y = tail.y + kArrowLength * std::sin(pose.theta);
    tip.z = 0.10;

    marker.points.push_back(tail);
    marker.points.push_back(tip);

    message.markers.push_back(std::move(marker));
  };

  // 地图边界，灰色
  const auto &grid = scene.environment.grid();
  const Vec2d origin = grid.origin();

  const std::array<Vec2d, 4> map_corners{
      Vec2d(origin.x(), origin.y()), Vec2d(grid.maxX(), origin.y()),
      Vec2d(grid.maxX(), grid.maxY()), Vec2d(origin.x(), grid.maxY())};

  addOutline("map_boundary", 0, map_corners, 0.7F, 0.7F, 0.7F);

  // 障碍物轮廓，红色
  const auto &obstacles = scene.environment.obstacles();
  if (obstacles.size() >
      static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::length_error("too many obstacles for ids");
  }

  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    addOutline("obstacles", static_cast<std::int32_t>(i),
               obstacles[i].vertices(), 0.9F, 0.25F, 0.2F);
  }

  // 目标车位，黄色
  addOutline("parking_space", 0, scene.parking_space.corners(), 1.0F, 0.8F,
             0.1F);

  // 起点车框绿色，目标点车框蓝色
  const auto start_footprint = scene.vehicle.footprint(scene.start);
  const auto goal_footprint = scene.vehicle.footprint(scene.goal);

  addOutline("start_vehicle", 0, start_footprint.corners(), 0.1F, 0.8F, 0.2F);
  addOutline("goal_vehicle", 0, goal_footprint.corners(), 0.2F, 0.6F, 1.0F);

  // 后轴中心与朝向，颜色和对应车框保持一致
  addArrow("start_pose", scene.start, 0.1F, 0.8F, 0.2F);
  addArrow("goal_pose", scene.goal, 0.2F, 0.6F, 1.0F);

  return message;
}

void Visualizer::publishScene(const core::scenario::Scenario &scene) {
  marker_pub_->publish(buildSceneMarkers(scene, node_->now()));
}

} // namespace parking_planner::visualization