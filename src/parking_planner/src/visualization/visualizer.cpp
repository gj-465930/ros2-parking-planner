#include "parking_planner/visualization/visualizer.h"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace parking_planner::visualization {

Visualizer::Visualizer(rclcpp::Node &node) : node_(&node) {
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/parking/markers", rclcpp::QoS(10));
}

void Visualizer::publishVehicle() {
  constexpr double half_length = 4.5 / 2.0;
  constexpr double half_width = 1.8 / 2.0;

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = node_->now();
  marker.header.frame_id = "map";

  marker.ns = "vehicle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;

  marker.color.r = 0.1F;
  marker.color.g = 0.8F;
  marker.color.b = 0.2F;
  marker.color.a = 1.0F;

  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;

  const auto add_point = [&marker](double x, double y) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    marker.points.push_back(point);
  };

  add_point(half_length, half_width);
  add_point(half_length, -half_width);
  add_point(-half_length, -half_width);
  add_point(-half_length, half_width);
  add_point(half_length, half_width);

  visualization_msgs::msg::MarkerArray message;
  message.markers.push_back(marker);
  marker_pub_->publish(message);
}

} // namespace parking_planner::visualization