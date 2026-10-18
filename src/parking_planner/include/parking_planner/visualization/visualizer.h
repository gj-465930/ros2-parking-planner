#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace parking_planner::visualization {

class Visualizer {
public:
  explicit Visualizer(rclcpp::Node &node);

  void publishVehicle();

private:
  rclcpp::Node *node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
};

} // namespace parking_planner::visualization