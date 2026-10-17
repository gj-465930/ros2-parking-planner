#pragma once

#include <rclcpp/rclcpp.hpp>

namespace parking_planner::runtime {

class ParkingNode : public rclcpp::Node {
public:
  ParkingNode();
};

} // namespace parking_planner::runtime