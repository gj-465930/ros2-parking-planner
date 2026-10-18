#pragma once

#include <memory>

#include "parking_planner/visualization/visualizer.h"
#include <rclcpp/rclcpp.hpp>

namespace parking_planner::runtime {

class ParkingNode : public rclcpp::Node {
public:
  ParkingNode();

private:
  std::unique_ptr<visualization::Visualizer> visualizer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

} // namespace parking_planner::runtime