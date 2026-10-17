#include "parking_planner/runtime/parking_node.h"

#include "parking_planner/core/core_smoke.h"

namespace parking_planner::runtime {

ParkingNode::ParkingNode() : rclcpp::Node("parking_node") {
  const int value = parking_planner::core::coreSmokeValue();
  RCLCPP_INFO(this->get_logger(), "Parking node ready, core value: %d", value);
}

} // namespace parking_planner::runtime