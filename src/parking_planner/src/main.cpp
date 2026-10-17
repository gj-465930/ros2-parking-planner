#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "parking_planner/runtime/parking_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<parking_planner::runtime::ParkingNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}