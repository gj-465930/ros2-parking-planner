#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "parking_planner/runtime/parking_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    auto node = std::make_shared<parking_planner::runtime::ParkingNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_main"), "Parking node failed: %s",
                 error.what());

    exit_code = 1;
  }

  rclcpp::shutdown();

  return exit_code;
}