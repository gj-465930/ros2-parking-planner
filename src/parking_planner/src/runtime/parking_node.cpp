#include "parking_planner/runtime/parking_node.h"

#include "parking_planner/core/core_smoke.h"

#include <chrono>
#include <memory>

namespace parking_planner::runtime {

ParkingNode::ParkingNode() : rclcpp::Node("parking_node") {
  const int value = parking_planner::core::coreSmokeValue();
  RCLCPP_INFO(this->get_logger(), "Parking node ready, core value: %d", value);

  visualizer_ = std::make_unique<visualization::Visualizer>(*this);
  visualization_timer_ =
      this->create_wall_timer(std::chrono::seconds(1), [this]() {
        this->visualizer_->publishVehicle();
      });
}

} // namespace parking_planner::runtime