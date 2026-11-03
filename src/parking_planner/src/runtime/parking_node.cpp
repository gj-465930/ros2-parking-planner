#include <memory>
#include <stdexcept>
#include <string>

#include "parking_planner/core/scenario/scenario_loader.h"
#include "parking_planner/runtime/parking_node.h"

namespace parking_planner::runtime {

ParkingNode::ParkingNode()
    : rclcpp::Node("parking_node"), scenario_(loadScenarioFromParameters()) {
  RCLCPP_INFO(this->get_logger(), "Loaded parking scenario: %s",
              scenario_.name.c_str());

  visualizer_ = std::make_unique<visualization::Visualizer>(*this);

  visualizer_->publishScene(scenario_);
}

core::scenario::Scenario ParkingNode::loadScenarioFromParameters() {
  const std::string file_path =
      this->declare_parameter<std::string>("scenario_file", "");

  if (file_path.empty()) {
    throw std::runtime_error("scenario_file must specify a scenario YAML file");
  }

  RCLCPP_INFO(this->get_logger(), "Loading scenario from: %s",
              file_path.c_str());

  return core::scenario::loadScenarioFromFile(file_path);
}

} // namespace parking_planner::runtime