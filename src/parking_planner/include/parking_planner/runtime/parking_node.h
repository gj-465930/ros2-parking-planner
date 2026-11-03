#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "parking_planner/core/scenario/scenario.h"
#include "parking_planner/visualization/visualizer.h"

namespace parking_planner::runtime {

/**
 * @brief 加载静态泊车场景，并在启动时发布可视化消息
 *
 * 场景在节点启动时加载一次，节点独立持有其数据
 * 图形消息的构造与发布由 Visualizer 负责
 */
class ParkingNode : public rclcpp::Node {
public:
  /**
   * @brief 创建节点、加载场景并发布完整场景
   * @throws std::runtime_error 场景无法读取或未通过校验
   */
  ParkingNode();

private:
  /**
   * @brief 读取节点的场景文件参数，并调用 core 加载器
   * @return 已通过校验的完整场景
   */
  core::scenario::Scenario loadScenarioFromParameters();

  const core::scenario::Scenario scenario_; ///< 启动时加载的场景

  std::unique_ptr<visualization::Visualizer> visualizer_;
};

} // namespace parking_planner::runtime