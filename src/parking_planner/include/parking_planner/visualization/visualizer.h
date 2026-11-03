#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "parking_planner/core/scenario/scenario.h"

namespace parking_planner::visualization {

/**
 * @brief 将泊车场景转换成可视化消息，并发布到 RViz。
 *
 * 借用 ROS 节点，不接管节点所有权。
 * 节点必须在 Visualizer 使用期间保持有效。
 */
class Visualizer {
public:
  /**
   * @brief 使用已有节点创建可视化发布器。
   * @param node 被借用的 ROS 节点。
   */
  explicit Visualizer(rclcpp::Node &node);

  /**
   * @brief 构造完整的静态场景可视化消息，不执行发布。
   *
   * 显示地图边界、障碍物、车位、起终车辆轮廓与后轴朝向。
   * 所有标记使用 map 坐标系和同一个时间戳。
   * 消息以 DELETEALL 开头，用于清理旧场景标记；
   * 对应可视化话题应由本 Visualizer 独占。
   *
   * @param scene 已通过加载校验的场景；位置为米，角度为弧度。
   * @param stamp 本批标记使用的 ROS 时间戳。
   * @return 包含清理指令和当前场景全部标记的消息。
   */
  static visualization_msgs::msg::MarkerArray
  buildSceneMarkers(const core::scenario::Scenario &scene,
                    const rclcpp::Time &stamp);

  /**
   * @brief 使用节点当前时间构造并发布完整场景。
   * @param scene 已通过加载校验的场景，只借用，不保存或复制。
   */
  void publishScene(const core::scenario::Scenario &scene);

private:
  rclcpp::Node *node_; ///< 借用的节点，不拥有其生命周期

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_; ///< 场景标记发布器
};

} // namespace parking_planner::visualization