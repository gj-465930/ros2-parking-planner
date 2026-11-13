#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "parking_planner/core/map/occupancy_grid.h"

namespace parking_planner::core::search {

/**
 * @brief 从固定终点构建二维栅格最短距离表
 *
 * 使用八邻域，直向移动代价为 resolution，
 * 斜向移动代价为 sqrt(2) * resolution，单位米
 * 禁止斜向切角：目标格及两侧正交格必须在地图内且空闲
 *
 * 保存构建时的地图快照，不持有外部地图的引用
 * 外部地图变化后，调用方需要重新构建
 * 仅考虑格子中心代表的点，不考虑车辆轮廓和运动学约束
 */
class DistanceToGoalMap {
public:
  /**
   * @brief 创建尚未构建的距离表
   */
  DistanceToGoalMap() = default;

  /**
   * @brief 从终点执行 Dijkstra，计算所有可达格子的最短距离
   *
   * 每次调用先使旧结果失效，再进行完整重建
   * 返回 false 或抛出异常后，对象保持未构建状态
   * 调用期间不得修改传入的地图
   *
   * @param grid 用于构建距离表的占据栅格
   * @param goal 终点格子，必须在地图内且空闲
   * @return 构建成功返回 true；终点越界或被占用返回 false
   *         终点与其他空闲区域不连通，仍可构建成功
   * @throws std::length_error 所需记录数量超过容器容量上限
   * @throws std::overflow_error 搜索所需的米制代价无法有限表示
   * @throws std::bad_alloc 内存分配失败
   */
  bool build(const map::OccupancyGrid &grid, const map::GridIndex &goal);

  /**
   * @brief 查询格子到终点的最短距离，时间复杂度为 O(1)
   *
   * 查询依据最近一次成功构建时的地图快照
   *
   * @param index 待查询的格子索引
   * @return 可达时返回有限距离，单位米，终点自身为零；
   *         格子在地图内且空闲，但不可达时返回正无穷大；
   *         格子越界或被占用时返回 std::nullopt
   * @throws std::logic_error 尚未成功构建，或最近一次构建失败
   */
  std::optional<double> cost(const map::GridIndex &index) const;

  /**
   * @brief 获取最近一次成功构建实际展开邻居的节点数量
   *
   * 包含终点，不包含过期或重复队列条目
   * 未构建或最近一次构建失败时返回零
   */
  std::size_t expandedNodes() const { return expanded_nodes_; }

private:
  std::optional<map::OccupancyGrid> grid_snapshot_; ///< 构建时的地图快照
  std::vector<double> cost_;       ///< 按 y * width + x 保存的米制距离
  std::size_t expanded_nodes_ = 0; ///< 实际展开的节点数量
};

} // namespace parking_planner::core::search