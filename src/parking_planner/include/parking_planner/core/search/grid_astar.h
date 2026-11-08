#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "parking_planner/core/map/occupancy_grid.h"

namespace parking_planner::core::search {

/**
 * @brief 二维栅格搜索使用的启发方式
 */
enum class GridHeuristic {
  Octile, ///< 忽略障碍物的八邻域最短距离，执行 A*
  Zero,   ///< 启发值恒为零，执行 Dijkstra
};

/**
 * @brief 一次成功的二维栅格搜索结果
 */
struct GridSearchResult {
  std::vector<map::GridIndex> path; ///< 起点到终点的格子序列，包含两端
  double path_cost = 0.0;           ///< 累计路径长度，单位米
  std::size_t expanded_nodes = 0; ///< 实际展开领居的节点数，不含终点和过期条目
};

/**
 * @brief 在只读占据栅格上执行八邻域最短路径搜索
 *
 * 直向移动代价为 resolution，斜向为 sqrt(2) * resolution，单位米
 * 禁止斜向切角：斜步的目标格及两侧正交格都必须在地图内且空闲
 * 搜索对象为格子中心代表的点，不考虑车辆轮廓与运动学约束
 */
class GridAStar {
public:
  /**
   * @brief 借用搜索使用的地图，不复制或接管地图所有权
   *
   * @param grid 占据栅格，生命周期必须覆盖本搜索对象
   * 搜索执行期间，调用方不得修改地图
   */
  explicit GridAStar(const map::OccupancyGrid &grid);

  /**
   * @brief 搜索从起点格子到终点格子的最短路径
   *
   * 每次调用独立建立搜索记录，可对同一对象重复调用
   *
   * @param start 起点格子，必须在地图内且空闲
   * @param goal 终点格子，必须在地图内且空闲
   * @param heuristic 启发方式，默认使用 Octile
   * @return 成功时返回路径、米制代价和扩展节点数；
   * 合法但无路时返回 std::nullopt
   * 起终点同格时返回单点路径，代价和扩展数为零
   * @throws std::invalid_argument
   * std::length_error
   * std::overflow_error
   * 起点或终点越界、被占用，或传入不支持的启发枚举值
   */
  std::optional<GridSearchResult>
  search(const map::GridIndex &start, const map::GridIndex &goal,
         GridHeuristic heuristic = GridHeuristic::Octile) const;

private:
  const map::OccupancyGrid &grid_; ///< 借用的只读地图
};

} // namespace parking_planner::core::search