#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include "parking_planner/core/map/occupancy_grid.h"
#include "parking_planner/core/search/grid_astar.h"

namespace parking_planner::core::search {
namespace {

constexpr std::size_t kNoparent = std::numeric_limits<std::size_t>::max();

struct NodeRecord {
  std::size_t parent_id = kNoparent;
  double g = std::numeric_limits<double>::infinity();
  bool closed = false;
};

struct QueueEntry {
  map::GridIndex index;
  double g;
  double h;
  std::size_t order;
};

struct Compare {
  bool operator()(const QueueEntry &left, const QueueEntry &right) {
    const double left_f = left.g + left.h;
    const double right_f = right.g + right.h;

    if (left_f != right_f) {
      return left_f > right_f;
    }

    if (left.h != right.h) {
      return left.h > right.h;
    }

    return left.order > right.order;
  }
};

// 启发函数
double estimateCost(const map::GridIndex &current_grid,
                    const map::GridIndex &goal_grid, const double resolution,
                    const GridHeuristic &heuristic) {
  if (heuristic == GridHeuristic::Zero) {
    return 0.0;
  }

  const double dx = std::abs(static_cast<double>(goal_grid.x) - current_grid.x);
  const double dy = std::abs(static_cast<double>(goal_grid.y) - current_grid.y);

  const double diagonal_steps = std::min(dx, dy);
  const double stright_steps = std::max(dx, dy) - diagonal_steps;

  const double cost =
      std::sqrt(2) * diagonal_steps * resolution + stright_steps * resolution;

  if (!std::isfinite(cost)) {
    throw std::overflow_error("grid heuristic cost is not representable");
  }

  return cost;
}

// 八领域栅格
constexpr std::array<map::GridIndex, 8> kNeighborOffsets{{
    {1, 0},
    {1, 1},
    {0, 1},
    {-1, 1},
    {-1, 0},
    {-1, -1},
    {0, -1},
    {1, -1},
}};

} // namespace

GridAStar::GridAStar(const map::OccupancyGrid &grid) : grid_(grid) {}

std::optional<GridSearchResult>
GridAStar::search(const map::GridIndex &start, const map::GridIndex &goal,
                  GridHeuristic heuristic) const {
  if (!grid_.contains(start) || !grid_.contains(goal)) {
    throw std::invalid_argument("search endpoint is outside grid");
  }

  if (grid_.isOccupied(start) || grid_.isOccupied(goal)) {
    throw std::invalid_argument("search endpoint is occupied");
  }

  if (heuristic != GridHeuristic::Octile && heuristic != GridHeuristic::Zero) {
    throw std::invalid_argument("unsupported grid heuristic");
  }

  const std::size_t width = static_cast<std::size_t>(grid_.width());
  const std::size_t height = static_cast<std::size_t>(grid_.height());

  std::vector<NodeRecord> nodes;
  if (width * height > nodes.max_size()) {
    throw std::length_error("search nodes count exceed container limit");
  }
  nodes.resize(width * height);

  // 根据栅格索引得到线性 id
  const auto linearIndex = [&width](const map::GridIndex &index) {
    return static_cast<std::size_t>(index.y) * width +
           static_cast<std::size_t>(index.x);
  };

  // 根据线性 id 得到栅格索引
  const auto indexLinear = [&width](std::size_t current_id) {
    map::GridIndex index;
    index.x = current_id % width;
    index.y = (current_id - index.x) / width;
    return index;
  };

  const auto isFree = [this](const map::GridIndex &index) {
    return grid_.contains(index) && !grid_.isOccupied(index);
  };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Compare> open;

  std::size_t next_order = 0;
  std::size_t expanded_nodes = 0;

  // 初始化
  const auto start_id = linearIndex(start);
  const auto goal_id = linearIndex(goal);
  nodes[start_id].g = 0.0;

  open.push(QueueEntry{start, 0.0,
                       estimateCost(start, goal, grid_.resolution(), heuristic),
                       next_order++});

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();

    std::size_t current_id = linearIndex(current.index);
    NodeRecord &current_record = nodes[current_id];

    // 跳过过期条目
    if (current.g > current_record.g) {
      continue;
    }

    if (current_record.closed) {
      continue;
    }

    if (current_id == goal_id) {
      GridSearchResult result;
      result.path_cost = nodes[current_id].g;
      result.expanded_nodes = expanded_nodes;

      std::vector<map::GridIndex> path;
      for (std::size_t i = current_id; i != kNoparent; i = nodes[i].parent_id) {
        const auto current_index = indexLinear(i);
        path.push_back({current_index.x, current_index.y});
      }

      std::reverse(path.begin(), path.end());
      result.path = path;

      return result;
    }

    // 一致启发函数下，有效弹出的节点可关闭，无需重开
    current_record.closed = true;
    ++expanded_nodes;

    // 检查八邻域栅格
    for (const map::GridIndex &offset : kNeighborOffsets) {
      const map::GridIndex next{
          current.index.x + offset.x,
          current.index.y + offset.y,
      };

      if (!isFree(next)) {
        continue;
      }

      // 判断这个领居是不是斜向的
      const bool diagonal = offset.x != 0 && offset.y != 0;

      if (diagonal) {
        // 如果是斜向还需判断当前格子垂直侧是否空间  禁止斜向切角
        const map::GridIndex horizontal{
            current.index.x + offset.x,
            current.index.y,
        };

        const map::GridIndex vertical{
            current.index.x,
            current.index.y + offset.y,
        };

        if (!isFree(horizontal) || !isFree(vertical)) {
          continue;
        }
      }

      const std::size_t next_id = linearIndex(next);
      NodeRecord &next_record = nodes[next_id];

      if (next_record.closed) {
        continue;
      }

      // 松弛
      const double step_cost =
          grid_.resolution() * (diagonal ? std::sqrt(2) : 1);
      const double candidate_g = nodes[current_id].g + step_cost;

      if (!std::isfinite(candidate_g)) {
        throw std::overflow_error("grid path cost is not representable");
      }

      if (candidate_g >= next_record.g) {
        continue;
      }

      const double h = estimateCost(next, goal, grid_.resolution(), heuristic);

      if (!std::isfinite(candidate_g + h)) {
        throw std::overflow_error("grid priority cost is not representable");
      }

      next_record.g = candidate_g;
      next_record.parent_id = current_id;

      open.push(QueueEntry{next, candidate_g, h, next_order++});
    }
  }

  return std::nullopt;
}

} // namespace parking_planner::core::search