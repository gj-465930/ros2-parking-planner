#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <vector>

#include "parking_planner/core/map/occupancy_grid.h"
#include "parking_planner/core/search/distance_to_goal_map.h"

namespace parking_planner::core::search {
namespace {

struct QueueEntry {
  map::GridIndex index;
  double g;
};

struct Record {
  double g = std::numeric_limits<double>::infinity();
  bool closed = false;
};

struct Compare {
  bool operator()(const QueueEntry &left, const QueueEntry &right) {
    if (left.g != right.g) {
      return left.g > right.g;
    }

    if (left.index.y != right.index.y) {
      return left.index.y > right.index.y;
    }

    return left.index.x > right.index.x;
  }
};

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

bool DistanceToGoalMap::build(const map::OccupancyGrid &grid,
                              const map::GridIndex &goal) {

  // 每次重建先使旧结果失效
  grid_snapshot_.reset();
  cost_.clear();
  expanded_nodes_ = 0;

  if (!grid.contains(goal) || grid.isOccupied(goal)) {
    return false;
  }

  const std::size_t width = static_cast<std::size_t>(grid.width());
  const std::size_t height = static_cast<std::size_t>(grid.height());

  const std::size_t cell_count = width * height;

  std::vector<Record> nodes;
  if (cell_count > nodes.max_size()) {
    throw std::length_error("nodes exceeds container limit");
  }

  nodes.resize(cell_count);

  const auto linearIndex = [&width](const map::GridIndex &index) {
    return static_cast<std::size_t>(index.y) * width +
           static_cast<std::size_t>(index.x);
  };

  const auto isFree = [&grid](const map::GridIndex &index) {
    return grid.contains(index) && !grid.isOccupied(index);
  };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Compare> open;
  std::size_t expanded_nodes = 0;

  // 初始化
  const std::size_t goal_id = linearIndex(goal);

  nodes[goal_id].g = 0.0;
  open.push({goal, 0.0});

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();

    const std::size_t current_id = linearIndex(current.index);

    if (current.g > nodes[current_id].g) {
      continue;
    }

    if (nodes[current_id].closed == true) {
      continue;
    }

    nodes[current_id].closed = true;
    ++expanded_nodes;

    for (const auto &offset : kNeighborOffsets) {
      const map::GridIndex next{current.index.x + offset.x,
                                current.index.y + offset.y};

      if (!isFree(next)) {
        continue;
      }

      const bool diagonal = offset.x != 0 && offset.y != 0;
      if (diagonal) {
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

      const double steps_cost =
          grid.resolution() * (diagonal ? std::sqrt(2.0) : 1.0);

      const double candidate_cost = nodes[current_id].g + steps_cost;

      if (!std::isfinite(candidate_cost)) {
        throw std::overflow_error(
            "distance map path cost is not representable");
      }

      if (candidate_cost >= nodes[next_id].g) {
        continue;
      }

      nodes[next_id].g = candidate_cost;
      open.push({next, candidate_cost});
    }
  }

  cost_.reserve(nodes.size());

  for (const Record &record : nodes) {
    cost_.push_back(record.g);
  }

  grid_snapshot_.emplace(grid);
  expanded_nodes_ = expanded_nodes;

  return true;
}

std::optional<double>
DistanceToGoalMap::cost(const map::GridIndex &index) const {
  if (!grid_snapshot_.has_value()) {
    throw std::logic_error("distance map has not been successfully built");
  }

  const map::OccupancyGrid &grid = *grid_snapshot_;
  if (!grid.contains(index) || grid.isOccupied(index)) {
    return std::nullopt;
  }

  const std::size_t width = static_cast<std::size_t>(grid.width());

  const std::size_t index_id = static_cast<std::size_t>(index.y) * width +
                               static_cast<std::size_t>(index.x);

  return cost_[index_id];
}

} // namespace parking_planner::core::search