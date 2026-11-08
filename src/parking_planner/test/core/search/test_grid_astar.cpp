#include "parking_planner/core/search/grid_astar.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace parking_planner::core::search {
namespace {

using geometry::Vec2d;
using map::GridIndex;
using map::OccupancyGrid;

void expectValidPath(const OccupancyGrid &grid, const GridSearchResult &result,
                     const GridIndex &start, const GridIndex &goal) {
  ASSERT_FALSE(result.path.empty());

  EXPECT_EQ(start.x, result.path.front().x);
  EXPECT_EQ(start.y, result.path.front().y);
  EXPECT_EQ(goal.x, result.path.back().x);
  EXPECT_EQ(goal.y, result.path.back().y);

  double calculated_cost = 0.0;

  for (std::size_t i = 0; i < result.path.size(); ++i) {
    const GridIndex &index = result.path[i];

    ASSERT_TRUE(grid.contains(index));
    EXPECT_FALSE(grid.isOccupied(index));

    if (i == 0) {
      continue;
    }

    const GridIndex &previous = result.path[i - 1];
    const int dx = std::abs(index.x - previous.x);
    const int dy = std::abs(index.y - previous.y);

    EXPECT_LE(dx, 1);
    EXPECT_LE(dy, 1);
    EXPECT_GT(dx + dy, 0);

    const bool diagonal = dx == 1 && dy == 1;
    calculated_cost += grid.resolution() * (diagonal ? std::sqrt(2.0) : 1.0);

    if (diagonal) {
      const GridIndex horizontal{index.x, previous.y};
      const GridIndex vertical{previous.x, index.y};

      ASSERT_TRUE(grid.contains(horizontal));
      ASSERT_TRUE(grid.contains(vertical));
      EXPECT_FALSE(grid.isOccupied(horizontal));
      EXPECT_FALSE(grid.isOccupied(vertical));
    }
  }

  EXPECT_NEAR(result.path_cost, calculated_cost, 1e-9);
}

TEST(GridAStarTest, FindsOctileShortestPathOnOpenGrid) {
  const OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 5, 4);
  const GridAStar searcher(grid);

  const GridIndex start{0, 0};
  const GridIndex goal{3, 2};
  const auto result = searcher.search(start, goal);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(1.0 + 2.0 * std::sqrt(2.0), result->path_cost, 1e-9);
  expectValidPath(grid, *result, start, goal);
}

TEST(GridAStarTest, ScalesPathCostWithGridResolution) {
  const OccupancyGrid grid(Vec2d(0.0, 0.0), 0.5, 5, 4);
  const GridAStar searcher(grid);

  const GridIndex start{0, 0};
  const GridIndex goal{3, 2};
  const auto result = searcher.search(start, goal);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(0.5 * (1.0 + 2.0 * std::sqrt(2.0)), result->path_cost, 1e-9);
  expectValidPath(grid, *result, start, goal);
}

TEST(GridAStarTest, ReturnsSingleCellWhenStartEqualsGoal) {
  const OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 3, 3);
  const GridAStar searcher(grid);

  const GridIndex start{1, 1};
  const auto result = searcher.search(start, start);

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(1U, result->path.size());
  EXPECT_EQ(start.x, result->path.front().x);
  EXPECT_EQ(start.y, result->path.front().y);
  EXPECT_DOUBLE_EQ(0.0, result->path_cost);
  EXPECT_EQ(0U, result->expanded_nodes);
}

TEST(GridAStarTest, DoesNotCutCornerWhenOneSideCellIsOccupied) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 2, 2);
  grid.setOccupied(GridIndex{1, 0}, true);
  const GridAStar searcher(grid);

  const GridIndex start{0, 0};
  const GridIndex goal{1, 1};
  const auto result = searcher.search(start, goal);

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(3U, result->path.size());
  EXPECT_EQ(0, result->path[1].x);
  EXPECT_EQ(1, result->path[1].y);
  EXPECT_NEAR(2.0, result->path_cost, 1e-9);
  expectValidPath(grid, *result, start, goal);
}

TEST(GridAStarTest, ReturnsNoPathWhenObstacleWallSeparatesEndpoints) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 5, 3);

  for (int y = 0; y < grid.height(); ++y) {
    grid.setOccupied(GridIndex{2, y}, true);
  }

  const GridAStar searcher(grid);
  const auto result = searcher.search(GridIndex{0, 1}, GridIndex{4, 1});

  EXPECT_FALSE(result.has_value());
}

TEST(GridAStarTest, RejectsOutOfBoundsEndpoints) {
  const OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 3, 3);
  const GridAStar searcher(grid);

  EXPECT_THROW(searcher.search(GridIndex{-1, 0}, GridIndex{2, 2}),
               std::invalid_argument);
  EXPECT_THROW(searcher.search(GridIndex{0, 0}, GridIndex{3, 2}),
               std::invalid_argument);
}

TEST(GridAStarTest, RejectsOccupiedEndpoints) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 3, 3);
  grid.setOccupied(GridIndex{0, 0}, true);
  grid.setOccupied(GridIndex{2, 2}, true);
  const GridAStar searcher(grid);

  EXPECT_THROW(searcher.search(GridIndex{0, 0}, GridIndex{1, 1}),
               std::invalid_argument);
  EXPECT_THROW(searcher.search(GridIndex{1, 1}, GridIndex{2, 2}),
               std::invalid_argument);
}

TEST(GridAStarTest, RejectsUnsupportedHeuristic) {
  const OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 3, 3);
  const GridAStar searcher(grid);
  const auto invalid_heuristic = static_cast<GridHeuristic>(-1);

  EXPECT_THROW(
      searcher.search(GridIndex{0, 0}, GridIndex{2, 2}, invalid_heuristic),
      std::invalid_argument);
}

TEST(GridAStarTest, AStarAndDijkstraReturnSameCost) {
  const OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 20, 5);
  const GridAStar searcher(grid);

  const GridIndex start{0, 2};
  const GridIndex goal{19, 2};

  const auto astar_result = searcher.search(start, goal, GridHeuristic::Octile);
  const auto dijkstra_result =
      searcher.search(start, goal, GridHeuristic::Zero);

  ASSERT_TRUE(astar_result.has_value());
  ASSERT_TRUE(dijkstra_result.has_value());

  EXPECT_NEAR(astar_result->path_cost, dijkstra_result->path_cost, 1e-9);
  EXPECT_LT(astar_result->expanded_nodes, dijkstra_result->expanded_nodes);
  expectValidPath(grid, *astar_result, start, goal);
  expectValidPath(grid, *dijkstra_result, start, goal);
}

TEST(GridAStarTest, RepeatedSearchesUseFreshSearchRecords) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 5, 3);
  const GridAStar searcher(grid);

  const GridIndex start{0, 1};
  const GridIndex goal{4, 1};

  const auto first_result = searcher.search(start, goal);
  ASSERT_TRUE(first_result.has_value());

  for (int y = 0; y < grid.height(); ++y) {
    grid.setOccupied(GridIndex{2, y}, true);
  }

  const auto second_result = searcher.search(start, goal);
  EXPECT_FALSE(second_result.has_value());
}

} // namespace
} // namespace parking_planner::core::search