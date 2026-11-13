#include "parking_planner/core/search/distance_to_goal_map.h"
#include "parking_planner/core/search/grid_astar.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace parking_planner::core::search {
namespace {

using geometry::Vec2d;
using map::GridIndex;
using map::OccupancyGrid;

constexpr double kTolerance = 1e-9;

// 先确认 optional 有值，再比较距离，避免解引用空 optional
void expectCost(const DistanceToGoalMap &distance_map, const GridIndex &index,
                double expected) {
  const auto actual = distance_map.cost(index);

  ASSERT_TRUE(actual.has_value());
  EXPECT_NEAR(expected, *actual, kTolerance);
}

TEST(DistanceToGoalMapTest, RejectsQueryBeforeBuild) {
  const DistanceToGoalMap distance_map;

  EXPECT_EQ(0U, distance_map.expandedNodes());
  EXPECT_THROW(distance_map.cost(GridIndex{0, 0}), std::logic_error);
}

TEST(DistanceToGoalMapTest, MatchesOctileDistanceOnOpenGrid) {
  const OccupancyGrid grid(Vec2d(-2.0, -1.0), 0.5, 5, 4);
  const GridIndex goal{3, 2};

  DistanceToGoalMap distance_map;
  ASSERT_TRUE(distance_map.build(grid, goal));

  // 无障碍时，每个格子都有已知的八邻域最短距离
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const int dx = std::abs(x - goal.x);
      const int dy = std::abs(y - goal.y);

      const int diagonal_steps = std::min(dx, dy);
      const int straight_steps = std::max(dx, dy) - diagonal_steps;

      const double expected =
          grid.resolution() *
          (std::sqrt(2.0) * diagonal_steps + straight_steps);

      expectCost(distance_map, GridIndex{x, y}, expected);
    }
  }

  expectCost(distance_map, goal, 0.0);
  EXPECT_EQ(20U, distance_map.expandedNodes());
}

TEST(DistanceToGoalMapTest, FindsHandCalculatedDetourAroundWall) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 3, 3);
  grid.setOccupied(GridIndex{1, 0}, true);
  grid.setOccupied(GridIndex{1, 1}, true);

  DistanceToGoalMap distance_map;
  ASSERT_TRUE(distance_map.build(grid, GridIndex{2, 0}));

  // 上行两格、右行两格、下行两格，禁止斜向切角
  expectCost(distance_map, GridIndex{0, 0}, 6.0);
  EXPECT_EQ(7U, distance_map.expandedNodes());
}

TEST(DistanceToGoalMapTest, ForbidsCornerCuttingAndAllowsIsolatedGoal) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 2, 2);
  grid.setOccupied(GridIndex{1, 0}, true);

  DistanceToGoalMap distance_map;
  ASSERT_TRUE(distance_map.build(grid, GridIndex{1, 1}));

  // 不能直接斜走，必须经过 (0,1)
  expectCost(distance_map, GridIndex{0, 0}, 2.0);
  EXPECT_EQ(3U, distance_map.expandedNodes());

  // 再封住另一侧，两个空闲格只能对角相望，无法连通
  grid.setOccupied(GridIndex{0, 1}, true);
  ASSERT_TRUE(distance_map.build(grid, GridIndex{1, 1}));

  const auto unreachable = distance_map.cost(GridIndex{0, 0});
  ASSERT_TRUE(unreachable.has_value());
  EXPECT_EQ(std::numeric_limits<double>::infinity(), *unreachable);

  expectCost(distance_map, GridIndex{1, 1}, 0.0);
  EXPECT_EQ(1U, distance_map.expandedNodes());
}

TEST(DistanceToGoalMapTest, ReturnsInfinityAcrossSeparatingWall) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 5, 3);

  for (int y = 0; y < grid.height(); ++y) {
    grid.setOccupied(GridIndex{2, y}, true);
  }

  DistanceToGoalMap distance_map;
  ASSERT_TRUE(distance_map.build(grid, GridIndex{4, 1}));

  const auto unreachable = distance_map.cost(GridIndex{0, 1});
  ASSERT_TRUE(unreachable.has_value());
  EXPECT_EQ(std::numeric_limits<double>::infinity(), *unreachable);

  expectCost(distance_map, GridIndex{3, 1}, 1.0);

  // 只有墙右边的 2 × 3 个空闲格可达
  EXPECT_EQ(6U, distance_map.expandedNodes());
}

TEST(DistanceToGoalMapTest, ReturnsNulloptForInvalidQueryCells) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 3, 3);
  grid.setOccupied(GridIndex{1, 1}, true);

  DistanceToGoalMap distance_map;
  ASSERT_TRUE(distance_map.build(grid, GridIndex{0, 0}));

  EXPECT_FALSE(distance_map.cost(GridIndex{-1, 0}).has_value());
  EXPECT_FALSE(distance_map.cost(GridIndex{0, -1}).has_value());
  EXPECT_FALSE(distance_map.cost(GridIndex{3, 0}).has_value());
  EXPECT_FALSE(distance_map.cost(GridIndex{0, 3}).has_value());
  EXPECT_FALSE(distance_map.cost(GridIndex{1, 1}).has_value());

  // 非法查询不应破坏已经构建的结果
  expectCost(distance_map, GridIndex{0, 0}, 0.0);
}

TEST(DistanceToGoalMapTest, InvalidGoalDiscardsPreviousResult) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 3, 3);
  grid.setOccupied(GridIndex{1, 1}, true);

  const std::array<GridIndex, 5> invalid_goals{{
      {-1, 0},
      {0, -1},
      {3, 0},
      {0, 3},
      {1, 1},
  }};

  DistanceToGoalMap distance_map;

  for (const GridIndex &goal : invalid_goals) {
    // 先有一份成功结果，再尝试非法重建
    ASSERT_TRUE(distance_map.build(grid, GridIndex{0, 0}));

    EXPECT_FALSE(distance_map.build(grid, goal));
    EXPECT_EQ(0U, distance_map.expandedNodes());
    EXPECT_THROW(distance_map.cost(GridIndex{0, 0}), std::logic_error);
  }
}

TEST(DistanceToGoalMapTest, UsesSnapshotUntilExplicitRebuild) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 4, 1);

  DistanceToGoalMap distance_map;
  ASSERT_TRUE(distance_map.build(grid, GridIndex{3, 0}));
  expectCost(distance_map, GridIndex{0, 0}, 3.0);

  grid.setOccupied(GridIndex{1, 0}, true);

  // 原地图改变，不影响已经保存的快照
  expectCost(distance_map, GridIndex{0, 0}, 3.0);
  expectCost(distance_map, GridIndex{1, 0}, 2.0);

  // 显式重建后，才使用新的占用状态
  ASSERT_TRUE(distance_map.build(grid, GridIndex{3, 0}));

  const auto unreachable = distance_map.cost(GridIndex{0, 0});
  ASSERT_TRUE(unreachable.has_value());
  EXPECT_EQ(std::numeric_limits<double>::infinity(), *unreachable);
  EXPECT_FALSE(distance_map.cost(GridIndex{1, 0}).has_value());
  EXPECT_EQ(2U, distance_map.expandedNodes());

  // 换成尺寸、分辨率都不同的地图，不能残留旧数据
  const OccupancyGrid smaller_grid(Vec2d(5.0, 5.0), 0.5, 2, 1);
  ASSERT_TRUE(distance_map.build(smaller_grid, GridIndex{0, 0}));

  expectCost(distance_map, GridIndex{0, 0}, 0.0);
  expectCost(distance_map, GridIndex{1, 0}, 0.5);
  EXPECT_FALSE(distance_map.cost(GridIndex{3, 0}).has_value());
  EXPECT_EQ(2U, distance_map.expandedNodes());
}

TEST(DistanceToGoalMapTest, RebuildsForDifferentGoal) {
  const OccupancyGrid grid(Vec2d(0.0, 0.0), 1.0, 4, 1);
  DistanceToGoalMap distance_map;

  ASSERT_TRUE(distance_map.build(grid, GridIndex{3, 0}));
  expectCost(distance_map, GridIndex{0, 0}, 3.0);

  ASSERT_TRUE(distance_map.build(grid, GridIndex{0, 0}));
  expectCost(distance_map, GridIndex{0, 0}, 0.0);
  expectCost(distance_map, GridIndex{3, 0}, 3.0);
}

TEST(DistanceToGoalMapTest, MatchesAStarFromMultipleStartCells) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 0.5, 5, 4);

  for (int y = 0; y < 3; ++y) {
    grid.setOccupied(GridIndex{2, y}, true);
  }

  const GridIndex goal{4, 0};
  const GridAStar searcher(grid);

  DistanceToGoalMap distance_map;
  ASSERT_TRUE(distance_map.build(grid, goal));

  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex start{x, y};

      if (grid.isOccupied(start)) {
        continue;
      }

      const auto result = searcher.search(start, goal);
      ASSERT_TRUE(result.has_value());

      expectCost(distance_map, start, result->path_cost);
    }
  }
}

TEST(DistanceToGoalMapTest, OverflowDiscardsPreviousResult) {
  DistanceToGoalMap distance_map;

  const OccupancyGrid normal_grid(Vec2d(0.0, 0.0), 1.0, 2, 1);
  ASSERT_TRUE(distance_map.build(normal_grid, GridIndex{0, 0}));

  // 地图边界仍能表示，但绕墙路径长度超过 double 上限
  const double resolution = std::numeric_limits<double>::max() / 4.0;

  OccupancyGrid huge_grid(Vec2d(0.0, 0.0), resolution, 3, 3);
  huge_grid.setOccupied(GridIndex{1, 0}, true);
  huge_grid.setOccupied(GridIndex{1, 1}, true);

  EXPECT_THROW(distance_map.build(huge_grid, GridIndex{2, 0}),
               std::overflow_error);

  EXPECT_EQ(0U, distance_map.expandedNodes());
  EXPECT_THROW(distance_map.cost(GridIndex{0, 0}), std::logic_error);
}

} // namespace
} // namespace parking_planner::core::search