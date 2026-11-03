#include "parking_planner/core/map/occupancy_grid.h"

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <limits>
#include <optional>
#include <stdexcept>

namespace parking_planner::core::map {
namespace {

using geometry::Vec2d;

void expectIndex(const std::optional<GridIndex> &actual, int expected_x,
                 int expected_y) {
  ASSERT_TRUE(actual.has_value());
  EXPECT_EQ(expected_x, actual->x);
  EXPECT_EQ(expected_y, actual->y);
}

TEST(OccupancyGridTest, StoresMetadataAndStartsFree) {
  const OccupancyGrid grid(Vec2d(-1.0, -1.0), 0.5, 4, 3);

  EXPECT_DOUBLE_EQ(-1.0, grid.origin().x());
  EXPECT_DOUBLE_EQ(-1.0, grid.origin().y());
  EXPECT_DOUBLE_EQ(0.5, grid.resolution());
  EXPECT_EQ(4, grid.width());
  EXPECT_EQ(3, grid.height());

  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      EXPECT_FALSE(grid.isOccupied(GridIndex{x, y}));
    }
  }
}

TEST(OccupancyGridTest, MapsWorldPointsToCells) {
  const OccupancyGrid grid(Vec2d(-1.0, -1.0), 0.5, 4, 3);

  expectIndex(grid.worldToGrid(Vec2d(-1.0, -1.0)), 0, 0);
  expectIndex(grid.worldToGrid(Vec2d(0.2, -0.3)), 2, 1);
  expectIndex(grid.worldToGrid(Vec2d(0.9, 0.4)), 3, 2);

  // 内部格线上的点属于右侧、上侧的格子
  expectIndex(grid.worldToGrid(Vec2d(0.0, -0.5)), 2, 1);
}

TEST(OccupancyGridTest, UsesHalfOpenMapBounds) {
  const OccupancyGrid grid(Vec2d(-1.0, -1.0), 0.5, 4, 3);

  // 地图覆盖 x∈[-1,1)，y∈[-1,0.5)
  EXPECT_FALSE(grid.worldToGrid(Vec2d(-1.1, 0.0)).has_value());
  EXPECT_FALSE(grid.worldToGrid(Vec2d(0.0, -1.1)).has_value());
  EXPECT_FALSE(grid.worldToGrid(Vec2d(1.0, 0.0)).has_value());
  EXPECT_FALSE(grid.worldToGrid(Vec2d(0.0, 0.5)).has_value());

  // nextafter 返回朝指定方向的下一个可表示浮点数
  const double just_inside_x = std::nextafter(1.0, 0.0);
  const double just_inside_y = std::nextafter(0.5, 0.0);

  expectIndex(grid.worldToGrid(Vec2d(just_inside_x, just_inside_y)), 3, 2);

  const double just_outside_x = std::nextafter(-1.0, -2.0);

  EXPECT_FALSE(grid.worldToGrid(Vec2d(just_outside_x, 0.0)).has_value());
}

TEST(OccupancyGridTest, RoundTripsCellCenters) {
  const OccupancyGrid grid(Vec2d(-1.0, -1.0), 0.5, 4, 3);

  const Vec2d center = grid.gridToWorld(GridIndex{2, 1});

  EXPECT_DOUBLE_EQ(0.25, center.x());
  EXPECT_DOUBLE_EQ(-0.25, center.y());

  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const Vec2d point = grid.gridToWorld(GridIndex{x, y});
      expectIndex(grid.worldToGrid(point), x, y);
    }
  }
}

TEST(OccupancyGridTest, UpdatesOccupancyWithoutChangingOtherCells) {
  OccupancyGrid grid(Vec2d(-1.0, -1.0), 0.5, 4, 3);

  grid.setOccupied(GridIndex{2, 1}, true);
  grid.setOccupied(GridIndex{3, 2}, true);

  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const bool expected = (x == 2 && y == 1) || (x == 3 && y == 2);

      EXPECT_EQ(expected, grid.isOccupied(GridIndex{x, y}));
    }
  }

  grid.setOccupied(GridIndex{2, 1}, false);

  EXPECT_FALSE(grid.isOccupied(GridIndex{2, 1}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{3, 2}));
}

TEST(OccupancyGridTest, RejectsInvalidIndices) {
  OccupancyGrid grid(Vec2d(-1.0, -1.0), 0.5, 4, 3);

  const GridIndex invalid_indices[] = {
      {-1, 0},
      {0, -1},
      {4, 0},
      {0, 3},
  };

  for (const auto &index : invalid_indices) {
    EXPECT_FALSE(grid.contains(index));
    EXPECT_THROW(grid.gridToWorld(index), std::out_of_range);
    EXPECT_THROW(grid.isOccupied(index), std::out_of_range);
    EXPECT_THROW(grid.setOccupied(index, true), std::out_of_range);
  }

  EXPECT_TRUE(grid.contains(GridIndex{0, 0}));
  EXPECT_TRUE(grid.contains(GridIndex{3, 2}));

  // 特别防止 (4,0) 被错误地当成下一行的 (0,1)
  EXPECT_FALSE(grid.isOccupied(GridIndex{0, 1}));
}

TEST(OccupancyGridTest, RejectsInvalidResolutions) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (double resolution : {0.0, -0.5, infinity, -infinity, nan}) {
    EXPECT_THROW((OccupancyGrid{Vec2d(0.0, 0.0), resolution, 4, 3}),
                 std::invalid_argument);
  }
}

TEST(OccupancyGridTest, RejectsNonPositiveDimensions) {
  // 使用 GridIndex 临时存放待测试的宽、高组合
  const GridIndex dimensions[] = {
      {0, 3},
      {-1, 3},
      {4, 0},
      {4, -1},
  };

  for (const auto &size : dimensions) {
    EXPECT_THROW((OccupancyGrid{Vec2d(0.0, 0.0), 0.5, size.x, size.y}),
                 std::invalid_argument);
  }
}

TEST(OccupancyGridTest, RejectsUnrepresentableMapBounds) {
  const double maximum = std::numeric_limits<double>::max();

  // 单个参数有限，但地图宽度或高度的计算溢出
  EXPECT_THROW((OccupancyGrid{Vec2d(0.0, 0.0), maximum, 2, 1}),
               std::invalid_argument);

  EXPECT_THROW((OccupancyGrid{Vec2d(0.0, 0.0), maximum, 1, 2}),
               std::invalid_argument);

  // 地图尺寸有限，但加上原点后上边界溢出
  EXPECT_THROW((OccupancyGrid{Vec2d(maximum, 0.0), maximum, 1, 1}),
               std::invalid_argument);

  // 原点过大，double 无法表示增加这点尺寸后的边界
  EXPECT_THROW((OccupancyGrid{Vec2d(1e20, 0.0), 0.5, 4, 3}),
               std::invalid_argument);
}

TEST(OccupancyGridTest, KeepsCellsInAdjacentRowsIndependent) {
  OccupancyGrid grid(Vec2d(0.0, 0.0), 0.5, 4, 3);

  grid.setOccupied(GridIndex{3, 0}, true);

  EXPECT_TRUE(grid.isOccupied(GridIndex{3, 0}));
  EXPECT_FALSE(grid.isOccupied(GridIndex{0, 1}));

  grid.setOccupied(GridIndex{0, 1}, true);
  grid.setOccupied(GridIndex{3, 0}, false);

  EXPECT_FALSE(grid.isOccupied(GridIndex{3, 0}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{0, 1}));
}

} // namespace
} // namespace parking_planner::core::map
