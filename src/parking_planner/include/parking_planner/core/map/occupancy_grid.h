#pragma once

#include "parking_planner/core/geometry/vec2d.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace parking_planner::core::map {

/**
 * @brief 二维格子索引，x 向右、y 向上，从零开始。
 */
struct GridIndex {
  int x = 0; ///< 水平方向索引
  int y = 0; ///< 竖直方向索引
};

/**
 * @brief 轴对齐的二维占据栅格地图。
 *
 * 原点为地图左下角的世界坐标，每格边长为 resolution。
 * 地图范围左下边界包含，右上边界不包含。
 * 格子仅表示空闲或占用，构造时全部为空闲。
 */
class OccupancyGrid {
public:
  /**
   * @brief 创建指定位置、分辨率和尺寸的空闲地图。
   *
   * @param origin 地图左下角的世界坐标，单位米。
   * @param resolution 每个格子的边长，单位米，必须有限且大于零。
   * @param width 水平方向格子数量，必须大于零。
   * @param height 竖直方向格子数量，必须大于零。
   * @throws std::invalid_argument 参数非法，或地图范围无法表示。
   * @throws std::length_error 格子总数超过容器容量上限。
   * @throws std::bad_alloc 内存分配失败。
   */
  OccupancyGrid(const geometry::Vec2d &origin, double resolution, int width,
                int height);

  /** @brief 获取地图左下角的世界坐标，单位米。 */
  geometry::Vec2d origin() const { return origin_; };

  /** @brief 获取格子边长，单位米。 */
  double resolution() const { return resolution_; };

  /** @brief 获取水平方向格子数量。 */
  int width() const { return width_; };

  /** @brief 获取竖直方向格子数量。 */
  int height() const { return height_; };

  /**
   * @brief 获取地图世界 x 上边界，单位米。
   * 地图的点归属范围不包含此边界。
   */
  double maxX() const { return max_x_; }

  /**
   * @brief 获取地图世界 y 上边界，单位米。
   * 地图的点归属范围不包含此边界。
   */
  double maxY() const { return max_y_; }

  /**
   * @brief 判断整数索引是否位于地图内。
   * @param index 待检查的格子索引。
   * @return 两个方向均未越界时返回 true。
   */
  bool contains(const GridIndex &index) const;

  /**
   * @brief 查询世界坐标点所在的格子。
   *
   * 使用 floor 划分格子，不添加几何容差。
   *
   * @param point 世界坐标点，单位米。
   * @return 地图内返回格子索引，地图外返回 std::nullopt。
   */
  std::optional<GridIndex> worldToGrid(const geometry::Vec2d &point) const;

  /**
   * @brief 将格子索引转换为格子中心的世界坐标。
   * @param index 地图内的格子索引。
   * @return 格子中心的世界坐标，单位米。
   * @throws std::out_of_range 索引越界。
   */
  geometry::Vec2d gridToWorld(const GridIndex &index) const;

  /**
   * @brief 查询指定格子是否被占用。
   * @param index 地图内的格子索引。
   * @return 被占用返回 true，空闲返回 false。
   * @throws std::out_of_range 索引越界。
   */
  bool isOccupied(const GridIndex &index) const;

  /**
   * @brief 设置指定格子的占用状态。
   * @param index 地图内的格子索引。
   * @param occupied true 表示占用，false 表示空闲。
   * @throws std::out_of_range 索引越界。
   */
  void setOccupied(const GridIndex &index, bool occupied);

private:
  std::size_t linearIndex(const GridIndex &index) const;

  geometry::Vec2d origin_; ///< 地图左下角世界坐标，单位米

  double resolution_; ///< 格子边长，单位米

  int width_; ///< 水平方向格子数量

  int height_; ///< 竖直方向格子数量

  double max_x_ = 0.0; ///< 不包含的世界 x 上边界

  double max_y_ = 0.0; ///< 不包含的世界 y 上边界

  // 按 y * width + x 存储，0 为空闲，1 为占用
  std::vector<std::uint8_t> cells_;
};

} // namespace parking_planner::core::map
