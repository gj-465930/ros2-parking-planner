#pragma once

#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/vec2d.h"

#include <vector>

namespace parking_planner::core::geometry {

/**
 * @brief 使用有序顶点表示的二维严格凸多边形。
 *
 * 顶点使用世界坐标，单位米，内部统一按逆时针顺序存储。
 * 不提供默认构造，不接受退化或带冗余共线顶点的输入。
 */
class ConvexPolygon2d {
public:
  /**
   * @brief 使用沿边界排列的顶点构造凸多边形。
   *
   * 任意两个顶点的距离必须大于 1e-9 米；
   * 统一为逆时针后，每条边的其他顶点到该边有向直线的
   * 左侧距离必须大于 1e-9 米。
   *
   * @param vertices 至少三个世界坐标顶点，不重复提供首点；
   *                 接受顺时针或逆时针排列。
   * @throws std::invalid_argument 输入重复、退化、近共线、
   *         非凸、自交，或无法满足上述数值约定。
   */
  explicit ConvexPolygon2d(const std::vector<Vec2d> &vertices);

  /**
   * @brief 只读访问按逆时针顺序排列的顶点。
   * @return 内部顶点容器的引用，生命周期依附于当前对象。
   */
  const std::vector<Vec2d> &vertices() const { return vertices_; };

  /**
   * @brief 使用分离轴定理判断与有向矩形是否重叠。
   *
   * 完全包含、边接触和角接触均视为重叠；
   * 各候选单位轴上不超过 1e-9 米的间隙按接触处理。
   *
   * @param rectangle 与当前多边形处于同一世界坐标系的矩形。
   * @return 未找到分离轴时返回 true。
   */
  bool hasOverlap(const OrientedRectangle2d &rectangle) const;

private:
  std::vector<Vec2d> vertices_; ///< 逆时针排列的世界坐标顶点，单位米
};

} // namespace parking_planner::core::geometry