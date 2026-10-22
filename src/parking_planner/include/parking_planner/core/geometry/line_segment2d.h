#pragma once

#include <optional>

#include "parking_planner/core/geometry/vec2d.h"

namespace parking_planner::core::geometry {

/**
 * @brief 二维有限线段
 *
 * 线段由起点和终点表示，坐标单位为米
 * 退化线段表示起点和终点重合
 */
class LineSegment2d {
public:
  /**
   * @brief 用起点和终点构造线段
   *
   * @param start 线段起点，单位米
   * @param end 线段终点，单位米
   */
  LineSegment2d(const Vec2d &start, const Vec2d &end);

  /**
   * @brief 获取线段起点
   *
   * @return 线段起点
   */
  Vec2d start() const { return start_; };

  /**
   * @brief 获取线段终点
   *
   * @return 线段终点
   */
  Vec2d end() const { return end_; };

  /**
   * @brief 获取从起点指向终点的位移向量
   *
   * 该向量未归一化 退化线段的方向向量为零向量
   *
   * @return 线段位移向量
   */
  Vec2d direction() const { return direction_; };

  /**
   * @brief 获取线段长度
   *
   * @return 线段长度，单位米
   */
  double length() const { return length_; };

  /**
   * @brief 计算点在线段所在直线上的原始投影参数
   *
   * 对非退化线段，投影点满足：
   *
   *   H = A + t * (B - A)
   *
   * t 小于 0 表示投影在起点外侧，t 大于 1 表示投影在终点外侧
   *
   * @param point 待投影点，单位米
   * @return 投影参数 t；线段退化时返回 std::nullopt
   */
  std::optional<double> projectionParameter(const Vec2d &point) const;

  /**
   * @brief 获取点到有限线段的最近点
   *
   * 如果无限直线上的投影落在线段外，则将投影截断到对应端点
   * 退化线段的最近点就是其唯一端点
   *
   * @param point 待查询点，单位米
   * @return 线段上的最近点，单位米
   */
  Vec2d nearestPoint(const Vec2d &point) const;

  /**
   * @brief 计算点到有限线段的欧氏距离
   *
   * @param point 待查询点，单位米
   * @return 距离，单位米
   */
  double distanceTo(const Vec2d &point) const;

  /**
   * @brief 判断点是否在线段上
   *
   * 判断包含端点和浮点容限范围内的点
   *
   * @param point 待判断点，单位米
   * @return 点在线段上时返回 true
   */
  bool isPointIn(const Vec2d &point) const;

  /**
   * @brief 判断当前线段是否与另一条线段相交
   *
   * 端点接触和共线重叠均视为相交
   * 退化线段按点处理
   *
   * @param other 另一条线段
   * @return 相交时返回 true
   */
  bool hasIntersect(const LineSegment2d &other) const;

private:
  Vec2d start_;
  Vec2d end_;
  Vec2d direction_;
  double length_;
};

} // namespace parking_planner::core::geometry