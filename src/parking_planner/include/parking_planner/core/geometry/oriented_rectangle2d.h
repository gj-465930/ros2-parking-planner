#pragma once

#include "parking_planner/core/geometry/vec2d.h"

#include <array>

namespace parking_planner::core::geometry {

/**
 * @brief 用几何中心、朝向和长宽表示的二维有向矩形
 *
 * 坐标和尺寸单位为米，朝向单位为弧度
 * 矩形局部 x 轴朝前、沿长度方向，局部 y 轴朝左、沿宽度方向
 * 长和宽严格为正，不表示退化的点或线段
 */
class OrientedRectangle2d {
public:
  /**
   * @brief 构造一个二维有向矩形。
   *
   * @param center 矩形几何中心的世界坐标，单位米
   * @param heading 长度方向相对世界 +x 轴的朝向，
   *                逆时针为正，允许任意有限弧度，不强制归一化
   * @param length 矩形长度，必须为正且有限，单位米
   * @param width 矩形宽度，必须为正且有限，单位米
   * @throws std::invalid_argument 朝向或尺寸不是有限数，
   *         或长度、宽度不大于零
   */
  OrientedRectangle2d(const Vec2d &center, double heading, double length,
                      double width);

  /**
   * @brief 获取矩形几何中心的世界坐标，单位米
   */
  Vec2d center() const { return center_; };

  /**
   * @brief 获取长度方向的朝向，单位弧度，逆时针为正
   */
  double heading() const { return heading_; };

  /**
   * @brief 获取矩形长度，单位米
   */
  double length() const { return length_; };

  /**
   * @brief 获取矩形宽度，单位米
   */
  double width() const { return width_; };

  /**
   * @brief 获取四个角点的世界坐标
   *
   * 前、左方向由矩形局部坐标系定义
   * @return 按左前、左后、右后、右前逆时针排列的四个角点，
   *         坐标单位为米，不重复返回第一个角点
   */
  std::array<Vec2d, 4> corners() const;

  /**
   * @brief 判断点是否位于矩形内部或边界上
   *
   * 使用长度容差处理浮点误差
   * @param point 待查询点的世界坐标，单位米
   * @return 点在内部、边界上或容差允许范围内时返回 true
   */
  bool isPointIn(const Vec2d &point) const;

  /**
   * @brief 使用分离轴定理判断两个矩形是否相交
   *
   * 边接触、角接触和完全包含均视为相交
   * 使用长度容差处理浮点误差
   * @param other 与当前矩形处于同一世界坐标系的另一个矩形
   * @return 所有候选轴上的投影均未分离时返回 true
   */
  bool hasOverlap(const OrientedRectangle2d &other) const;

private:
  Vec2d center_;   ///< 几何中心的世界坐标，单位米
  double heading_; ///< 长度方向的朝向，单位弧度，逆时针为正
  double length_;  ///< 矩形长度，严格为正，单位米
  double width_;   ///< 矩形宽度，严格为正，单位米
};

} // namespace parking_planner::core::geometry