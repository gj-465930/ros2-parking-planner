#pragma once

#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"

namespace parking_planner::core::vehicle {

/**
 * @brief 保存车辆尺寸、转向限制及派生几何量。
 *
 * 长度单位为米，角度单位为弧度。
 * 车辆局部坐标以后轴中心为原点，x 朝前、y 朝左。
 * 构造后仅提供查询接口，保持基础参数与缓存量一致。
 */
class VehicleParams {
public:
  /**
   * @brief 根据基础参数构造车辆几何。
   *
   * @param wheelbase 轴距，有限且严格为正，单位米
   * @param width 车宽，有限且严格为正，单位米
   * @param front_overhang 前轴到车头的距离，有限且非负，单位米
   * @param rear_overhang 后轴到车尾的距离，有限且非负，单位米
   * @param max_steering_angle 左右对称的最大等效前轮转角幅值，
   *        严格位于 (0, pi/2)，单位弧度，不是方向盘角
   * @throws std::invalid_argument 输入不满足约束，
   *         或派生量非有限、总长或最小转弯半径不为正
   */
  VehicleParams(double wheelbase, double width, double front_overhang,
                double rear_overhang, double max_steering_angle);

  /** @brief 获取轴距，单位米。 */
  double wheelbase() const { return wheelbase_; };

  /** @brief 获取车宽，单位米。 */
  double width() const { return width_; };

  /** @brief 获取前悬，单位米。 */
  double frontOverhang() const { return front_overhang_; };

  /** @brief 获取后悬，单位米。 */
  double rearOverhang() const { return rear_overhang_; };

  /** @brief 获取最大等效前轮转角幅值，单位弧度。 */
  double maxSteeringAngle() const { return max_steering_angle_; };

  /** @brief 获取车身总长，单位米。 */
  double length() const { return length_; };

  /**
   * @brief 获取后轴到车框中心沿局部 x 轴的有符号偏移，单位米。
   *
   * 正值表示中心位于后轴前方，允许为零或负。
   */
  double centerOffset() const { return center_offset_; };
  
  /**
   * @brief 获取低速无侧滑模型下后轴中心的最小转弯半径，单位米。
   *
   * 返回正值，不表示车身外侧的转弯半径。
   */
  double minTurningRadius() const { return min_turning_radius_; };

  /**
   * @brief 根据后轴中心的世界位姿生成车辆矩形。
   *
   * @param rear_axle_pose 后轴中心世界位姿；位置单位米，
   *        朝向允许任意有限弧度，逆时针为正
   * @return 世界坐标系中的车身矩形，保留输入航向，不添加安全膨胀
   * @throws std::invalid_argument 位姿或计算得到的中心包含非有限值
   */
  geometry::OrientedRectangle2d
  footprint(const geometry::Pose2d &rear_axle_pose) const;

private:
  double wheelbase_;          ///< 轴距，米
  double width_;              ///< 车宽，米
  double front_overhang_;     ///< 前悬，米
  double rear_overhang_;      ///< 后悬，米
  double max_steering_angle_; ///< 最大等效前轮转幅值，弧度

  double length_ = 0.0;             ///< 缓存的车身总长，米
  double center_offset_ = 0.0;      ///< 缓存的中心纵向偏移，米
  double min_turning_radius_ = 0.0; ///< 缓存的后轴最小转弯半径，米
};

} // namespace parking_planner::core::vehicle