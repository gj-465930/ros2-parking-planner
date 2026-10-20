#pragma once

namespace parking_planner::core::geometry {

/**
 * @brief 二维位姿，描述位置与朝向。
 *
 */
struct Pose2d {
  double x = 0.0;     ///< x 坐标，单位米
  double y = 0.0;     ///< y 坐标，单位米
  double theta = 0.0; ///< 相对参考坐标系+x轴方向，单位弧度，逆时针为正
};

} // namespace parking_planner::core::geometry