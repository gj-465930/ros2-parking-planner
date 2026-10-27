#pragma once

#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

namespace parking_planner::core::vehicle {

/**
 * @brief 在低速、无侧滑假设下，计算固定前轮转角后的后轴位姿
 *
 * 使用稳定的圆弧表达式，支持直行、左右转和倒车
 * 不修改输入，不检查运动过程中的碰撞
 *
 * @param pose 当前后轴中心的世界位姿，分量必须有限；
 *        位置单位米，航向单位弧度，逆时针为正
 * @param params 已通过构造校验的车辆参数
 * @param steering_angle 等效前轮转角，单位弧度，左转为正；
 *        必须有限且绝对值不超过车辆转角上限
 * @param distance 后轴中心的有符号行驶距离，单位米；
 *        前进为正，倒车为负，必须有限
 * @return 新的后轴世界位姿，航向归一化至 [-pi, pi)；
 *         零距离时也会归一化航向
 * @throws std::invalid_argument 输入非法或计算产生非有限值
 */
geometry::Pose2d propagateExact(const geometry::Pose2d &pose,
                                const VehicleParams &params,
                                double steering_angle, double distance);

/**
 * @brief 用一步显式欧拉近似计算后轴位姿
 *
 * 位置增量使用这一步开始时的航向
 * 多步近似由调用方拆分距离并重复调用
 * 参数、单位、有效范围、输出航向及异常约定与 propagateExact 相同
 *
 * @param pose 当前后轴中心的世界位姿
 * @param params 车辆参数
 * @param steering_angle 这一步固定的等效前轮转角，单位弧度
 * @param distance 这一步的有符号行驶距离，单位米
 * @return 新的后轴世界位姿，航向位于 [-pi, pi)
 * @throws std::invalid_argument 输入非法或计算产生非有限值
 */
geometry::Pose2d propagateEuler(const geometry::Pose2d &pose,
                                const VehicleParams &params,
                                double steering_angle, double distance);

} // namespace parking_planner::core::vehicle