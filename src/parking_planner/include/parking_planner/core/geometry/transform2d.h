#pragma once

#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"

namespace parking_planner::core::geometry {

/**
 * @brief 将点的局部坐标转换为全局坐标
 *
 * @param local_point 在局部坐标系中的坐标，单位米
 * @param reference_pose 局部坐标系在全局坐标系中的位姿：
 * @return Vec2d 点在全局坐标系中的坐标，单位米
 */
Vec2d transformToGlobal(const Vec2d &local_point, const Pose2d &reference_pose);

/**
 * @brief 将点的全局坐标转换为局部坐标
 *
 * @param global_point 点在全局坐标系中的坐标，单位米
 * @param reference_pose 局部坐标系在全局坐标系中的位姿
 * @return Vec2d 点在局部坐标系中的坐标，单位米
 */
Vec2d transformToLocal(const Vec2d &global_point, const Pose2d &reference_pose);

} // namespace parking_planner::core::geometry