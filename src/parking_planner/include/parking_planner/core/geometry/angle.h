#pragma once

namespace parking_planner::core::geometry {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2 * kPi;

// 将有限角度归一化到 [-π, π)。
double normalizeAngle(double angel);

double angleDiff(double from, double to);

// 沿最短角差插值
double interpolateAngle(double from, double to, double t);

} // namespace parking_planner::core::geometry