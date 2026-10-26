#include "parking_planner/core/geometry/convex_polygon2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace parking_planner::core::geometry {
namespace {

constexpr double kGeometryEpsilon = 1e-9;

bool isSeparatedOnAxis(const std::vector<Vec2d> &polygon_vertices,
                       const std::array<Vec2d, 4> &rectangle_corners,
                       const Vec2d &axis) {
  const Vec2d origin_point = polygon_vertices.front();

  double polygon_min = 0.0;
  double polygon_max = 0.0;

  for (const auto &vertex : polygon_vertices) {
    const double projection = (vertex - origin_point).dot(axis);
    polygon_min = std::min(polygon_min, projection);
    polygon_max = std::max(polygon_max, projection);
  }

  const double first_projection =
      (rectangle_corners.front() - origin_point).dot(axis);

  double rectangle_min = first_projection;
  double rectangle_max = first_projection;

  for (const Vec2d &corner : rectangle_corners) {
    const double projection = (corner - origin_point).dot(axis);
    rectangle_min = std::min(rectangle_min, projection);
    rectangle_max = std::max(rectangle_max, projection);
  }

  return rectangle_min - polygon_max > kGeometryEpsilon ||
         polygon_min - rectangle_max > kGeometryEpsilon;
}

} // namespace

ConvexPolygon2d::ConvexPolygon2d(const std::vector<Vec2d> &vertices)
    : vertices_(vertices) {
  std::size_t count = vertices.size();

  if (count < 3) {
    throw std::invalid_argument(
        "ConvexPolygon2d requires at least three vertices");
  }

  // 检查点是否重复
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      if ((vertices[i] - vertices[j]).length() <= kGeometryEpsilon) {
        throw std::invalid_argument(
            "ConvexPolygon2d contains duplicate vertices");
      }
    }
  }

  double twice_signed_area = 0.0;

  // 计算有向面积
  for (std::size_t i = 1; i + 1 < count; ++i) {
    const Vec2d first = vertices[i] - vertices[0];
    const Vec2d second = vertices[i + 1] - vertices[0];

    twice_signed_area += first.cross(second);
  }

  if (!std::isfinite(twice_signed_area) || twice_signed_area == 0.0) {
    throw std::invalid_argument("ConvexPolygon2d has invalid or zero area");
  }

  if (twice_signed_area < 0.0) {
    std::reverse(vertices_.begin(), vertices_.end());
  }

  // 检测是否是凸多边形并且按照逆时针排列
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t next = (i + 1) % count;
    const Vec2d edge = vertices_[next] - vertices_[i];
    const double edge_length = edge.length();

    // 每条有向边的其他顶点全部位于左侧
    for (std::size_t j = 0; j < count; ++j) {
      if (j == i || j == next) {
        continue;
      }

      const Vec2d offset = vertices_[j] - vertices_[i];

      const double signed_distance = edge.cross(offset) / edge_length;

      if (!std::isfinite(signed_distance) ||
          signed_distance <= kGeometryEpsilon) {
        throw std::invalid_argument(
            "Vertices must form an ordered strictly convex polygon");
      }
    }
  }
}

bool ConvexPolygon2d::hasOverlap(const OrientedRectangle2d &rectangle) const {
  const auto rectangle_corners = rectangle.corners();

  // 检查矩形两个法向量
  const double cos_theta = std::cos(rectangle.heading());
  const double sin_theta = std::sin(rectangle.heading());

  const Vec2d longitudinal_axis(cos_theta, sin_theta);
  const Vec2d lateral_axis(-sin_theta, cos_theta);

  if (isSeparatedOnAxis(vertices_, rectangle_corners, longitudinal_axis) ||
      isSeparatedOnAxis(vertices_, rectangle_corners, lateral_axis)) {
    return false;
  }

  // 检查多边形每条边的法向量
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    const std::size_t next = (i + 1) % vertices_.size();

    const Vec2d edge = vertices_[next] - vertices_[i];
    const double edge_length = edge.length();

    const Vec2d perpendicular_unit_vector(edge.y() / edge_length,
                                          -edge.x() / edge_length);

    if (isSeparatedOnAxis(vertices_, rectangle_corners,
                          perpendicular_unit_vector)) {
      return false;
    }
  }

  return true;
}

} // namespace parking_planner::core::geometry