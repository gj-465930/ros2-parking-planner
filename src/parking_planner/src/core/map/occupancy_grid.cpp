#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>

#include "parking_planner/core/geometry/vec2d.h"
#include "parking_planner/core/map/occupancy_grid.h"

namespace parking_planner::core::map {

OccupancyGrid::OccupancyGrid(const geometry::Vec2d &origin, double resolution,
                             int width, int height)
    : origin_(origin), resolution_(resolution), width_(width), height_(height) {
  if (!std::isfinite(resolution_) || resolution_ <= 0.0) {
    throw std::invalid_argument("grid resolution must be finite and positive ");
  }

  if (width_ <= 0 || height_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive and finite");
  }

  const std::size_t columns = static_cast<std::size_t>(width_);
  const std::size_t row = static_cast<std::size_t>(height_);

  if (columns * row > cells_.max_size()) {
    throw std::length_error("grid cell count exceeds container limit");
  }

  const double world_width = static_cast<int>(width_) * resolution;
  const double world_height = static_cast<int>(height_) * resolution;

  max_x_ = origin_.x() + world_width;
  max_y_ = origin_.y() + world_height;

  if (!std::isfinite(world_width) || !std::isfinite(world_height) ||
      !std::isfinite(max_x_) || !std::isfinite(max_y_) ||
      max_x_ <= origin.x() || max_y_ <= origin_.y()) {
    throw std::invalid_argument("grid world bounds cannot be represented");
  }

  cells_.assign(columns * row, 0);
}

bool OccupancyGrid::contains(const GridIndex &index) const {
  return index.x >= 0 && index.x < width_ && index.y >= 0 && index.y < height_;
}

std::optional<GridIndex>
OccupancyGrid::worldToGrid(const geometry::Vec2d &point) const {
  if (point.x() < origin_.x() || point.x() >= max_x_ ||
      point.y() < origin_.y() || point.y() >= max_y_) {
    return std::nullopt;
  }

  const double column = std::floor((point.x() - origin_.x()) / resolution_);
  const double row = std::floor((point.y() - origin_.y()) / resolution_);

  const int x =
      static_cast<int>(std::min(column, static_cast<double>(width_ - 1)));
  const int y =
      static_cast<int>(std::min(row, static_cast<double>(height_ - 1)));

  return GridIndex{x, y};
}

geometry::Vec2d OccupancyGrid::gridToWorld(const GridIndex &index) const {
  if (!contains(index)) {
    throw std::out_of_range("grid index is outside map");
  }

  const double x =
      (static_cast<double>(index.x) + 0.5) * resolution_ + origin_.x();

  const double y =
      (static_cast<double>(index.y) + 0.5) * resolution_ + origin_.y();

  return geometry::Vec2d(x, y);
}

std::size_t OccupancyGrid::linearIndex(const GridIndex &index) const {
  if (!contains(index)) {
    throw std::out_of_range("grid index is outside map");
  }

  return static_cast<std::size_t>(index.y) * static_cast<std::size_t>(width_) +
         static_cast<std::size_t>(index.x);
}

bool OccupancyGrid::isOccupied(const GridIndex &index) const {
  if (!contains(index)) {
    throw std::out_of_range("grid index is outside map");
  }

  return cells_[linearIndex(index)] == 1;
}

void OccupancyGrid::setOccupied(const GridIndex &index, bool occupied) {
  if (!contains(index)) {
    throw std::out_of_range("grid index is outside map");
  }

  cells_[linearIndex(index)] = occupied ? 1 : 0;
}

} // namespace parking_planner::core::map
