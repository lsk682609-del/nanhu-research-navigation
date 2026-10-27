#include "camera_boundary_navigation/safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

namespace camera_boundary_navigation
{
namespace
{
// Constant-curvature paths are checked as sampled polylines.  Their accumulated
// chord length is microscopically shorter than the requested arc length.
constexpr double kPathLengthTolerance = 1e-3;
}

std::string toString(SafetyState state)
{
  switch (state) {
    case SafetyState::DISABLED: return "DISABLED";
    case SafetyState::PAUSED: return "PAUSED";
    case SafetyState::WAITING_FOR_INPUT: return "WAITING_FOR_INPUT";
    case SafetyState::BOUNDARY_INVALID: return "BOUNDARY_INVALID";
    case SafetyState::FRONT_TOO_CLOSE: return "FRONT_TOO_CLOSE";
    case SafetyState::PATH_NOT_ON_GROUND: return "PATH_NOT_ON_GROUND";
    case SafetyState::PATH_BLOCKED: return "PATH_BLOCKED";
    case SafetyState::TRACKING: return "TRACKING";
  }
  return "UNKNOWN";
}

SafetyGate::SafetyGate(SafetyConfig config)
: config_(std::move(config))
{
}

std::int64_t SafetyGate::key(int gx, int gy) const
{
  const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(gx));
  const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(gy));
  return static_cast<std::int64_t>((ux << 32U) | uy);
}

std::pair<int, int> SafetyGate::cell(const Point2d & point) const
{
  return {
    static_cast<int>(std::floor(point.x / config_.grid_resolution)),
    static_cast<int>(std::floor(point.y / config_.grid_resolution))};
}

void SafetyGate::updateGroundCells(const std::vector<Point2d> & ground_cells)
{
  ground_cells_.clear();
  ground_cells_.reserve(ground_cells.size());
  for (const auto & point : ground_cells) {
    const auto [gx, gy] = cell(point);
    ground_cells_.insert(key(gx, gy));
  }
}

void SafetyGate::updateObstacleCells(const std::vector<Point2d> & obstacle_points)
{
  obstacle_cells_.clear();
  obstacle_cells_.reserve(obstacle_points.size());
  immediate_obstacle_ = false;
  std::unordered_map<std::int64_t, int> cell_counts;
  cell_counts.reserve(obstacle_points.size());
  for (const auto & point : obstacle_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0) {
      continue;
    }
    const auto [gx, gy] = cell(point);
    ++cell_counts[key(gx, gy)];
  }
  for (const auto & point : obstacle_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0) {
      continue;
    }
    const auto [gx, gy] = cell(point);
    const auto point_key = key(gx, gy);
    if (cell_counts[point_key] < config_.minimum_obstacle_points_per_cell) {
      continue;
    }
    obstacle_cells_.insert(point_key);
    if (point.x <= config_.immediate_obstacle_distance &&
      std::abs(point.y) <= config_.immediate_obstacle_half_width)
    {
      immediate_obstacle_ = true;
    }
  }
}

void SafetyGate::setFrontRange(double front_range)
{
  front_range_ = front_range;
}

void SafetyGate::setInputsFresh(bool boundary_fresh, bool grid_fresh, bool obstacle_fresh)
{
  boundary_fresh_ = boundary_fresh;
  grid_fresh_ = grid_fresh;
  obstacle_fresh_ = obstacle_fresh;
}

void SafetyGate::markNewFrame()
{
  ++frame_sequence_;
}

bool SafetyGate::nearSet(
  const Point2d & point, double radius,
  const std::unordered_set<std::int64_t> & cells) const
{
  const auto [center_x, center_y] = cell(point);
  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(radius / config_.grid_resolution)));
  const double squared_radius = radius * radius;
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      if (cells.find(key(center_x + dx, center_y + dy)) == cells.end()) {
        continue;
      }
      const double cell_center_x =
        (static_cast<double>(center_x + dx) + 0.5) * config_.grid_resolution;
      const double cell_center_y =
        (static_cast<double>(center_y + dy) + 0.5) * config_.grid_resolution;
      const double squared_distance =
        (point.x - cell_center_x) * (point.x - cell_center_x) +
        (point.y - cell_center_y) * (point.y - cell_center_y);
      if (squared_distance <= squared_radius) {
        return true;
      }
    }
  }
  return false;
}

bool SafetyGate::pathIsOnGround(const ReferencePath & path) const
{
  if (!config_.require_ground_grid) {
    return true;
  }
  if (ground_cells_.empty()) {
    return false;
  }
  const double matching_radius = 1.6 * config_.grid_resolution;
  for (const auto & point : path.points) {
    if (point.x < config_.minimum_ground_check_x) {
      continue;
    }
    // Sample the footprint disk; unknown ground is not free space. The near-camera
    // blind zone remains explicit and never exempts obstacle collision checks.
    const int steps = std::max(1, static_cast<int>(std::ceil(
      config_.ground_half_width / (config_.grid_resolution * 0.5))));
    const double step = config_.ground_half_width / steps;
    const int extent = config_.ground_half_width > 0.0 ? steps : 0;
    for (int ix = -extent; ix <= extent; ++ix) {
      for (int iy = -extent; iy <= extent; ++iy) {
        const double dx = ix * step;
        const double dy = iy * step;
        if (std::hypot(dx, dy) > config_.ground_half_width + 1e-6) {continue;}
        const Point2d sample{point.x + dx, point.y + dy};
        if (sample.x < config_.minimum_ground_check_x) {continue;}
        const double tolerance = config_.ground_half_width > 0.0 ?
          0.72 * config_.grid_resolution : matching_radius;
        if (!nearSet(sample, tolerance, ground_cells_)) {return false;}
      }
    }
  }
  return true;
}

bool SafetyGate::pathIsBlocked(const ReferencePath & path) const
{
  if (immediate_obstacle_) {
    return true;
  }
  if (!config_.require_obstacle_cloud) {
    return false;
  }
  for (const auto & point : path.points) {
    if (point.x < 0.15) {
      continue;
    }
    if (nearSet(point, config_.path_clearance + 0.71 * config_.grid_resolution, obstacle_cells_)) {
      return true;
    }
  }
  return false;
}

SafetyState SafetyGate::checkPathGeometry(const ReferencePath & path) const
{
  if (!path.valid) {
    return SafetyState::BOUNDARY_INVALID;
  }
  if (!pathIsOnGround(path)) {
    return SafetyState::PATH_NOT_ON_GROUND;
  }
  if (pathIsBlocked(path)) {
    return SafetyState::PATH_BLOCKED;
  }
  return SafetyState::TRACKING;
}

SafetyState SafetyGate::evaluate(
  bool enabled, bool paused, const BoundaryCurve & curve, const ReferencePath & path,
  bool allow_boundaryless)
{
  if (!enabled) {
    resetRecovery();
    return SafetyState::DISABLED;
  }
  if (paused) {
    resetRecovery();
    return SafetyState::PAUSED;
  }
  if (!boundary_fresh_ || (config_.require_ground_grid && !grid_fresh_) ||
    (config_.require_obstacle_cloud && !obstacle_fresh_))
  {
    resetRecovery();
    return SafetyState::WAITING_FOR_INPUT;
  }
  if (!curve.valid && !allow_boundaryless) {
    resetRecovery();
    return SafetyState::BOUNDARY_INVALID;
  }
  if ((!config_.directional_front_check &&
    (!std::isfinite(front_range_) || front_range_ < config_.minimum_front_range)) ||
    (config_.directional_front_check &&
    freePathLength(path) + kPathLengthTolerance < config_.minimum_front_range))
  {
    resetRecovery();
    return SafetyState::FRONT_TOO_CLOSE;
  }
  const SafetyState path_state = checkPathGeometry(path);
  if (path_state != SafetyState::TRACKING) {
    resetRecovery();
    return path_state;
  }
  if (frame_sequence_ != last_counted_frame_sequence_) {
    ++consecutive_valid_frames_;
    last_counted_frame_sequence_ = frame_sequence_;
  }
  if (consecutive_valid_frames_ < config_.recovery_valid_frames) {
    return SafetyState::WAITING_FOR_INPUT;
  }
  return SafetyState::TRACKING;
}

std::vector<Point2d> SafetyGate::obstacleCenters() const
{
  std::vector<Point2d> result;
  result.reserve(obstacle_cells_.size());
  for (const auto cell_key : obstacle_cells_) {
    const auto bits = static_cast<std::uint64_t>(cell_key);
    const auto gx = static_cast<std::int32_t>(bits >> 32U);
    const auto gy = static_cast<std::int32_t>(bits & 0xffffffffU);
    result.push_back({(gx + 0.5) * config_.grid_resolution,
      (gy + 0.5) * config_.grid_resolution});
  }
  return result;
}

double SafetyGate::freePathLength(const ReferencePath & path) const
{
  if (!path.valid || immediate_obstacle_) {return 0.0;}
  double length = 0.0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    ReferencePath sample;
    sample.valid = true;
    sample.points = {path.points[i]};
    if (checkPathGeometry(sample) != SafetyState::TRACKING) {break;}
    length += std::hypot(path.points[i].x - path.points[i - 1].x,
      path.points[i].y - path.points[i - 1].y);
  }
  return length;
}

void SafetyGate::resetRecovery()
{
  consecutive_valid_frames_ = 0;
  last_counted_frame_sequence_ = frame_sequence_;
}

}  // camera_boundary_navigation命名空间
