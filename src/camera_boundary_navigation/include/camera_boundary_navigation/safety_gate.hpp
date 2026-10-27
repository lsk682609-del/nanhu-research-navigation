#ifndef CAMERA_BOUNDARY_NAVIGATION__SAFETY_GATE_HPP_
#define CAMERA_BOUNDARY_NAVIGATION__SAFETY_GATE_HPP_

#include <cstdint>
#include <unordered_set>
#include <vector>

#include "camera_boundary_navigation/types.hpp"

namespace camera_boundary_navigation
{

struct SafetyConfig
{
  double grid_resolution = 0.15;
  double path_sample_step = 0.10;
  double path_clearance = 0.40;
  double ground_half_width = 0.0;
  bool directional_front_check = false;
  double minimum_ground_check_x = 0.60;
  double minimum_front_range = 1.0;
  double immediate_obstacle_distance = 0.70;
  double immediate_obstacle_half_width = 0.40;
  int minimum_obstacle_points_per_cell = 3;
  bool require_ground_grid = true;
  bool require_obstacle_cloud = true;
  int recovery_valid_frames = 5;
};

class SafetyGate
{
public:
  explicit SafetyGate(SafetyConfig config);

  void updateGroundCells(const std::vector<Point2d> & ground_cells);
  void updateObstacleCells(const std::vector<Point2d> & obstacle_points);
  void setFrontRange(double front_range);
  void setInputsFresh(bool boundary_fresh, bool grid_fresh, bool obstacle_fresh);
  void markNewFrame();
  SafetyState checkPathGeometry(const ReferencePath & path) const;
  SafetyState evaluate(
    bool enabled, bool paused, const BoundaryCurve & curve, const ReferencePath & path,
    bool allow_boundaryless = false);
  void resetRecovery();
  std::vector<Point2d> obstacleCenters() const;
  double freePathLength(const ReferencePath & path) const;

private:
  std::int64_t key(int gx, int gy) const;
  std::pair<int, int> cell(const Point2d & point) const;
  bool nearSet(
    const Point2d & point, double radius,
    const std::unordered_set<std::int64_t> & cells) const;
  bool pathIsOnGround(const ReferencePath & path) const;
  bool pathIsBlocked(const ReferencePath & path) const;

  SafetyConfig config_;
  std::unordered_set<std::int64_t> ground_cells_;
  std::unordered_set<std::int64_t> obstacle_cells_;
  double front_range_ = 0.0;
  bool boundary_fresh_ = false;
  bool grid_fresh_ = false;
  bool obstacle_fresh_ = false;
  bool immediate_obstacle_ = false;
  int consecutive_valid_frames_ = 0;
  std::uint64_t frame_sequence_ = 0U;
  std::uint64_t last_counted_frame_sequence_ = 0U;
};

}  // camera_boundary_navigation命名空间

#endif  // 头文件保护宏结束
