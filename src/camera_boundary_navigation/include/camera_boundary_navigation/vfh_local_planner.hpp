#ifndef CAMERA_BOUNDARY_NAVIGATION__VFH_LOCAL_PLANNER_HPP_
#define CAMERA_BOUNDARY_NAVIGATION__VFH_LOCAL_PLANNER_HPP_

#include <string>
#include <vector>
#include "camera_boundary_navigation/boundary_controller.hpp"
#include "camera_boundary_navigation/safety_gate.hpp"

namespace camera_boundary_navigation
{
struct VfhConfig
{
  bool enabled = true;
  int angular_bins = 61;
  int return_confirmation_frames = 8;
  double half_fov = 0.75;  // Radians; ground support also limits actual visibility.
  double horizon = 2.0;
  double minimum_path_length = 1.0;
  double inflation_radius = 0.51;
  double target_weight = 4.0;
  double smooth_weight = 1.0;
  double obstacle_weight = 0.8;
  double reference_weight = 1.0;
  double side_switch_penalty = 2.0;
  double return_curvature_step = 0.05;
  double braking_deceleration = 0.5;
  double reaction_time = 0.35;
  double stop_margin = 0.25;
  double slowdown_clearance = 0.5;
  double preferred_clearance = 0.20;
  double clearance_weight = 12.0;
  double corridor_weight = 4.0;
};

struct VfhCandidate
{
  ReferencePath path;
  bool safe = false;
  double cost = 0.0;
};

struct VfhDecision
{
  ReferencePath path;
  std::string mode = "FOLLOW";
  std::string reason = "nominal_path_safe";
  double curvature = 0.0;
  double speed_limit = 0.0;
  double free_length = 0.0;
  double clearance = 0.0;
  std::vector<double> histogram;
  std::vector<VfhCandidate> candidates;
};

class VfhLocalPlanner
{
public:
  VfhLocalPlanner(VfhConfig config, BoundaryControllerConfig controller_config);
  VfhDecision plan(const ReferencePath & nominal, const ReferencePath & boundary,
    double visible_horizon, const SafetyGate & safety,
    const BoundaryCorridor & corridor = BoundaryCorridor{});
  bool commandIsSafe(const ControlCommand & command, const SafetyGate & safety,
    const BoundaryCorridor & corridor = BoundaryCorridor{}) const;
  double pathClearance(const ReferencePath & path, const SafetyGate & safety) const;
  void reset();

private:
  double speedLimit(const ReferencePath & path, const SafetyGate & safety) const;
  VfhConfig config_;
  BoundaryControllerConfig controller_config_;
  BoundaryController geometry_;
  bool avoiding_ = false;
  int clear_frames_ = 0;
  int previous_side_ = 0;
  double previous_curvature_ = 0.0;
};
}  // namespace camera_boundary_navigation
#endif
