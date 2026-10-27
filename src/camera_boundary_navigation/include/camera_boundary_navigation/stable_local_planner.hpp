#ifndef CAMERA_BOUNDARY_NAVIGATION__STABLE_LOCAL_PLANNER_HPP_
#define CAMERA_BOUNDARY_NAVIGATION__STABLE_LOCAL_PLANNER_HPP_

#include "camera_boundary_navigation/types.hpp"

namespace camera_boundary_navigation
{

struct StablePlannerConfig
{
  double preferred_boundary_distance = 0.65;
  double boundary_distance_deadband = 0.20;
  int turn_confirmation_frames = 5;
  int straight_confirmation_frames = 8;
  double minimum_turn_curvature = 0.05;
  double curvature_tracking_weight = 4.0;
  double curvature_change_weight = 12.0;
  double maximum_curvature_change_per_frame = 0.05;
};

struct StablePlannerDecision
{
  bool select_straight = true;
  bool turning = false;
  double selected_curvature = 0.0;
  int confirmation_count = 0;
};

class StableLocalPlanner
{
public:
  StableLocalPlanner(StablePlannerConfig config, ReferenceSide side);

  StablePlannerDecision update(
    bool straight_safe, double boundary_distance, double boundary_curvature);
  void reset();

private:
  int requiredTurnDirection(
    bool straight_safe, double boundary_distance, double boundary_curvature) const;
  double regularizeCurvature(double desired_curvature);

  StablePlannerConfig config_;
  ReferenceSide side_ = ReferenceSide::LEFT;
  bool turning_ = false;
  int pending_turn_direction_ = 0;
  int turn_confirmation_count_ = 0;
  int straight_confirmation_count_ = 0;
  double previous_curvature_ = 0.0;
};

}  // camera_boundary_navigation命名空间

#endif  // 头文件保护宏结束
