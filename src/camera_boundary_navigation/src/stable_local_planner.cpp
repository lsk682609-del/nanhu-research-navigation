#include "camera_boundary_navigation/stable_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace camera_boundary_navigation
{

StableLocalPlanner::StableLocalPlanner(StablePlannerConfig config, ReferenceSide side)
: config_(std::move(config)), side_(side)
{
}

int StableLocalPlanner::requiredTurnDirection(
  bool straight_safe, double boundary_distance, double boundary_curvature) const
{
  const double lower =
    config_.preferred_boundary_distance - config_.boundary_distance_deadband;
  const double upper =
    config_.preferred_boundary_distance + config_.boundary_distance_deadband;

  if (std::isfinite(boundary_distance) && boundary_distance < lower) {
    // 距参考边界太近时向道路内部转弯：左边界向右，右边界向左。
    return side_ == ReferenceSide::LEFT ? -1 : 1;
  }
  if (std::isfinite(boundary_distance) && boundary_distance > upper) {
    // 距参考边界太远时缓慢回到参考区域：左边界向左，右边界向右。
    return side_ == ReferenceSide::LEFT ? 1 : -1;
  }
  if (!straight_safe && std::isfinite(boundary_curvature) &&
    std::abs(boundary_curvature) >= config_.minimum_turn_curvature)
  {
    // 边界距离仍在死区内但直线已经不安全时，使用边界路径提供的转弯方向。
    return boundary_curvature > 0.0 ? 1 : -1;
  }
  return 0;
}

double StableLocalPlanner::regularizeCurvature(double desired_curvature)
{
  const double weight_sum =
    config_.curvature_tracking_weight + config_.curvature_change_weight;
  double selected =
    (config_.curvature_tracking_weight * desired_curvature +
    config_.curvature_change_weight * previous_curvature_) / weight_sum;
  selected = std::clamp(
    selected,
    previous_curvature_ - config_.maximum_curvature_change_per_frame,
    previous_curvature_ + config_.maximum_curvature_change_per_frame);
  previous_curvature_ = selected;
  return selected;
}

StablePlannerDecision StableLocalPlanner::update(
  bool straight_safe, double boundary_distance, double boundary_curvature)
{
  StablePlannerDecision decision;
  const int required_direction = requiredTurnDirection(
    straight_safe, boundary_distance, boundary_curvature);

  if (!turning_) {
    if (required_direction == 0) {
      pending_turn_direction_ = 0;
      turn_confirmation_count_ = 0;
      previous_curvature_ = 0.0;
      return decision;
    }

    if (required_direction != pending_turn_direction_) {
      pending_turn_direction_ = required_direction;
      turn_confirmation_count_ = 1;
    } else {
      ++turn_confirmation_count_;
    }
    decision.confirmation_count = turn_confirmation_count_;
    if (turn_confirmation_count_ < config_.turn_confirmation_frames) {
      // 直线不安全时仍返回直线路径，由安全门保持停车，直到转弯证据足够稳定。
      return decision;
    }
    turning_ = true;
    straight_confirmation_count_ = 0;
  }

  if (turning_ && required_direction == 0 && straight_safe) {
    ++straight_confirmation_count_;
    if (straight_confirmation_count_ >= config_.straight_confirmation_frames) {
      reset();
      return decision;
    }
  } else {
    straight_confirmation_count_ = 0;
  }

  int turn_direction = required_direction;
  if (turn_direction == 0) {
    turn_direction = previous_curvature_ >= 0.0 ? 1 : -1;
  }
  double desired_curvature = boundary_curvature;
  if (!std::isfinite(desired_curvature) || desired_curvature * turn_direction <= 0.0) {
    desired_curvature = 0.0;
  }
  desired_curvature = static_cast<double>(turn_direction) * std::max(
    std::abs(desired_curvature), config_.minimum_turn_curvature);

  decision.select_straight = false;
  decision.turning = true;
  decision.confirmation_count = turn_confirmation_count_;
  decision.selected_curvature = regularizeCurvature(desired_curvature);
  return decision;
}

void StableLocalPlanner::reset()
{
  turning_ = false;
  pending_turn_direction_ = 0;
  turn_confirmation_count_ = 0;
  straight_confirmation_count_ = 0;
  previous_curvature_ = 0.0;
}

}  // camera_boundary_navigation命名空间
