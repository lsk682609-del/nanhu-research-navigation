// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "nav2_a_star_rpp/costmap_path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nav2_a_star_rpp
{

CostmapPathSmoother::CostmapPathSmoother(const CostmapPathSmootherParameters & parameters)
: parameters_(parameters)
{
}

void CostmapPathSmoother::setParameters(const CostmapPathSmootherParameters & parameters)
{
  parameters_ = parameters;
}

void CostmapPathSmoother::setPathOrientations(nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return;
  }

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const size_t before = i == 0 ? 0 : i - 1;
    const size_t after = i + 1 < path.poses.size() ? i + 1 : i;
    const auto & a = path.poses[before].pose.position;
    const auto & b = path.poses[after].pose.position;
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    path.poses[i].pose.orientation.x = 0.0;
    path.poses[i].pose.orientation.y = 0.0;
    path.poses[i].pose.orientation.z = std::sin(0.5 * yaw);
    path.poses[i].pose.orientation.w = std::cos(0.5 * yaw);
  }
}

bool CostmapPathSmoother::validate(
  const nav_msgs::msg::Path & path, const PoseIsFree & pose_is_free,
  SmoothingDiagnostics * diagnostics) const
{
  if (!pose_is_free || path.poses.size() < 2 || parameters_.collision_check_step <= 0.0) {
    return false;
  }

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    const int samples = std::max(
      1, static_cast<int>(std::ceil(distance / parameters_.collision_check_step)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      if (!pose_is_free(
          a.x + ratio * (b.x - a.x),
          a.y + ratio * (b.y - a.y), yaw))
      {
        if (diagnostics) {
          diagnostics->has_failed_pose = true;
          diagnostics->failed_segment = i - 1;
          diagnostics->x = a.x + ratio * (b.x - a.x);
          diagnostics->y = a.y + ratio * (b.y - a.y);
          diagnostics->yaw = yaw;
        }
        return false;
      }
    }
  }
  return true;
}

bool CostmapPathSmoother::smooth(
  const nav_msgs::msg::Path & input,
  nav_msgs::msg::Path & output,
  const PoseIsFree & pose_is_free, SmoothingDiagnostics * diagnostics) const
{
  if (diagnostics) {
    *diagnostics = SmoothingDiagnostics{};
    diagnostics->reason = "路径点不足或平滑参数无效";
  }
  output = input;
  if (input.poses.size() < 3 || !pose_is_free || parameters_.max_iterations <= 0 ||
    parameters_.tolerance < 0.0 || parameters_.data_weight < 0.0 ||
    parameters_.smooth_weight < 0.0 || parameters_.max_deviation <= 0.0 ||
    parameters_.collision_check_step <= 0.0)
  {
    return false;
  }

  nav_msgs::msg::Path smoothed = input;
  for (int iteration = 0; iteration < parameters_.max_iterations; ++iteration) {
    if (diagnostics) {diagnostics->iterations = iteration + 1;}
    nav_msgs::msg::Path next = smoothed;
    double total_change = 0.0;

    for (size_t i = 1; i + 1 < smoothed.poses.size(); ++i) {
      const auto & original = input.poses[i].pose.position;
      const auto & current = smoothed.poses[i].pose.position;
      const auto & previous = smoothed.poses[i - 1].pose.position;
      const auto & following = smoothed.poses[i + 1].pose.position;

      double candidate_x = current.x + parameters_.data_weight * (original.x - current.x) +
        parameters_.smooth_weight * (previous.x + following.x - 2.0 * current.x);
      double candidate_y = current.y + parameters_.data_weight * (original.y - current.y) +
        parameters_.smooth_weight * (previous.y + following.y - 2.0 * current.y);

      const double deviation = std::hypot(candidate_x - original.x, candidate_y - original.y);
      if (deviation > parameters_.max_deviation) {
        const double scale = parameters_.max_deviation / deviation;
        candidate_x = original.x + scale * (candidate_x - original.x);
        candidate_y = original.y + scale * (candidate_y - original.y);
      }

      const double yaw = std::atan2(following.y - previous.y, following.x - previous.x);
      if (!pose_is_free(candidate_x, candidate_y, yaw)) {
        if (diagnostics) {++diagnostics->rejected_points;}
        continue;
      }

      next.poses[i].pose.position.x = candidate_x;
      next.poses[i].pose.position.y = candidate_y;
      total_change += std::hypot(candidate_x - current.x, candidate_y - current.y);
    }

    smoothed = std::move(next);
    if (total_change < parameters_.tolerance) {
      break;
    }
  }

  setPathOrientations(smoothed);
  if (validate(smoothed, pose_is_free, diagnostics)) {
    if (diagnostics) {diagnostics->reason = "平滑通过";}
    output = std::move(smoothed);
    return true;
  }

  // 圆滑拐角可能切出狭窄的无障碍通道，因此逐步向原始 Theta* 折线回退，
  // 并采用第一个通过完整碰撞复检的混合结果。
  double blend = 0.5;
  for (int attempt = 0; attempt < parameters_.backtracking_steps; ++attempt, blend *= 0.5) {
    if (diagnostics) {diagnostics->backtracking_attempts = attempt + 1;}
    nav_msgs::msg::Path candidate = input;
    for (size_t i = 1; i + 1 < candidate.poses.size(); ++i) {
      candidate.poses[i].pose.position.x += blend *
        (smoothed.poses[i].pose.position.x - input.poses[i].pose.position.x);
      candidate.poses[i].pose.position.y += blend *
        (smoothed.poses[i].pose.position.y - input.poses[i].pose.position.y);
    }
    setPathOrientations(candidate);
    if (validate(candidate, pose_is_free, diagnostics)) {
      if (diagnostics) {diagnostics->reason = "缩小平滑量后通过";}
      output = std::move(candidate);
      return true;
    }
  }

  output = input;
  if (diagnostics) {diagnostics->reason = "平滑及混合回退路径碰撞复检均未通过";}
  return false;
}

}  // namespace nav2_a_star_rpp
