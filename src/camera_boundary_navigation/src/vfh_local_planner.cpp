#include "camera_boundary_navigation/vfh_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace camera_boundary_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kPathLengthTolerance = 1e-3;
int side(double curvature) {return curvature > 0.02 ? 1 : (curvature < -0.02 ? -1 : 0);}
double square(double value) {return value * value;}
}

VfhLocalPlanner::VfhLocalPlanner(VfhConfig config, BoundaryControllerConfig controller_config)
: config_(std::move(config)), controller_config_(std::move(controller_config)),
  geometry_(controller_config_)
{
  for (const double value : {config_.half_fov, config_.horizon, config_.minimum_path_length,
    config_.inflation_radius, config_.return_curvature_step, config_.braking_deceleration,
    config_.reaction_time, config_.stop_margin, config_.slowdown_clearance, config_.preferred_clearance,
    controller_config_.maximum_abs_curvature, controller_config_.maximum_angular_velocity,
    controller_config_.maximum_linear_velocity, controller_config_.lookahead_distance})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("VFH geometry and braking parameters must be finite and positive");
    }
  }
  for (const double value : {config_.target_weight, config_.smooth_weight,
    config_.obstacle_weight, config_.reference_weight, config_.side_switch_penalty,
    config_.clearance_weight, config_.corridor_weight})
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("VFH weights must be finite and nonnegative");
    }
  }
  if (config_.angular_bins < 3 || config_.angular_bins > 361 ||
    config_.return_confirmation_frames < 1 || config_.half_fov >= kPi / 2.0 ||
    config_.minimum_path_length < 0.3 || config_.horizon < config_.minimum_path_length)
  {
    throw std::invalid_argument("invalid VFH sampling or horizon parameters");
  }
}

void VfhLocalPlanner::reset()
{
  avoiding_ = false;
  clear_frames_ = 0;
  previous_side_ = 0;
  previous_curvature_ = 0.0;
}

double VfhLocalPlanner::pathClearance(const ReferencePath & path, const SafetyGate & safety) const
{
  // Clearance is measured over the swept path, not a narrow ray at the target angle.
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : safety.obstacleCenters()) {
    for (const auto & point : path.points) {
      clearance = std::min(clearance,
        std::hypot(point.x - obstacle.x, point.y - obstacle.y) - config_.inflation_radius);
    }
  }
  return clearance;
}

double VfhLocalPlanner::speedLimit(const ReferencePath & path, const SafetyGate & safety) const
{
  const double distance = std::max(0.0, safety.freePathLength(path) - config_.stop_margin);
  const double at = config_.braking_deceleration * config_.reaction_time;
  double limit = std::min(controller_config_.maximum_linear_velocity,
    std::sqrt(at * at + 2.0 * config_.braking_deceleration * distance) - at);
  const double clearance = pathClearance(path, safety);
  if (std::isfinite(clearance)) {
    limit = std::min(limit, controller_config_.maximum_linear_velocity *
      std::clamp(clearance / config_.slowdown_clearance, 0.0, 1.0));
  }
  const double curvature = std::abs(geometry_.targetCurvature(path));
  if (curvature > 1e-6) {
    limit = std::min(limit, controller_config_.maximum_angular_velocity / curvature);
  }
  return limit;
}

VfhDecision VfhLocalPlanner::plan(
  const ReferencePath & nominal, const ReferencePath & boundary,
  double visible_horizon, const SafetyGate & safety, const BoundaryCorridor & corridor)
{
  VfhDecision result;
  if (corridor.valid && !corridor.usable()) {
    result.mode = "BLOCKED";
    result.reason = "corridor_too_narrow";
    return result;
  }
  if (!boundary.valid || !std::isfinite(visible_horizon)) {
    reset();
    result.mode = "BLOCKED";
    result.reason = "boundary_invalid";
    return result;
  }
  const double horizon = std::min(config_.horizon, visible_horizon);
  if (horizon < config_.minimum_path_length) {
    result.mode = "BLOCKED";
    result.reason = "insufficient_visible_distance";
    return result;
  }
  const double nominal_clearance = pathClearance(nominal, safety);
  const bool nominal_geometry_safe = corridor.contains(nominal) &&
    safety.checkPathGeometry(nominal) == SafetyState::TRACKING &&
    safety.freePathLength(nominal) + kPathLengthTolerance >= config_.minimum_path_length &&
    speedLimit(nominal, safety) > 1e-4;
  const double nominal_curvature = geometry_.targetCurvature(nominal);
  const bool nominal_safe = nominal_geometry_safe && nominal_clearance >= config_.preferred_clearance &&
    (!corridor.valid || std::abs(nominal_curvature - geometry_.targetCurvature(boundary)) < 0.05);
  if (!avoiding_ && nominal_safe) {
    result.path = nominal;
    result.curvature = nominal_curvature;
    result.speed_limit = speedLimit(nominal, safety);
    result.free_length = safety.freePathLength(nominal);
    result.clearance = nominal_clearance;
    previous_curvature_ = nominal_curvature;
    return result;
  }
  avoiding_ = true;
  clear_frames_ = nominal_safe ? std::min(clear_frames_ + 1, config_.return_confirmation_frames) : 0;
  const bool returning = clear_frames_ >= config_.return_confirmation_frames;
  const double desired_curvature = returning ?
    previous_curvature_ + std::clamp(nominal_curvature - previous_curvature_,
    -config_.return_curvature_step, config_.return_curvature_step) :
    geometry_.targetCurvature(boundary);

  const double bin_step = 2.0 * config_.half_fov / (config_.angular_bins - 1);
  result.histogram.assign(config_.angular_bins, 0.0);
  for (const auto & obstacle : safety.obstacleCenters()) {
    const double range = std::hypot(obstacle.x, obstacle.y);
    if (range > horizon + config_.inflation_radius) {continue;}
    const double angle = std::atan2(obstacle.y, obstacle.x);
    const double inflation = std::asin(std::clamp(config_.inflation_radius /
      std::max(range, 1e-6), 0.0, 1.0));
    for (int bin = 0; bin < config_.angular_bins; ++bin) {
      const double theta = -config_.half_fov + bin * bin_step;
      if (std::abs(theta - angle) <= inflation + bin_step * 0.5) {
        // Max over occupied cells makes scores independent of point density.
        result.histogram[bin] = std::max(result.histogram[bin],
          1.0 / std::max(0.2, range));
      }
    }
  }

  std::vector<double> curvatures{desired_curvature, previous_curvature_, nominal_curvature, 0.0};
  const double target_distance = std::min(controller_config_.lookahead_distance, horizon);
  for (int bin = 0; bin < config_.angular_bins; ++bin) {
    const double theta = -config_.half_fov + bin * bin_step;
    curvatures.push_back(2.0 * std::sin(theta) / target_distance);
  }
  // Additional curvature samples retain reachable arcs even when polar bins are coarse.
  for (int bin = 0; bin < config_.angular_bins; ++bin) {
    curvatures.push_back(controller_config_.maximum_abs_curvature *
      (2.0 * bin / (config_.angular_bins - 1) - 1.0));
  }
  double best_cost = std::numeric_limits<double>::infinity();
  const double goal_angle = std::atan2(boundary.target.y, boundary.target.x);
  for (const double curvature : curvatures) {
    if (std::abs(curvature) > controller_config_.maximum_abs_curvature) {continue;}

    // A long fixed horizon can reject every sharp turn: the far end may leave the
    // camera Ground support even though a safe prefix is long enough to execute and
    // replan on the next frame.  Limit each curvature by the camera half-FOV, then
    // shorten it one path sample at a time down to minimum_path_length.  The first
    // safe result is therefore the longest currently observable safe prefix for that
    // curvature.  Ground coverage and obstacle clearance remain hard constraints.
    double candidate_horizon = horizon;
    if (std::abs(curvature) > 1e-6) {
      candidate_horizon = std::min(
        candidate_horizon, 2.0 * config_.half_fov / std::abs(curvature));
    }
    if (candidate_horizon + 1e-6 < config_.minimum_path_length) {continue;}

    VfhCandidate candidate;
    double candidate_speed = 0.0;
    while (true) {
      candidate.path = geometry_.buildConstantCurvaturePath(curvature, candidate_horizon);
      candidate_speed = speedLimit(candidate.path, safety);
      // A shorter observable prefix carries less look-ahead assurance.  Preserve
      // the braking/clearance cap and additionally reduce the command ceiling in
      // proportion to the selected prefix instead of driving it at nominal speed.
      candidate_speed = std::min(
        candidate_speed, controller_config_.maximum_linear_velocity *
        std::clamp(candidate_horizon / horizon, 0.0, 1.0));
      candidate.safe = corridor.contains(candidate.path) &&
        safety.checkPathGeometry(candidate.path) == SafetyState::TRACKING &&
        candidate_speed > 1e-4;
      if (candidate.safe ||
        candidate_horizon <= config_.minimum_path_length + 1e-6)
      {
        break;
      }
      candidate_horizon = std::max(
        config_.minimum_path_length,
        candidate_horizon - controller_config_.path_sample_step);
    }

    const double angle = std::atan2(candidate.path.target.y, candidate.path.target.x);
    if (std::abs(angle) > config_.half_fov + 1e-6) {continue;}
    const int bin = std::clamp(static_cast<int>(std::lround(
      (angle + config_.half_fov) / bin_step)), 0, config_.angular_bins - 1);
    // Histogram ranks directions; the full arc/footprint is the collision hard constraint.
    candidate.cost = config_.target_weight * square(angle - goal_angle) +
      config_.smooth_weight * square(curvature - previous_curvature_) +
      config_.obstacle_weight * (result.histogram[bin] + 4.0 *
      square(1.0 - candidate_speed / controller_config_.maximum_linear_velocity)) +
      config_.reference_weight * square(curvature - desired_curvature);
    const double clearance = pathClearance(candidate.path, safety);
    candidate.cost += config_.clearance_weight * square(std::clamp(
      (config_.preferred_clearance - clearance) / config_.preferred_clearance, 0.0, 1.0));
    if (corridor.valid) {
      double cost = 0.0;
      int count = 0;
      for (const auto & p : candidate.path.points) {
        if (p.x < corridor.xMin() || p.x > corridor.xMax()) {continue;}
        const double lo = corridor.lower(p.x), hi = corridor.upper(p.x);
        cost += square((p.y - (lo + hi) * 0.5) / std::max(0.05, (hi - lo) * 0.5));
        ++count;
      }
      if (count > 0) {candidate.cost += config_.corridor_weight * cost / count;}
    }
    if (!returning && previous_side_ != 0 && side(curvature) != 0 &&
      side(curvature) != previous_side_) {candidate.cost += config_.side_switch_penalty;}
    if (returning) {
      candidate.cost += 100.0 * square(curvature - desired_curvature);
    }
    if (candidate.safe && candidate.cost < best_cost) {
      best_cost = candidate.cost;
      result.path = candidate.path;
      result.curvature = curvature;
      result.speed_limit = candidate_speed;
    }
    result.candidates.push_back(std::move(candidate));
  }
  if (!result.path.valid) {
    result.mode = "BLOCKED";
    result.reason = "no_safe_forward_trajectory";
    return result;
  }
  result.mode = returning ? "RETURN" : "AVOID";
  result.reason = returning ? "nominal_clear_confirmed" :
    (nominal_geometry_safe ? "nominal_clearance_low" : "nominal_path_obstructed");
  if (returning && std::abs(result.curvature - nominal_curvature) <= config_.return_curvature_step &&
    nominal_safe)
  {
    result.path = nominal;
    result.curvature = nominal_curvature;
    result.mode = "FOLLOW";
    result.reason = "returned_to_nominal";
    reset();
  }
  previous_curvature_ = result.curvature;
  if (side(result.curvature) != 0) {previous_side_ = side(result.curvature);}
  if (result.mode == "FOLLOW") {
    result.speed_limit = speedLimit(result.path, safety);
  }
  result.free_length = safety.freePathLength(result.path);
  result.clearance = pathClearance(result.path, safety);
  return result;
}

bool VfhLocalPlanner::commandIsSafe(
  const ControlCommand & command, const SafetyGate & safety, const BoundaryCorridor & corridor) const
{
  if (!command.valid || !std::isfinite(command.linear) || !std::isfinite(command.angular) ||
    command.linear < 0.0) {return false;}
  if (command.linear <= 1e-6) {return std::abs(command.angular) <= 1e-6;}
  const double distance = command.linear * config_.reaction_time +
    square(command.linear) / (2.0 * config_.braking_deceleration) + config_.stop_margin;
  const double curvature = command.angular / command.linear;
  if (std::abs(curvature) > controller_config_.maximum_abs_curvature + 1e-6) {return false;}
  const auto path = geometry_.buildConstantCurvaturePath(curvature, std::max(0.35, distance));
  return corridor.contains(path) && safety.checkPathGeometry(path) == SafetyState::TRACKING;
}
}  // namespace camera_boundary_navigation
