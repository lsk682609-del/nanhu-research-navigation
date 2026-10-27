#include "camera_boundary_navigation/boundary_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace camera_boundary_navigation
{

namespace
{

Point2d cubicBezier(
  const Point2d & p0, const Point2d & p1, const Point2d & p2, const Point2d & p3,
  double t)
{
  const double one_minus_t = 1.0 - t;
  const double b0 = one_minus_t * one_minus_t * one_minus_t;
  const double b1 = 3.0 * one_minus_t * one_minus_t * t;
  const double b2 = 3.0 * one_minus_t * t * t;
  const double b3 = t * t * t;
  return {
    b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
    b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y};
}

void selectLookaheadTarget(ReferencePath & path, double lookahead_distance)
{
  if (path.points.size() < 2U) {
    return;
  }
  path.target = path.points.back();
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    if (std::hypot(path.points[i].x, path.points[i].y) >= lookahead_distance) {
      path.target = path.points[i];
      break;
    }
  }
  path.lookahead_distance = std::hypot(path.target.x, path.target.y);
  path.valid = path.lookahead_distance >= 0.3;
}

}  // 匿名命名空间

BoundaryController::BoundaryController(BoundaryControllerConfig config)
: config_(std::move(config))
{
}

ReferencePath BoundaryController::buildReferencePath(const BoundaryCurve & curve) const
{
  ReferencePath path;
  if (!curve.valid || curve.x_max <= curve.x_min || config_.path_sample_step <= 0.0) {
    return path;
  }
  std::vector<Point2d> offset_points;
  for (double x = curve.x_min; x <= curve.x_max + 1e-6; x += config_.path_sample_step) {
    const double derivative = curve.derivative(x);
    const double normalizer = std::hypot(1.0, derivative);
    Point2d inward_normal;
    if (config_.side == ReferenceSide::LEFT) {
      inward_normal = {derivative / normalizer, -1.0 / normalizer};
    } else {
      inward_normal = {-derivative / normalizer, 1.0 / normalizer};
    }
    const Point2d reference{
      x + config_.boundary_offset * inward_normal.x,
      curve.y(x) + config_.boundary_offset * inward_normal.y};
    if (reference.x > 0.05 && std::isfinite(reference.x) && std::isfinite(reference.y)) {
      offset_points.push_back(reference);
    }
  }
  if (offset_points.size() < 2U) {
    return path;
  }

  // 构造从相机/车辆原点到第一个偏移边界点的三次Bé塞尔过渡曲线。
  // P1保证曲线沿车辆x轴方向离开原点，P2保证到达时与偏移边界曲线切向一致。
  // 该过渡曲线替代原来的直线连接，消除第一个边界点处可见的切向突变。
  const Point2d origin{0.0, 0.0};
  const Point2d join = offset_points.front();
  Point2d join_tangent{
    offset_points[1].x - offset_points[0].x,
    offset_points[1].y - offset_points[0].y};
  const double tangent_norm = std::hypot(join_tangent.x, join_tangent.y);
  if (tangent_norm <= 1e-6) {
    return path;
  }
  join_tangent.x /= tangent_norm;
  join_tangent.y /= tangent_norm;

  const double chord_length = std::hypot(join.x, join.y);
  if (chord_length <= 1e-6) {
    return path;
  }
  // 根据连接点的前向距离限制控制柄长度，避免横向偏差较大时过渡曲线向后弯折。
  const double handle_length = std::min(
    config_.bezier_handle_ratio * chord_length, 0.45 * join.x);
  if (handle_length <= 1e-6) {
    return path;
  }
  const Point2d departure_control{handle_length, 0.0};
  const Point2d arrival_control{
    join.x - handle_length * join_tangent.x,
    join.y - handle_length * join_tangent.y};
  const double control_polygon_length =
    std::hypot(departure_control.x - origin.x, departure_control.y - origin.y) +
    std::hypot(
      arrival_control.x - departure_control.x,
      arrival_control.y - departure_control.y) +
    std::hypot(join.x - arrival_control.x, join.y - arrival_control.y);
  // 三次Bé塞尔曲线在显示和安全检查时会离散为折线。过渡段采用普通路径一半的
  // 采样间距，使离散折线的末段方向仍接近理论上一阶连续的到达切向。
  const int connector_segments = std::max(
    2, static_cast<int>(std::ceil(
      control_polygon_length / (0.5 * config_.path_sample_step))));

  path.points.reserve(
    static_cast<std::size_t>(connector_segments) + offset_points.size());
  path.points.push_back(origin);
  for (int i = 1; i <= connector_segments; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(connector_segments);
    path.points.push_back(cubicBezier(
      origin, departure_control, arrival_control, join, t));
  }
  path.points.insert(path.points.end(), offset_points.begin() + 1, offset_points.end());

  selectLookaheadTarget(path, config_.lookahead_distance);
  return path;
}

ReferencePath BoundaryController::buildConstantCurvaturePath(
  double curvature, double horizon) const
{
  ReferencePath path;
  if (!std::isfinite(curvature) || !std::isfinite(horizon) || horizon < 0.3 ||
    config_.path_sample_step <= 0.0)
  {
    return path;
  }

  const auto point_at_distance = [curvature](double distance) {
      if (std::abs(curvature) <= 1e-6) {
        return Point2d{distance, 0.0};
      }
      return Point2d{
        std::sin(curvature * distance) / curvature,
        (1.0 - std::cos(curvature * distance)) / curvature};
    };

  path.points.reserve(
    static_cast<std::size_t>(std::ceil(horizon / config_.path_sample_step)) + 2U);
  path.points.push_back({0.0, 0.0});
  double distance = config_.path_sample_step;
  for (; distance < horizon; distance += config_.path_sample_step) {
    path.points.push_back(point_at_distance(distance));
  }
  path.points.push_back(point_at_distance(horizon));
  selectLookaheadTarget(path, config_.lookahead_distance);
  return path;
}

double BoundaryController::targetCurvature(const ReferencePath & path) const
{
  if (!path.valid || path.lookahead_distance <= 1e-6) {
    return 0.0;
  }
  const double squared_distance =
    path.target.x * path.target.x + path.target.y * path.target.y;
  return 2.0 * path.target.y / squared_distance;
}

ControlCommand BoundaryController::computeCommand(
  const ReferencePath & path, double dt, double speed_limit)
{
  ControlCommand command;
  if (!path.valid || path.lookahead_distance <= 1e-6 || dt <= 0.0) {
    return command;
  }
  command.curvature = targetCurvature(path);
  if (!std::isfinite(command.curvature) ||
    std::abs(command.curvature) > config_.maximum_abs_curvature)
  {
    return ControlCommand{};
  }

  const double raw_linear = std::min(std::max(0.0, speed_limit), std::clamp(
    config_.nominal_linear_velocity /
    (1.0 + config_.curvature_slowdown_gain * std::abs(command.curvature)),
    config_.minimum_linear_velocity, config_.maximum_linear_velocity));
  double raw_angular = std::clamp(
    raw_linear * command.curvature,
    -config_.maximum_angular_velocity, config_.maximum_angular_velocity);

  if (command_ready_) {
    const double maximum_change = config_.maximum_angular_acceleration * dt;
    raw_angular = std::clamp(
      raw_angular, previous_command_.angular - maximum_change,
      previous_command_.angular + maximum_change);
    const double alpha = std::clamp(config_.command_filter_alpha, 0.0, 1.0);
    command.linear = (1.0 - alpha) * previous_command_.linear + alpha * raw_linear;
    command.angular = (1.0 - alpha) * previous_command_.angular + alpha * raw_angular;
  } else {
    command.linear = raw_linear;
    command.angular = raw_angular;
  }
  // Apply the safety cap after filtering, preserving curvature when reducing speed.
  if (command.linear > std::max(0.0, speed_limit)) {
    command.angular *= std::max(0.0, speed_limit) / command.linear;
    command.linear = std::max(0.0, speed_limit);
  }
  if (command.linear <= 1e-9) {command.angular = 0.0;}
  command.valid = true;
  previous_command_ = command;
  command_ready_ = true;
  return command;
}

void BoundaryController::reset()
{
  previous_command_ = ControlCommand{};
  command_ready_ = false;
}

}  // camera_boundary_navigation命名空间
