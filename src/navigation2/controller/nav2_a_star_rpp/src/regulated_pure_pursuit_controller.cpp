// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright 2026 Wei Wang -> shaxikai@outlook.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav2_a_star_rpp/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <fmt/format.h>
#include "tf2/LinearMath/Matrix3x3.h"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace nav2_a_star_rpp
{
namespace
{
// 若本周期计算异常退出，上层会立即下发安全急停零速度，内部历史指令也必须同步归零。
class LinearCommandExceptionGuard
{
public:
  LinearCommandExceptionGuard(double & command, bool diagnostics, rclcpp::Logger logger)
  : command_(command), uncaught_exceptions_(std::uncaught_exceptions()),
    diagnostics_(diagnostics), logger_(logger) {}

  ~LinearCommandExceptionGuard()
  {
    if (std::uncaught_exceptions() > uncaught_exceptions_) {
      if (diagnostics_) {
        RCLCPP_WARN(logger_, "[speed_diag/reset] reason=controller_exception previous=%.4f next=0", command_);
      }
      command_ = 0.0;
    }
  }

private:
  double & command_;
  int uncaught_exceptions_;
  bool diagnostics_;
  rclcpp::Logger logger_;
};

// 按弧长比较汇入点之前的重叠路线，避免切片起点移动、采样密度变化造成误重置。
struct RouteProjection
{
  double distance{std::numeric_limits<double>::infinity()};
  double progress{0.0};
  size_t next{0};
};

std::vector<double> routeLengths(const nav_msgs::msg::Path & path)
{
  std::vector<double> lengths(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    lengths[i] = lengths[i - 1] + hypot(b.x - a.x, b.y - a.y);
  }
  return lengths;
}

RouteProjection projectJoin(
  const nav_msgs::msg::Path & path, const std::vector<double> & lengths,
  const geometry_msgs::msg::Point & join)
{
  RouteProjection result;
  // 第一个点可能是随机器人移动的连接锚点，不参与路线一致性判断。
  for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length < 1.0e-12) {continue;}
    const double ratio = std::clamp(
      ((join.x - a.x) * dx + (join.y - a.y) * dy) / squared_length, 0.0, 1.0);
    const double distance = hypot(join.x - a.x - ratio * dx, join.y - a.y - ratio * dy);
    if (distance < result.distance) {
      result.distance = distance;
      result.progress = lengths[i] + ratio * std::sqrt(squared_length);
      result.next = ratio >= 1.0 - 1.0e-6 ? i + 2 : i + 1;
    }
  }
  return result;
}

geometry_msgs::msg::Point routePoint(
  const nav_msgs::msg::Path & path, const std::vector<double> & lengths, double progress)
{
  auto upper = std::upper_bound(lengths.begin(), lengths.end(), progress);
  if (upper == lengths.end()) {return path.poses.back().pose.position;}
  const size_t i = std::max<size_t>(1, std::distance(lengths.begin(), upper));
  const double ratio = (progress - lengths[i - 1]) / (lengths[i] - lengths[i - 1]);
  auto point = path.poses[i - 1].pose.position;
  const auto & end = path.poses[i].pose.position;
  point.x += ratio * (end.x - point.x);
  point.y += ratio * (end.y - point.y);
  return point;
}

bool sameRouteToJoin(
  const nav_msgs::msg::Path & previous, const nav_msgs::msg::Path & updated,
  const geometry_msgs::msg::Point & join, size_t & next)
{
  if (previous.poses.size() < 3 || updated.poses.size() < 3) {return false;}
  for (const auto * path : {&previous, &updated}) {
    for (const auto & pose : path->poses) {
      if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) {
        return false;
      }
    }
  }
  const auto old_lengths = routeLengths(previous);
  const auto new_lengths = routeLengths(updated);
  const auto old_join = projectJoin(previous, old_lengths, join);
  const auto new_join = projectJoin(updated, new_lengths, join);
  constexpr double tolerance = 0.20;
  if (old_join.distance > tolerance || new_join.distance > tolerance) {return false;}
  const double overlap = std::min(
    old_join.progress - old_lengths[1], new_join.progress - new_lengths[1]);
  // 临近汇入点时，上游切片会缩短。仍比较所有剩余上游，并在不足
  // 0.5 m 时使用共同下游补充有向路线证据，不能只凭汇入点接近就保留。
  const double forward_overlap = std::min(
    old_lengths.back() - old_join.progress, new_lengths.back() - new_join.progress);
  if (overlap < 0.50 && overlap + forward_overlap < 0.50) {return false;}
  const int samples = std::max(1, static_cast<int>(std::ceil(overlap / 0.10)));
  for (int i = 0; i <= samples; ++i) {
    const double back = overlap * i / samples;
    const auto a = routePoint(previous, old_lengths, old_join.progress - back);
    const auto b = routePoint(updated, new_lengths, new_join.progress - back);
    if (hypot(a.x - b.x, a.y - b.y) > tolerance) {return false;}
  }
  if (overlap < 0.50) {
    const int forward_samples = std::max(1, static_cast<int>(std::ceil(forward_overlap / 0.10)));
    for (int i = 0; i <= forward_samples; ++i) {
      const double ahead = forward_overlap * i / forward_samples;
      const auto a = routePoint(previous, old_lengths, old_join.progress + ahead);
      const auto b = routePoint(updated, new_lengths, new_join.progress + ahead);
      if (hypot(a.x - b.x, a.y - b.y) > tolerance) {return false;}
    }
  }
  next = new_join.next;
  return true;
}
}  // namespace

void RegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  planner_ = std::make_unique<theta_star::ThetaStar>();
  planner_->costmap_ = costmap_ros->getCostmap();

  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_decel", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".speed_diagnostics_enabled", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_collision_detection",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lethal_cost", rclcpp::ParameterValue(252));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".traversal_cost_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".start_relax_radius", rclcpp::ParameterValue(0.80));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".prefer_planner_polyline_when_clear",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".polyline_collision_check_resolution",
    rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".polyline_track_max_length",
    rclcpp::ParameterValue(15.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_global_path_slice_freeze", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_merge_connection_point", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_smoothing_enabled", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_max_iterations", rclcpp::ParameterValue(100));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_tolerance", rclcpp::ParameterValue(1.0e-4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_data_weight", rclcpp::ParameterValue(0.20));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_smooth_weight", rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_max_deviation", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_collision_step", rclcpp::ParameterValue(0.025));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_backtracking_steps", rclcpp::ParameterValue(6));

  // 门区三阶段：默认关闭，需在参数文件中显式开启。
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_servo_enabled", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_cloud_topic", rclcpp::ParameterValue(std::string("cur_lid")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_cloud_timeout", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_detect_range", rclcpp::ParameterValue(2.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_detect_half_width", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_min_height", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_max_height", rclcpp::ParameterValue(2.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_min_width", rclcpp::ParameterValue(0.55));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_max_width", rclcpp::ParameterValue(1.15));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_enter_distance", rclcpp::ParameterValue(2.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_align_angle", rclcpp::ParameterValue(0.08));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_align_rate", rclcpp::ParameterValue(0.45));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_cross_speed", rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_lateral_gain", rclcpp::ParameterValue(1.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_max_angular", rclcpp::ParameterValue(0.45));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_exit_margin", rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_exit_hold", rclcpp::ParameterValue(0.40));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_hold_distance", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_min_points", rclcpp::ParameterValue(5));
  // 只有检测到的门心落在当前路线上，才允许门区伺服接管。
  // 门区伺服会锁存航向直行，若被路旁的其它“缝隙”误触发，会盲目前进撞墙。
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_route_tolerance", rclcpp::ParameterValue(0.40));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".door_positions", rclcpp::ParameterValue(std::vector<double>{}));

  // 卡死自救：顶住障碍、持续预测碰撞或局部规划连续无解时，先沿车尾方向
  // 退出一小段再重新规划。后退前必须用 footprint 校验车尾走廊，不能把车退进墙里。
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_enabled", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_backup_distance", rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_backup_speed", rclcpp::ParameterValue(0.12));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_rear_clearance", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_stuck_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_stuck_translation", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_max_cycles", rclcpp::ParameterValue(2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_retrigger_window", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_collision_cycles", rclcpp::ParameterValue(5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_no_path_cycles", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".front_stop_clearance", rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".front_slow_clearance", rclcpp::ParameterValue(0.95));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".front_stop_half_width", rclcpp::ParameterValue(0.20));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".recovery_front_block_cycles", rclcpp::ParameterValue(3));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_linear_decel", max_linear_decel_);
  node->get_parameter(plugin_name_ + ".speed_diagnostics_enabled", speed_diagnostics_enabled_);
  if (max_linear_accel_ <= 0.0 || max_linear_decel_ <= 0.0) {
    throw nav2_core::PlannerException(
            "max_linear_accel and max_linear_decel must be positive");
  }
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity",
    min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist",
    approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(
    plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    max_allowed_time_to_collision_up_to_carrot_);
  node->get_parameter(
    plugin_name_ + ".use_collision_detection",
    use_collision_detection_);
  node->get_parameter(
    plugin_name_ + ".use_regulated_linear_velocity_scaling",
    use_regulated_linear_velocity_scaling_);
  node->get_parameter(
    plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    use_cost_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
  node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
  node->get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor",
    inflation_cost_scaling_factor_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius",
    regulated_linear_scaling_min_radius_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_speed",
    regulated_linear_scaling_min_speed_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  int lethal_cost = 252;
  node->get_parameter(plugin_name_ + ".lethal_cost", lethal_cost);
  planner_->lethal_cost_ = lethal_cost;
  node->get_parameter(
    plugin_name_ + ".traversal_cost_weight", planner_->w_traversal_cost_);
  node->get_parameter(plugin_name_ + ".start_relax_radius", planner_->start_relax_radius_);
  node->get_parameter("controller_frequency", control_frequency);
  node->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist",
    max_robot_pose_search_dist_);
  node->get_parameter(
    plugin_name_ + ".use_interpolation",
    use_interpolation_);
  node->get_parameter(
    plugin_name_ + ".prefer_planner_polyline_when_clear",
    prefer_planner_polyline_when_clear_);
  node->get_parameter(
    plugin_name_ + ".polyline_collision_check_resolution",
    polyline_collision_check_resolution_);
  node->get_parameter(
    plugin_name_ + ".polyline_track_max_length",
    polyline_track_max_length_);
  node->get_parameter(
    plugin_name_ + ".publish_global_path_slice_freeze", publish_global_path_slice_freeze_);
  node->get_parameter(
    plugin_name_ + ".publish_merge_connection_point", publish_merge_connection_point_);
  node->get_parameter(plugin_name_ + ".path_smoothing_enabled", path_smoothing_enabled_);
  node->get_parameter(
    plugin_name_ + ".smoothing_max_iterations", smoothing_parameters_.max_iterations);
  node->get_parameter(plugin_name_ + ".smoothing_tolerance", smoothing_parameters_.tolerance);
  node->get_parameter(plugin_name_ + ".smoothing_data_weight", smoothing_parameters_.data_weight);
  node->get_parameter(
    plugin_name_ + ".smoothing_smooth_weight", smoothing_parameters_.smooth_weight);
  node->get_parameter(
    plugin_name_ + ".smoothing_max_deviation", smoothing_parameters_.max_deviation);
  node->get_parameter(
    plugin_name_ + ".smoothing_collision_step", smoothing_parameters_.collision_check_step);
  node->get_parameter(
    plugin_name_ + ".smoothing_backtracking_steps", smoothing_parameters_.backtracking_steps);

  node->get_parameter(plugin_name_ + ".door_servo_enabled", door_servo_enabled_);
  node->get_parameter(plugin_name_ + ".door_cloud_topic", door_cloud_topic_);
  node->get_parameter(plugin_name_ + ".door_cloud_timeout", door_cloud_timeout_);
  node->get_parameter(plugin_name_ + ".door_detect_range", door_detect_range_);
  node->get_parameter(plugin_name_ + ".door_detect_half_width", door_detect_half_width_);
  node->get_parameter(plugin_name_ + ".door_min_height", door_min_height_);
  node->get_parameter(plugin_name_ + ".door_max_height", door_max_height_);
  node->get_parameter(plugin_name_ + ".door_min_width", door_min_width_);
  node->get_parameter(plugin_name_ + ".door_max_width", door_max_width_);
  node->get_parameter(plugin_name_ + ".door_enter_distance", door_enter_distance_);
  node->get_parameter(plugin_name_ + ".door_align_angle", door_align_angle_);
  node->get_parameter(plugin_name_ + ".door_align_rate", door_align_rate_);
  node->get_parameter(plugin_name_ + ".door_cross_speed", door_cross_speed_);
  node->get_parameter(plugin_name_ + ".door_lateral_gain", door_lateral_gain_);
  node->get_parameter(plugin_name_ + ".door_max_angular", door_max_angular_);
  node->get_parameter(plugin_name_ + ".door_exit_margin", door_exit_margin_);
  node->get_parameter(plugin_name_ + ".door_exit_hold", door_exit_hold_);
  node->get_parameter(plugin_name_ + ".door_route_tolerance", door_route_tolerance_);
  node->get_parameter(plugin_name_ + ".door_hold_distance", door_hold_distance_);
  node->get_parameter(plugin_name_ + ".door_min_points", door_min_points_);
  node->get_parameter(plugin_name_ + ".door_positions", door_positions_);
  node->get_parameter(plugin_name_ + ".recovery_enabled", recovery_enabled_);
  node->get_parameter(plugin_name_ + ".recovery_backup_distance", recovery_backup_distance_);
  node->get_parameter(plugin_name_ + ".recovery_backup_speed", recovery_backup_speed_);
  node->get_parameter(plugin_name_ + ".recovery_rear_clearance", recovery_rear_clearance_);
  node->get_parameter(plugin_name_ + ".recovery_stuck_time", recovery_stuck_time_);
  node->get_parameter(plugin_name_ + ".recovery_stuck_translation", recovery_stuck_translation_);
  node->get_parameter(plugin_name_ + ".recovery_max_cycles", recovery_max_cycles_);
  node->get_parameter(plugin_name_ + ".recovery_retrigger_window", recovery_retrigger_window_);
  node->get_parameter(plugin_name_ + ".recovery_collision_cycles", recovery_collision_cycles_threshold_);
  node->get_parameter(plugin_name_ + ".recovery_no_path_cycles", recovery_no_path_cycles_threshold_);
  node->get_parameter(plugin_name_ + ".front_stop_clearance", front_stop_clearance_);
  node->get_parameter(plugin_name_ + ".front_slow_clearance", front_slow_clearance_);
  node->get_parameter(plugin_name_ + ".front_stop_half_width", front_stop_half_width_);
  node->get_parameter(plugin_name_ + ".recovery_front_block_cycles", recovery_front_block_cycles_);
  if (recovery_backup_distance_ <= 0.0 || recovery_backup_speed_ <= 0.0 ||
    recovery_max_cycles_ < 0)
  {
    RCLCPP_WARN(
      logger_, "recovery_backup_distance/speed must be positive; 卡死自救已禁用");
    recovery_enabled_ = false;
  }
  if (smoothing_parameters_.max_iterations <= 0 || smoothing_parameters_.tolerance < 0.0 ||
    smoothing_parameters_.data_weight < 0.0 || smoothing_parameters_.data_weight > 1.0 ||
    smoothing_parameters_.smooth_weight <= 0.0 || smoothing_parameters_.smooth_weight > 0.5 ||
    smoothing_parameters_.max_deviation <= 0.0 ||
    smoothing_parameters_.collision_check_step <= 0.0 ||
    smoothing_parameters_.backtracking_steps < 0)
  {
    throw nav2_core::PlannerException("Invalid Theta* path smoothing parameters");
  }
  path_smoother_.setParameters(smoothing_parameters_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  if (inflation_cost_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }

  /** Possible to drive in reverse direction if and only if
   "use_rotate_to_heading" parameter is set to false **/

  if (use_rotate_to_heading_ && allow_reversing_) {
    RCLCPP_WARN(
      logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
      "parameter cannot be set to true. By default setting use_rotate_to_heading true");
    allow_reversing_ = false;
  }

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  theta_raw_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "theta_star_path_raw", rclcpp::QoS(1).transient_local());
  theta_smoothed_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "theta_star_path_smoothed", rclcpp::QoS(1).transient_local());
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
  freeze_slice_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "global_path_slice_freeze", rclcpp::QoS(1).transient_local());
  record_pub_ = node->create_publisher<std_msgs::msg::String>("/log/record", 10);
  if (publish_merge_connection_point_) {
    merge_connection_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "merge_path_connection_pose", rclcpp::QoS(1).transient_local());
  }

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);

  local_planner_type_ = nav2_scan_planner::parameter(*node,
    plugin_name_ + ".local_planner_type", std::string("theta_star"));
  if (local_planner_type_ == "scan") {
    if (!use_collision_detection_ || use_cost_regulated_linear_velocity_scaling_ || allow_reversing_) {
      throw nav2_core::PlannerException(
        "SCAN requires collision detection, forward tracking, and cost-based scaling disabled");
    }
    scan_ = std::make_unique<nav2_scan_planner::ScanBackend>(*node, plugin_name_ + ".scan",
      tf_, costmap_ros_->getGlobalFrameID(), costmap_ros_->getBaseFrameID());
  } else if (local_planner_type_ != "theta_star") {
    throw nav2_core::PlannerException("local_planner_type must be theta_star or scan");
  }
  RCLCPP_INFO(logger_, "Local planning backend: %s", local_planner_type_.c_str());
}

void RegulatedPurePursuitController::cleanup()
{
  scan_.reset();
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " nav2_a_star_rpp::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  theta_raw_pub_.reset();
  theta_smoothed_pub_.reset();
  carrot_pub_.reset();
  carrot_arc_pub_.reset();
  freeze_slice_pub_.reset();
  record_pub_.reset();
  merge_connection_pose_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
  // 每次激活都从静止状态建立速度斜坡。
  last_linear_command_ = 0.0;
  door_phase_ = DoorPhase::NONE;
  door_obs_ = DoorObservation{};
  door_obs_time_ = -1.0;
  door_last_time_ = -1.0;
  if (scan_) scan_->activate();
  avoid_flag_ = false;
  track_path_.poses.clear();
  recovery_phase_ = RecoveryPhase::NONE;
  recovery_travel_ = 0.0;
  recovery_target_ = 0.0;
  recovery_start_time_ = -1.0;
  recovery_anchor_valid_ = false;
  recovery_cycles_ = 0;
  recovery_last_trigger_time_ = -1.0;
  recovery_collision_cycles_ = 0;
  recovery_no_path_cycles_ = 0;
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "nav2_a_star_rpp::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  theta_raw_pub_->on_activate();
  theta_smoothed_pub_->on_activate();
  carrot_pub_->on_activate();
  carrot_arc_pub_->on_activate();
  if (freeze_slice_pub_) {
    freeze_slice_pub_->on_activate();
    if (prefer_planner_polyline_when_clear_) {
      std_msgs::msg::Bool msg;
      msg.data = false;
      freeze_slice_pub_->publish(msg);
    }
  }
  if (merge_connection_pose_pub_) {
    merge_connection_pose_pub_->on_activate();
  }
  record_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  if (door_servo_enabled_ && node && !door_cloud_sub_) {
    door_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      door_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &RegulatedPurePursuitController::doorCloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[门区] 门框测距伺服已启用, 点云话题=%s", door_cloud_topic_.c_str());
  }
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &RegulatedPurePursuitController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void RegulatedPurePursuitController::deactivate()
{
  if (scan_) scan_->deactivate();
  door_cloud_sub_.reset();
  door_phase_ = DoorPhase::NONE;
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "nav2_a_star_rpp::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  theta_raw_pub_->on_deactivate();
  theta_smoothed_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_arc_pub_->on_deactivate();
  if (freeze_slice_pub_) {
    freeze_slice_pub_->on_deactivate();
  }
  if (merge_connection_pose_pub_) {
    merge_connection_pose_pub_->on_deactivate();
  }
  record_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  LinearCommandExceptionGuard reset_command_on_exception(
    last_linear_command_, speed_diagnostics_enabled_, logger_);
  const bool diagnostic_avoid_before = avoid_flag_;
  const double diagnostic_previous_command = last_linear_command_;

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()), std::defer_lock);
  if (!scan_) lock.lock();

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // 卡死自救必须先于门区阶段判定：门区一旦接管就会提前返回本函数，
  // 若把自救放在门区之后，门区里顶住障碍时将永远检测不到（只会等进度检查超时）。
  // 进入后退时若正处于门区，会一并复位门区阶段，出门后再按已知门位重新对准。
  if (recovery_enabled_) {
    geometry_msgs::msg::TwistStamped recovery_cmd;
    if (updateRecovery(pose, speed, recovery_cmd)) {
      return recovery_cmd;
    }
  }

  // 前向停车保护：门区伺服与常规折线跟踪都必须先满足「车头净空」这一安全约束，
  // 否则会先顶上障碍再靠自救倒退，表现为反复撞墙、且停车时贴得过近。
  const double front_clear = frontClearDistance(pose);

  // 门区三阶段优先于常规折线跟踪：0.70 m 门洞的净宽不足以通过
  // isPolylineSegmentFree 的 253 判据，必须由门框测距伺服直接给出速度。
  if (door_servo_enabled_) {
    updateDoorPhase(pose, speed);
    if (door_phase_ != DoorPhase::NONE) {
      geometry_msgs::msg::TwistStamped door_cmd;
      door_cmd.header = pose.header;
      computeDoorCommand(door_obs_, door_cmd.twist.linear.x, door_cmd.twist.angular.z);
      if (door_cmd.twist.linear.x > 0.0) {
        try {
          if (frontStopActive(pose, front_clear)) {
            door_cmd.twist.linear.x = 0.0;
          } else {
            door_cmd.twist.linear.x *= frontClearanceScale(front_clear);
          }
        } catch (const nav2_core::PlannerException &) {
          // 交回全局重规划前把门区阶段复位，避免新路径继续被旧门区状态接管。
          resetDoorPhase();
          throw;
        }
      }
      last_linear_command_ = door_cmd.twist.linear.x;
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "[门区] phase=%d x=%.2f y=%.2f width=%.2f v=%.2f w=%.2f",
        static_cast<int>(door_phase_), door_obs_.x, door_obs_.y, door_obs_.width,
        door_cmd.twist.linear.x, door_cmd.twist.angular.z);
      if (record_pub_) {
        std_msgs::msg::String message;
        message.data = fmt::format(
          "{:.6f},ctrl,door,{},x={:.3f},y={:.3f},w={:.3f},v={:.3f},wz={:.3f}",
          clock_->now().seconds(), static_cast<int>(door_phase_), door_obs_.x, door_obs_.y,
          door_obs_.width, door_cmd.twist.linear.x, door_cmd.twist.angular.z);
        record_pub_->publish(message);
      }
      return door_cmd;
    }
  }

  // get local plan
  diagnostic_plan_source_ = scan_ ? "scan" : "empty";
  if (scan_) {
    try {
      if (!scan_->ready()) throw std::runtime_error("SCAN GridMap not ready or sensor data stale");
      local_plan_ = getScanLocalPlan(pose, speed);
    } catch (const std::exception &e) {
      local_plan_.poses.clear();
      throw nav2_core::PlannerException(e.what());
    }
  } else {
    local_plan_ = getLocalPlan(pose);
  }
  
  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  const char* cvc_method = "APC";
  const double diagnostic_nan = std::numeric_limits<double>::quiet_NaN();
  double diagnostic_pose_cost = diagnostic_nan;
  double diagnostic_target_velocity = diagnostic_nan;
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
    cvc_method = "RTG";
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "[绕障诊断/原地对准路径] avoid=%d angle=%.3f threshold=%.3f carrot=(%.3f,%.3f) track_points=%zu join=%zu",
      avoid_flag_, angle_to_heading, rotate_to_heading_min_angle_,
      carrot_pose.pose.position.x, carrot_pose.pose.position.y, track_path_.poses.size(), join_idx_);
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
    cvc_method = "RTP";
  } else {
    diagnostic_pose_cost = costAtPose(pose.pose.position.x, pose.pose.position.y);
    applyConstraints(
      curvature, speed,
      diagnostic_pose_cost, transformed_plan,
      linear_vel, sign);
    diagnostic_target_velocity = linear_vel;

    // 无条件以上一次成功下发的控制指令累积速度斜坡，不再依赖实测速度、有效标志或超时判断。
    // 原地转向和安全急停仍可立即置零。
    linear_vel = limitLinearVelocity(linear_vel, last_linear_command_);

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
  }

  // 前向停车保护：车头净空取自代价地图 LETHAL_OBSTACLE(254)，不受窄道/门区
  // 内切带(253)饱和的影响。净空不足时停止前向并触发自救；净空偏小时线性降速，
  // 避免以巡航速度顶到障碍后才发现，也能让停车位置与障碍保持安全间距。
  if (linear_vel > 0.0) {
    if (frontStopActive(pose, front_clear)) {
      linear_vel = 0.0;
      angular_vel = 0.0;
    } else {
      linear_vel *= frontClearanceScale(front_clear);
      angular_vel = linear_vel * curvature;
    }
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  const bool collision = use_collision_detection_ &&
    isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist);
  if (speed_diagnostics_enabled_) {
    const auto & end = transformed_plan.poses.back().pose.position;
    const bool constrained = std::string(cvc_method) == "APC";
    double join_distance = diagnostic_nan;
    if (!track_path_.poses.empty() && track_path_.header.frame_id == pose.header.frame_id) {
      const auto & join = track_path_.poses[std::min(join_idx_, track_path_.poses.size()-1)].pose.position;
      join_distance = hypot(join.x-pose.pose.position.x, join.y-pose.pose.position.y);
    }
    double slice_end_distance = diagnostic_nan;
    if (!global_plan_.poses.empty() && global_plan_.header.frame_id == pose.header.frame_id) {
      const auto & goal = global_plan_.poses.back().pose.position;
      slice_end_distance = hypot(goal.x-pose.pose.position.x, goal.y-pose.pose.position.y);
    }
    const double local_length = nav2_util::geometry_utils::calculate_path_length(transformed_plan);
    const double approach_factor = approachVelocityScalingFactor(transformed_plan);
    // A separate tagged record preserves the existing ctrl,cvc CSV schema.
    const auto detail = fmt::format(
      "{:.6f},ctrl,speed_diag_v1,plan_seq={},backend={},source={},mode={},avoid_before={},avoid_after={},"
      "join={},track_points={},join_dist={:.4f},local_points={},local_length={:.4f},"
      "local_end_dist={:.4f},slice_end_dist={:.4f},carrot_x={:.4f},carrot_y={:.4f},"
      "heading_error={:.4f},curvature={:.4f},pose_cost={:.1f},v_desired={:.4f},"
      "v_curve={:.4f},v_cost={:.4f},v_pre_approach={:.4f},approach_factor={:.4f},"
      "v_target={:.4f},v_previous={:.4f},v_computed={:.4f},w_computed={:.4f},"
      "odom_vx={:.4f},odom_vy={:.4f},odom_wz={:.4f},control_dt={:.4f},"
      "max_accel={:.4f},max_decel={:.4f},collision_rejected={}",
      clock_->now().seconds(), diagnostic_plan_sequence_, scan_ ? "SCAN" : "COSTMAP",
      diagnostic_plan_source_, cvc_method, diagnostic_avoid_before, avoid_flag_, join_idx_, track_path_.poses.size(),
      join_distance, transformed_plan.poses.size(), local_length, hypot(end.x,end.y),
      slice_end_distance, carrot_pose.pose.position.x, carrot_pose.pose.position.y,
      atan2(carrot_pose.pose.position.y,carrot_pose.pose.position.x), curvature, diagnostic_pose_cost,
      desired_linear_vel_, constrained ? diagnostic_curvature_velocity_ : diagnostic_nan,
      constrained ? diagnostic_cost_velocity_ : diagnostic_nan,
      constrained ? diagnostic_pre_approach_velocity_ : diagnostic_nan, approach_factor,
      diagnostic_target_velocity, diagnostic_previous_command, linear_vel, angular_vel,
      speed.linear.x, speed.linear.y, speed.angular.z, control_duration_,
      max_linear_accel_, max_linear_decel_, collision);
    if (record_pub_) {
      std_msgs::msg::String message;
      message.data = detail;
      record_pub_->publish(message);
    }
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 500, "[speed_diag] %s", detail.c_str());
  }
  if (collision) {
    throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  // recorder
  if (record_pub_) {
    double rx = pose.pose.position.x;
    double ry = pose.pose.position.y;
    double rz = pose.pose.position.z;
    double rr, rpi, ryaw;
    tf2::Matrix3x3(
      tf2::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                      pose.pose.orientation.z, pose.pose.orientation.w))
      .getRPY(rr, rpi, ryaw);
    // carrot_pose is already in base_link frame
    double cx = carrot_pose.pose.position.x;
    double cy = carrot_pose.pose.position.y;
    double cz = carrot_pose.pose.position.z;
    auto m = std::make_shared<std_msgs::msg::String>();
    m->data = fmt::format("{:.6f},{},{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{}",
      clock_->now().seconds(), "ctrl", "cvc", static_cast<int>(exec_status_),
      rx, ry, rz, rr, rpi, ryaw,
      cx, cy, cz,
      speed.linear.x, speed.angular.z,
      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cvc_method);
    record_pub_->publish(*m);
  }

  // 仅在本次计算成功后保存指令；原地转向时 linear_vel 为零，因此会立即保存零。
  last_linear_command_ = linear_vel;
  return cmd_vel;
}

double RegulatedPurePursuitController::limitLinearVelocity(
  double target_velocity, double current_velocity) const
{
  if (!std::isfinite(current_velocity)) {
    current_velocity = 0.0;
  }

  const bool changing_direction = target_velocity * current_velocity < 0.0;
  const bool increasing_speed =
    std::abs(target_velocity) > std::abs(current_velocity) && !changing_direction;
  const double rate_limit = increasing_speed ? max_linear_accel_ : max_linear_decel_;
  const double max_change = rate_limit * control_duration_;
  return current_velocity + std::clamp(
    target_velocity - current_velocity, -max_change, max_change);
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  // 阈值必须明显小于 xy_goal_tolerance：若两者相等，车恰好停在容差边界上时
  // 会被强制 linear_vel=0，而 goal checker 又因差一点点判不到，形成原地等死的死锁。
  return use_rotate_to_heading_ &&
         dist_to_goal < std::min(goal_dist_tol_, 0.05);
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop = std::sqrt(2 * max_angular_accel_ * fabs(angle_to_path));
  if (fabs(angular_vel) > max_vel_to_stop) {
    angular_vel = sign * max_vel_to_stop;
  }
}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

bool RegulatedPurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel,
  const double & carrot_dist)
{
  // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
  // odom frame and the carrot_pose is in robot base frame.

  // check current point is OK
  const double current_footprint_cost = footprintCostAt(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation));
  // 车辆足迹已经压在致命格上时，控制器不能把所有指令一律否决：那样会原地冻结，
  // 上层只能反复执行恢复行为（清代价地图/原地旋转/倒车），永远出不来。
  // 这里切换为「脱困模式」：只否决让碰撞继续恶化的预测位姿，允许原地转向和朝外移动。
  const bool already_in_collision =
    current_footprint_cost >= static_cast<double>(LETHAL_OBSTACLE);
  if (already_in_collision) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[RPP当前位姿碰撞] frame=%s x=%.3f y=%.3f yaw=%.3f footprint_cost=%.0f v=%.3f w=%.3f carrot=%.3f；"
      "进入脱困模式，只否决加重碰撞的预测位姿",
      costmap_ros_->getGlobalFrameID().c_str(), robot_pose.pose.position.x,
      robot_pose.pose.position.y, tf2::getYaw(robot_pose.pose.orientation),
      current_footprint_cost, linear_vel, angular_vel, carrot_dist);
  }

  // 脱困方向：从最近的致命格指向车体。footprint 代价在 254 处饱和，
  // 单看代价无法区分「朝障碍里面顶」和「向外退出」，因此再要求预测位移
  // 在该方向上的投影不为负（原地转向投影为 0，始终允许）。
  double escape_dx = 0.0;
  double escape_dy = 0.0;
  if (already_in_collision) {
    unsigned int rmx = 0;
    unsigned int rmy = 0;
    const double rx = robot_pose.pose.position.x;
    const double ry = robot_pose.pose.position.y;
    if (costmap_->worldToMap(rx, ry, rmx, rmy)) {
      const int cells = std::max(
        1, static_cast<int>(std::ceil(0.60 / costmap_->getResolution())));
      double best = std::numeric_limits<double>::infinity();
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          const int mx = static_cast<int>(rmx) + dx;
          const int my = static_cast<int>(rmy) + dy;
          if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
            my >= static_cast<int>(costmap_->getSizeInCellsY())) {continue;}
          if (costmap_->getCost(mx, my) < LETHAL_OBSTACLE) {continue;}
          const double d = std::hypot(dx, dy);
          if (d < best) {
            best = d;
            escape_dx = -dx;
            escape_dy = -dy;
          }
        }
      }
      const double n = std::hypot(escape_dx, escape_dy);
      if (n > 1e-6) {
        escape_dx /= n;
        escape_dy /= n;
      }
    }
  }

  // visualization messages
  nav_msgs::msg::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  double projection_time = 0.0;
  if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
    // rotating to heading at goal or toward path
    // Equation finds the angular distance required for the largest
    // part of the robot radius to move to another costmap cell:
    // theta_min = 2.0 * sin ((res/2) / r_max)
    // via isosceles triangle r_max-r_max-resolution,
    // dividing by angular_velocity gives us a timestep.
    double max_radius = scan_ ? scan_->radius() : costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    projection_time =
      2.0 * sin(((scan_ ? scan_->resolution() : costmap_->getResolution()) / 2) / max_radius) / fabs(angular_vel);
  } else {
    // Normal path tracking
    projection_time = (scan_ ? scan_->resolution() : costmap_->getResolution()) / fabs(linear_vel);
  }
  if (scan_) {
    // Bound motion of the complete body, including combined translation and rotation.
    projection_time = std::min(0.05, scan_->resolution() * 0.5 /
      std::max(1e-6, fabs(linear_vel) + scan_->radius() * fabs(angular_vel)));
  }

  const geometry_msgs::msg::Point & robot_xy = robot_pose.pose.position;
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

  // only forward simulate within time requested
  int i = 1;
  while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_) {
    i++;

    // apply velocity at curr_pose over distance
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    // check if past carrot pose, where no longer a thoughtfully valid command
    if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist) {
      break;
    }

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
      if (already_in_collision &&
        footprintCostAt(curr_pose.x, curr_pose.y, curr_pose.theta) <=
        current_footprint_cost + 1e-6)
      {
        const double move_x = curr_pose.x - robot_xy.x;
        const double move_y = curr_pose.y - robot_xy.y;
        if (std::hypot(move_x, move_y) < 1e-6 ||
          move_x * escape_dx + move_y * escape_dy >= -1e-6)
        {
          // 没有加重碰撞，且位移不朝障碍内部（原地转向、沿边缘滑出）：
          // 属于脱困所需动作，放行。
          continue;
        }
      }
      // 预测碰撞被持续拦截是“顶住障碍”的直接信号，累计后由自救阶段处理。
      ++recovery_collision_cycles_;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[RPP预测碰撞] frame=%s x=%.3f y=%.3f yaw=%.3f t=%.3f v=%.3f w=%.3f carrot=%.3f",
        costmap_ros_->getGlobalFrameID().c_str(), curr_pose.x, curr_pose.y,
        curr_pose.theta, (i - 1) * projection_time, linear_vel, angular_vel, carrot_dist);
      carrot_arc_pub_->publish(arc_pts_msg);
      return true;
    }
  }

  carrot_arc_pub_->publish(arc_pts_msg);

  return false;
}

bool RegulatedPurePursuitController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
{
  if (scan_) return scan_->collision(x, y, theta);
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 30000,
      "The dimensions of the costmap is too small to successfully check for "
      "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
      "increase your costmap size.");
    return false;
  }

  return footprintCostAt(x, y, theta) >= static_cast<double>(LETHAL_OBSTACLE);
}

double RegulatedPurePursuitController::footprintCostAt(
  const double & x,
  const double & y,
  const double & theta)
{
  if (scan_) return 0.0;
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {return 0.0;}
  const double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());
  if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
  {
    return 0.0;
  }
  return footprint_cost;
}

double RegulatedPurePursuitController::costAtPose(const double & x, const double & y)
{
  if (scan_) return 0.0;  // GridMap occupancy is not a Nav2 inflation cost.
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the costmap is too small to fully include your robot's footprint, "
      "thusly the robot cannot proceed further");
    throw nav2_core::PlannerException(
            "RegulatedPurePursuitController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void RegulatedPurePursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path,
  double & linear_vel
) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel;
  double cost_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // limit the linear velocity by proximity to obstacles
  if (use_cost_regulated_linear_velocity_scaling_ &&
    pose_cost != static_cast<double>(NO_INFORMATION) &&
    pose_cost != static_cast<double>(FREE_SPACE))
  {
    const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
      std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

    if (min_distance_to_obstacle < cost_scaling_dist_) {
      cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
    }
  }

  // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  if (speed_diagnostics_enabled_) {
    diagnostic_curvature_velocity_ = curvature_vel;
    diagnostic_cost_velocity_ = cost_vel;
    diagnostic_pre_approach_velocity_ = linear_vel;
  }

  applyApproachVelocityScaling(path, linear_vel);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}

  // 没有正在执行的绕障路径：按原流程更新
  // 正在绕障：检查有向重叠路线，忽略移动锚点；上游不足0.5m时，
  // 使用汇入点后的共同路线补充证据（sameRouteToJoin）。
  // 成功：保留绕障段、汇入点及 avoid_flag_，只替换汇入点之后的路径
  // 匹配失败：按原流程重置，接受新路线
void RegulatedPurePursuitController::doorCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(door_cloud_mutex_);
  door_cloud_ = cloud;
}

void RegulatedPurePursuitController::resetDoorPhase()
{
  door_phase_ = DoorPhase::NONE;
  door_obs_ = DoorObservation{};
  door_obs_time_ = -1.0;
  door_cross_travel_ = 0.0;
  door_cross_target_ = 0.0;
  door_cross_locked_ = false;
  door_corridor_heading_err_ = 0.0;
  door_corridor_lateral_err_ = 0.0;
  // 门区期间 computeVelocityCommands 提前返回，规划器完全不参与，绕障跟踪态
  // 在整段门区被冻结。进门时它是在门西侧规划出来的，出门时车已到门东侧，
  // 若继续沿用会产生"车辆到最近点连线受阻"并让车原地掉头。门区接管运动权
  // 时即判定该规划结果作废，退出后下一拍从当前位姿重新评估全局路径。
  if (avoid_flag_ || !track_path_.poses.empty()) {
    RCLCPP_INFO(
      logger_, "[门区] 清空冻结的绕障跟踪态 avoid=%d track_points=%zu join=%zu",
      avoid_flag_, track_path_.poses.size(), join_idx_);
  }
  avoid_flag_ = false;
  track_path_.poses.clear();
  global_path_tail_start_idx_ = 0;
  join_idx_ = 0;
}

double RegulatedPurePursuitController::rearClearDistance(
  const geometry_msgs::msg::PoseStamped & pose, double max_dist)
{
  // scan 后端下 costAtPose 恒为 0（代价由 GridMap 判定），无法做车尾校验，
  // 此时不做限制，保持原有行为。
  if (scan_) {return max_dist;}
  const double yaw = tf2::getYaw(pose.pose.orientation);
  // 从车尾外沿起算：只用单点代价探测车尾走廊，不用整车 footprint。
  // 顶住前方障碍时整车 footprint 的前端本就贴着障碍，若按整车判定会把
  // “向后远离障碍”误判成“后退会撞上”，导致永远不敢后退。
  const double rear_x = pose.pose.position.x - 0.23 * std::cos(yaw);
  const double rear_y = pose.pose.position.y - 0.23 * std::sin(yaw);
  const double step = 0.05;
  for (double d = step; d <= max_dist + 1e-9; d += step) {
    const double x = rear_x - d * std::cos(yaw);
    const double y = rear_y - d * std::sin(yaw);
    // 253 起即为内切障碍：车尾一旦贴上就立即停，绝不把车退进墙里。
    if (costAtPose(x, y) >= 253.0) {return d - step;}
  }
  return max_dist;
}

double RegulatedPurePursuitController::frontClearDistance(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // scan 后端没有 Nav2 代价地图（costAtPose 恒为 0），保持原有行为。
  if (scan_) {return std::numeric_limits<double>::infinity();}

  // 只探测车宽范围内的前向格：窄门两侧门框在横向 0.35 m 处，会被半宽过滤掉，
  // 因此门洞里不会误判为有障碍，而真正挡在车头前的箱体/墙一定落在探测带内。
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cs = std::cos(yaw);
  const double sn = std::sin(yaw);
  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;
  const double front_x = 0.275;  // footprint 前缘相对 base_link
  const double step = 0.05;
  // 只探到开始降速的距离即可：更远的障碍不影响停车/降速判定。
  for (double d = step; d <= front_slow_clearance_; d += step) {
    for (const double lateral : {0.0, -front_stop_half_width_, front_stop_half_width_}) {
      const double x = rx + (front_x + d) * cs - lateral * sn;
      const double y = ry + (front_x + d) * sn + lateral * cs;
      // 253 是内切膨胀带（窄道里整片地面都是），254 才是坡度层标记的真障碍。
      if (costAtPose(x, y) >= 254.0) {return d;}
    }
  }
  return std::numeric_limits<double>::infinity();
}

double RegulatedPurePursuitController::frontClearanceScale(double front_clear) const
{
  if (front_clear >= front_slow_clearance_) {return 1.0;}
  if (front_clear <= front_stop_clearance_) {return 0.0;}
  return (front_clear - front_stop_clearance_) /
         (front_slow_clearance_ - front_stop_clearance_);
}

bool RegulatedPurePursuitController::frontStopActive(
  const geometry_msgs::msg::PoseStamped & pose, double front_clear)
{
  // 净空低于停车阈值，或降速系数已低到无法产生有效位移时，都按“受阻”处理：
  // 否则车会以几毫米每秒的速度贴着障碍爬行，既走不动也不触发自救。
  const bool blocked = front_clear < front_stop_clearance_ ||
                       frontClearanceScale(front_clear) < 0.10;
  if (!blocked) {
    front_block_cycles_ = 0;
    return false;
  }
  if (++front_block_cycles_ < std::max(1, recovery_front_block_cycles_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[安全] 车头前方 %.2f m 有障碍（要求 %.2f m），停止前向", front_clear, front_stop_clearance_);
    return true;
  }
  front_block_cycles_ = 0;
  RCLCPP_WARN(
    logger_, "[安全] 车头净空 %.2f m 持续不足（要求 %.2f m），触发层1后退自救",
    front_clear, front_stop_clearance_);
  // 停住时 last_linear_command_ 为 0，“有速度无位移”判据不会触发，
  // 因此前向受阻必须自己触发自救，否则车会一直停在障碍前不重规划。
  if (startRecovery("车头净空持续不足", pose)) {return true;}
  RCLCPP_WARN(
    logger_, "[恢复] 车头持续受阻且自救次数已用尽，交回全局路径重新规划");
  throw nav2_core::PlannerException(
          "front blocked: clearance below limit after recovery exhausted");
}

bool RegulatedPurePursuitController::startRecovery(
  const char * reason, const geometry_msgs::msg::PoseStamped & pose)
{
  if (recovery_cycles_ >= recovery_max_cycles_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[恢复] 已达最大自救次数(%d)，不再后退：%s", recovery_max_cycles_, reason);
    return false;
  }

  const double clear = rearClearDistance(
    pose, recovery_backup_distance_ + recovery_rear_clearance_);
  const double usable = clear - recovery_rear_clearance_;
  if (usable < 0.05) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[恢复] 车尾 %.2f m 内受阻，不后退（%s）", clear, reason);
    return false;
  }

  recovery_phase_ = RecoveryPhase::BACKUP;
  if (door_phase_ != DoorPhase::NONE) {
    // 后退会破坏门区已建立的对准，门区阶段整体复位；退出后退后按已知门位重新 PRE_ALIGN。
    RCLCPP_WARN(
      logger_, "[恢复] 门区阶段卡死(phase=%d)，复位门区后先后退",
      static_cast<int>(door_phase_));
    resetDoorPhase();
  }
  recovery_travel_ = 0.0;
  recovery_target_ = std::min(recovery_backup_distance_, usable);
  recovery_lock_yaw_ = tf2::getYaw(pose.pose.orientation);
  recovery_start_x_ = pose.pose.position.x;
  recovery_start_y_ = pose.pose.position.y;
  recovery_start_time_ = clock_->now().seconds();
  recovery_last_progress_travel_ = 0.0;
  recovery_progress_time_ = recovery_start_time_;
  ++recovery_cycles_;
  recovery_last_trigger_time_ = clock_->now().seconds();
  recovery_no_path_cycles_ = 0;
  recovery_anchor_valid_ = false;
  RCLCPP_WARN(
    logger_, "[恢复] 触发后退：reason=%s 后退 %.2f m（车尾净空 %.2f m）cycle=%d",
    reason, recovery_target_, clear, recovery_cycles_);
  return true;
}

void RegulatedPurePursuitController::finishRecovery()
{
  recovery_phase_ = RecoveryPhase::NONE;
  recovery_travel_ = 0.0;
  recovery_target_ = 0.0;
  recovery_start_time_ = -1.0;
  // 后退前的局部规划结果已失效，清空后由下一拍从新位姿重新评估。
  avoid_flag_ = false;
  track_path_.poses.clear();
  global_path_tail_start_idx_ = 0;
  join_idx_ = 0;
  RCLCPP_INFO(logger_, "[恢复] 后退结束，清空绕障态并重新规划");
}

bool RegulatedPurePursuitController::updateRecovery(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*speed*/,
  geometry_msgs::msg::TwistStamped & cmd)
{
  const double now = clock_->now().seconds();

  // ---- BACKUP：沿锁存航向直线后退，每拍校验车尾走廊 ----
  if (recovery_phase_ == RecoveryPhase::BACKUP) {
    const double clear = rearClearDistance(
      pose, recovery_backup_distance_ + recovery_rear_clearance_);
    if (clear <= recovery_rear_clearance_) {
      RCLCPP_WARN(logger_, "[恢复] 后退中止：车尾仅剩 %.2f m 净空", clear);
      finishRecovery();
      recovery_collision_cycles_ = 0;
      return false;
    }
    // 后退进度只统计沿锁存航向朝车尾方向的分量：顶住障碍后车可能沿障碍面
    // 侧滑或摆动，位移大小照样增长，只有"真正往后退了多少"才有意义。
    const double dx = pose.pose.position.x - recovery_start_x_;
    const double dy = pose.pose.position.y - recovery_start_y_;
    recovery_travel_ = std::max(0.0, -(dx * std::cos(recovery_lock_yaw_) +
      dy * std::sin(recovery_lock_yaw_)));
    const double yaw_err = std::remainder(
      recovery_lock_yaw_ - tf2::getYaw(pose.pose.orientation), 2.0 * 3.14159265358979323846);
    cmd.header = pose.header;
    cmd.twist.linear.x = -recovery_backup_speed_;
    cmd.twist.angular.z = std::clamp(1.5 * yaw_err, -0.25, 0.25);
    last_linear_command_ = cmd.twist.linear.x;
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 500,
      "[恢复] BACKUP travel=%.2f/%.2f v=%.2f w=%.2f clear=%.2f",
      recovery_travel_, recovery_target_, cmd.twist.linear.x, cmd.twist.angular.z, clear);
    // 后退受阻兜底：本仿真 Livox 只发前半球点云（front_half_only=true），
    // 车尾净空恒为上限，无法靠代价地图发现后方障碍，只能用"退不动"来判断。
    // 判据取"位移不再增长"而不是"总位移接近 0"，这样退到一半被顶住也能停。
    if (recovery_travel_ >= recovery_last_progress_travel_ + 0.01) {
      recovery_last_progress_travel_ = recovery_travel_;
      recovery_progress_time_ = now;
    } else if (now - recovery_progress_time_ >= 0.6) {
      RCLCPP_WARN(
        logger_, "[恢复] 后退受阻：%.1f s 内位移无增长（当前 %.3f m），中止后退（后方可能有障碍）",
        now - recovery_start_time_, recovery_travel_);
      finishRecovery();
      recovery_collision_cycles_ = 0;
      return false;
    }
    if (recovery_travel_ >= recovery_target_) {finishRecovery();}
    recovery_collision_cycles_ = 0;
    return true;
  }

  // ---- 触发判定 ----
  const char * reason = nullptr;
  if (recovery_collision_cycles_ >= std::max(1, recovery_collision_cycles_threshold_)) {
    reason = "预测碰撞持续拦截";
  } else if (recovery_no_path_cycles_ >= std::max(1, recovery_no_path_cycles_threshold_)) {
    reason = "局部规划连续无解";
  } else {
    const double v_cmd = std::abs(last_linear_command_);
    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    if (v_cmd >= 0.05) {
      if (!recovery_anchor_valid_) {
        recovery_anchor_x_ = px;
        recovery_anchor_y_ = py;
        recovery_anchor_time_ = now;
        recovery_anchor_valid_ = true;
      } else if (now - recovery_anchor_time_ >= recovery_stuck_time_) {
        if (std::hypot(px - recovery_anchor_x_, py - recovery_anchor_y_) <
          recovery_stuck_translation_)
        {
          reason = "有速度指令但无位移";
        } else {
          recovery_anchor_x_ = px;
          recovery_anchor_y_ = py;
          recovery_anchor_time_ = now;
        }
      }
    } else {
      recovery_anchor_valid_ = false;
    }
  }
  recovery_collision_cycles_ = 0;

  if (reason == nullptr) {return false;}

  if (recovery_last_trigger_time_ > 0.0 &&
    now - recovery_last_trigger_time_ > recovery_retrigger_window_)
  {
    recovery_cycles_ = 0;
  }
  if (!startRecovery(reason, pose)) {
    recovery_no_path_cycles_ = 0;
    recovery_anchor_valid_ = false;
    return false;
  }

  cmd.header = pose.header;
  cmd.twist.linear.x = -recovery_backup_speed_;
  cmd.twist.angular.z = 0.0;
  last_linear_command_ = cmd.twist.linear.x;
  RCLCPP_INFO(
    logger_, "[恢复] BACKUP travel=%.2f/%.2f v=%.2f",
    recovery_travel_, recovery_target_, cmd.twist.linear.x);
  return true;
}

bool RegulatedPurePursuitController::detectDoor(
  const geometry_msgs::msg::PoseStamped & pose, DoorObservation & obs) const
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(door_cloud_mutex_);
    cloud = door_cloud_;
  }
  if (!cloud || cloud->width == 0 || cloud->height == 0 || cloud->header.frame_id.empty()) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "[门区诊断] 无点云或 frame_id 为空");
    return false;
  }
  const rclcpp::Time stamp(cloud->header.stamp);
  if (stamp.nanoseconds() == 0) {return false;}
  const double age = (clock_->now() - stamp).seconds();
  if (age < -0.10 || age > door_cloud_timeout_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000, "[门区诊断] 点云超时 age=%.2f frame=%s",
      age, cloud->header.frame_id.c_str());
    return false;
  }

  sensor_msgs::msg::PointCloud2 global_cloud;
  try {
    const auto transform = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), cloud->header.frame_id, stamp,
      tf2::Duration(std::chrono::milliseconds(50)));
    tf2::doTransform(*cloud, global_cloud, transform);
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000, "[门区诊断] TF 失败 %s->%s: %s",
      cloud->header.frame_id.c_str(), costmap_ros_->getGlobalFrameID().c_str(), e.what());
    return false;
  }

  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cs = std::cos(yaw);
  const double sn = std::sin(yaw);

  // 横向按 0.05 m 分桶；只统计门柱高度带，滤掉地面与车体自身回波。
  const double bin = 0.05;
  const int half_bins = static_cast<int>(std::ceil(door_detect_half_width_ / bin));
  const int bins = 2 * half_bins + 1;
  std::vector<int> count(bins, 0);
  std::vector<double> sum_x(bins, 0.0);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(global_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(global_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(global_cloud, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const double gx = *it_x;
    const double gy = *it_y;
    const double gz = *it_z;
    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {continue;}
    if (gz < door_min_height_ || gz > door_max_height_) {continue;}
    const double dx = (gx - rx) * cs + (gy - ry) * sn;
    const double dy = -(gx - rx) * sn + (gy - ry) * cs;
    if (dx < 0.25 || dx > door_detect_range_) {continue;}
    if (std::abs(dy) > door_detect_half_width_) {continue;}
    const int k = static_cast<int>(std::lround(dy / bin));
    if (k < -half_bins || k > half_bins) {continue;}
    count[k + half_bins] += 1;
    sum_x[k + half_bins] += dx;
  }

  // 从车体中心线向两侧找最近的成片门柱，其内缘即为门洞边界。
  bool has_right = false;
  bool has_left = false;
  int right_k = 0;
  int left_k = 0;
  for (int k = -1; k >= -half_bins; --k) {
    if (count[k + half_bins] >= door_min_points_) {right_k = k; has_right = true; break;}
  }
  for (int k = 1; k <= half_bins; ++k) {
    if (count[k + half_bins] >= door_min_points_) {left_k = k; has_left = true; break;}
  }
  if (!has_left || !has_right) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000, "[门区诊断] 只找到单侧门柱 left=%d right=%d", has_left, has_right);
    return false;
  }

  const double right_inner = right_k * bin;
  const double left_inner = left_k * bin;
  const double width = left_inner - right_inner;
  if (width < door_min_width_ || width > door_max_width_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000, "[门区诊断] 净宽超限 width=%.2f (%.2f..%.2f)",
      width, door_min_width_, door_max_width_);
    return false;
  }

  double x_sum = 0.0;
  int x_count = 0;
  for (int k = -half_bins; k <= half_bins; ++k) {
    if (count[k + half_bins] >= door_min_points_) {
      x_sum += sum_x[k + half_bins];
      // 必须是点数，而不是被占用的桶数，否则门距会被桶和放大。
      x_count += count[k + half_bins];
    }
  }
  if (x_count == 0) {return false;}

  obs.valid = true;
  obs.x = x_sum / x_count;
  obs.y = 0.5 * (left_inner + right_inner);
  obs.width = width;
  obs.left_inner = left_inner;
  obs.right_inner = right_inner;
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000, "[门区诊断] 门框 x=%.2f y=%.2f width=%.2f",
    obs.x, obs.y, obs.width);
  return true;
}

bool RegulatedPurePursuitController::detectKnownDoor(
  const geometry_msgs::msg::PoseStamped & pose,
  double & corridor_heading, double & lateral_err, double & along_track) const
{
  if (door_positions_.empty() || (door_positions_.size() % 2) != 0) {return false;}
  const auto & route = global_plan_.poses.empty() ? track_path_ : global_plan_;
  if (route.poses.size() < 2) {return false;}

  const double rx = pose.pose.position.x;
  const double ry = pose.pose.position.y;

  // 预计算逐段长度与累计弧长，供投影与沿程距离用。
  const size_t n_seg = route.poses.size() - 1;
  std::vector<double> seg_len(n_seg, 0.0);
  std::vector<double> cum(route.poses.size(), 0.0);
  for (size_t i = 0; i < n_seg; ++i) {
    seg_len[i] = std::hypot(
      route.poses[i + 1].pose.position.x - route.poses[i].pose.position.x,
      route.poses[i + 1].pose.position.y - route.poses[i].pose.position.y);
    cum[i + 1] = cum[i] + seg_len[i];
  }

  // 把 (px,py) 投影到折线，返回最近距离并输出所在段与投影点弧长。
  auto project = [&](double px, double py, size_t & seg_idx, double & s_along) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n_seg; ++i) {
      const auto & a = route.poses[i].pose.position;
      const auto & b = route.poses[i + 1].pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len2 = dx * dx + dy * dy;
      const double t = len2 > 1e-12 ?
        std::clamp(((px - a.x) * dx + (py - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
      const double d = std::hypot(px - a.x - t * dx, py - a.y - t * dy);
      if (d < best) {best = d; seg_idx = i; s_along = cum[i] + t * seg_len[i];}
    }
    return best;
  };

  for (size_t k = 0; k + 1 < door_positions_.size(); k += 2) {
    const double gx = door_positions_[k];
    const double gy = door_positions_[k + 1];

    size_t door_seg = 0;
    double door_s = 0.0;
    // 门心必须紧贴当前路线，否则视为路旁无关几何。
    if (project(gx, gy, door_seg, door_s) > door_route_tolerance_) {continue;}

    const auto & a = route.poses[door_seg].pose.position;
    const auto & b = route.poses[door_seg + 1].pose.position;
    corridor_heading = std::atan2(b.y - a.y, b.x - a.x);

    size_t car_seg = 0;
    double car_s = 0.0;
    project(rx, ry, car_seg, car_s);
    // 沿程：车必须在门投影点之前（尚未越过门），且在触发距离内。
    along_track = door_s - car_s;
    if (along_track < 0.0 || along_track > door_enter_distance_) {continue;}

    // 车到走廊轴线的有符号横向距离（左正）。轴线过门心、方向 corridor_heading。
    lateral_err = -(rx - gx) * std::sin(corridor_heading) +
      (ry - gy) * std::cos(corridor_heading);
    return true;
  }
  return false;
}

void RegulatedPurePursuitController::updateDoorPhase(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed)
{
  const double now = clock_->now().seconds();
  double dt = 0.0;
  if (door_last_time_ > 0.0) {dt = std::clamp(now - door_last_time_, 0.0, 0.2);}
  door_last_time_ = now;

  DoorObservation obs;
  const bool seen = detectDoor(pose, obs);
  if (seen) {
    door_obs_ = obs;
    door_obs_time_ = now;
  }
  // 允许短时点云断流，超过 0.25 s 才认为门框丢失。
  const bool fresh = seen || (door_obs_time_ > 0.0 && now - door_obs_time_ <= 0.25);
  const double bearing = fresh ?
    std::atan2(door_obs_.y, std::max(door_obs_.x, 0.05)) : 0.0;

  // 门区伺服一旦进入 CROSS 会锁存航向盲目前进，必须确认观测到的门确实位于
  // 当前路线上。否则路旁任意两团几何（门柱+箱子、墙角等）都可能被误判成门，
  // 车会沿错误方向直行撞进障碍。
  const auto door_on_route = [&]() {
    const auto & route = global_plan_.poses.empty() ? track_path_ : global_plan_;
    if (route.poses.empty()) {return true;}
    const double yaw_r = tf2::getYaw(pose.pose.orientation);
    const double gx = pose.pose.position.x + door_obs_.x * std::cos(yaw_r) -
      door_obs_.y * std::sin(yaw_r);
    const double gy = pose.pose.position.y + door_obs_.x * std::sin(yaw_r) +
      door_obs_.y * std::cos(yaw_r);
    double best = std::numeric_limits<double>::infinity();
    if (route.poses.size() == 1) {
      best = std::hypot(gx - route.poses[0].pose.position.x, gy - route.poses[0].pose.position.y);
    }
    for (size_t i = 0; i + 1 < route.poses.size(); ++i) {
      const auto & a = route.poses[i].pose.position;
      const auto & b = route.poses[i + 1].pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len2 = dx * dx + dy * dy;
      const double t = len2 > 1e-12 ? std::clamp(
        ((gx - a.x) * dx + (gy - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
      best = std::min(best, std::hypot(gx - a.x - t * dx, gy - a.y - t * dy));
    }
    if (best > door_route_tolerance_) {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 2000,
        "[门区诊断] 门心 (%.2f,%.2f) 距路线 %.2f m，超过容差 %.2f m，不启用门区伺服",
        gx, gy, best, door_route_tolerance_);
      return false;
    }
    return true;
  };

  // 已知门走廊：即便 LiDAR 尚未看到门柱，也能据固定门位启动 PRE_ALIGN，
  // 让车先转到走廊方向，再交给既有 ALIGN/CROSS 精对准与穿越。
  double corridor_heading = 0.0;
  double corridor_lateral = 0.0;
  double corridor_along = 0.0;
  const bool known = detectKnownDoor(pose, corridor_heading, corridor_lateral, corridor_along);
  if (known) {
    const double yaw_now = tf2::getYaw(pose.pose.orientation);
    const double he = corridor_heading - yaw_now;
    door_corridor_heading_err_ = std::atan2(std::sin(he), std::cos(he));
    door_corridor_lateral_err_ = corridor_lateral;
  }

  if (door_phase_ == DoorPhase::CROSS) {
    door_cross_travel_ += std::max(0.0, speed.linear.x) * dt;
  }

  switch (door_phase_) {
    case DoorPhase::NONE:
      if (fresh && door_obs_.x > 0.0 && door_obs_.x <= door_enter_distance_ &&
        std::abs(door_obs_.y) < door_detect_half_width_ && door_on_route())
      {
        door_phase_ = DoorPhase::ALIGN;
        RCLCPP_INFO(
          logger_, "[门区] NONE->ALIGN x=%.2f y=%.2f width=%.2f",
          door_obs_.x, door_obs_.y, door_obs_.width);
      } else if (known) {
        door_phase_ = DoorPhase::PRE_ALIGN;
        RCLCPP_INFO(
          logger_, "[门区] NONE->PRE_ALIGN he=%.2f lat=%.2f along=%.2f",
          door_corridor_heading_err_, door_corridor_lateral_err_, corridor_along);
      }
      break;
    case DoorPhase::PRE_ALIGN:
      // 门走廊固定：先在原地转到走廊方向并收敛横向偏差；
      // 一旦 LiDAR 正对门框并认定其为路线上的门，交还既有 ALIGN 精对准。
      if (!known) {
        resetDoorPhase();
        RCLCPP_WARN(logger_, "[门区] 已知门不再位于路线前方，退出 PRE_ALIGN");
      } else if (seen && door_obs_.x > 0.0 &&
        std::abs(door_obs_.y) < door_detect_half_width_ && door_on_route())
      {
        door_phase_ = DoorPhase::ALIGN;
        RCLCPP_INFO(
          logger_, "[门区] PRE_ALIGN->ALIGN x=%.2f y=%.2f width=%.2f",
          door_obs_.x, door_obs_.y, door_obs_.width);
      }
      break;
    case DoorPhase::ALIGN:
      if (!fresh) {
        resetDoorPhase();
        RCLCPP_WARN(logger_, "[门区] 门框丢失，退出 ALIGN");
      } else if (std::abs(bearing) < door_align_angle_ && std::abs(door_obs_.y) < 0.10) {
        door_phase_ = DoorPhase::CROSS;
        door_cross_travel_ = 0.0;
        door_cross_target_ = door_obs_.x + door_exit_margin_;
        RCLCPP_INFO(
          logger_, "[门区] ALIGN->CROSS y=%.2f bearing=%.3f target=%.2f",
          door_obs_.y, bearing, door_cross_target_);
      }
      break;
    case DoorPhase::CROSS:
      // 门柱内缘近距离会擦过激光视线：测宽收缩、单侧丢失都属正常现象。
      // 此时保持进入 CROSS 时的航向直行，靠累计行程走完门洞，不再回退常规规划。
      if (!door_cross_locked_ &&
        (!fresh || door_obs_.x < door_hold_distance_))
      {
        door_cross_locked_ = true;
        RCLCPP_INFO(
          logger_, "[门区] 锁存直行 x=%.2f y=%.2f width=%.2f fresh=%d",
          door_obs_.x, door_obs_.y, door_obs_.width, static_cast<int>(fresh));
      }
      if (door_cross_travel_ >= door_cross_target_) {
        door_phase_ = DoorPhase::EXIT;
        door_exit_deadline_ = now + door_exit_hold_;
        RCLCPP_INFO(
          logger_, "[门区] CROSS->EXIT 已前进 %.2f m", door_cross_travel_);
      }
      break;
    case DoorPhase::EXIT:
      if (now >= door_exit_deadline_) {
        resetDoorPhase();
        RCLCPP_INFO(logger_, "[门区] EXIT->NONE，恢复 Pure Pursuit");
      }
      break;
  }
}

void RegulatedPurePursuitController::computeDoorCommand(
  const DoorObservation & obs, double & linear_vel, double & angular_vel) const
{
  linear_vel = 0.0;
  angular_vel = 0.0;
  const double bearing = std::atan2(obs.y, std::max(obs.x, 0.05));

  switch (door_phase_) {
    case DoorPhase::PRE_ALIGN: {
      // 依据已知门走廊对准：航向偏差大时原地转向，对准后低速前进并收敛横向。
      const double he = door_corridor_heading_err_;
      const double lat = door_corridor_lateral_err_;
      if (std::abs(he) > 0.15) {
        angular_vel = std::clamp(2.0 * he, -door_max_angular_, door_max_angular_);
        if (std::abs(angular_vel) < 0.05) {
          angular_vel = std::copysign(0.05, he);
        }
        linear_vel = 0.0;
      } else {
        linear_vel = std::min(door_cross_speed_, 0.15);
        angular_vel = std::clamp(
          door_lateral_gain_ * (-lat), -door_max_angular_, door_max_angular_);
      }
      break;
    }
    case DoorPhase::ALIGN:
      // 偏差大时原地转向；对准后以低速蠕行，继续用门心方向修正横向偏差。
      if (std::abs(bearing) > 0.35) {
        angular_vel = (bearing > 0.0 ? 1.0 : -1.0) * door_align_rate_;
      } else {
        linear_vel = std::min(door_cross_speed_, 0.15);
        angular_vel = std::clamp(
          door_lateral_gain_ * bearing, -door_max_angular_, door_max_angular_);
      }
      break;
    case DoorPhase::CROSS: {
      linear_vel = door_cross_speed_;
      if (door_cross_locked_) {
        // 已贴近门框：门柱测距不可用，保持进入时的航向直行穿门。
        angular_vel = 0.0;
        break;
      }
      // 门框测距伺服：近距离直接用左右门柱内缘的均值作横向误差。
      double y_err = obs.y;
      if (std::abs(obs.x) < 0.60) {y_err = 0.5 * (obs.left_inner + obs.right_inner);}
      angular_vel = std::clamp(
        door_lateral_gain_ * std::atan2(y_err, std::max(obs.x, 0.15)),
        -door_max_angular_, door_max_angular_);
      break;
    }
    case DoorPhase::EXIT:
      // 整车完全出门前保持严格直行，避免后段被门柱刮蹭。
      linear_vel = door_cross_speed_;
      angular_vel = 0.0;
      break;
    case DoorPhase::NONE:
      break;
  }
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> guard(mutex_);
  ++diagnostic_plan_sequence_;
  if (speed_diagnostics_enabled_) {
    RCLCPP_INFO(logger_,
      "[speed_diag/setPlan] seq=%llu avoid=%d points=%zu previous_command=%.4f (not reset here)",
      diagnostic_plan_sequence_, avoid_flag_, path.poses.size(), last_linear_command_);
  }
  if (!scan_ && avoid_flag_ && !track_path_.poses.empty() &&
    path.header.frame_id == global_plan_.header.frame_id &&
    track_path_.header.frame_id == path.header.frame_id)
  {
    // 没有全局尾段时join_idx_等于路径长度，此时实际汇入点是最后一个点。
    const size_t connection_index = std::min(join_idx_, track_path_.poses.size() - 1);
    size_t next = 0;
    if (sameRouteToJoin(global_plan_, path,
      track_path_.poses[connection_index].pose.position, next))
    {
      track_path_.poses.resize(connection_index + 1);
      track_path_.poses.insert(track_path_.poses.end(), path.poses.begin() + next, path.poses.end());
      track_path_.header = path.header;
      join_idx_ = connection_index;
      global_path_tail_start_idx_ = next;
      global_plan_ = path;
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
        "[绕障诊断/路径更新保留] backend=COSTMAP join=%zu tail_start=%zu track_points=%zu；同路线刷新，保持绕障段和汇入点",
        join_idx_, next, track_path_.poses.size());
      return;
    }
  }
  if (scan_ && avoid_flag_ && join_idx_ < track_path_.poses.size() &&
    path.header.frame_id == global_plan_.header.frame_id && path.poses.size() > 1)
  {
    const auto connection = track_path_.poses[join_idx_];
    size_t match = findClosestPlanIndex(path, connection);
    bool same_route = hypot(path.poses[match].pose.position.x - connection.pose.position.x,
      path.poses[match].pose.position.y - connection.pose.position.y) < 0.2;
    // Validate the overlapping upstream route, excluding its moving robot anchor.
    for (size_t i = 1; same_route && i <= match; ++i) {
      double best = std::numeric_limits<double>::infinity();
      const auto &p = path.poses[i].pose.position;
      for (size_t j = 1; j < global_plan_.poses.size(); ++j) {
        const auto &a = global_plan_.poses[j-1].pose.position;
        const auto &b = global_plan_.poses[j].pose.position;
        double dx=b.x-a.x, dy=b.y-a.y;
        double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/std::max(1e-12,dx*dx+dy*dy),0.0,1.0);
        best=std::min(best,hypot(p.x-a.x-t*dx,p.y-a.y-t*dy));
      }
      same_route = best < 0.2;
    }
    if (same_route) {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 2000,
        "[绕障诊断/路径更新保留] backend=SCAN join=%zu match=%zu old_points=%zu new_points=%zu",
        join_idx_, match, global_plan_.poses.size(), path.poses.size());
      track_path_.poses.resize(join_idx_ + 1);
      track_path_.poses.insert(track_path_.poses.end(), path.poses.begin()+match+1,path.poses.end());
      global_plan_ = path;
      return;
    }
  }
  // 状态清空事件逐次记录，避免限频掩盖提前退出绕障的直接原因。
  if (avoid_flag_ || !track_path_.poses.empty()) {
    RCLCPP_WARN(
      logger_,
      "[绕障诊断/路径更新重置] backend=%s avoid=%d track_points=%zu join=%zu "
      "old_points=%zu new_points=%zu old_frame=%s new_frame=%s；setPlan清空绕障状态",
      scan_ ? "SCAN" : "COSTMAP", avoid_flag_, track_path_.poses.size(), join_idx_,
      global_plan_.poses.size(), path.poses.size(), global_plan_.header.frame_id.c_str(),
      path.header.frame_id.c_str());
  }
  global_plan_ = path;
  avoid_flag_ = false;
  track_path_.poses.clear();
  global_path_tail_start_idx_ = 0;
  join_idx_ = 0;
  // 新路径意味着上层已重规划，正在进行的后退立即中止，交由新路径决定走向。
  if (recovery_phase_ == RecoveryPhase::BACKUP) {
    RCLCPP_INFO(logger_, "[恢复] 新路径下发，中止后退");
  }
  recovery_phase_ = RecoveryPhase::NONE;
  recovery_travel_ = 0.0;
  recovery_target_ = 0.0;
  recovery_start_time_ = -1.0;
  recovery_anchor_valid_ = false;
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  const double previous_desired = desired_linear_vel_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
  if (speed_diagnostics_enabled_) {
    RCLCPP_INFO(logger_,
      "[speed_diag/speedLimit] input=%.4f percentage=%d desired_before=%.4f desired_after=%.4f",
      speed_limit, percentage, previous_desired, desired_linear_vel_);
  }
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (local_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(local_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    local_plan_.poses.begin(), local_plan_.poses.end(), max_robot_pose_search_dist_);

  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    local_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, local_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & local_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = local_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = local_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  local_plan_.poses.erase(begin(local_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

bool RegulatedPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double RegulatedPurePursuitController::getCostmapMaxExtent() const
{
  if (scan_) return scan_->horizon();
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

size_t RegulatedPurePursuitController::findClosestPlanIndex(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot) const
{
  if (path.poses.empty()) {
    return 0;
  }
  geometry_msgs::msg::PoseStamped robot_plan;
  if (robot.header.frame_id == path.header.frame_id) {
    robot_plan = robot;
  } else if (!transformPose(path.header.frame_id, robot, robot_plan)) {
    return 0;
  }
  const double rx = robot_plan.pose.position.x;
  const double ry = robot_plan.pose.position.y;
  size_t best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - rx;
    const double dy = path.poses[i].pose.position.y - ry;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

bool RegulatedPurePursuitController::isPolylineSegmentFree(
  double x0, double y0, double x1, double y1) const
{
  if (scan_) return scan_->segmentFree(x0,y0,x1,y1);
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len = std::hypot(dx, dy);
  const double res = std::max(polyline_collision_check_resolution_, 1e-4);
  const int n = std::max(1, static_cast<int>(std::ceil(len / res)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const double x = x0 + t * dx;
    const double y = y0 + t * dy;
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return false;
    }
    if (costmap_->getCost(mx, my) >= INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
  }
  return true;
}

bool RegulatedPurePursuitController::isPathPolylineClear(
  const nav_msgs::msg::Path & path, size_t start_idx, size_t end_idx) const
{
  if (path.poses.empty() || end_idx >= path.poses.size() || end_idx < start_idx) {
    return false;
  }
  if (start_idx == end_idx) {
    return true;
  }
  for (size_t i = start_idx; i < end_idx; ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    if (!isPolylineSegmentFree(a.x, a.y, b.x, b.y)) {
      return false;
    }
  }
  return true;
}

bool RegulatedPurePursuitController::isGlobalPlanPolylineClear(
  size_t start_idx, size_t end_idx) const
{
  return isPathPolylineClear(global_plan_, start_idx, end_idx);
}

nav_msgs::msg::Path RegulatedPurePursuitController::densePathSlice(
  const nav_msgs::msg::Path & path, size_t start_idx, size_t end_idx, double step_m) const
{
  nav_msgs::msg::Path out;
  out.header = path.header;
  if (path.poses.empty() || start_idx > end_idx || end_idx >= path.poses.size()) {
    return out;
  }

  const double step = std::max(step_m, 0.02);

  if (start_idx == end_idx) {
    geometry_msgs::msg::PoseStamped p = path.poses[start_idx];
    p.header = out.header;
    out.poses.push_back(p);
    return out;
  }

  double total_len = 0.0;
  for (size_t j = start_idx; j < end_idx; ++j) {
    const auto & a = path.poses[j].pose.position;
    const auto & b = path.poses[j + 1].pose.position;
    total_len += hypot(b.x - a.x, b.y - a.y);
  }

  if (total_len < 1e-6) {
    geometry_msgs::msg::PoseStamped p = path.poses[start_idx];
    p.header = out.header;
    out.poses.push_back(p);
    return out;
  }

  auto append_pose = [&out](double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = out.header;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    ps.pose.orientation = tf2::toMsg(q);
    out.poses.push_back(ps);
  };

  for (double d = 0.0; ; ) {
    double acc = 0.0;
    size_t k = start_idx;
    double x = path.poses[end_idx].pose.position.x;
    double y = path.poses[end_idx].pose.position.y;
    double yaw = tf2::getYaw(path.poses[end_idx].pose.orientation);
    bool found_seg = false;
    while (k < end_idx) {
      const auto & a = path.poses[k].pose.position;
      const auto & b = path.poses[k + 1].pose.position;
      const double seg_len = hypot(b.x - a.x, b.y - a.y);
      if (acc + seg_len >= d - 1e-9) {
        const double t = seg_len > 1e-9 ? (d - acc) / seg_len : 0.0;
        const double tt = std::min(1.0, std::max(0.0, t));
        x = a.x + tt * (b.x - a.x);
        y = a.y + tt * (b.y - a.y);
        yaw = std::atan2(b.y - a.y, b.x - a.x);
        found_seg = true;
        break;
      }
      acc += seg_len;
      ++k;
    }
    if (!found_seg) {
      const auto & pe = path.poses[end_idx].pose;
      x = pe.position.x;
      y = pe.position.y;
      yaw = tf2::getYaw(pe.orientation);
    }
    append_pose(x, y, yaw);
    if (d >= total_len - 1e-9) {
      break;
    }
    d = std::min(d + step, total_len);
  }

  return out;
}

nav_msgs::msg::Path RegulatedPurePursuitController::denseGlobalPlanSlice(
  size_t start_idx, size_t end_idx, double step_m) const
{
  return densePathSlice(global_plan_, start_idx, end_idx, step_m);
}

nav_msgs::msg::Path RegulatedPurePursuitController::getScanLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed)
{
  if (global_plan_.poses.size() < 2) throw std::runtime_error("SCAN reference path is empty");
  if (global_plan_.header.frame_id != costmap_ros_->getGlobalFrameID())
    throw std::runtime_error("SCAN reference must use the configured global frame");
  geometry_msgs::msg::PoseStamped robot;
  if (!transformPose(global_plan_.header.frame_id, pose, robot))
    throw std::runtime_error("Cannot transform SCAN start pose");
  const size_t closest = findClosestPlanIndex(global_plan_, robot);
  size_t goal_index = closest;
  bool found = false;
  double arc = hypot(global_plan_.poses[closest].pose.position.x - robot.pose.position.x,
    global_plan_.poses[closest].pose.position.y - robot.pose.position.y);
  for (size_t i = closest; i < global_plan_.poses.size(); ++i) {
    const auto &p = global_plan_.poses[i].pose.position;
    if (i > closest) {
      const auto &prev = global_plan_.poses[i-1].pose.position;
      arc += hypot(p.x-prev.x,p.y-prev.y);
    }
    if (arc > scan_->horizon() || !scan_->inside(p.x,p.y)) break;
    const size_t next = std::min(i+1, global_plan_.poses.size()-1);
    const auto &q = global_plan_.poses[next].pose.position;
    const double yaw = next == i ? tf2::getYaw(global_plan_.poses[i].pose.orientation) : atan2(q.y-p.y,q.x-p.x);
    if (!scan_->collision(p.x,p.y,yaw)) { goal_index=i; found=true; }
  }
  if (!found || goal_index < closest) throw std::runtime_error("No free local SCAN goal in GridMap");

  if (avoid_flag_ && join_idx_ < track_path_.poses.size()) {
    const auto &join = track_path_.poses[join_idx_].pose.position;
    const size_t near = findClosestPlanIndex(track_path_,robot);
    if (hypot(join.x-robot.pose.position.x,join.y-robot.pose.position.y) < 0.2 || near > join_idx_) {
      avoid_flag_=false; track_path_.poses.clear();
    } else if (scan_->segmentFree(robot.pose.position.x,robot.pose.position.y,
        track_path_.poses[near].pose.position.x,track_path_.poses[near].pose.position.y) &&
      isPathPolylineClear(track_path_,near,join_idx_))
    {
      return densePathSlice(track_path_,near,join_idx_,scan_->resolution()*0.5);
    }
  }

  auto reference = densePathSlice(global_plan_,closest,goal_index,scan_->resolution()*0.5);
  const auto &first = reference.poses.front().pose.position;
  nav_msgs::msg::Path result;
  if (!avoid_flag_ && scan_->segmentFree(robot.pose.position.x,robot.pose.position.y,first.x,first.y) &&
    isPathPolylineClear(reference,0,reference.poses.size()-1))
  {
    result=reference;
  } else {
    // Publish/cache only a fully successful result. A failure leaves no usable command this cycle.
    result=scan_->plan(robot,speed,reference,global_plan_.poses[goal_index]);
    track_path_=result;
    join_idx_=result.poses.size()-1;
    track_path_.poses.insert(track_path_.poses.end(),global_plan_.poses.begin()+goal_index+1,global_plan_.poses.end());
    avoid_flag_=true;
    if (merge_connection_pose_pub_) merge_connection_pose_pub_->publish(result.poses.back());
  }
  result.header.stamp=clock_->now();
  if (freeze_slice_pub_) { std_msgs::msg::Bool msg; msg.data=avoid_flag_; freeze_slice_pub_->publish(msg); }
  return result;
}

nav_msgs::msg::Path RegulatedPurePursuitController::getLocalPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  local_plan_.poses.clear();

  const auto pub_freeze = [this](bool frozen) {
    if (publish_global_path_slice_freeze_ && freeze_slice_pub_ && prefer_planner_polyline_when_clear_) {
      std_msgs::msg::Bool msg;
      msg.data = frozen;
      freeze_slice_pub_->publish(msg);
    }
  };

  if (global_plan_.poses.size() < 1) {
    RCLCPP_WARN(logger_, "Global plan is empty, cannot find goal");
    pub_freeze(false);
    return local_plan_;
  }

  // 绕障结束判断
  if (avoid_flag_ && !track_path_.poses.empty()) {
    const size_t cm = findClosestPlanIndex(track_path_, robot_pose);
    // 距离仅作诊断，不改变原有的最近点索引退出条件。
    const auto & join = track_path_.poses[
      std::min(join_idx_, track_path_.poses.size() - 1)].pose.position;
    const double join_distance = hypot(
      robot_pose.pose.position.x - join.x, robot_pose.pose.position.y - join.y);
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[绕障诊断/跟踪进度] cm=%zu join=%zu points=%zu join_distance=%.3f "
      "robot=(%.3f,%.3f) join_xy=(%.3f,%.3f) robot_frame=%s path_frame=%s",
      cm, join_idx_, track_path_.poses.size(), join_distance,
      robot_pose.pose.position.x, robot_pose.pose.position.y, join.x, join.y,
      robot_pose.header.frame_id.c_str(), track_path_.header.frame_id.c_str());
    bool reached_local_goal = false;
    if (join_idx_ < track_path_.poses.size()) {
      reached_local_goal = (cm >= join_idx_);
    } else if (!track_path_.poses.empty()) {
      reached_local_goal = (cm + 1 >= track_path_.poses.size());
    }
    if (reached_local_goal) {
      RCLCPP_WARN(
        logger_,
        "[绕障诊断/索引退出] cm=%zu join=%zu points=%zu join_distance=%.3f reason=%s",
        cm, join_idx_, track_path_.poses.size(), join_distance,
        join_idx_ < track_path_.poses.size() ? "最近点达到或越过汇入索引" : "汇入索引越界，最近点达到末点");
      avoid_flag_ = false;
      track_path_.poses.clear();
      global_path_tail_start_idx_ = 0;
      join_idx_ = 0;
      pub_freeze(false);
    }
  }

  // local_goal（Theta* 终点）：从车辆在全局路径上的最近点向前扫描。
  // global_plan_ 在一个 FollowPath 生命周期内可以保持不变，而 rolling costmap
  // 会随车移动，因此路径开头的历史点最终必然离开 costmap。不能因为首个历史点
  // worldToMap() 失败就终止扫描，否则长距离 FollowPath 会在行驶半个窗口后失效。
  bool found_local_goal = false;
  geometry_msgs::msg::PoseStamped local_goal;
  size_t local_goal_idx = 0;
  const size_t closest_global_idx = findClosestPlanIndex(global_plan_, robot_pose);
  size_t first_in_map_idx = global_plan_.poses.size();
  size_t last_in_map_idx = global_plan_.poses.size();
  size_t skipped_before_map = 0;
  bool entered_costmap = false;
  auto global_scan_it = global_plan_.poses.cbegin() + closest_global_idx;
  for (; global_scan_it != global_plan_.poses.cend(); ++global_scan_it) {
    const size_t scan_idx = static_cast<size_t>(
      std::distance(global_plan_.poses.cbegin(), global_scan_it));
    unsigned int cx, cy;
    if (!costmap_->worldToMap(global_scan_it->pose.position.x, global_scan_it->pose.position.y, cx, cy)) {
      if (entered_costmap) {
        // The forward path has left the rolling window. Points beyond this are
        // not candidates for the current local goal.
        break;
      }
      // Tolerate a closest sample just outside the map because of discretization
      // or localization/path sampling error; continue until the path enters it.
      ++skipped_before_map;
      continue;
    }
    if (!entered_costmap) {
      entered_costmap = true;
      first_in_map_idx = scan_idx;
    }
    last_in_map_idx = scan_idx;
    if (costmap_->getCost(cx, cy) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      local_goal = *global_scan_it;
      found_local_goal = true;
      local_goal_idx = scan_idx;
    }
  }

  const std::string first_in_map_text =
    first_in_map_idx < global_plan_.poses.size() ? std::to_string(first_in_map_idx) : "none";
  const std::string last_in_map_text =
    last_in_map_idx < global_plan_.poses.size() ? std::to_string(last_in_map_idx) : "none";
  const std::string selected_text = found_local_goal ? std::to_string(local_goal_idx) : "none";
  if (closest_global_idx > 0 || skipped_before_map > 0) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "[局部目标诊断] closest=%zu first_in_map=%s last_in_map=%s selected=%s "
      "skipped_before_map=%zu points=%zu robot=(%.3f,%.3f) "
      "costmap_origin=(%.3f,%.3f) costmap_size=(%.3f,%.3f)",
      closest_global_idx,
      first_in_map_text.c_str(), last_in_map_text.c_str(), selected_text.c_str(),
      skipped_before_map, global_plan_.poses.size(),
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      costmap_->getOriginX(), costmap_->getOriginY(),
      costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  }

  if (!found_local_goal) {
    const auto & closest = global_plan_.poses[closest_global_idx].pose.position;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "No valid local goal found within local costmap! closest=%zu closest_xy=(%.3f,%.3f) "
      "first_in_map=%s last_in_map=%s skipped_before_map=%zu points=%zu "
      "robot=(%.3f,%.3f) plan_frame=%s robot_frame=%s "
      "costmap_origin=(%.3f,%.3f) costmap_size=(%.3f,%.3f)",
      closest_global_idx, closest.x, closest.y,
      first_in_map_text.c_str(), last_in_map_text.c_str(),
      skipped_before_map, global_plan_.poses.size(),
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      global_plan_.header.frame_id.c_str(), robot_pose.header.frame_id.c_str(),
      costmap_->getOriginX(), costmap_->getOriginY(),
      costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
    pub_freeze(false);
    return local_plan_;
  }

  // path_obj_check
  const auto path_obj_check = [&](
    const nav_msgs::msg::Path & track_path,
    size_t polyline_goal_idx,
    bool have_polyline_goal_idx,
    bool validate_generated_detour = false) -> bool
  {
    // 即使禁用原路折线复用，新生成的Theta*拼接输出仍需通过验证。
    if ((!prefer_planner_polyline_when_clear_ && !validate_generated_detour) ||
      !have_polyline_goal_idx)
    {
      return false;
    }
    const size_t closest_idx = findClosestPlanIndex(track_path, robot_pose);
    // polyline_goal_idx 与全局扫描一样：从路径起点向前累加的「远点」。车已驶过该点后
    // closest_idx 会大于 polyline_goal_idx，并非遇障，否则每拍都会走 Theta* 重拼合并线。
    const size_t path_last = track_path.poses.empty() ? 0 : track_path.poses.size() - 1;
    // 绕障段已拼接原路径尾段。汇入点只是状态切换位置，不是停车终点；
    // 将尾段也纳入预瞄/终点减速使用的局部路径，并照常判障。
    const bool merged_detour = &track_path == &track_path_;
    const size_t eff_poly_goal = merged_detour ? path_last :
      std::min(path_last, std::max(polyline_goal_idx, closest_idx));
    if (closest_idx > eff_poly_goal) {
      return false;
    }
    size_t track_end_idx = closest_idx;
    double cum_len = 0.0;
    while (track_end_idx < eff_poly_goal) {
      const auto & p0 = track_path.poses[track_end_idx].pose.position;
      const auto & p1 = track_path.poses[track_end_idx + 1].pose.position;
      unsigned int mx, my;
      if (merged_detour && !costmap_->worldToMap(p1.x, p1.y, mx, my)) {
        break;
      }
      const double seg = hypot(p1.x - p0.x, p1.y - p0.y);
      if (cum_len + seg > polyline_track_max_length_) {
        break;
      }
      cum_len += seg;
      ++track_end_idx;
    }
    geometry_msgs::msg::PoseStamped robot_in_plan;
    if (!transformPose(track_path.header.frame_id, robot_pose, robot_in_plan)) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "[绕障诊断/路径检查失败] avoid=%d reason=坐标变换失败", avoid_flag_);
      return false;
    }
    const double rx = robot_in_plan.pose.position.x;
    const double ry = robot_in_plan.pose.position.y;
    const auto & sp = track_path.poses[closest_idx].pose.position;
    // 车辆自身所在格已处于 lethal 带内时，「车辆→最近点」这一段在任何重规划下
    // 都不可能通过判障；若照常拒绝就会形成「无法重规划→原地永久卡死」的死锁。
    // 这里只豁免起步段，并把折线复检起点后移到 start_relax_radius_ 之外，
    // 逃逸段之后的路径仍逐段严格判障（RPP 的 254 足迹碰撞检测也仍然生效）。
    unsigned int robot_mx = 0, robot_my = 0;
    const bool robot_in_lethal =
      costmap_->worldToMap(rx, ry, robot_mx, robot_my) &&
      static_cast<int>(costmap_->getCost(robot_mx, robot_my)) >= planner_->lethal_cost_;
    if (robot_in_lethal) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[绕障诊断/起点豁免] 车辆所在格代价=%d 已达 lethal=%d，跳过车辆→最近点判障，"
        "向外 %.2f m 以外的路径仍逐段复检 closest=%zu robot=(%.3f,%.3f)",
        static_cast<int>(costmap_->getCost(robot_mx, robot_my)), planner_->lethal_cost_,
        planner_->start_relax_radius_, closest_idx, rx, ry);
    } else if (!isPolylineSegmentFree(rx, ry, sp.x, sp.y)) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "[绕障诊断/路径检查失败] avoid=%d reason=车辆到最近点连线受阻 closest=%zu robot=(%.3f,%.3f) closest_xy=(%.3f,%.3f)",
        avoid_flag_, closest_idx, rx, ry, sp.x, sp.y);
      return false;
    }
    size_t check_start_idx = closest_idx;
    while (robot_in_lethal && check_start_idx < track_end_idx) {
      const auto & q = track_path.poses[check_start_idx].pose.position;
      if (std::hypot(q.x - rx, q.y - ry) >= planner_->start_relax_radius_) {
        break;
      }
      ++check_start_idx;
    }
    if (!isPathPolylineClear(track_path, check_start_idx, track_end_idx)) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
        "[绕障诊断/路径检查失败] avoid=%d reason=前方路径段检查未通过 closest=%zu end=%zu join=%zu",
        avoid_flag_, closest_idx, track_end_idx, polyline_goal_idx);
      return false;
    }
    local_plan_ = densePathSlice(
      track_path, closest_idx, track_end_idx, polyline_collision_check_resolution_);
    diagnostic_plan_source_ = merged_detour ? "detour_with_tail" : "global_polyline";
    return !local_plan_.poses.empty();
  };

  // Theta* + track_path = local_path + global 剩余（merged）；遇障进入/保持 avoid_flag；冻结由本拍末尾 pub_freeze 统一发
  const auto run_theta_star_merge_tail = [&](const char * reason) -> bool {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[绕障诊断/触发重规划] reason=%s avoid=%d old_points=%zu old_join=%zu goal_idx=%zu goal=(%.3f,%.3f)",
      reason, avoid_flag_, track_path_.poses.size(), join_idx_, local_goal_idx,
      local_goal.pose.position.x, local_goal.pose.position.y);
    // a.hpp: local_path — Theta* 原始输出在 local_plan_；再拼成 track_path_ (= local_path + global 剩余)
    // 偏离全局路径时，汇入点只放到最近路径点前方约 1 m 处，先回到路径再谈远端绕障；
    // 否则 Theta* 会直接奔向远端目标，绕过门区这类唯一通道。
    geometry_msgs::msg::PoseStamped theta_goal = local_goal;
    size_t theta_goal_idx = local_goal_idx;
    // 导航器会把当前位姿前插成全局路径首点。若直接拿首点算「离路径距离」，
    // 这个距离恒为 0，就近汇入分支永远不生效，Theta* 就会直扑远端目标、
    // 绕过门区这种唯一通道（车会贴着门柱钻进膨胀区）。
    // 因此跳过与车体重合的前插点，从真正的前方路径起算。
    size_t anchor_idx = 0;
    while (anchor_idx + 1 < global_plan_.poses.size()) {
      const auto & q = global_plan_.poses[anchor_idx].pose.position;
      if (std::hypot(q.x - robot_pose.pose.position.x,
        q.y - robot_pose.pose.position.y) > 0.35) {break;}
      ++anchor_idx;
    }
    // 「离路径距离」必须用横向偏距（车体到锚点之后路径折线的垂直距离），
    // 而不是到单个前方点的直线距离。后者会把「沿路径已前进 0.35~0.45 m」这种
    // 纵向距离也计入，导致车即便贴着路径也会被误判成偏离 >0.40 m，每一拍都把
    // 汇入点收回门前 1 m，永远够不到门区伺服触发范围（原地反复减速/转向）。
    double off_path_dist = std::numeric_limits<double>::infinity();
    if (anchor_idx < global_plan_.poses.size()) {
      const double rx = robot_pose.pose.position.x;
      const double ry = robot_pose.pose.position.y;
      if (anchor_idx + 1 >= global_plan_.poses.size()) {
        const auto & a = global_plan_.poses[anchor_idx].pose.position;
        off_path_dist = std::hypot(rx - a.x, ry - a.y);
      } else {
        for (size_t i = anchor_idx; i + 1 < global_plan_.poses.size(); ++i) {
          const auto & a = global_plan_.poses[i].pose.position;
          const auto & b = global_plan_.poses[i + 1].pose.position;
          const double dx = b.x - a.x;
          const double dy = b.y - a.y;
          const double len2 = dx * dx + dy * dy;
          const double t = len2 > 1e-12 ? std::clamp(
            ((rx - a.x) * dx + (ry - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
          off_path_dist = std::min(off_path_dist,
            std::hypot(rx - a.x - t * dx, ry - a.y - t * dy));
        }
      }
    }
    if (off_path_dist > 0.40) {
      double acc = 0.0;
      size_t idx = anchor_idx;
      while (idx + 1 < global_plan_.poses.size()) {
        const auto & a = global_plan_.poses[idx].pose.position;
        const auto & b = global_plan_.poses[idx + 1].pose.position;
        acc += std::hypot(b.x - a.x, b.y - a.y);
        ++idx;
        if (acc >= 1.0) {break;}
      }
      theta_goal = global_plan_.poses[idx];
      theta_goal_idx = idx;
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "[绕障诊断/就近汇入] 距路径 %.2f m，汇入点由 idx=%zu 收回到 idx=%zu (%.3f,%.3f)",
        off_path_dist, local_goal_idx, theta_goal_idx,
        theta_goal.pose.position.x, theta_goal.pose.position.y);
    }
    // Theta* 的终点必须落在可通行格。窄门这类唯一通道的内切(253)带会把
    // 全局路径上的局部目标整片盖住，直接拿它当终点必然「无解」；
    // 此时沿路径把终点回退到最近的自由格，让规划器至少能走到门口。
    const auto cell_cost = [&](const geometry_msgs::msg::PoseStamped & p) -> int {
        unsigned int mx = 0, my = 0;
        if (!costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {return 255;}
        return static_cast<int>(costmap_->getCost(mx, my));
      };
    const int theta_goal_cost = cell_cost(theta_goal);
    if (theta_goal_cost >= planner_->lethal_cost_) {
      const int blocked_cost = theta_goal_cost;
      size_t back = theta_goal_idx;
      while (back > 0) {
        const auto & p = global_plan_.poses[back].pose.position;
        if (cell_cost(global_plan_.poses[back]) < planner_->lethal_cost_) {break;}
        if (std::hypot(p.x - robot_pose.pose.position.x,
          p.y - robot_pose.pose.position.y) <= planner_->start_relax_radius_) {break;}
        --back;
      }
      if (cell_cost(global_plan_.poses[back]) < planner_->lethal_cost_) {
        theta_goal = global_plan_.poses[back];
        theta_goal_idx = back;
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "[绕障诊断/终点回退] Theta* 终点 cell_cost=%d 已达 lethal=%d，沿路径回退到 idx=%zu (%.3f,%.3f)",
          blocked_cost, planner_->lethal_cost_, theta_goal_idx,
          theta_goal.pose.position.x, theta_goal.pose.position.y);
      }
    }

    planner_->setStartAndGoal(robot_pose, theta_goal);
    getPlan(local_plan_);
    diagnostic_plan_source_ = "theta_star_to_join";
    if (local_plan_.poses.empty()) {
      // 无解时不能清空跟踪路径：清空后本拍和后续每一拍都只拿到空局部路径，
      // 控制器下发零速度，进度检查器 10 s 后 abort，表现为「原地卡死、无法重规划」。
      // 退回全局路径切片继续跟踪，门区随后由门框测距伺服接管。
      // 切片必须伸过门区触发半径（door_enter_distance），否则车停在门前、
      // 永远够不到伺服 ALIGN 触发点。多留 0.5 m 余量确保已越过门框观测范围。
      const double fallback_slice_len = door_enter_distance_ + 0.5;
      size_t slice_to = closest_global_idx;
      double acc = 0.0;
      while (slice_to + 1 < global_plan_.poses.size()) {
        const auto & a = global_plan_.poses[slice_to].pose.position;
        const auto & b = global_plan_.poses[slice_to + 1].pose.position;
        acc += std::hypot(b.x - a.x, b.y - a.y);
        ++slice_to;
        if (acc >= fallback_slice_len) {break;}
      }
      local_plan_ = densePathSlice(
        global_plan_, closest_global_idx, slice_to, polyline_collision_check_resolution_);
      diagnostic_plan_source_ = "global_fallback_theta_empty";
      ++recovery_no_path_cycles_;
      avoid_flag_ = false;
      track_path_.poses.clear();
      global_path_tail_start_idx_ = 0;
      join_idx_ = 0;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[绕障诊断/重规划为空] reason=%s goal=(%.3f,%.3f) cell_cost=%d；不清空路径，"
        "退回全局路径 idx=%zu..%zu 继续跟踪（points=%zu）",
        reason, theta_goal.pose.position.x, theta_goal.pose.position.y,
        cell_cost(theta_goal), closest_global_idx, slice_to, local_plan_.poses.size());
      return false;
    }
    const size_t theta_n = local_plan_.poses.size();
    recovery_no_path_cycles_ = 0;
    track_path_.header = global_plan_.header;
    track_path_.poses = local_plan_.poses;
    global_path_tail_start_idx_ = theta_goal_idx + 1;
    for (size_t gi = global_path_tail_start_idx_; gi < global_plan_.poses.size(); ++gi) {
      track_path_.poses.push_back(global_plan_.poses[gi]);
    }
    join_idx_ = theta_n;
    // 首次重规划的这一拍也跟踪经过判障的拼接路线，避免只有后续周期
    // 才得到尾段。尾段未通过判障时（如门洞这类唯一通道被内切带封死），
    // 只保留已验证的 Theta* 段继续跟踪，而不是抛异常中止导航。
    if (!path_obj_check(track_path_, join_idx_, true, true)) {
      track_path_.poses.resize(theta_n);
      join_idx_ = theta_n;
      global_path_tail_start_idx_ = 0;
      diagnostic_plan_source_ = "theta_star_only";
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[绕障诊断/尾段回退] 合并尾段未通过判障，仅跟踪已验证的 Theta* 段 points=%zu",
        theta_n);
    } else {
      diagnostic_plan_source_ = "theta_star_with_tail";
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "[绕障诊断/重规划完成] theta_points=%zu track_points=%zu join=%zu tail_start=%zu",
        theta_n, track_path_.poses.size(), join_idx_, global_path_tail_start_idx_);
    }
    return true;
  };


  // 尚未回到全局路径起点时，先走「机器人→路径起点」的接入段，不启动绕障分支：
  // 否则 Theta* 会直接规划到远端目标，绕过门区这类唯一通道，冲进膨胀区卡死。
  bool rejoining_path = false;
  double rejoin_dist = 0.0;
  if (!avoid_flag_ && !global_plan_.poses.empty() && closest_global_idx == 0) {
    const auto & p0 = global_plan_.poses.front().pose.position;
    rejoin_dist = std::hypot(
      robot_pose.pose.position.x - p0.x, robot_pose.pose.position.y - p0.y);
    rejoining_path = rejoin_dist > 0.30 && rejoin_dist <= 1.50;
  }
  if (rejoining_path) {
    nav_msgs::msg::Path rejoin;
    rejoin.header = global_plan_.header;
    rejoin.poses.push_back(robot_pose);
    double cum = 0.0;
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
      rejoin.poses.push_back(global_plan_.poses[i]);
      if (i > 0) {
        const auto & a = global_plan_.poses[i - 1].pose.position;
        const auto & b = global_plan_.poses[i].pose.position;
        cum += std::hypot(b.x - a.x, b.y - a.y);
      }
      if (cum >= 1.0) {break;}
    }
    local_plan_ = densePathSlice(
      rejoin, 0, rejoin.poses.size() - 1, polyline_collision_check_resolution_);
    diagnostic_plan_source_ = "rejoin_global_path";
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[接入诊断] 距全局路径起点 %.2f m，先接入路径再判障 avoid=%d",
      rejoin_dist, static_cast<int>(avoid_flag_));
  }

  // 非绕障期间：全局路径折线判障
  bool obj_flag = false;
  if (rejoining_path) {
    obj_flag = false;
  } else if (!avoid_flag_) {
    obj_flag = !path_obj_check(global_plan_, local_goal_idx, found_local_goal);
    if (obj_flag) {
      avoid_flag_ = run_theta_star_merge_tail("全局路径检查未通过");
    }
  } else {
    if (track_path_.poses.empty()) {
      avoid_flag_ = run_theta_star_merge_tail("绕障状态下跟踪路径为空");
    } else {
      obj_flag = !path_obj_check(
        track_path_, join_idx_, true);
      if (obj_flag) {
        avoid_flag_ = run_theta_star_merge_tail("当前绕障路径检查未通过");
      }
    }
  }

  // a.hpp L25: avoid_flag 为 false 时清空 track_path_
  if (!avoid_flag_) {
    track_path_.poses.clear();
    global_path_tail_start_idx_ = 0;
    join_idx_ = 0;
  }

  if (local_plan_.poses.empty()) {
    pub_freeze(false);
    return local_plan_;
  }

  if (global_scan_it == global_plan_.poses.cend()) {
    local_plan_.poses.back().pose.orientation = global_plan_.poses.back().pose.orientation;
  }

  local_plan_.header.stamp = clock_->now();
  local_plan_.header.frame_id = global_plan_.header.frame_id;

  pub_freeze(avoid_flag_ || obj_flag);

  if (publish_merge_connection_point_ && merge_connection_pose_pub_ &&
    !track_path_.poses.empty() &&
    join_idx_ < track_path_.poses.size())
  {
    geometry_msgs::msg::PoseStamped conn =
      track_path_.poses[join_idx_];
    conn.header.stamp = clock_->now();
    conn.header.frame_id = track_path_.header.frame_id;
    merge_connection_pose_pub_->publish(conn);
  }

  return local_plan_;
}

void RegulatedPurePursuitController::getPlan(nav_msgs::msg::Path & path)
{
  std::vector<coordsW> tmp_path;
  if (planner_->generatePath(tmp_path)) {
    if (planner_->start_relaxed_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[Theta*起点松弛] 机器人所在格已达 lethal=%d，就近退到安全格 grid=(%d,%d) 起步后规划成功",
        planner_->lethal_cost_, planner_->src_.x, planner_->src_.y);
    }
    nav_msgs::msg::Path theta_path =
      linearInterpolation(tmp_path, planner_->costmap_->getResolution());
    theta_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    theta_path.header.stamp = clock_->now();
    for (auto & pose : theta_path.poses) {
      pose.header = theta_path.header;
      pose.pose.orientation.w = 1.0;
    }
    theta_raw_pub_->publish(theta_path);
    if (!path_smoothing_enabled_) {
      path = theta_path;
      theta_smoothed_pub_->publish(path);
      return;
    }

    nav_msgs::msg::Path smoothed_path;
    const auto pose_is_free = [this](double x, double y, double yaw) {
        return isSmoothingPoseFree(x, y, yaw);
      };
    SmoothingDiagnostics diagnostics;
    if (path_smoother_.smooth(theta_path, smoothed_path, pose_is_free, &diagnostics)) {
      path = std::move(smoothed_path);
      if (diagnostics.backtracking_attempts > 0) {
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 2000,
          "[平滑回退成功] iterations=%d rejected_points=%d backtracking=%d，采用缩小平滑量后的路径",
          diagnostics.iterations, diagnostics.rejected_points, diagnostics.backtracking_attempts);
      }
    } else {
      path = theta_path;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[平滑失败] reason=%s points=%zu iterations=%d rejected_points=%d backtracking=%d "
        "has_failed_pose=%d segment=%zu frame=%s x=%.3f y=%.3f yaw=%.3f detail=%s；直接采用原始局部路径",
        diagnostics.reason, theta_path.poses.size(), diagnostics.iterations,
        diagnostics.rejected_points, diagnostics.backtracking_attempts,
        diagnostics.has_failed_pose, diagnostics.failed_segment,
        costmap_ros_->getGlobalFrameID().c_str(), diagnostics.x, diagnostics.y, diagnostics.yaw,
        (diagnostics.has_failed_pose ?
        describeSmoothingPose(diagnostics.x, diagnostics.y, diagnostics.yaw) :
        std::string("未进入路径碰撞复检")).c_str());
    }
    theta_smoothed_pub_->publish(path);
  } else {
    RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
    // 栅格索引先检查边界，避免诊断本身越界读取地图。
    const auto describe_cell = [this](const coordsM & cell) {
        if (cell.x < 0 || cell.y < 0 ||
          cell.x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
          cell.y >= static_cast<int>(costmap_->getSizeInCellsY()))
        {
          return fmt::format("grid=({},{}),地图外", cell.x, cell.y);
        }
        double x, y;
        costmap_->mapToWorld(cell.x, cell.y, x, y);
        return fmt::format("grid=({},{}),world=({:.3f},{:.3f}),cost={}",
          cell.x, cell.y, x, y, static_cast<int>(costmap_->getCost(cell.x, cell.y)));
      };
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[Theta*无解] frame=%s start={%s} goal={%s} lethal_cost=%d allow_unknown=%d；未进入平滑阶段",
      costmap_ros_->getGlobalFrameID().c_str(), describe_cell(planner_->src_).c_str(),
      describe_cell(planner_->dst_).c_str(), planner_->lethal_cost_, planner_->allow_unknown_);
    path.poses.clear();
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    empty_path.header.stamp = clock_->now();
    theta_raw_pub_->publish(empty_path);
    theta_smoothed_pub_->publish(empty_path);
  }
}

std::string RegulatedPurePursuitController::describeSmoothingPose(
  double x, double y, double /*yaw*/) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return "中心点超出地图";
  }
  const int center_cost = costmap_->getCost(mx, my);
  const char * reason = "中心点可通行";
  if (center_cost == NO_INFORMATION) {
    reason = "中心点未知区域不允许通行";
  } else if (center_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    reason = "中心代价达到局部目标点阈值";
  }
  return fmt::format("{} check=center grid=({},{}) center_cost={} blocked_cost={} allow_unknown=false",
    reason, mx, my, center_cost,
    static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
}

bool RegulatedPurePursuitController::isSmoothingPoseFree(
  double x, double y, double /*yaw*/) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return false;
  }

  // 与 local_goal 选择保持一致：由膨胀层提供车身余量，只检查中心点。
  // 253（内切膨胀障碍）、254（致命障碍）、255（未知区域）均不可通行。
  return costmap_->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

nav_msgs::msg::Path RegulatedPurePursuitController::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  geometry_msgs::msg::PoseStamped p1;
  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    coordsW pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    pa.poses.push_back(p1);

    coordsW pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }

  return pa;
}

rcl_interfaces::msg::SetParametersResult
RegulatedPurePursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto &parameter : parameters) {
    const auto &name = parameter.get_name();
    if (name == plugin_name_ + ".local_planner_type" ||
      name.rfind(plugin_name_ + ".scan.", 0) == 0 ||
      (scan_ && (name == plugin_name_ + ".use_collision_detection" ||
      name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling" ||
      name == plugin_name_ + ".allow_reversing")))
    {
      result.successful = false;
      result.reason = "Planning/map backend and SCAN safety settings require navigation restart";
      return result;
    }
  }
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".inflation_cost_scaling_factor") {
        if (parameter.as_double() <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
            "it should be >0. Ignoring parameter update.");
          continue;
        }
        inflation_cost_scaling_factor_ = parameter.as_double();
      } else if (name == plugin_name_ + ".desired_linear_vel") {
        desired_linear_vel_ = parameter.as_double();
        base_desired_linear_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_linear_accel") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "max_linear_accel must be positive";
          return result;
        }
        max_linear_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_linear_decel") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "max_linear_decel must be positive";
          return result;
        }
        max_linear_decel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot") {
        max_allowed_time_to_collision_up_to_carrot_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_dist") {
        cost_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_gain") {
        cost_scaling_gain_ = parameter.as_double();
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_radius") {
        regulated_linear_scaling_min_radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_speed") {
        regulated_linear_scaling_min_speed_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angular_accel") {
        max_angular_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_min_angle") {
        rotate_to_heading_min_angle_ = parameter.as_double();
      } else if (name == plugin_name_ + ".polyline_collision_check_resolution") {
        polyline_collision_check_resolution_ = parameter.as_double();
      } else if (name == plugin_name_ + ".polyline_track_max_length") {
        polyline_track_max_length_ = parameter.as_double();
      } else if (name == plugin_name_ + ".smoothing_tolerance") {
        if (parameter.as_double() < 0.0) {
          result.successful = false;
          result.reason = "smoothing_tolerance must be non-negative";
          return result;
        }
        smoothing_parameters_.tolerance = parameter.as_double();
      } else if (name == plugin_name_ + ".smoothing_data_weight") {
        if (parameter.as_double() < 0.0 || parameter.as_double() > 1.0) {
          result.successful = false;
          result.reason = "smoothing_data_weight must be in [0, 1]";
          return result;
        }
        smoothing_parameters_.data_weight = parameter.as_double();
      } else if (name == plugin_name_ + ".smoothing_smooth_weight") {
        if (parameter.as_double() <= 0.0 || parameter.as_double() > 0.5) {
          result.successful = false;
          result.reason = "smoothing_smooth_weight must be in (0, 0.5]";
          return result;
        }
        smoothing_parameters_.smooth_weight = parameter.as_double();
      } else if (name == plugin_name_ + ".smoothing_max_deviation") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "smoothing_max_deviation must be positive";
          return result;
        }
        smoothing_parameters_.max_deviation = parameter.as_double();
      } else if (name == plugin_name_ + ".smoothing_collision_step") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "smoothing_collision_step must be positive";
          return result;
        }
        smoothing_parameters_.collision_check_step = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".speed_diagnostics_enabled") {
        speed_diagnostics_enabled_ = parameter.as_bool();
        RCLCPP_INFO(logger_, "[speed_diag] enabled=%d record_topic=/log/record",
          speed_diagnostics_enabled_);
      } else if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_regulated_linear_velocity_scaling") {
        use_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling") {
        use_cost_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        if (parameter.as_bool() && allow_reversing_) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          continue;
        }
        use_rotate_to_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".allow_reversing") {
        if (use_rotate_to_heading_ && parameter.as_bool()) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          continue;
        }
        allow_reversing_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".prefer_planner_polyline_when_clear") {
        prefer_planner_polyline_when_clear_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".publish_global_path_slice_freeze") {
        publish_global_path_slice_freeze_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".publish_merge_connection_point") {
        publish_merge_connection_point_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".path_smoothing_enabled") {
        path_smoothing_enabled_ = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".smoothing_max_iterations") {
        if (parameter.as_int() <= 0) {
          result.successful = false;
          result.reason = "smoothing_max_iterations must be positive";
          return result;
        }
        smoothing_parameters_.max_iterations = static_cast<int>(parameter.as_int());
      } else if (name == plugin_name_ + ".smoothing_backtracking_steps") {
        if (parameter.as_int() < 0) {
          result.successful = false;
          result.reason = "smoothing_backtracking_steps must be non-negative";
          return result;
        }
        smoothing_parameters_.backtracking_steps = static_cast<int>(parameter.as_int());
      }
    }
  }

  path_smoother_.setParameters(smoothing_parameters_);
  result.successful = true;
  return result;
}

}  // namespace nav2_a_star_rpp

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_a_star_rpp::RegulatedPurePursuitController,
  nav2_core::Controller)
