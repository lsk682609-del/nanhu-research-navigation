#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "camera_boundary_navigation/boundary_controller.hpp"
#include "camera_boundary_navigation/boundary_model.hpp"
#include "camera_boundary_navigation/safety_gate.hpp"
#include "camera_boundary_navigation/stable_local_planner.hpp"
#include "camera_boundary_navigation/vfh_local_planner.hpp"

namespace camera_boundary_navigation
{

class CameraBoundaryNavigationNode : public rclcpp::Node
{
public:
  CameraBoundaryNavigationNode()
  : Node("camera_boundary_navigation_node")
  {
    loadParameters();
    active_reference_side_ = controller_config_.side;

    boundary_model_ = std::make_unique<BoundaryModel>(model_config_);
    left_boundary_model_ = std::make_unique<BoundaryModel>(model_config_);
    right_boundary_model_ = std::make_unique<BoundaryModel>(model_config_);
    controller_ = std::make_unique<BoundaryController>(controller_config_);
    safety_gate_ = std::make_unique<SafetyGate>(safety_config_);
    stable_planner_ = std::make_unique<StableLocalPlanner>(
      stable_planner_config_, controller_config_.side);

    vfh_planner_ = std::make_unique<VfhLocalPlanner>(vfh_config_, controller_config_);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
    boundary_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      boundary_topic_, sensor_qos,
      std::bind(&CameraBoundaryNavigationNode::boundaryCallback, this, std::placeholders::_1));
    grid_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      grid_topic_, sensor_qos,
      std::bind(&CameraBoundaryNavigationNode::gridCallback, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, sensor_qos,
      std::bind(&CameraBoundaryNavigationNode::obstacleCallback, this, std::placeholders::_1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::QoS(10),
      std::bind(&CameraBoundaryNavigationNode::enableCallback, this, std::placeholders::_1));
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
      pause_topic_, rclcpp::QoS(10),
      std::bind(&CameraBoundaryNavigationNode::pauseCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(reference_path_topic_, rclcpp::QoS(10));
    nominal_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/camera_navigation/nominal_path", rclcpp::QoS(10));
    avoidance_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/camera_navigation/avoidance_markers", rclcpp::QoS(2));
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(target_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);

    last_boundary_receipt_ = zeroTime();
    last_grid_receipt_ = zeroTime();
    last_obstacle_receipt_ = zeroTime();
    last_control_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CameraBoundaryNavigationNode::controlTimer, this));

    RCLCPP_WARN(
      get_logger(),
      "odometry-free boundary follower ready: side=%s offset=%.2fm cmd=%s enabled=%s; "
      "non-zero commands require fresh boundary/grid/obstacle inputs",
      reference_side_name_.c_str(), controller_config_.boundary_offset,
      cmd_vel_topic_.c_str(), enabled_ ? "true" : "false");
  }

private:
  rclcpp::Time zeroTime() const
  {
    return rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  static bool hasField(const sensor_msgs::msg::PointCloud2 & message, const std::string & name)
  {
    return std::any_of(
      message.fields.begin(), message.fields.end(),
      [&name](const auto & field) {return field.name == name;});
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  bool validCloud(const sensor_msgs::msg::PointCloud2 & message, bool labels) const
  {
    const double age = (now() - rclcpp::Time(message.header.stamp,
      get_clock()->get_clock_type())).seconds();
    if (message.header.frame_id != output_frame_ || age < -future_tolerance_ ||
      age > input_timeout_ ||
      message.is_bigendian || message.point_step == 0 ||
      message.row_step != static_cast<std::uint64_t>(message.width) * message.point_step ||
      message.data.size() < static_cast<std::uint64_t>(message.row_step) * message.height)
    {return false;}
    for (const std::string name : {"x", "y", "intensity"}) {
      if (name == "intensity" && !labels) {continue;}
      const auto it = std::find_if(message.fields.begin(), message.fields.end(),
        [&name](const auto & field) {return field.name == name;});
      if (it == message.fields.end() || it->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        it->count != 1 || static_cast<std::uint64_t>(it->offset) + sizeof(float) > message.point_step)
      {return false;}
    }
    return true;
  }

  void loadParameters()
  {
    boundary_topic_ = declare_parameter<std::string>(
      "boundary_topic", "/traversability/costmap/boundary_constraints");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/traversability/costmap/depth_obstacles");
    grid_topic_ = declare_parameter<std::string>("grid_topic", "/traversability/grid");
    enable_topic_ = declare_parameter<std::string>(
      "enable_topic", "/camera_navigation/enable");
    pause_topic_ = declare_parameter<std::string>("pause_topic", "/pause");
    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic", "/camera_navigation/cmd_vel");
    reference_path_topic_ = declare_parameter<std::string>(
      "reference_path_topic", "/camera_navigation/reference_path");
    target_topic_ = declare_parameter<std::string>(
      "target_topic", "/camera_navigation/target_point");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/camera_navigation/status");
    output_frame_ = declare_parameter<std::string>("output_frame", "gravity_aligned_base");
    enabled_ = declare_parameter<bool>("motion.enabled_on_startup", false);
    publish_zero_when_stopped_ = declare_parameter<bool>(
      "motion.publish_zero_when_stopped", true);
    control_frequency_ = declare_parameter<double>("control.frequency", 20.0);
    input_timeout_ = declare_parameter<double>("safety.input_timeout", 0.25);
    future_tolerance_ = declare_parameter<double>("safety.future_tolerance", 0.0);

    reference_side_name_ = declare_parameter<std::string>("reference_side", "left");
    if (reference_side_name_ == "left") {
      controller_config_.side = ReferenceSide::LEFT;
      selected_boundary_label_ = 1;
    } else if (reference_side_name_ == "right") {
      controller_config_.side = ReferenceSide::RIGHT;
      selected_boundary_label_ = 2;
    } else {
      throw std::runtime_error("reference_side must be 'left' or 'right'");
    }

    model_config_.fit_x_min = declare_parameter<double>("boundary.fit_x_min", 0.6);
    model_config_.fit_x_max = declare_parameter<double>("boundary.fit_x_max", 3.5);
    model_config_.minimum_points = declare_parameter<int>("boundary.minimum_points", 8);
    model_config_.minimum_x_span = declare_parameter<double>("boundary.minimum_x_span", 1.0);
    model_config_.maximum_rms_residual = declare_parameter<double>(
      "boundary.maximum_fit_residual", 0.12);
    model_config_.outlier_threshold = declare_parameter<double>(
      "boundary.outlier_threshold", 0.18);
    model_config_.robust_iterations = declare_parameter<int>("boundary.robust_iterations", 3);
    model_config_.filter_alpha = declare_parameter<double>("boundary.filter_alpha", 0.3);

    controller_config_.boundary_offset = declare_parameter<double>("boundary.offset", 0.65);
    controller_config_.path_sample_step = declare_parameter<double>(
      "control.path_sample_step", 0.10);
    controller_config_.bezier_handle_ratio = declare_parameter<double>(
      "control.bezier_handle_ratio", 0.35);
    controller_config_.lookahead_distance = declare_parameter<double>(
      "control.lookahead_distance", 1.5);
    controller_config_.nominal_linear_velocity = declare_parameter<double>(
      "control.nominal_linear_velocity", 0.25);
    controller_config_.minimum_linear_velocity = declare_parameter<double>(
      "control.minimum_linear_velocity", 0.10);
    controller_config_.maximum_linear_velocity = declare_parameter<double>(
      "control.maximum_linear_velocity", 0.30);
    controller_config_.maximum_angular_velocity = declare_parameter<double>(
      "control.maximum_angular_velocity", 0.35);
    controller_config_.maximum_angular_acceleration = declare_parameter<double>(
      "control.maximum_angular_acceleration", 0.50);
    controller_config_.maximum_abs_curvature = declare_parameter<double>(
      "control.maximum_abs_curvature", 1.50);
    controller_config_.curvature_slowdown_gain = declare_parameter<double>(
      "control.curvature_slowdown_gain", 1.50);
    controller_config_.command_filter_alpha = declare_parameter<double>(
      "control.command_filter_alpha", 0.30);

    stability_enabled_ = declare_parameter<bool>("stability.enabled", true);
    stability_path_horizon_ = declare_parameter<double>("stability.path_horizon", 2.5);
    boundary_evaluation_x_ = declare_parameter<double>(
      "stability.boundary_evaluation_x", 0.8);
    stable_planner_config_.preferred_boundary_distance = controller_config_.boundary_offset;
    stable_planner_config_.boundary_distance_deadband = declare_parameter<double>(
      "stability.boundary_distance_deadband", 0.20);
    stable_planner_config_.turn_confirmation_frames = declare_parameter<int>(
      "stability.turn_confirmation_frames", 5);
    stable_planner_config_.straight_confirmation_frames = declare_parameter<int>(
      "stability.straight_confirmation_frames", 8);
    stable_planner_config_.minimum_turn_curvature = declare_parameter<double>(
      "stability.minimum_turn_curvature", 0.05);
    stable_planner_config_.curvature_tracking_weight = declare_parameter<double>(
      "stability.curvature_tracking_weight", 4.0);
    stable_planner_config_.curvature_change_weight = declare_parameter<double>(
      "stability.curvature_change_weight", 12.0);
    stable_planner_config_.maximum_curvature_change_per_frame = declare_parameter<double>(
      "stability.maximum_curvature_change_per_frame", 0.05);

    safety_config_.grid_resolution = declare_parameter<double>("safety.grid_resolution", 0.15);
    safety_config_.path_sample_step = controller_config_.path_sample_step;
    safety_config_.path_clearance = declare_parameter<double>("safety.path_clearance", 0.40);
    safety_config_.minimum_ground_check_x = declare_parameter<double>(
      "safety.minimum_ground_check_x", 0.60);
    safety_config_.minimum_front_range = declare_parameter<double>(
      "safety.minimum_front_range", 1.0);
    safety_config_.immediate_obstacle_distance = declare_parameter<double>(
      "safety.obstacle_stop_distance", 0.70);
    safety_config_.immediate_obstacle_half_width = declare_parameter<double>(
      "safety.obstacle_half_width", 0.40);
    safety_config_.minimum_obstacle_points_per_cell = declare_parameter<int>(
      "safety.minimum_obstacle_points_per_cell", 1);
    safety_config_.require_ground_grid = declare_parameter<bool>(
      "safety.require_ground_grid", true);
    safety_config_.require_obstacle_cloud = declare_parameter<bool>(
      "safety.require_obstacle_cloud", true);
    safety_config_.recovery_valid_frames = declare_parameter<int>(
      "safety.recovery_valid_frames", 5);

    vfh_config_.enabled = declare_parameter<bool>("avoidance.enabled", true);
    boundaryless_enabled_ = declare_parameter<bool>("boundaryless.enabled", true);
    vfh_config_.angular_bins = declare_parameter<int>("avoidance.angular_bins", 61);
    vfh_config_.return_confirmation_frames = declare_parameter<int>(
      "avoidance.return_confirmation_frames", 8);
    vfh_config_.half_fov = declare_parameter<double>("avoidance.half_fov", 0.75);
    vfh_config_.horizon = declare_parameter<double>("avoidance.horizon", 2.0);
    vfh_config_.minimum_path_length = declare_parameter<double>("avoidance.minimum_path_length", 1.0);
    vfh_config_.target_weight = declare_parameter<double>("avoidance.target_weight", 4.0);
    vfh_config_.smooth_weight = declare_parameter<double>("avoidance.smooth_weight", 1.0);
    vfh_config_.obstacle_weight = declare_parameter<double>("avoidance.obstacle_weight", 0.8);
    vfh_config_.reference_weight = declare_parameter<double>("avoidance.reference_weight", 1.0);
    vfh_config_.side_switch_penalty = declare_parameter<double>("avoidance.side_switch_penalty", 2.0);
    vfh_config_.return_curvature_step = declare_parameter<double>("avoidance.return_curvature_step", 0.05);
    vfh_config_.braking_deceleration = declare_parameter<double>("avoidance.braking_deceleration", 0.5);
    vfh_config_.reaction_time = declare_parameter<double>("avoidance.reaction_time", 0.35);
    vfh_config_.stop_margin = declare_parameter<double>("avoidance.stop_margin", 0.25);
    vfh_config_.slowdown_clearance = declare_parameter<double>("avoidance.slowdown_clearance", 0.5);
    vfh_config_.preferred_clearance = declare_parameter<double>("avoidance.preferred_clearance", 0.20);
    vfh_config_.clearance_weight = declare_parameter<double>("avoidance.clearance_weight", 12.0);
    vfh_config_.corridor_weight = declare_parameter<double>("avoidance.corridor_weight", 4.0);
    const double ground_half_width = declare_parameter<double>("avoidance.ground_half_width", 0.30);
    safety_config_.ground_half_width = vfh_config_.enabled ? ground_half_width : 0.0;
    safety_config_.directional_front_check = vfh_config_.enabled;
    vfh_config_.inflation_radius = safety_config_.path_clearance +
      0.71 * safety_config_.grid_resolution;
    if (vfh_config_.enabled && (!safety_config_.require_ground_grid ||
      !safety_config_.require_obstacle_cloud || !std::isfinite(ground_half_width) ||
      ground_half_width <= 0.0 || ground_half_width > safety_config_.path_clearance ||
      vfh_config_.minimum_path_length < safety_config_.minimum_front_range ||
      vfh_config_.reaction_time < input_timeout_ + 1.0 / control_frequency_ ||
      output_frame_.empty()))
    {throw std::runtime_error("avoidance requires ground/obstacles, footprint and sufficient reaction time");}
    if (boundaryless_enabled_ && !vfh_config_.enabled) {
      throw std::runtime_error("boundaryless forward mode requires avoidance.enabled");
    }

    for (const double number : {control_frequency_, input_timeout_, future_tolerance_,
      safety_config_.grid_resolution,
      safety_config_.path_clearance, safety_config_.minimum_front_range,
      safety_config_.minimum_ground_check_x, safety_config_.immediate_obstacle_distance,
      safety_config_.immediate_obstacle_half_width, controller_config_.path_sample_step,
      controller_config_.minimum_linear_velocity, controller_config_.nominal_linear_velocity,
      controller_config_.maximum_angular_acceleration, controller_config_.command_filter_alpha})
    {
      if (!std::isfinite(number)) {throw std::runtime_error("nonfinite navigation parameter");}
    }
    if (safety_config_.path_clearance <= 0.0 || safety_config_.minimum_front_range <= 0.0 ||
      safety_config_.immediate_obstacle_distance <= 0.0 ||
      safety_config_.immediate_obstacle_half_width <= 0.0 ||
      controller_config_.maximum_angular_acceleration <= 0.0 ||
      controller_config_.command_filter_alpha <= 0.0 || controller_config_.command_filter_alpha > 1.0 ||
      (vfh_config_.enabled && controller_config_.path_sample_step > safety_config_.grid_resolution))
    {throw std::runtime_error("invalid safety geometry or command smoothing parameters");}

    if (control_frequency_ <= 0.0 || input_timeout_ <= 0.0 || future_tolerance_ < 0.0 ||
      model_config_.fit_x_max <= model_config_.fit_x_min || model_config_.minimum_points < 3 ||
      controller_config_.boundary_offset <= 0.0 ||
      controller_config_.path_sample_step <= 0.0 ||
      controller_config_.bezier_handle_ratio <= 0.0 ||
      controller_config_.bezier_handle_ratio > 1.0 ||
      controller_config_.lookahead_distance <= 0.0 ||
      controller_config_.minimum_linear_velocity < 0.0 ||
      controller_config_.maximum_linear_velocity < controller_config_.minimum_linear_velocity ||
      stability_path_horizon_ < 0.3 || boundary_evaluation_x_ < 0.0 ||
      stable_planner_config_.boundary_distance_deadband < 0.0 ||
      stable_planner_config_.turn_confirmation_frames < 1 ||
      stable_planner_config_.straight_confirmation_frames < 1 ||
      stable_planner_config_.minimum_turn_curvature <= 0.0 ||
      stable_planner_config_.curvature_tracking_weight <= 0.0 ||
      stable_planner_config_.curvature_change_weight < 0.0 ||
      stable_planner_config_.maximum_curvature_change_per_frame <= 0.0 ||
      safety_config_.grid_resolution <= 0.0 ||
      safety_config_.minimum_ground_check_x < 0.0 ||
      safety_config_.minimum_obstacle_points_per_cell < 1 ||
      safety_config_.recovery_valid_frames < 1)
    {
      throw std::runtime_error("invalid camera boundary navigation parameters");
    }
  }

  void boundaryCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!validCloud(*message, true)) {
      pending_boundary_stamp_ = -1;
      last_boundary_receipt_ = zeroTime();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejected boundary cloud: frame, stamp or PointCloud2 layout invalid");
      return;
    }
    pending_boundary_stamp_ = stampNanoseconds(message->header.stamp);
    pending_boundary_header_ = message->header;
    pending_boundary_points_.clear();
    pending_left_points_.clear();
    pending_right_points_.clear();
    pending_front_range_ = 0.0;
    if (!hasField(*message, "x") || !hasField(*message, "y") ||
      !hasField(*message, "intensity"))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "boundary cloud must contain x, y and intensity");
      tryCommitFrame();
      return;
    }

    double front_range = std::numeric_limits<double>::infinity();
    double selected_boundary_max_x = 0.0;
    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> intensity(*message, "intensity");
    for (; x != x.end(); ++x, ++y, ++intensity) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*intensity)) {
        continue;
      }
      const int label = static_cast<int>(std::lround(*intensity));
      if (label == 1) {pending_left_points_.push_back({*x, *y});}
      if (label == 2) {pending_right_points_.push_back({*x, *y});}
      if (label == selected_boundary_label_) {
        pending_boundary_points_.push_back({*x, *y});
        selected_boundary_max_x = std::max(selected_boundary_max_x, static_cast<double>(*x));
      } else if (label == 3 && *x > 0.0F) {
        front_range = std::min(front_range, static_cast<double>(*x));
      }
    }
    pending_front_range_ =
      std::isfinite(front_range) ? front_range : selected_boundary_max_x;
    tryCommitFrame();
  }

  void gridCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!validCloud(*message, true)) {
      pending_grid_stamp_ = -1;
      last_grid_receipt_ = zeroTime();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejected grid cloud: frame, stamp or PointCloud2 layout invalid");
      return;
    }
    pending_grid_stamp_ = stampNanoseconds(message->header.stamp);
    pending_ground_cells_.clear();
    if (!hasField(*message, "x") || !hasField(*message, "y") ||
      !hasField(*message, "intensity"))
    {
      tryCommitFrame();
      return;
    }
    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> intensity(*message, "intensity");
    for (; x != x.end(); ++x, ++y, ++intensity) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*intensity) &&
        static_cast<int>(std::lround(*intensity)) == 1)
      {
        pending_ground_cells_.push_back({*x, *y});
      }
    }
    tryCommitFrame();
  }

  void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!validCloud(*message, false)) {
      pending_obstacle_stamp_ = -1;
      last_obstacle_receipt_ = zeroTime();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejected obstacle cloud: frame, stamp or PointCloud2 layout invalid");
      return;
    }
    pending_obstacle_stamp_ = stampNanoseconds(message->header.stamp);
    pending_obstacles_.clear();
    if (!hasField(*message, "x") || !hasField(*message, "y")) {
      tryCommitFrame();
      return;
    }
    pending_obstacles_.reserve(static_cast<std::size_t>(message->width) * message->height);
    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    for (; x != x.end(); ++x, ++y) {
      if (std::isfinite(*x) && std::isfinite(*y) && *x >= 0.0F &&
        *x <= static_cast<float>(model_config_.fit_x_max + 1.0) &&
        std::abs(*y) <= 3.0F)
      {
        pending_obstacles_.push_back({*x, *y});
      }
    }
    tryCommitFrame();
  }

  void tryCommitFrame()
  {
    const bool grid_ready = !safety_config_.require_ground_grid ||
      (pending_grid_stamp_ >= 0 && pending_boundary_stamp_ == pending_grid_stamp_);
    const bool obstacle_ready = !safety_config_.require_obstacle_cloud ||
      (pending_obstacle_stamp_ >= 0 && pending_boundary_stamp_ == pending_obstacle_stamp_);
    if (pending_boundary_stamp_ < 0 || !grid_ready || !obstacle_ready ||
      pending_boundary_stamp_ <= last_committed_stamp_)
    {
      return;
    }
    const rclcpp::Time frame_time(pending_boundary_stamp_, get_clock()->get_clock_type());
    if (!fresh(frame_time, now())) {return;}
    safety_gate_->setFrontRange(pending_front_range_);
    safety_gate_->updateGroundCells(
      pending_boundary_stamp_ == pending_grid_stamp_ ? pending_ground_cells_ :
      std::vector<Point2d>{});
    safety_gate_->updateObstacleCells(
      pending_boundary_stamp_ == pending_obstacle_stamp_ ? pending_obstacles_ :
      std::vector<Point2d>{});
    safety_gate_->markNewFrame();

    // Fit only a contiguous observed segment. Never bridge a FOV veto gap.
    const auto contiguous = [this](std::vector<Point2d> points) {
      points.erase(std::remove_if(points.begin(), points.end(), [this](const Point2d & p) {
        return p.x < model_config_.fit_x_min || p.x > model_config_.fit_x_max;
      }), points.end());
      std::sort(points.begin(), points.end(), [](const Point2d & a, const Point2d & b) {return a.x < b.x;});
      std::size_t begin = 0, best_begin = 0, best_end = 0;
      for (std::size_t i = 0; i <= points.size(); ++i) {
        if (i == points.size() || (i > 0 && points[i].x - points[i - 1].x > 0.20)) {
          if (i - begin > best_end - best_begin) {best_begin = begin; best_end = i;}
          begin = i;
        }
      }
      return std::vector<Point2d>(points.begin() + best_begin, points.begin() + best_end);
    };
    const auto left_points = contiguous(pending_left_points_);
    const auto right_points = contiguous(pending_right_points_);
    // Input adaptation already smooths boundaries; refit each frame without carrying
    // coefficients across visibility gaps or changes in the chosen segment.
    left_boundary_model_->reset();
    right_boundary_model_->reset();
    const auto left = left_boundary_model_->fit(left_points);
    const auto right = right_boundary_model_->fit(right_points);
    latest_corridor_ = BoundaryCorridor{};
    latest_curve_ = controller_config_.side == ReferenceSide::LEFT ? left : right;
    if (!latest_curve_.valid) {latest_curve_ = left.valid ? left : right;}
    auto path_config = controller_config_;
    if (left.valid && right.valid &&
      std::min(left.x_max, right.x_max) - std::max(left.x_min, right.x_min) >= model_config_.minimum_x_span)
    {
      latest_corridor_.left = left;
      latest_corridor_.right = right;
      latest_corridor_.margin = vfh_config_.inflation_radius;
      latest_corridor_.valid = true;
      latest_curve_ = left;
      latest_curve_.a0 = (left.a0 + right.a0) * 0.5;
      latest_curve_.a1 = (left.a1 + right.a1) * 0.5;
      latest_curve_.a2 = (left.a2 + right.a2) * 0.5;
      latest_curve_.x_min = latest_corridor_.xMin();
      latest_curve_.x_max = latest_corridor_.xMax();
      path_config.boundary_offset = 0.0;
    } else {
      path_config.side = left.valid && !right.valid ? ReferenceSide::LEFT :
        (right.valid && !left.valid ? ReferenceSide::RIGHT : controller_config_.side);
    }
    const BoundaryController reference_builder(path_config);
    if (path_config.side != active_reference_side_) {
      active_reference_side_ = path_config.side;
      stable_planner_ = std::make_unique<StableLocalPlanner>(stable_planner_config_, active_reference_side_);
    }
    ReferencePath boundary_path = reference_builder.buildReferencePath(latest_curve_);
    boundaryless_active_ = !latest_curve_.valid && boundaryless_enabled_;
    latest_straight_safe_ = false;
    latest_boundary_distance_ = std::numeric_limits<double>::quiet_NaN();
    latest_decision_ = StablePlannerDecision{};
    if (boundaryless_active_) {
      // A forward target is independent of boundary fitting. Ground/obstacles
      // still verify every candidate; do not fabricate a valid boundary curve.
      stable_planner_->reset();
      boundary_model_->reset();
      boundary_path = controller_->buildConstantCurvaturePath(0.0, vfh_config_.horizon);
      latest_path_ = boundary_path;
      latest_straight_safe_ =
        safety_gate_->checkPathGeometry(latest_path_) == SafetyState::TRACKING;
    } else if (latest_corridor_.valid || !stability_enabled_) {
      stable_planner_->reset();
      latest_path_ = boundary_path;
      latest_decision_.select_straight = false;
      latest_decision_.selected_curvature = controller_->targetCurvature(boundary_path);
    } else if (!latest_curve_.valid) {
      stable_planner_->reset();
      latest_path_ = ReferencePath{};
    } else {
      double horizon = std::min(stability_path_horizon_, latest_curve_.x_max);
      if (!vfh_config_.enabled && std::isfinite(pending_front_range_) && pending_front_range_ > 0.0) {
        horizon = std::min(horizon, pending_front_range_);
      }
      const ReferencePath straight_path =
        controller_->buildConstantCurvaturePath(0.0, horizon);
      latest_straight_safe_ =
        safety_gate_->checkPathGeometry(straight_path) == SafetyState::TRACKING &&
        vfh_planner_->pathClearance(straight_path, *safety_gate_) >= vfh_config_.preferred_clearance;
      const double evaluation_x = std::clamp(
        boundary_evaluation_x_, latest_curve_.x_min, latest_curve_.x_max);
      latest_boundary_distance_ = active_reference_side_ == ReferenceSide::LEFT ?
        latest_curve_.y(evaluation_x) : -latest_curve_.y(evaluation_x);
      const double boundary_curvature = controller_->targetCurvature(boundary_path);
      latest_decision_ = stable_planner_->update(
        latest_straight_safe_, latest_boundary_distance_, boundary_curvature);
      latest_path_ = latest_decision_.select_straight ? straight_path :
        controller_->buildConstantCurvaturePath(
        latest_decision_.selected_curvature, horizon);
    }

    latest_nominal_path_ = latest_path_;
    if (vfh_config_.enabled) {
      latest_avoidance_ = vfh_planner_->plan(
        latest_nominal_path_, boundary_path,
        boundaryless_active_ ? vfh_config_.horizon : latest_curve_.x_max,
        *safety_gate_, latest_corridor_);
      latest_path_ = latest_avoidance_.path;
    }
    latest_header_ = pending_boundary_header_;
    const rclcpp::Time receipt = frame_time;
    last_boundary_receipt_ = receipt;
    last_grid_receipt_ = receipt;
    last_obstacle_receipt_ = receipt;
    last_committed_stamp_ = pending_boundary_stamp_;
    publishDebugPath();
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    enabled_ = message->data;
    safety_gate_->resetRecovery();
    controller_->reset();
    stable_planner_->reset();
    vfh_planner_->reset();
    RCLCPP_WARN(get_logger(), "camera navigation enable=%s", enabled_ ? "true" : "false");
  }

  void pauseCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    paused_ = message->data;
    safety_gate_->resetRecovery();
    controller_->reset();
    stable_planner_->reset();
    vfh_planner_->reset();
    RCLCPP_WARN(get_logger(), "camera navigation pause=%s", paused_ ? "true" : "false");
  }

  bool fresh(const rclcpp::Time & receipt, const rclcpp::Time & current) const
  {
    const double age = (current - receipt).seconds();
    return receipt.nanoseconds() > 0 && age >= -future_tolerance_ && age <= input_timeout_;
  }

  void controlTimer()
  {
    const rclcpp::Time current = now();
    safety_gate_->setInputsFresh(
      fresh(last_boundary_receipt_, current), fresh(last_grid_receipt_, current),
      fresh(last_obstacle_receipt_, current));
    SafetyState state = safety_gate_->evaluate(
      enabled_, paused_, latest_curve_, latest_path_, boundaryless_active_);
    if (state == SafetyState::TRACKING && !latest_corridor_.contains(latest_path_)) {
      state = SafetyState::PATH_BLOCKED;
    }

    if (vfh_config_.enabled && latest_avoidance_.mode == "BLOCKED" &&
      (latest_curve_.valid || boundaryless_active_) &&
      enabled_ && !paused_ && fresh(last_boundary_receipt_, current) &&
      fresh(last_grid_receipt_, current) && fresh(last_obstacle_receipt_, current))
    {state = SafetyState::PATH_BLOCKED;}
    ControlCommand command;
    if (state == SafetyState::TRACKING) {
      const double dt = std::clamp((current - last_control_time_).seconds(), 1e-3, 0.5);
      command = controller_->computeCommand(latest_path_, dt, vfh_config_.enabled ?
        latest_avoidance_.speed_limit : std::numeric_limits<double>::infinity());
      if (vfh_config_.enabled && !vfh_planner_->commandIsSafe(command, *safety_gate_, latest_corridor_)) {
        state = SafetyState::PATH_BLOCKED;
        command = ControlCommand{};
        controller_->reset();
        safety_gate_->resetRecovery();
      }
      if (!command.valid && state == SafetyState::TRACKING) {
        state = SafetyState::BOUNDARY_INVALID;
        controller_->reset();
      }
    } else {
      controller_->reset();
    }
    if (state == SafetyState::WAITING_FOR_INPUT &&
      (!fresh(last_boundary_receipt_, current) || !fresh(last_grid_receipt_, current) ||
      !fresh(last_obstacle_receipt_, current))) {vfh_planner_->reset();}
    last_control_time_ = current;

    geometry_msgs::msg::Twist output;
    if (state == SafetyState::TRACKING && command.valid) {
      output.linear.x = command.linear;
      output.angular.z = command.angular;
    }
    if (state == SafetyState::TRACKING || publish_zero_when_stopped_ ||
      last_state_ == SafetyState::TRACKING)
    {
      cmd_pub_->publish(output);
    }
    publishStatus(current, state, command);

    if (state != last_state_) {
      if (state == SafetyState::TRACKING) {
        RCLCPP_INFO(get_logger(), "camera navigation state -> %s", toString(state).c_str());
      } else {
        RCLCPP_WARN(get_logger(), "camera navigation state -> %s; commanding stop", toString(state).c_str());
      }
      last_state_ = state;
    }
  }

  nav_msgs::msg::Path pathMessage(const ReferencePath & path) const
  {
    nav_msgs::msg::Path message;
    message.header = latest_header_;
    message.header.frame_id = output_frame_;
    if (!path.valid) {return message;}
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = path.points[i].x;
      pose.pose.position.y = path.points[i].y;
      double yaw = 0.0;
      if (i + 1 < path.points.size()) {
        yaw = std::atan2(path.points[i + 1].y - path.points[i].y,
          path.points[i + 1].x - path.points[i].x);
      }
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      message.poses.push_back(pose);
    }
    return message;
  }

  void publishDebugPath()
  {
    path_pub_->publish(pathMessage(latest_path_));
    nominal_path_pub_->publish(pathMessage(latest_nominal_path_));
    geometry_msgs::msg::PointStamped target;
    target.header = latest_header_;
    target.header.frame_id = output_frame_;
    target.point.x = latest_path_.target.x;
    target.point.y = latest_path_.target.y;
    target_pub_->publish(target);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int id = 0;
    for (const auto & candidate : latest_avoidance_.candidates) {
      visualization_msgs::msg::Marker marker;
      marker.header = target.header;
      marker.ns = "candidates";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.012;
      marker.color.r = candidate.safe ? 0.0 : 1.0;
      marker.color.g = candidate.safe ? 1.0 : 0.0;
      marker.color.a = 0.35;
      for (const auto & p : candidate.path.points) {
        geometry_msgs::msg::Point point;
        point.x = p.x; point.y = p.y; point.z = 0.03;
        marker.points.push_back(point);
      }
      markers.markers.push_back(marker);
    }
    for (std::size_t i = 0; i < latest_avoidance_.histogram.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header = target.header;
      marker.ns = "histogram";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.02;
      marker.color.r = 1.0; marker.color.g = 0.5; marker.color.a = 0.8;
      const double angle = -vfh_config_.half_fov + 2.0 * vfh_config_.half_fov * i /
        (latest_avoidance_.histogram.size() - 1);
      geometry_msgs::msg::Point origin, tip;
      tip.x = std::min(2.0, latest_avoidance_.histogram[i]) * std::cos(angle);
      tip.y = std::min(2.0, latest_avoidance_.histogram[i]) * std::sin(angle);
      marker.points = {origin, tip};
      markers.markers.push_back(marker);
    }
    avoidance_markers_pub_->publish(markers);
  }

  static diagnostic_msgs::msg::KeyValue value(const std::string & key, const std::string & data)
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = data;
    return result;
  }

  void publishStatus(
    const rclcpp::Time & stamp, SafetyState state, const ControlCommand & command)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "camera_boundary_navigation";
    status.hardware_id = "depth_camera_boundary_follower";
    status.message = toString(state);
    status.level = state == SafetyState::TRACKING ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      (state == SafetyState::DISABLED || state == SafetyState::WAITING_FOR_INPUT ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status.values.push_back(value("enabled", enabled_ ? "true" : "false"));
    status.values.push_back(value("paused", paused_ ? "true" : "false"));
    status.values.push_back(value("reference_side", reference_side_name_));
    status.values.push_back(value("active_reference_side",
      active_reference_side_ == ReferenceSide::LEFT ? "left" : "right"));
    status.values.push_back(value("navigation_mode",
      boundaryless_active_ ? "BOUNDARYLESS_FORWARD" :
      (latest_corridor_.valid ? "CORRIDOR_CENTER" : "BOUNDARY_FOLLOW")));
    status.values.push_back(value("path_clearance", std::to_string(latest_avoidance_.clearance)));
    status.values.push_back(value("corridor_valid", latest_corridor_.valid ? "true" : "false"));
    status.values.push_back(value("corridor_usable", latest_corridor_.usable() ? "true" : "false"));
    status.values.push_back(value("boundary_valid", latest_curve_.valid ? "true" : "false"));
    status.values.push_back(value("boundary_confidence", std::to_string(latest_curve_.confidence)));
    status.values.push_back(value("boundary_rms", std::to_string(latest_curve_.rms_residual)));
    status.values.push_back(value("linear_command", std::to_string(command.linear)));
    status.values.push_back(value("angular_command", std::to_string(command.angular)));
    status.values.push_back(value("curvature", std::to_string(command.curvature)));
    status.values.push_back(value(
      "planner_mode", latest_decision_.select_straight ? "STRAIGHT" : "TURNING"));
    status.values.push_back(value(
      "straight_safe", latest_straight_safe_ ? "true" : "false"));
    status.values.push_back(value(
      "boundary_distance", std::to_string(latest_boundary_distance_)));
    status.values.push_back(value(
      "selected_path_curvature", std::to_string(latest_decision_.selected_curvature)));
    status.values.push_back(value(
      "turn_confirmation_count", std::to_string(latest_decision_.confirmation_count)));
    status.values.push_back(value("avoidance_mode", vfh_config_.enabled ? latest_avoidance_.mode : "OFF"));
    status.values.push_back(value("avoidance_reason", latest_avoidance_.reason));
    status.values.push_back(value("avoidance_speed_limit", std::to_string(latest_avoidance_.speed_limit)));
    status.values.push_back(value("avoidance_free_length", std::to_string(latest_avoidance_.free_length)));
    status.values.push_back(value("avoidance_curvature", std::to_string(latest_avoidance_.curvature)));
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  VfhConfig vfh_config_;
  VfhDecision latest_avoidance_;
  ReferencePath latest_nominal_path_;
  std::unique_ptr<VfhLocalPlanner> vfh_planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nominal_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr avoidance_markers_pub_;
  BoundaryModelConfig model_config_;
  BoundaryControllerConfig controller_config_;
  SafetyConfig safety_config_;
  StablePlannerConfig stable_planner_config_;
  std::unique_ptr<BoundaryModel> boundary_model_;
  std::unique_ptr<BoundaryModel> left_boundary_model_, right_boundary_model_;
  BoundaryCorridor latest_corridor_;
  std::unique_ptr<BoundaryController> controller_;
  std::unique_ptr<SafetyGate> safety_gate_;
  std::unique_ptr<StableLocalPlanner> stable_planner_;

  std::string boundary_topic_, obstacle_topic_, grid_topic_;
  std::string enable_topic_, pause_topic_, cmd_vel_topic_;
  std::string reference_path_topic_, target_topic_, status_topic_, output_frame_;
  std::string reference_side_name_;
  int selected_boundary_label_ = 1;
  ReferenceSide active_reference_side_ = ReferenceSide::RIGHT;
  bool enabled_ = false;
  bool paused_ = false;
  bool publish_zero_when_stopped_ = true;
  double control_frequency_ = 20.0;
  double input_timeout_ = 0.25;
  double future_tolerance_ = 0.0;
  bool stability_enabled_ = true;
  bool boundaryless_enabled_ = true;
  bool boundaryless_active_ = false;
  double stability_path_horizon_ = 2.5;
  double boundary_evaluation_x_ = 0.8;

  BoundaryCurve latest_curve_;
  ReferencePath latest_path_;
  StablePlannerDecision latest_decision_;
  bool latest_straight_safe_ = false;
  double latest_boundary_distance_ = std::numeric_limits<double>::quiet_NaN();
  std_msgs::msg::Header latest_header_;
  std_msgs::msg::Header pending_boundary_header_;
  std::vector<Point2d> pending_boundary_points_;
  std::vector<Point2d> pending_left_points_, pending_right_points_;
  std::vector<Point2d> pending_ground_cells_;
  std::vector<Point2d> pending_obstacles_;
  double pending_front_range_ = 0.0;
  std::int64_t pending_boundary_stamp_ = -1;
  std::int64_t pending_grid_stamp_ = -1;
  std::int64_t pending_obstacle_stamp_ = -1;
  std::int64_t last_committed_stamp_ = -1;
  rclcpp::Time last_boundary_receipt_;
  rclcpp::Time last_grid_receipt_;
  rclcpp::Time last_obstacle_receipt_;
  rclcpp::Time last_control_time_;
  SafetyState last_state_ = SafetyState::DISABLED;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // camera_boundary_navigation命名空间

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera_boundary_navigation::CameraBoundaryNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
