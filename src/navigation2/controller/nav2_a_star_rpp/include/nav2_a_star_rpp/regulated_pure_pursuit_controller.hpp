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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_a_star_rpp/costmap_path_smoother.hpp"
#include "nav2_a_star_rpp/theta_star.hpp"
#include "nav2_scan_planner/scan_backend.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_common/status.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nav2_a_star_rpp
{

/**
 * @class nav2_a_star_rpp::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_a_star_rpp::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for nav2_a_star_rpp::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param carrot_pose Pose of carrot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @param carrot_dist Distance to the carrot for PP
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &,
    const double &);

  /**
   * @brief checks for collision at projected pose
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @param theta orientation of Yaw
   * @return Whether in collision
   */
  bool inCollision(
    const double & x,
    const double & y,
    const double & theta);

  /**
   * @brief footprint cost at a pose; 0 when the scan backend does the checking
   */
  double footprintCostAt(
    const double & x,
    const double & y,
    const double & theta);
  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

  double approachVelocityScalingFactor(
    const nav_msgs::msg::Path & path
  ) const;

  void applyApproachVelocityScaling(
    const nav_msgs::msg::Path & path,
    double & linear_vel
  ) const;

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, const nav_msgs::msg::Path & path,
    double & linear_vel, double & sign);

  /** 根据上一周期输出指令和控制周期限制正常跟踪时的线加速度与线减速度。 */
  double limitLinearVelocity(double target_velocity, double current_velocity) const;

  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return Lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findVelocitySignChange(const nav_msgs::msg::Path & transformed_plan);

  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /** Index of path pose closest (XY) to robot; transforms robot into path frame if needed. */
  size_t findClosestPlanIndex(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot) const;

  /** Sample segment in costmap; false if off-map or cost >= inscribed inflated obstacle. */
  bool isPolylineSegmentFree(double x0, double y0, double x1, double y1) const;

  /** True if path[start_idx..end_idx] polyline is free at sampling resolution (path in costmap frame). */
  bool isPathPolylineClear(
    const nav_msgs::msg::Path & path, size_t start_idx, size_t end_idx) const;

  /** True if polyline global_plan_[start_idx..end_idx] is free at sampling resolution. */
  bool isGlobalPlanPolylineClear(size_t start_idx, size_t end_idx) const;

  /** Dense poses along path polyline from start_idx to end_idx (inclusive end). */
  nav_msgs::msg::Path densePathSlice(
    const nav_msgs::msg::Path & path, size_t start_idx, size_t end_idx, double step_m) const;

  /** Dense poses along global_plan polyline from start_idx to end_idx (inclusive end). */
  nav_msgs::msg::Path denseGlobalPlanSlice(
    size_t start_idx, size_t end_idx, double step_m) const;

  nav_msgs::msg::Path getLocalPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose);

  void getPlan(nav_msgs::msg::Path & path);

  /** Theta* 平滑器使用的中心点检查，与局部目标点相同，车身余量由膨胀层提供。 */
  bool isSmoothingPoseFree(double x, double y, double yaw) const;
  // 仅用于失败诊断，不改变碰撞判定或路径选择。
  std::string describeSmoothingPose(double x, double y, double yaw) const;

  nav_msgs::msg::Path getScanLocalPlan(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed);

  nav_msgs::msg::Path linearInterpolation(
    const std::vector<coordsW> & raw_path,
    const double & dist_bw_points);


  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief 门区三阶段状态机的观测结果（全部在机器人 base 坐标系下）。
   */
  struct DoorObservation
  {
    bool valid{false};
    double x{0.0};             // 门中心的前向距离
    double y{0.0};             // 门中心的左向（横向）偏移，左正右负
    double width{0.0};         // 两侧门柱内缘之间的净宽
    double left_inner{0.0};    // 左侧门柱内缘的横向坐标（正）
    double right_inner{0.0};   // 右侧门柱内缘的横向坐标（负）
  };

  /** 门区阶段：NONE 正常跟踪；PRE_ALIGN 依据已知门位对准走廊；ALIGN 对准；CROSS 居中穿越；EXIT 出门保持直行。 */
  enum class DoorPhase
  {
    NONE,
    PRE_ALIGN,
    ALIGN,
    CROSS,
    EXIT
  };

  /** 卡死自救阶段：NONE 正常；BACKUP 沿锁存航向直线后退（车尾方向先做 footprint 校验）。 */
  enum class RecoveryPhase
  {
    NONE,
    BACKUP
  };

  /** 收到点云后仅缓存，实际解算在控制周期内完成。 */
  void doorCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  /** 从最新点云中检测前方门框；成功时返回门中心与净宽（base 系）。 */
  bool detectDoor(
    const geometry_msgs::msg::PoseStamped & pose, DoorObservation & obs) const;

  /** 依据已知门位推算走廊对准误差：门须落在当前路线上，且车须位于门投影点
   *  之后的沿程距离 (0, door_enter_distance_] 内。输出走廊方向角（map 系）、
   *  车到走廊轴线的有符号横向距离（左正）与沿程距离。 */
  bool detectKnownDoor(
    const geometry_msgs::msg::PoseStamped & pose,
    double & corridor_heading, double & lateral_err, double & along_track) const;

  /** 按当前观测推进 ALIGN/CROSS/EXIT 状态机。 */
  void updateDoorPhase(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed);

  /** 门区阶段直接给出速度，绕开全局折线与 Theta*（门洞宽度不足以支撑常规判障）。 */
  void computeDoorCommand(
    const DoorObservation & obs, double & linear_vel, double & angular_vel) const;

  /** 退出门区模式并清空观测。 */
  void resetDoorPhase();

  /** 卡死检测与后退脱困。返回 true 表示本拍已由自救阶段接管（cmd 已填充）。 */
  bool updateRecovery(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    geometry_msgs::msg::TwistStamped & cmd);

  /** 车尾方向沿 -x_body 的可用净空（m），用 footprint 逐点校验，上限 max_dist。 */
  double rearClearDistance(
    const geometry_msgs::msg::PoseStamped & pose, double max_dist);

  /**
   * 车头正前方（车宽内）到最近致命障碍的净空（m），从车头外沿起算。
   * 只认代价地图里的 LETHAL_OBSTACLE(254)：253 是内切膨胀带，窄门/走廊里
   * 整片地面都被染成 253，用它测距会把地面当成墙；254 才是坡度层真正标记的
   * 障碍，能区分「前方有障碍」与「两侧门框/可通行坡面」。
   * 探测范围内无障碍时返回 +inf，保持原有行为。
   */
  double frontClearDistance(const geometry_msgs::msg::PoseStamped & pose);

  /** 净空不足时按比例降速；低于停车净空返回 0。 */
  double frontClearanceScale(double front_clear) const;

  /**
   * 前向停车保护：净空低于阈值时返回 true（调用方须把前向速度置零）。
   * 连续受阻达到阈值后触发层 1 后退自救；自救次数用尽则抛出
   * PlannerException，交回 net_navigator 重新规划全局路径。
   */
  bool frontStopActive(const geometry_msgs::msg::PoseStamped & pose, double front_clear);

  /** 进入后退阶段；失败（车尾受阻/次数用尽）时返回 false。 */
  bool startRecovery(const char * reason, const geometry_msgs::msg::PoseStamped & pose);

  /** 结束后退：清空冻结的绕障跟踪态，让下一拍从新位姿重新规划。 */
  void finishRecovery();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_, base_desired_linear_vel_;
  double max_linear_accel_{0.3};
  double max_linear_decel_{0.6};
  double last_linear_command_{0.0};
  // Opt-in telemetry only; never feeds back into velocity computation.
  bool speed_diagnostics_enabled_{false};
  unsigned long long diagnostic_plan_sequence_{0};
  double diagnostic_curvature_velocity_{0.0};
  double diagnostic_cost_velocity_{0.0};
  double diagnostic_pre_approach_velocity_{0.0};
  const char * diagnostic_plan_source_{"unset"};
  double lookahead_dist_;
  double rotate_to_heading_angular_vel_;
  double max_lookahead_dist_;
  double min_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_dist_;
  tf2::Duration transform_tolerance_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  double control_duration_;
  double max_allowed_time_to_collision_up_to_carrot_;
  bool use_collision_detection_;
  bool use_regulated_linear_velocity_scaling_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double cost_scaling_dist_;
  double cost_scaling_gain_;
  double inflation_cost_scaling_factor_;
  double regulated_linear_scaling_min_radius_;
  double regulated_linear_scaling_min_speed_;
  bool use_rotate_to_heading_;
  double max_angular_accel_;
  double rotate_to_heading_min_angle_;
  double goal_dist_tol_;
  bool allow_reversing_;
  double max_robot_pose_search_dist_;
  bool use_interpolation_;
  /** If true and polyline is clear on costmap, track densified global plan; else Theta* (avoidance). */
  bool prefer_planner_polyline_when_clear_{false};
  double polyline_collision_check_resolution_{0.05};
  /** Max arc length (m) along global plan to densify from closest index toward goal. */
  double polyline_track_max_length_{15.0};

  /** 对加密后的 Theta* 路径进行可选的碰撞约束平滑。 */
  bool path_smoothing_enabled_{false};
  CostmapPathSmootherParameters smoothing_parameters_;
  CostmapPathSmoother path_smoother_;

  /** Publish std_msgs/Bool: true while global slices should stay frozen (see avoid_flag_). */
  bool publish_global_path_slice_freeze_{true};
  /** If true, publish merge_path_connection_pose (PoseStamped) at Theta* / global tail splice for RViz. */
  bool publish_merge_connection_point_{true};

  /** a.hpp: global_path — upstream plan from planner / navigator. */
  nav_msgs::msg::Path global_plan_;
  /** a.hpp: local_path / 本拍输出 — 控制器给出的局部跟线路径（折线密化或 Theta* 段等）。 */
  nav_msgs::msg::Path local_plan_;

  /** a.hpp: avoid_flag — 避障态：跟 track_path_；冻结全局切片直到到达衔接点。 */
  bool avoid_flag_{false};

  /** a.hpp: track_path — Theta* 段 + global 剩余；遇障跟线几何。 */
  nav_msgs::msg::Path track_path_;
  /** a.hpp: 拼接 global 剩余时，从 global_plan_ 该下标开始（= global 远点下标 + 1）。 */
  size_t global_path_tail_start_idx_{0};
  /** a.hpp: 衔接点 join_idx_ — track_path_ 上全局尾巴起点下标（== 本拍 Theta* pose 数）。 */
  size_t join_idx_{0};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> theta_raw_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> theta_smoothed_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> freeze_slice_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> record_pub_;
  nav2_common::ExecStatus exec_status_{nav2_common::ExecStatus::OK};
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>
  merge_connection_pose_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  std::unique_ptr<theta_star::ThetaStar> planner_;
  std::unique_ptr<nav2_scan_planner::ScanBackend> scan_;
  std::string local_planner_type_{"theta_star"};

  // ---- 门区三阶段（ALIGN / CROSS / EXIT）与门框测距伺服 ----
  bool door_servo_enabled_{false};
  std::string door_cloud_topic_{"cur_lid"};
  double door_cloud_timeout_{0.5};
  double door_detect_range_{2.6};
  double door_detect_half_width_{1.2};
  double door_min_height_{0.4};
  double door_max_height_{2.6};
  double door_min_width_{0.55};
  double door_max_width_{1.15};
  double door_enter_distance_{2.2};
  double door_align_angle_{0.08};
  double door_align_rate_{0.45};
  double door_cross_speed_{0.30};
  double door_lateral_gain_{1.6};
  double door_max_angular_{0.45};
  double door_exit_margin_{0.35};
  double door_exit_hold_{0.40};
  double door_hold_distance_{0.50};
  int door_min_points_{5};
  /** 门心到当前路线的最大允许距离；超出则该“门”不视为路线上的门。 */
  double door_route_tolerance_{0.40};
  /** 已知门心（map 系，平坦 [x1,y1,x2,y2,...]）。门是固定静态特征：即便车尚未
   *  面向门、LiDAR 看不到门柱，也能据此启动 PRE_ALIGN，打破“须先看到才能对准”的死循环。 */
  std::vector<double> door_positions_;
  /** PRE_ALIGN 阶段：已知门走廊方向与车航向之差（map 系，已归一化到 [-pi, pi]）。 */
  double door_corridor_heading_err_{0.0};
  /** PRE_ALIGN 阶段：车到已知门走廊轴线的有符号横向距离（左正，map 系）。 */
  double door_corridor_lateral_err_{0.0};

  DoorPhase door_phase_{DoorPhase::NONE};
  DoorObservation door_obs_;
  /** 最近一次有效门观测的时间戳（秒）。 */
  double door_obs_time_{-1.0};
  /** CROSS 阶段自进入以来累计前进距离，用于判断整车是否已完全出门。 */
  double door_cross_travel_{0.0};
  /** CROSS 阶段需要累计前进的距离（进门距离 + 出门余量）。 */
  double door_cross_target_{0.0};
  /** 贴近门框后锁定门框观测，避免门柱擦过激光时测宽收缩导致误判丢失。 */
  bool door_cross_locked_{false};
  /** EXIT 阶段保持直行的截止时刻。 */
  double door_exit_deadline_{0.0};
  /** 上一控制周期的时间戳，用于积分前进距离。 */
  double door_last_time_{-1.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr door_cloud_sub_;
  mutable std::mutex door_cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr door_cloud_;

  // ---- 卡死自救（后退脱困）----
  bool recovery_enabled_{false};
  /** 一次后退的目标距离（m）。 */
  double recovery_backup_distance_{0.35};
  /** 后退速度（m/s，取正值，下发时取负）。 */
  double recovery_backup_speed_{0.12};
  /** 后退终点之外要求保留的净空（m），防止把车尾顶到障碍上。 */
  double recovery_rear_clearance_{0.10};
  /** 有速度指令却持续无位移多久判定为卡死（s）。 */
  double recovery_stuck_time_{1.5};
  /** 上述窗口内小于该位移即视为无位移（m）。 */
  double recovery_stuck_translation_{0.05};
  /** 同一路段允许的最大自救次数，超过后不再后退，交由上层处理。 */
  int recovery_max_cycles_{2};
  /** 距上次自救超过该时间即重置次数（s），避免一次失败后永久失效。 */
  double recovery_retrigger_window_{10.0};
  /** 连续被预测碰撞拦截多少控制周期后触发自救。 */
  int recovery_collision_cycles_threshold_{5};
  /** 连续局部规划无解多少控制周期后触发自救。 */
  int recovery_no_path_cycles_threshold_{3};

  RecoveryPhase recovery_phase_{RecoveryPhase::NONE};
  /** BACKUP 期间锁存的航向（map 系），用于直线后退。 */
  double recovery_lock_yaw_{0.0};
  /** 本次已后退距离（m），按位姿实测位移累计，而不是按指令速度积分。 */
  double recovery_travel_{0.0};
  /** 本次计划后退距离（m），受车尾净空限制。 */
  double recovery_target_{0.0};
  /** 后退起点（map 系），用于按实测位移判断后退是否真的发生。 */
  double recovery_start_x_{0.0};
  double recovery_start_y_{0.0};
  /** 本次后退开始时刻（s），用于判断"给了后退指令但车没动"。 */
  double recovery_start_time_{-1.0};
  /** 上次判定"仍在后退"时的位移，以及该时刻，用于检测后退中途被顶住。 */
  double recovery_last_progress_travel_{0.0};
  double recovery_progress_time_{-1.0};
  double recovery_anchor_x_{0.0};
  double recovery_anchor_y_{0.0};
  double recovery_anchor_time_{-1.0};
  bool recovery_anchor_valid_{false};
  int recovery_cycles_{0};
  double recovery_last_trigger_time_{-1.0};
  /** 本拍被预测碰撞拦截的次数，下一拍用于判定。 */
  int recovery_collision_cycles_{0};
  /** 连续局部规划无解的控制周期数。 */
  int recovery_no_path_cycles_{0};

  // ---- 前向停车保护（只认代价地图 LETHAL_OBSTACLE=254，绕过饱和的 253 内切带）----
  /** 车头到前方障碍的最小允许净空（m），低于该值不再前向行驶。 */
  double front_stop_clearance_{0.35};
  /** 开始线性降速的净空（m）：从该值降到停车净空时速度线性减到 0，避免高速顶墙。 */
  double front_slow_clearance_{0.95};
  /** 前向探测横向半宽（m）：只覆盖车体宽度（0.23），把窄门两侧 0.35 m 的门框排除在外。 */
  double front_stop_half_width_{0.20};
  /** 连续受阻的控制周期数，达到后触发自救（10 Hz 下 3 拍约 0.3 s）。 */
  int recovery_front_block_cycles_{3};
  /** 本拍之前连续受阻的周期计数。 */
  int front_block_cycles_{0};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_a_star_rpp

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
