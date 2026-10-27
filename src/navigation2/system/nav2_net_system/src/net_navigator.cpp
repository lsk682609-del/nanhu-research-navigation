// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#include "net_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <filesystem>

#include <sstream>
#include <fmt/format.h>
#include "nav2_msgs/msg/log.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "rclcpp_action/types.hpp"

namespace
{
const char * stateStr(nav2_net_system::NavigatorState s)
{
  using NS = nav2_net_system::NavigatorState;
  switch (s) {
    case NS::IDLE:      return "IDLE";
    case NS::PLANNING:  return "PLANNING";
    case NS::EXECUTING:    return "EXECUTING";
    case NS::SUCCESS:      return "SUCCESS";
    case NS::CANCELED:     return "CANCELED";
    case NS::PLAN_FAILED:  return "PLAN_FAILED";
    case NS::EXEC_FAILED:  return "EXEC_FAILED";
    default:               return "UNKNOWN";
  }
}

const char * actionResultCodeStr(const rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::UNKNOWN:    return "UNKNOWN";
    case rclcpp_action::ResultCode::SUCCEEDED:  return "SUCCEEDED";
    case rclcpp_action::ResultCode::CANCELED:   return "CANCELED";
    case rclcpp_action::ResultCode::ABORTED:    return "ABORTED";
    default:                                    return "UNRECOGNIZED";
  }
}
}  // namespace

namespace nav2_net_system
{

void NetNavigator::publishLog(const std::string& level, const std::string& msg)
{
  auto m = std::make_shared<nav2_msgs::msg::Log>();
  m->level = level;
  m->node_name = "net_navigator";
  m->message = msg;
  log_pub_->publish(*m);
}

void NetNavigator::publishNavStatus()
{
  if (nav_status_pub_) {
    auto legacy = std_msgs::msg::Int32();
    legacy.data = static_cast<int>(state_);
    nav_status_pub_->publish(legacy);
  }
  if (!navigation_status_pub_) {
    return;
  }
  nav2_msgs::msg::NavigationStatus msg;
  switch (state_) {
    case NavigatorState::IDLE:
      msg.state = nav2_msgs::msg::NavigationStatus::IDLE; break;
    case NavigatorState::PLANNING:
      msg.state = nav2_msgs::msg::NavigationStatus::PLANNING; break;
    case NavigatorState::EXECUTING:
      msg.state = nav2_msgs::msg::NavigationStatus::NAVIGATING; break;
    case NavigatorState::SUCCESS:
      msg.state = nav2_msgs::msg::NavigationStatus::SUCCEEDED; break;
    case NavigatorState::CANCELED:
      msg.state = nav2_msgs::msg::NavigationStatus::CANCELED; break;
    case NavigatorState::PLAN_FAILED:
    case NavigatorState::EXEC_FAILED:
      msg.state = nav2_msgs::msg::NavigationStatus::FAILED; break;
  }
  msg.distance_remaining = distance_remaining_;
  msg.message = status_message_;
  navigation_status_pub_->publish(msg);
}

void NetNavigator::setFailure(NavigatorState state, const std::string & reason)
{
  status_message_ = reason;
  distance_remaining_ = -1.0F;
  setState(state);
}

void NetNavigator::startNavigation(const geometry_msgs::msg::PoseStamped & goal)
{
  cancelAllActive();
  latest_goal_ = goal;
  distance_remaining_ = -1.0F;
  status_message_ = "正在规划";
  exec_recovery_attempts_ = 0;
  setState(NavigatorState::PLANNING);
}

void NetNavigator::onNavigationRequest(
  const std::shared_ptr<nav2_msgs::srv::NavigationRequest::Request> request,
  std::shared_ptr<nav2_msgs::srv::NavigationRequest::Response> response)
{
  response->command_id = request->command_id;
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    response->message = "导航节点未激活";
    return;
  }
  if (request->cmd == "Cancel" || request->cmd == "cancel") {
    cancelAllActive();
    status_message_ = "导航已取消";
    distance_remaining_ = -1.0F;
    setState(NavigatorState::CANCELED);
    response->success = true;
    response->message = "取消指令已执行";
    return;
  }
  if (request->cmd != "New" && request->cmd != "new") {
    response->message = "不支持的命令，仅支持 New 或 Cancel";
    return;
  }
  const auto & p = request->target_pose.pose.position;
  const auto & q = request->target_pose.pose.orientation;
  if (request->target_pose.header.frame_id.empty()) {
    response->message = "目标坐标系不能为空";
    return;
  }
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    response->message = "目标位姿包含非法数值";
    return;
  }
  if (!compute_client_->action_server_is_ready()) {
    response->message = "路网规划服务未就绪";
    return;
  }
  if (!follow_client_->action_server_is_ready()) {
    response->message = "路径控制服务未就绪";
    return;
  }
  startNavigation(request->target_pose);
  response->success = true;
  response->message = "导航目标已受理";
}

NetNavigator::NetNavigator(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("net_navigator", "", options),
  last_follow_send_time_(std::chrono::steady_clock::time_point{}),
  nav_to_pose_start_time_(std::chrono::steady_clock::time_point{})
{
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("planning_period_s", 1.0);
  declare_parameter("goal_match_tolerance", 0.05);
  declare_parameter("controller_id", std::string("FollowPath"));
  declare_parameter("goal_checker_id", std::string("general_goal_checker"));
  declare_parameter("follow_path_action", std::string("/follow_path"));
  declare_parameter("compute_path_action", std::string("/compute_path_to_pose"));
  declare_parameter("follow_slice_horizon_m", 30.0);
  declare_parameter("path_progress_min_translation_m", 0.05);
  declare_parameter("path_progress_max_projection_distance_m", 1.0);
  declare_parameter("path_progress_passed_point_margin_m", 0.10);
  declare_parameter("executing_freq_hz", 1.0);
  declare_parameter("planning_timeout_s", 10.0);
  declare_parameter("recovery_max_attempts", 2);
  declare_parameter("log_event_topic", std::string("log/event"));
  declare_parameter("log_record_topic", std::string("log/record"));
}

nav2_util::CallbackReturn
NetNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_base_frame", robot_frame_);
  get_parameter("planning_period_s", planning_period_s_);
  get_parameter("goal_match_tolerance", goal_match_tolerance_);
  get_parameter("controller_id", controller_id_);
  get_parameter("goal_checker_id", goal_checker_id_);
  get_parameter("follow_path_action", follow_path_action_);
  get_parameter("compute_path_action", compute_path_action_);
  get_parameter("follow_slice_horizon_m", follow_slice_horizon_m_);
  get_parameter("path_progress_min_translation_m", path_progress_min_translation_m_);
  get_parameter("path_progress_max_projection_distance_m", path_progress_max_projection_distance_m_);
  get_parameter("path_progress_passed_point_margin_m", path_progress_passed_point_margin_m_);
  get_parameter("executing_freq_hz", executing_freq_hz_);
  get_parameter("planning_timeout_s", planning_timeout_s_);
  get_parameter("recovery_max_attempts", recovery_max_attempts_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::QoS(10),
    std::bind(&NetNavigator::onGoal, this, std::placeholders::_1));

  cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
    "cancel_navigation", rclcpp::QoS(1),
    std::bind(&NetNavigator::onCancelNavigation, this, std::placeholders::_1));

  global_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "global_path", rclcpp::QoS(1).transient_local());

  log_event_topic_ = get_parameter("log_event_topic").as_string();
  log_record_topic_ = get_parameter("log_record_topic").as_string();
  log_pub_ = create_publisher<nav2_msgs::msg::Log>(log_event_topic_, 10);
  record_pub_ = create_publisher<std_msgs::msg::String>(log_record_topic_, 10);

  nav_status_pub_ = create_publisher<std_msgs::msg::Int32>("nav_status", 10);
  navigation_status_pub_ = create_publisher<nav2_msgs::msg::NavigationStatus>(
    "navigation_status", rclcpp::QoS(1).reliable().transient_local());
  navigation_request_service_ = create_service<nav2_msgs::srv::NavigationRequest>(
    "navigation_request",
    std::bind(&NetNavigator::onNavigationRequest, this,
      std::placeholders::_1, std::placeholders::_2));
  status_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&NetNavigator::publishNavStatus, this));

  follow_client_ = rclcpp_action::create_client<FollowPath>(
    this, follow_path_action_);
  compute_client_ = rclcpp_action::create_client<ComputePathToPose>(
    this, compute_path_action_);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),  // 100Hz 状态机 tick
    std::bind(&NetNavigator::onPlanTick, this));
  timer_->cancel();

  nav_to_pose_action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "navigate_to_pose",
    [this](const rclcpp_action::GoalUUID & uuid,
           std::shared_ptr<const NavigateToPose::Goal> goal) {
      return navToPoseGoalCallback(uuid, goal);
    },
    [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      return navToPoseCancelCallback(goal_handle);
    },
    [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      navToPoseAcceptedCallback(goal_handle);
    });

  publishLog("info", fmt::format("configured map={} base={} compute={} follow={} period={:.2f}s",
    global_frame_, robot_frame_, compute_path_action_,
    follow_path_action_, planning_period_s_));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NetNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  createBond();
  if (timer_) {
    timer_->reset();
  }
  if (status_timer_) {
    status_timer_->reset();
  }
  publishLog("info", fmt::format("Activated (state={})", stateStr(state_)));
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NetNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  if (timer_) {
    timer_->cancel();
  }
  if (status_timer_) {
    status_timer_->cancel();
  }

  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  cancelAllActive();
  destroyBond();
  publishLog("info", "Deactivated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NetNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  goal_sub_.reset();
  global_path_pub_.reset();
  follow_client_.reset();
  compute_client_.reset();
  timer_.reset();
  nav_to_pose_action_server_.reset();
  cancel_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  nav_status_pub_.reset();
  navigation_status_pub_.reset();
  navigation_request_service_.reset();
  status_timer_.reset();

  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  active_follow_handle_.reset();
  active_compute_handle_.reset();
  active_nav_to_pose_handle_.reset();
  mission_route_.poses.clear();
  remaining_path_.poses.clear();
  have_last_path_progress_pose_ = false;
  ++navigation_generation_;
  final_follow_pending_ = false;
  goal_snapshot_ = geometry_msgs::msg::PoseStamped();
  last_follow_send_time_ = std::chrono::steady_clock::time_point{};

  setState(NavigatorState::IDLE);

  publishLog("info", "Cleaned up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NetNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  publishLog("info", "Shutdown");
  return nav2_util::CallbackReturn::SUCCESS;
}

// ============================================================================
// 状态机核心
// ============================================================================

void NetNavigator::setState(NavigatorState new_state)
{
  if (state_ != new_state) {
    publishLog("info", fmt::format("state_change {} -> {}", stateStr(state_), stateStr(new_state)));
    state_ = new_state;
    if (new_state == NavigatorState::SUCCESS) {
      distance_remaining_ = 0.0F;
      status_message_ = "导航成功";
    } else if (new_state == NavigatorState::EXECUTING) {
      status_message_ = "正在导航";
    } else if (new_state == NavigatorState::IDLE) {
      distance_remaining_ = -1.0F;
      status_message_ = "空闲";
    }
    publishNavStatus();  // 状态变化时立即发布
  }
}

// ============================================================================
// NavigateToPose Action Server
// ============================================================================

rclcpp_action::GoalResponse
NetNavigator::navToPoseGoalCallback(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  const auto & p = goal->pose.pose.position;
  publishLog("info", fmt::format("NavigateToPose goal (x={:.2f} y={:.2f})", p.x, p.y));
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
NetNavigator::navToPoseCancelCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  publishLog("info", "NavigateToPose canceled");
  (void)goal_handle;

  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  cancelAllActive();
  setState(NavigatorState::IDLE);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void NetNavigator::navToPoseAcceptedCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  cancelAllActive();

  active_nav_to_pose_handle_ = goal_handle;
  nav_to_pose_start_time_ = std::chrono::steady_clock::now();

  // 触发 PLANNING
  latest_goal_ = goal_handle->get_goal()->pose;

  {
    const auto & gp = latest_goal_.pose.position;
    const auto & go = latest_goal_.pose.orientation;
    double gr, gpi, gy;
    tf2::Matrix3x3(tf2::Quaternion(go.x, go.y, go.z, go.w)).getRPY(gr, gpi, gy);
    auto m = std_msgs::msg::String();
    m.data = fmt::format("{:.6f},{},{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}",
      now().seconds(), "net", "goal", static_cast<int>(exec_status_),
      gp.x, gp.y, gp.z, gr, gpi, gy);
    record_pub_->publish(m);
  }

  setState(NavigatorState::PLANNING);
}

void NetNavigator::publishNavigateToPoseFeedback(
  std::shared_ptr<GoalHandleNavigateToPose> goal_handle, float distance_remaining)
{
  if (!goal_handle || !goal_handle->is_active()) {
    return;
  }

  geometry_msgs::msg::PoseStamped robot;
  if (!getRobotPose(robot)) {
    return;
  }

  NavigateToPose::Feedback feedback;
  feedback.current_pose = robot;
  feedback.navigation_time = rclcpp::Duration::from_seconds(
    std::chrono::duration<double>(std::chrono::steady_clock::now() - nav_to_pose_start_time_).count());
  feedback.distance_remaining = distance_remaining;

  goal_handle->publish_feedback(std::make_shared<NavigateToPose::Feedback>(feedback));
}

void NetNavigator::answerNavigateToPoseResult(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToPose>> goal_handle,
  const rclcpp_action::ResultCode code,
  const NavigateToPose::Result & /*result*/)
{
  if (!goal_handle || !goal_handle->is_active()) {
    return;
  }

  const char * result_str = "unknown";
  if (code == rclcpp_action::ResultCode::SUCCEEDED) result_str = "succeeded";
  else if (code == rclcpp_action::ResultCode::CANCELED) result_str = "canceled";
  else if (code == rclcpp_action::ResultCode::ABORTED) result_str = "aborted";
  publishLog("info", fmt::format("NavigateToPose result={}", result_str));

  auto result_msg = std::make_shared<NavigateToPose::Result>();
  if (code == rclcpp_action::ResultCode::SUCCEEDED) {
    goal_handle->succeed(result_msg);
  } else if (code == rclcpp_action::ResultCode::CANCELED) {
    if (goal_handle->is_executing()) {
      try {
        goal_handle->canceled(result_msg);
      } catch (const rclcpp::exceptions::RCLError &) {
        goal_handle->abort(result_msg);
      }
    } else {
      goal_handle->abort(result_msg);
    }
  } else {
    goal_handle->abort(result_msg);
  }
}

// ============================================================================
// 事件处理
// ============================================================================

void NetNavigator::onCancelNavigation(std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  cancelAllActive();
  const auto & last = latest_goal_.pose.position;
  publishLog("info", fmt::format("Goal canceled (x={:.2f} y={:.2f})", last.x, last.y));
  status_message_ = "导航已取消";
  distance_remaining_ = -1.0F;
  setState(NavigatorState::CANCELED);
}

void NetNavigator::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 非激活状态忽略 goal
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    publishLog("warn", "Ignoring goal: not active");
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  latest_goal_ = *msg;
  cancelAllActive();
  const auto & last = msg->pose.position;
  publishLog("info", fmt::format("Goal received (x={:.2f} y={:.2f})", last.x, last.y));

  {
    const auto & gp = msg->pose.position;
    const auto & go = msg->pose.orientation;
    double gr, gpi, gy;
    tf2::Matrix3x3(tf2::Quaternion(go.x, go.y, go.z, go.w)).getRPY(gr, gpi, gy);
    auto m = std_msgs::msg::String();
    m.data = fmt::format("{:.6f},{},{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}",
      now().seconds(), "net", "goal", static_cast<int>(exec_status_),
      gp.x, gp.y, gp.z, gr, gpi, gy);
    record_pub_->publish(m);
  }

  status_message_ = "正在规划";
  distance_remaining_ = -1.0F;
  exec_recovery_attempts_ = 0;
  setState(NavigatorState::PLANNING);
}

// ============================================================================
// 主循环：分派到各状态处理
// ============================================================================

void NetNavigator::onPlanTick()
{
  // 非激活状态不做任何事
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(state_mutex_);

  switch (state_) {
    case NavigatorState::IDLE:
      handleIdle();
      break;
    case NavigatorState::PLANNING:
      // PLANNING 需要当前目标（转换到 map 坐标系）
      {
        geometry_msgs::msg::PoseStamped goal_in_map;
        {
          std::lock_guard<std::recursive_mutex> lock(state_mutex_);
          goal_in_map = latest_goal_;
        }
        if (!transformGoalToGlobalFrame(goal_in_map, goal_in_map)) {
          status_message_ = "目标坐标转换失败";
          publishNavStatus();
          return;
        }
        handlePlanning(goal_in_map);
      }
      break;
    case NavigatorState::EXECUTING:
      handleExecuting();
      break;
    case NavigatorState::SUCCESS:
    case NavigatorState::CANCELED:
    case NavigatorState::PLAN_FAILED:
    case NavigatorState::EXEC_FAILED:
      // 等待外部下发新目标，新 goal 会触发 PLANNING
      break;
  }
}

// ============================================================================
// 各状态处理
// ============================================================================

void NetNavigator::handleIdle()
{
  // IDLE：无事可做
}

void NetNavigator::handlePlanning(const geometry_msgs::msg::PoseStamped & goal)
{
  // 检查是否有未完成的规划请求
  if (compute_request_sent_) {
    // 检查超时
    auto elapsed = std::chrono::steady_clock::now() - compute_request_sent_time_;
    if (std::chrono::duration<double>(elapsed).count() > planning_timeout_s_) {
      publishLog("warn", fmt::format("planning_timeout ({:.1f}s > {:.1f}s)",
        std::chrono::duration<double>(elapsed).count(), planning_timeout_s_));
      compute_request_sent_ = false;
      setFailure(NavigatorState::PLAN_FAILED, "路网规划超时");
    }
    return;
  }

  // 检查 action server 就绪
  if (!compute_client_->action_server_is_ready()) {
    publishLog("warn", "compute_path_not_ready");
    return;
  }
  if (!follow_client_->action_server_is_ready()) {
    publishLog("warn", "follow_path_not_ready");
    return;
  }

  // 保存目标快照，用于后续判断目标是否在规划期间发生变化
  goal_snapshot_ = goal;
  const auto navigation_generation = navigation_generation_;
  const auto request_sequence = ++compute_request_sequence_;
  active_compute_request_sequence_ = request_sequence;

  // 发送规划请求
  ComputePathToPose::Goal cp_goal;
  cp_goal.use_start = false;              // 规划器自动找最近路网节点
  cp_goal.goal = goal;
  cp_goal.goal.header.stamp = now();
  cp_goal.goal.header.frame_id = global_frame_;

  rclcpp_action::Client<ComputePathToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this, navigation_generation, request_sequence](GoalHandleComputePath::SharedPtr handle) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (navigation_generation != navigation_generation_ ||
        request_sequence != active_compute_request_sequence_ ||
        state_ != NavigatorState::PLANNING)
      {
        if (handle && compute_client_) {
          compute_client_->async_cancel_goal(handle);
        }
        return;
      }
      if (!handle) {
        publishLog("warn", "ComputePathToPose rejected");
        compute_request_sent_ = false;
        setFailure(NavigatorState::PLAN_FAILED, "路网规划请求被拒绝");
        return;
      }
      active_compute_handle_ = handle;
    };

  opts.result_callback =
    [this, navigation_generation, request_sequence](
      const GoalHandleComputePath::WrappedResult & wrapped) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (navigation_generation != navigation_generation_ ||
        request_sequence != active_compute_request_sequence_ ||
        state_ != NavigatorState::PLANNING)
      {
        return;
      }
      active_compute_handle_.reset();
      compute_request_sent_ = false;

      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
        publishLog("warn", fmt::format("ComputePathToPose failed ({})", actionResultCodeStr(wrapped.code)));
        setFailure(NavigatorState::PLAN_FAILED, "路网规划失败");
        if (active_nav_to_pose_handle_) {
          NavigateToPose::Result result;
          answerNavigateToPoseResult(active_nav_to_pose_handle_,
            wrapped.code, result);
          active_nav_to_pose_handle_.reset();
        }
        return;
      }

      const auto & path = wrapped.result->path;
      if (path.poses.empty()) {
        publishLog("warn", "ComputePathToPose returned empty path");
        setFailure(NavigatorState::PLAN_FAILED, "路网规划返回空路径");
        if (active_nav_to_pose_handle_) {
          NavigateToPose::Result result;
          answerNavigateToPoseResult(active_nav_to_pose_handle_,
            rclcpp_action::ResultCode::ABORTED, result);
          active_nav_to_pose_handle_.reset();
        }
        return;
      }

      if (path.header.frame_id != global_frame_ ||
        !sameGoalXY(path.poses.back(), goal_snapshot_, goal_match_tolerance_))
      {
        const auto & end = path.poses.back().pose.position;
        const auto & goal = goal_snapshot_.pose.position;
        publishLog("warn", fmt::format(
          "ComputePathToPose endpoint mismatch: frame={} end=({:.3f},{:.3f}) "
          "task_goal=({:.3f},{:.3f})",
          path.header.frame_id, end.x, end.y, goal.x, goal.y));
        setFailure(NavigatorState::PLAN_FAILED, "规划路径终点与任务目标不一致");
        if (active_nav_to_pose_handle_) {
          NavigateToPose::Result result;
          answerNavigateToPoseResult(active_nav_to_pose_handle_,
            rclcpp_action::ResultCode::ABORTED, result);
          active_nav_to_pose_handle_.reset();
        }
        return;
      }

      // 只保存路径、切换状态。下一帧 handleExecuting 会负责切片和发送 FollowPath
      mission_route_ = path;
      remaining_path_ = path;  // 初始化剩余路径
      have_last_path_progress_pose_ = false;
      nav_to_pose_start_time_ = std::chrono::steady_clock::now();
      setState(NavigatorState::EXECUTING);
    };

  compute_client_->async_send_goal(cp_goal, opts);
  compute_request_sent_ = true;
  compute_request_sent_time_ = std::chrono::steady_clock::now();
  publishLog("debug", "Sent ComputePathToPose request");
}

void NetNavigator::handleExecuting()
{
  // =========================================================================
  // 1. 节流
  // =========================================================================
  auto now_time = std::chrono::steady_clock::now();
  auto executing_period = std::chrono::milliseconds(static_cast<int>(1000.0 / executing_freq_hz_));

  if (now_time - last_follow_send_time_ < executing_period) {
    return;
  }
  last_follow_send_time_ = now_time;

  exec_status_ = nav2_common::ExecStatus::OK;

  geometry_msgs::msg::PoseStamped robot;
  if (!getRobotPose(robot)) {
    exec_status_ = nav2_common::ExecStatus::NET_NO_ROBOT_POS;
    status_message_ = "无法获取机器人位置";
    distance_remaining_ = -1.0F;
  }

  float distance_remaining = 0.0f;
  if (exec_status_ == nav2_common::ExecStatus::OK) {
    double projection_dist = 0.0;
    double robot_arc = 0.0;
    size_t seg_idx = 0;
    projectRobotOnPath(remaining_path_, robot.pose.position.x,
                       robot.pose.position.y, projection_dist, robot_arc, seg_idx);
    if (!have_last_path_progress_pose_) {
      last_path_progress_pose_ = robot;
      have_last_path_progress_pose_ = true;
    }
    const double movement = std::hypot(
      robot.pose.position.x - last_path_progress_pose_.pose.position.x,
      robot.pose.position.y - last_path_progress_pose_.pose.position.y);
    size_t trimmed_points = 0;
    if (remaining_path_.poses.size() > 2 &&
      movement >= path_progress_min_translation_m_ &&
      projection_dist <= path_progress_max_projection_distance_m_)
    {
      // Keep the last passed waypoint as the path's rear anchor. In particular,
      // never drop the first point merely because the robot projects onto its segment.
      double waypoint_arc = 0.0;
      for (size_t i = 1; i + 1 < remaining_path_.poses.size(); ++i) {
        const auto & previous = remaining_path_.poses[i - 1].pose.position;
        const auto & current = remaining_path_.poses[i].pose.position;
        waypoint_arc += std::hypot(current.x - previous.x, current.y - previous.y);
        if (waypoint_arc + path_progress_passed_point_margin_m_ > robot_arc) {
          break;
        }
        trimmed_points = i;
      }
      if (trimmed_points > 0) {
        remaining_path_.poses.erase(
          remaining_path_.poses.begin(), remaining_path_.poses.begin() + trimmed_points);
        last_path_progress_pose_ = robot;
      }
    }

    // Keep a separate progress record so a stop or a rotation can be checked
    // against the actual path cursor without changing the existing exec CSV schema.
    if (record_pub_) {
      std_msgs::msg::String progress_record;
      progress_record.data = fmt::format(
        "{:.6f},net,path_progress,movement={:.3f},projection={:.3f},arc={:.3f},"
        "trimmed={},remaining={}",
        now().seconds(), movement, projection_dist, robot_arc,
        trimmed_points, remaining_path_.poses.size());
      record_pub_->publish(progress_record);
    }

    nav_msgs::msg::Path remaining_path_r = remaining_path_;
    remaining_path_r.poses.insert(remaining_path_r.poses.begin(), robot);
    if (global_path_pub_) {
      nav_msgs::msg::Path viz = remaining_path_r;
      viz.header.stamp = now();
      global_path_pub_->publish(viz);
    }

    double total_remaining = 0;
    for (size_t i = 0; i + 1 < remaining_path_r.poses.size(); ++i) {
      const auto & a = remaining_path_r.poses[i].pose.position;
      const auto & b = remaining_path_r.poses[i + 1].pose.position;
      total_remaining += std::hypot(b.x - a.x, b.y - a.y);
    }
    distance_remaining = static_cast<float>(total_remaining);
    distance_remaining_ = distance_remaining;

    if (active_nav_to_pose_handle_) {
      publishNavigateToPoseFeedback(active_nav_to_pose_handle_, distance_remaining);
    }

    if (final_follow_pending_) {
      // The controller owns arrival at the task goal. Do not preempt its final
      // FollowPath with an identical slice while waiting for its action result.
    } else if (!follow_client_->action_server_is_ready()) {
      publishLog("warn", "FollowPath not ready, skipping slice");
      exec_status_ = nav2_common::ExecStatus::NET_FOLLOW_NOT_READY;
    }
    else {
      auto sliced = sliceRouteAhead(
        remaining_path_, robot.pose.position.x, robot.pose.position.y, follow_slice_horizon_m_);

      if (sliced.path.poses.size() < 2) {
        publishLog("warn", "Route slice too short, replan");
        setState(NavigatorState::PLANNING);
        exec_status_ = nav2_common::ExecStatus::NET_SLICE_TOO_SHORT;
      }
      else {
        sliced.path.header.stamp = now();
        sendFollowPath(sliced.path, sliced.reaches_task_goal);
      }
    }
  }

  {
    double rx = 0, ry = 0, rz = 0, rr = 0, rpi = 0, ryaw = 0;
    if (getRobotPose(robot)) {
      rx = robot.pose.position.x;
      ry = robot.pose.position.y;
      rz = robot.pose.position.z;
      const auto & ro = robot.pose.orientation;
      tf2::Matrix3x3(tf2::Quaternion(ro.x, ro.y, ro.z, ro.w)).getRPY(rr, rpi, ryaw);
    }
    double tx = 0, ty = 0, tz = 0, tr = 0, tpi = 0, tyaw = 0;
    if (!remaining_path_.poses.empty()) {
      tx = remaining_path_.poses.back().pose.position.x;
      ty = remaining_path_.poses.back().pose.position.y;
      tz = remaining_path_.poses.back().pose.position.z;
      const auto & go = remaining_path_.poses.back().pose.orientation;
      tf2::Matrix3x3(tf2::Quaternion(go.x, go.y, go.z, go.w)).getRPY(tr, tpi, tyaw);
    }
    auto m = std_msgs::msg::String();
    m.data = fmt::format("{:.6f},{},{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}",
      now().seconds(), "net", "exec", static_cast<int>(exec_status_),
      rx, ry, rz, rr, rpi, ryaw,
      distance_remaining,
      tx, ty, tz, tr, tpi, tyaw);
    record_pub_->publish(m);
  }
}

// ============================================================================
// 工具方法
// ============================================================================

bool NetNavigator::getRobotPose(geometry_msgs::msg::PoseStamped & robot_out)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);
    robot_out.header.frame_id = global_frame_;
    robot_out.header.stamp = tf.header.stamp;
    robot_out.pose.position.x = tf.transform.translation.x;
    robot_out.pose.position.y = tf.transform.translation.y;
    robot_out.pose.position.z = tf.transform.translation.z;
    robot_out.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    publishLog("warn", fmt::format("tf_lookup_failed error={}", ex.what()));
    return false;
  }
}

bool NetNavigator::transformGoalToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & goal_in,
  geometry_msgs::msg::PoseStamped & goal_out)
{
  if (goal_in.header.frame_id == global_frame_) {
    goal_out = goal_in;
    return true;
  }
  try {
    goal_out = tf_buffer_->transform(goal_in, global_frame_, tf2::durationFromSec(0.2));
    return true;
  } catch (const tf2::TransformException & ex) {
    publishLog("warn", fmt::format("tf_lookup_failed goal={}", ex.what()));
    return false;
  }
}

void NetNavigator::cancelAllActive()
{
  ++navigation_generation_;
  final_follow_pending_ = false;
  compute_request_sent_ = false;
  last_follow_send_time_ = std::chrono::steady_clock::time_point{};

  if (active_compute_handle_ && compute_client_) {
    compute_client_->async_cancel_goal(active_compute_handle_);
    active_compute_handle_.reset();
  }

  if (active_follow_handle_ && follow_client_) {
    follow_client_->async_cancel_goal(active_follow_handle_);
    active_follow_handle_.reset();
  }

  if (active_nav_to_pose_handle_) {
    auto result_msg = std::make_shared<NavigateToPose::Result>();
    try {
      active_nav_to_pose_handle_->canceled(result_msg);
    } catch (const rclcpp::exceptions::RCLError &) {
      // Goal not in cancelable state, let framework handle it
    }
    active_nav_to_pose_handle_.reset();
  }
}

void NetNavigator::sendFollowPath(
  const nav_msgs::msg::Path & path, const bool reaches_task_goal)
{
  FollowPath::Goal goal_msg;
  goal_msg.path = path;
  goal_msg.controller_id = controller_id_;
  goal_msg.goal_checker_id = goal_checker_id_;

  const auto navigation_generation = navigation_generation_;
  const auto request_sequence = ++follow_request_sequence_;
  active_follow_request_sequence_ = request_sequence;
  final_follow_pending_ = reaches_task_goal;

  rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this, navigation_generation, request_sequence, reaches_task_goal](
      GoalHandleFollowPath::SharedPtr handle) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (navigation_generation != navigation_generation_ ||
        request_sequence != active_follow_request_sequence_ ||
        state_ != NavigatorState::EXECUTING)
      {
        if (handle && follow_client_) {
          follow_client_->async_cancel_goal(handle);
        }
        return;
      }
      if (!handle) {
        final_follow_pending_ = false;
        publishLog("warn", fmt::format(
          "FollowPath goal rejected (request={} final={})",
          request_sequence, reaches_task_goal));
        setFailure(NavigatorState::EXEC_FAILED, "路径控制请求被拒绝");
        return;
      }
      active_follow_handle_ = handle;
      accepted_follow_request_sequence_ = request_sequence;
      const auto & id = handle->get_goal_id();
      publishLog("debug", fmt::format(
        "FollowPath accepted (request={} final={} id={:02x}{:02x}{:02x}{:02x})",
        request_sequence, reaches_task_goal,
        static_cast<unsigned>(id[0]), static_cast<unsigned>(id[1]),
        static_cast<unsigned>(id[2]), static_cast<unsigned>(id[3])));
    };

  opts.result_callback =
    [this, navigation_generation, request_sequence, reaches_task_goal](
      const GoalHandleFollowPath::WrappedResult & result) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (navigation_generation != navigation_generation_ ||
        request_sequence != active_follow_request_sequence_ ||
        state_ != NavigatorState::EXECUTING)
      {
        return;
      }
      if (active_follow_handle_ &&
        accepted_follow_request_sequence_ == request_sequence &&
        active_follow_handle_->get_goal_id() != result.goal_id)
      {
        publishLog("warn", fmt::format(
          "FollowPath result ID mismatch (request={})", request_sequence));
        if (reaches_task_goal) {
          final_follow_pending_ = false;
          setState(NavigatorState::PLANNING);
        }
        return;
      }
      active_follow_handle_.reset();
      const auto & id = result.goal_id;
      publishLog("info", fmt::format(
        "FollowPath finished (request={} final={} code={} id={:02x}{:02x}{:02x}{:02x})",
        request_sequence, reaches_task_goal, actionResultCodeStr(result.code),
        static_cast<unsigned>(id[0]), static_cast<unsigned>(id[1]),
        static_cast<unsigned>(id[2]), static_cast<unsigned>(id[3])));

      if (!reaches_task_goal) {
        return;
      }
      final_follow_pending_ = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        const auto & goal = goal_snapshot_.pose.position;
        publishLog("info", fmt::format(
          "Task goal reached by controller (x={:.2f} y={:.2f})", goal.x, goal.y));
        if (active_nav_to_pose_handle_) {
          NavigateToPose::Result nav_result;
          answerNavigateToPoseResult(active_nav_to_pose_handle_,
            rclcpp_action::ResultCode::SUCCEEDED, nav_result);
          active_nav_to_pose_handle_.reset();
        }
        setState(NavigatorState::SUCCESS);
      } else {
        publishLog("warn", "Final FollowPath did not succeed");
        // 全局层恢复：控制器自身自救（后退/局部重规划）已用尽仍失败，
        // 此处重新走一次路网规划再下发 FollowPath，给控制器一条新路径。
        if (exec_recovery_attempts_ < recovery_max_attempts_) {
          ++exec_recovery_attempts_;
          publishLog("warn", fmt::format(
            "[恢复] 路径控制失败（{}），重新规划重试 {}/{}",
            actionResultCodeStr(result.code),
            exec_recovery_attempts_, recovery_max_attempts_));
          status_message_ = "路径控制失败，正在重新规划";
          setState(NavigatorState::PLANNING);
        } else {
          setFailure(NavigatorState::EXEC_FAILED,
            fmt::format("路径控制失败：{}（重试 {} 次后放弃）",
              actionResultCodeStr(result.code), recovery_max_attempts_));
        }
      }
    };

  follow_client_->async_send_goal(goal_msg, opts);
  const auto & last = path.poses.back().pose.position;
  publishLog("debug", fmt::format(
    "Sent FollowPath (request={} final={} poses={} last_x={:.2f} last_y={:.2f})",
    request_sequence, reaches_task_goal, path.poses.size(), last.x, last.y));
}

bool NetNavigator::sameGoalXY(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b,
  const double tol)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return (dx * dx + dy * dy) <= tol * tol;
}

NetNavigator::RouteSlice NetNavigator::sliceRouteAhead(
  const nav_msgs::msg::Path & route, double rx, double ry, double horizon_m)
{
  RouteSlice slice;
  auto & out = slice.path;
  out.header = route.header;
  if (route.poses.size() < 2 || horizon_m <= 0.0) {
    return slice;
  }

  const size_t n = route.poses.size();
  std::vector<double> cum(n, 0.0);
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & a = route.poses[i].pose.position;
    const auto & b = route.poses[i + 1].pose.position;
    cum[i + 1] = cum[i] + std::hypot(b.x - a.x, b.y - a.y);
  }
  const double total = cum.back();
  if (total < 1e-6) {
    return slice;
  }

  double distance = 0.0;
  double best_arc = 0.0;
  size_t seg_idx_dummy = 0;
  projectRobotOnPath(route, rx, ry, distance, best_arc, seg_idx_dummy);

  const double end_arc = std::min(total, best_arc + horizon_m);
  if (end_arc <= best_arc + 1e-6) {
    return slice;
  }
  slice.reaches_task_goal = end_arc >= total - 1e-6;

  auto append_interp = [&out, &route, &cum](double s) {
    size_t k = 0;
    const size_t nv = route.poses.size();
    while (k + 1 < nv && cum[k + 1] < s - 1e-9) { ++k; }
    const auto & pa = route.poses[k].pose.position;
    const auto & pb = route.poses[k + 1].pose.position;
    const double seg_len = std::hypot(pb.x - pa.x, pb.y - pa.y);
    const double ds = s - cum[k];
    const double t = (seg_len > 1e-9) ? std::clamp(ds / seg_len, 0.0, 1.0) : 0.0;
    const double yaw = (seg_len > 1e-9) ? std::atan2(pb.y - pa.y, pb.x - pa.x) : 0.0;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = out.header;
    ps.pose.position.x = pa.x + t * (pb.x - pa.x);
    ps.pose.position.y = pa.y + t * (pb.y - pa.y);
    ps.pose.position.z = route.poses.front().pose.position.z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    ps.pose.orientation = tf2::toMsg(q);
    out.poses.push_back(ps);
  };

  const double step = 0.15;
  for (double s = best_arc; s <= end_arc + 1e-9; ) {
    append_interp(s);
    if (s >= end_arc - 1e-9) { break; }
    s = std::min(s + step, end_arc);
  }

  return slice;
}

void NetNavigator::projectRobotOnPath(
  const nav_msgs::msg::Path & path, double rx, double ry,
  double & distance_out, double & arc_out, size_t & seg_idx_out)
{
  if (path.poses.size() < 2) {
    distance_out = 1000.0;
    arc_out = 0.0;
    seg_idx_out = 0;
    return;
  }

  const size_t n = path.poses.size();
  std::vector<double> cum(n, 0.0);
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    cum[i + 1] = cum[i] + std::hypot(b.x - a.x, b.y - a.y);
  }

  double best_d2 = std::numeric_limits<double>::max();
  double best_arc = 0.0;
  size_t best_seg_idx = 0;
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double apx = rx - a.x;
    const double apy = ry - a.y;
    const double ab2 = abx * abx + aby * aby;
    double t = (ab2 > 1e-12) ? std::clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0) : 0.0;
    const double px = a.x + t * abx;
    const double py = a.y + t * aby;
    const double d2 = (rx - px) * (rx - px) + (ry - py) * (ry - py);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_arc = cum[i] + t * std::sqrt(ab2);
      best_seg_idx = i;
    }
  }

  distance_out = std::sqrt(best_d2);
  arc_out = best_arc;
  seg_idx_out = best_seg_idx;
}

}  // namespace nav2_net_system
