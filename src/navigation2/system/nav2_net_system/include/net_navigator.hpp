// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#ifndef NAV2_NET_SYSTEM__NET_NAVIGATOR_HPP_
#define NAV2_NET_SYSTEM__NET_NAVIGATOR_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_common/status.hpp"
#include "nav2_msgs/msg/log.hpp"
#include "nav2_msgs/msg/navigation_status.hpp"
#include "nav2_msgs/srv/navigation_request.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_net_system
{

/**
 * 导航状态机
 *
 *   IDLE ──(新goal)──► PLANNING ──(规划成功)──► EXECUTING
 *    ▲                     │                         │
 *    │                     │ 规划失败/被取消          │ goal 完成/被取消
 *    │                     ▼                         ▼
 *    └─────────────────── IDLE ◄─────────────────── IDLE
 */
enum class NavigatorState {
  IDLE = 0, PLANNING = 1, EXECUTING = 2, SUCCESS = 3,
  PLAN_FAILED = 4, EXEC_FAILED = 5, CANCELED = 6
};

/// Subscribes to goal_pose, calls topology ComputePathToPose, then FollowPath on controller_server.
class NetNavigator : public nav2_util::LifecycleNode
{
public:
  explicit NetNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NetNavigator() override = default;

  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:

  // 状态机核心
  NavigatorState state_{NavigatorState::IDLE};
  void setState(NavigatorState new_state);

  // 事件处理
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onCancelNavigation(std_msgs::msg::Empty::SharedPtr msg);

  // NavigateToPose action callbacks
  rclcpp_action::GoalResponse navToPoseGoalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse navToPoseCancelCallback(
    std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void navToPoseAcceptedCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void publishNavigateToPoseFeedback(
    std::shared_ptr<GoalHandleNavigateToPose> goal_handle, float distance_remaining);
  void answerNavigateToPoseResult(std::shared_ptr<GoalHandleNavigateToPose> goal_handle,
    const rclcpp_action::ResultCode code, const NavigateToPose::Result & result);
  void onPlanTick();

  // 各状态的处理入口（每帧只执行其中一个）
  void handleIdle();
  void handlePlanning(const geometry_msgs::msg::PoseStamped & goal);
  void handleExecuting();

  // 工具方法
  bool getRobotPose(geometry_msgs::msg::PoseStamped & robot_out);
  bool transformGoalToGlobalFrame(const geometry_msgs::msg::PoseStamped & goal_in,
                                   geometry_msgs::msg::PoseStamped & goal_out);
  void sendFollowPath(const nav_msgs::msg::Path & path, bool reaches_task_goal);
  void cancelActiveFollowPath();
  void cancelAllActive();
  static bool sameGoalXY(const geometry_msgs::msg::PoseStamped & a,
                         const geometry_msgs::msg::PoseStamped & b,
                         double tol);

  struct RouteSlice
  {
    nav_msgs::msg::Path path;
    bool reaches_task_goal{false};
  };

  /** 从 route 上机器人投影处往前截取 horizon_m 长度的路径段 */
  RouteSlice sliceRouteAhead(
    const nav_msgs::msg::Path & route, double rx, double ry, double horizon_m);

  // ============ 参数 ============
  std::string global_frame_;
  std::string robot_frame_;
  double planning_period_s_;
  double goal_match_tolerance_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string follow_path_action_;
  std::string compute_path_action_;
  double follow_slice_horizon_m_{30.0};
  double path_progress_min_translation_m_{0.05};
  double path_progress_max_projection_distance_m_{1.0};
  double path_progress_passed_point_margin_m_{0.10};
  int recovery_max_attempts_{2};
  // ============ ROS 资源 ============
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_client_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_to_pose_action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Log>::SharedPtr log_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<nav2_msgs::msg::NavigationStatus>::SharedPtr navigation_status_pub_;
  rclcpp::Service<nav2_msgs::srv::NavigationRequest>::SharedPtr navigation_request_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::string log_event_topic_;
  std::string log_record_topic_;

  // ============ 导航数据 ============
  std::recursive_mutex state_mutex_;
  geometry_msgs::msg::PoseStamped latest_goal_;
  geometry_msgs::msg::PoseStamped goal_snapshot_;  // 进入 PLANNING 时保存的目标快照
  nav_msgs::msg::Path mission_route_;              // 缓存的完整路网路径
  nav_msgs::msg::Path remaining_path_;            // 剩余路径（动态裁剪，用于切片）
  geometry_msgs::msg::PoseStamped last_path_progress_pose_;
  bool have_last_path_progress_pose_{false};

  // ============ 工具方法 ============
  void projectRobotOnPath(
    const nav_msgs::msg::Path & path, double rx, double ry,
    double & distance_out, double & arc_out, size_t & seg_idx_out);

  // ============ EXECUTING 状态下发 FollowPath 的节流控制 ============
  std::chrono::steady_clock::time_point last_follow_send_time_;
  double executing_freq_hz_{1.0};

  // Action handles
  GoalHandleFollowPath::SharedPtr active_follow_handle_;
  GoalHandleComputePath::SharedPtr active_compute_handle_;
  std::shared_ptr<GoalHandleNavigateToPose> active_nav_to_pose_handle_;
  std::chrono::steady_clock::time_point nav_to_pose_start_time_;
  uint64_t navigation_generation_{0};
  uint64_t compute_request_sequence_{0};
  uint64_t active_compute_request_sequence_{0};
  uint64_t follow_request_sequence_{0};
  uint64_t active_follow_request_sequence_{0};
  uint64_t accepted_follow_request_sequence_{0};
  bool final_follow_pending_{false};
  // 全局层恢复：路径控制失败后重新规划重试的次数（每次导航任务内计数）
  int exec_recovery_attempts_{0};

  // Planning request flag to avoid duplicate requests
  nav2_common::ExecStatus exec_status_{nav2_common::ExecStatus::OK};
  bool compute_request_sent_{false};
  std::chrono::steady_clock::time_point compute_request_sent_time_;
  double planning_timeout_s_{10.0};
  float distance_remaining_{-1.0F};
  std::string status_message_{"空闲"};

  void publishLog(const std::string& level, const std::string& msg);
  void publishNavStatus();
  void onNavigationRequest(
    const std::shared_ptr<nav2_msgs::srv::NavigationRequest::Request> request,
    std::shared_ptr<nav2_msgs::srv::NavigationRequest::Response> response);
  void startNavigation(const geometry_msgs::msg::PoseStamped & goal);
  void setFailure(NavigatorState state, const std::string & reason);
};

}  // namespace nav2_net_system

#endif  // NAV2_NET_SYSTEM__NET_NAVIGATOR_HPP_
