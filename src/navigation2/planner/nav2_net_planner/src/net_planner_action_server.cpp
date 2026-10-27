// Copyright 2026
// Standalone ComputePathToPose action server: topology path using map->base_link TF.
// LifecycleNode — managed by lifecycle_manager for ordered startup/shutdown & heartbeat.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_net_planner/path_dense_utils.hpp"
#include "nav2_net_planner/topo_map.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_common/sp_timer.h"
#include "nav2_msgs/msg/log.hpp"
#include <fmt/format.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

namespace nav2_net_planner
{

class NetPlannerActionServer : public nav2_util::LifecycleNode
{
public:
  explicit NetPlannerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2_util::LifecycleNode("net_planner_action_server", "", options)
  {
    declare_parameter<std::string>("topo_map_path", "");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_link");
    declare_parameter<double>("max_snap_distance", 15.0);
    declare_parameter<bool>("edge_snap_enabled", false);
    declare_parameter<double>("transform_tolerance", 0.5);
    declare_parameter<double>("dense_path_step", 0.1);
    declare_parameter<double>("max_dense_path_length", 0.0);
    declare_parameter<bool>("publish_planner_path", true);
    declare_parameter<std::string>("planner_path_topic", "topology_plan");
    declare_parameter<std::string>("navigation_global_path_topic", "navigation_global_path");
  }

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    topo_map_path_ = get_parameter("topo_map_path").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    max_snap_distance_ = get_parameter("max_snap_distance").as_double();
    edge_snap_enabled_ = get_parameter("edge_snap_enabled").as_bool();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();
    dense_path_step_ = get_parameter("dense_path_step").as_double();
    max_dense_path_length_ = get_parameter("max_dense_path_length").as_double();
    publish_planner_path_ = get_parameter("publish_planner_path").as_bool();
    planner_path_topic_ = get_parameter("planner_path_topic").as_string();
    navigation_global_path_topic_ = get_parameter("navigation_global_path_topic").as_string();

    if (topo_map_path_.empty() || !topo_.loadTopoMap(topo_map_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load topology: %s", topo_map_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded topology %s (%zu nodes)", topo_map_path_.c_str(), topo_.nodes.size());
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this, "compute_path_to_pose",
      std::bind(&NetPlannerActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NetPlannerActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&NetPlannerActionServer::handleAccepted, this, std::placeholders::_1));

    if (publish_planner_path_) {
      plan_pub_ = create_publisher<nav_msgs::msg::Path>(
        planner_path_topic_, rclcpp::QoS(1).transient_local());
      RCLCPP_INFO(get_logger(), "Publishing planner path on %s", plan_pub_->get_topic_name());
    }
    sparse_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
      navigation_global_path_topic_, rclcpp::QoS(1).reliable().transient_local());

    log_pub_ = create_publisher<nav2_msgs::msg::Log>("log/event", 10);

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    action_server_.reset();
    plan_pub_.reset();
    sparse_plan_pub_.reset();
    log_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    return on_cleanup(state);
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    (void)goal;
    if (topo_.nodes.empty()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<ServerGoalHandle> handle)
  {
    (void)handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<ServerGoalHandle> handle)
  {
    std::thread{std::bind(&NetPlannerActionServer::execute, this, handle)}.detach();
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & out) const
  {
    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = robot_base_frame_;
    in.header.stamp.sec = 0;
    in.header.stamp.nanosec = 0u;
    in.pose.orientation.w = 1.0;
    try {
      tf_buffer_->transform(in, out, global_frame_, tf2::durationFromSec(transform_tolerance_));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF %s->%s failed: %s", robot_base_frame_.c_str(), global_frame_.c_str(), e.what());
      return false;
    }
  }

  static nav_msgs::msg::Path buildPath(
    const TopoMap & topo,
    const std::vector<int> & node_ids,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::unordered_map<int, TopoMap::Node> * node_overlay = nullptr)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;

    // 路径首点：起点实际位置
    path.poses.push_back(start);

    std::unordered_map<int, TopoMap::Node> id2n;
    for (const auto & n : topo.nodes) {
      id2n[n.id] = n;
    }
    if (node_overlay) {
      for (const auto & kv : *node_overlay) {
        id2n[kv.first] = kv.second;
      }
    }

    for (size_t i = 0; i < node_ids.size(); ++i) {
      const auto it = id2n.find(node_ids[i]);
      if (it == id2n.end()) {
        continue;
      }
      const TopoMap::Node & nd = it->second;
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = static_cast<double>(nd.x);
      ps.pose.position.y = static_cast<double>(nd.y);
      ps.pose.position.z = static_cast<double>(nd.z);

      double yaw = 0.0;
      if (i + 1 < node_ids.size()) {
        const auto itn = id2n.find(node_ids[i + 1]);
        if (itn != id2n.end()) {
          const double dx = static_cast<double>(itn->second.x - nd.x);
          const double dy = static_cast<double>(itn->second.y - nd.y);
          yaw = std::atan2(dy, dx);
        }
      } else {
        const double dx = goal.pose.position.x - static_cast<double>(nd.x);
        const double dy = goal.pose.position.y - static_cast<double>(nd.y);
        yaw = (dx * dx + dy * dy > 1e-6) ? std::atan2(dy, dx) : tf2::getYaw(goal.pose.orientation);
      }
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      ps.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(ps);
    }
    // 路径末点：终点实际位置
    path.poses.push_back(goal);
    return path;
  }

  static nav_msgs::msg::Path maybeDensify(
    const nav_msgs::msg::Path & sparse, double dense_path_step, double max_dense_path_length)
  {
    if (dense_path_step <= 0.0 || sparse.poses.size() < 2) {
      return sparse;
    }
    return densifyPolyline(sparse, dense_path_step, max_dense_path_length);
  }

  /// SNN 规划结果状态码 → 可读名称，用于算法测试日志。
  static const char * snnStatusName(SnnPlanResult::Status status)
  {
    switch (status) {
      case SnnPlanResult::Status::Success: return "Success";
      case SnnPlanResult::Status::TopologyNotCached: return "TopologyNotCached";
      case SnnPlanResult::Status::InvalidStart: return "InvalidStart";
      case SnnPlanResult::Status::InvalidGoal: return "InvalidGoal";
      case SnnPlanResult::Status::InvalidDynamics: return "InvalidDynamics";
      case SnnPlanResult::Status::Unreachable: return "Unreachable";
    }
    return "Unknown";
  }

  /// 同时写终端与 /log/event（由 nav2_log_record 落盘）。
  void logSnnEvent(const std::string & message)
  {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    if (log_pub_) {
      auto m = std::make_shared<nav2_msgs::msg::Log>();
      m->level = "info";
      m->node_name = "snn";
      m->message = message;
      log_pub_->publish(*m);
    }
  }

  void execute(const std::shared_ptr<ServerGoalHandle> handle)
  {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<ComputePathToPose::Result>();
    const auto t0 = get_clock()->now();

    SPTimer timer(__func__, [this](const std::string& name, double ms) {
      if (!log_pub_) return;
      auto m = std::make_shared<nav2_msgs::msg::Log>();
      m->level = "info";
      m->node_name = "net_planner";
      m->message = fmt::format("{} time={:.1f}ms", name, ms);
      log_pub_->publish(*m);
    });

    geometry_msgs::msg::PoseStamped start = goal->start;
    if (!goal->use_start) {
      if (!getRobotPose(start)) {
        RCLCPP_WARN(
          get_logger(), "compute_path_to_pose: abort — TF %s→%s failed (cannot get robot pose)",
          robot_base_frame_.c_str(), global_frame_.c_str());
        result->path.header.frame_id = global_frame_;
        result->path.header.stamp = get_clock()->now();
        handle->abort(result);
        return;
      }
    }

    const auto & g = goal->goal;
    std::unordered_map<int, TopoMap::Node> extra_nodes;
    std::unordered_map<int, std::vector<std::pair<int, float>>> extra_adj;

    // ===== SNN 算法测试日志：调用前 =====
    logSnnEvent(fmt::format(
      "[SNN] plan request: start=({:.3f},{:.3f},{:.3f}) goal=({:.3f},{:.3f},{:.3f}) "
      "max_snap_dist={:.2f} edge_snap={}",
      start.pose.position.x, start.pose.position.y, start.pose.position.z,
      g.pose.position.x, g.pose.position.y, g.pose.position.z,
      max_snap_distance_, edge_snap_enabled_));

    // 用 steady_clock 计时：仿真下 use_sim_time 的 ROS 时钟每 100ms 才跳一次，
    // 会把微秒级的 SNN 耗时抹成 0.00ms。
    const auto snn_t0 = std::chrono::steady_clock::now();
    SnnPlanResult snn_result;
    std::vector<int> ids = topo_.planNodeIds(
        start.pose.position.x, start.pose.position.y, start.pose.position.z,
        g.pose.position.x, g.pose.position.y, g.pose.position.z,
        max_snap_distance_, edge_snap_enabled_, extra_nodes, extra_adj, &snn_result);
    const double snn_elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - snn_t0).count();

    // ===== SNN 算法测试日志：调用后 =====
    logSnnEvent(fmt::format(
      "[SNN] plan result: status={} planned_nodes={} total_delay={:.3f} "
      "start_spike_time={:.3f} fired_neurons={} synaptic_events={} ignored_edges={} "
      "elapsed={:.2f}ms",
      snnStatusName(snn_result.status), snn_result.nodeIds.size(),
      snn_result.totalDelay, snn_result.startSpikeTime,
      snn_result.firedNeuronCount, snn_result.processedSynapticEventCount,
      snn_result.ignoredEdgeCount, snn_elapsed_ms));

    if (ids.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "compute_path_to_pose: abort — snap or path failed (max_dist=%.2f, edge_snap=%d)",
        max_snap_distance_, edge_snap_enabled_);
      handle->abort(result);
      return;
    }

    const std::unordered_map<int, TopoMap::Node> * overlay =
      extra_nodes.empty() ? nullptr : &extra_nodes;
    nav_msgs::msg::Path sparse =
      buildPath(topo_, ids, global_frame_, get_clock()->now(), start, g, overlay);
    nav_msgs::msg::Path minimal;
    minimal.header = sparse.header;
    // buildPath adds the live robot pose and exact goal for control. The
    // operator topic deliberately contains only the planned topology nodes.
    for (size_t i = 1; i + 1 < sparse.poses.size(); ++i) {
      const auto & pose = sparse.poses[i];
      if (!minimal.poses.empty()) {
        const auto & previous = minimal.poses.back().pose.position;
        const auto & current = pose.pose.position;
        if (std::hypot(current.x - previous.x, current.y - previous.y) < 1e-4) {
          minimal.poses.back() = pose;
          continue;
        }
      }
      minimal.poses.push_back(pose);
    }
    sparse_plan_pub_->publish(minimal);
    result->path = maybeDensify(sparse, dense_path_step_, max_dense_path_length_);
    result->path.header.stamp = get_clock()->now();
    if (plan_pub_ && !result->path.poses.empty()) {
      plan_pub_->publish(result->path);
    }
    result->planning_time = static_cast<builtin_interfaces::msg::Duration>(get_clock()->now() - t0);
    handle->succeed(result);
  }

  TopoMap topo_;
  std::string topo_map_path_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double max_snap_distance_;
  bool edge_snap_enabled_{false};
  double transform_tolerance_;
  double dense_path_step_;
  double max_dense_path_length_;
  bool publish_planner_path_{true};
  std::string planner_path_topic_{"topology_plan"};
  std::string navigation_global_path_topic_{"navigation_global_path"};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sparse_plan_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Log>::SharedPtr log_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
};

}  // namespace nav2_net_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_net_planner::NetPlannerActionServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
