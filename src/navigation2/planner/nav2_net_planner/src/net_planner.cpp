// Copyright 2026

#include "nav2_net_planner/net_planner.hpp"
#include "nav2_net_planner/path_dense_utils.hpp"

#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace nav2_net_planner
{

NetPlanner::NetPlanner() = default;

NetPlanner::~NetPlanner()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type NetPlanner", name_.c_str());
}

void NetPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();
  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring plugin %s of type NetPlanner", name_.c_str());

  declare_parameter_if_not_declared(
    node, name + ".topo_map_path", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name + ".topo_map_path", topo_map_path_);

  declare_parameter_if_not_declared(node, name + ".use_tf_robot_pose", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_tf_robot_pose", use_tf_robot_pose_);

  declare_parameter_if_not_declared(
    node, name + ".robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  node->get_parameter(name + ".robot_base_frame", robot_base_frame_);

  declare_parameter_if_not_declared(node, name + ".max_snap_distance", rclcpp::ParameterValue(15.0));
  node->get_parameter(name + ".max_snap_distance", max_snap_distance_);

  declare_parameter_if_not_declared(node, name + ".edge_snap_enabled", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".edge_snap_enabled", edge_snap_enabled_);

  declare_parameter_if_not_declared(node, name + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".transform_tolerance", transform_tolerance_);

  declare_parameter_if_not_declared(node, name + ".dense_path_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".dense_path_step", dense_path_step_);
  declare_parameter_if_not_declared(
    node, name + ".max_dense_path_length", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".max_dense_path_length", max_dense_path_length_);

  if (topo_map_path_.empty()) {
    RCLCPP_ERROR(logger_, "NetPlanner: topo_map_path is empty; load a topology file.");
  } else if (!topo_.loadTopoMap(topo_map_path_)) {
    RCLCPP_ERROR(logger_, "NetPlanner: failed to load topology: %s", topo_map_path_.c_str());
  } else {
    RCLCPP_INFO(
      logger_, "NetPlanner: loaded topology %s (%zu nodes)", topo_map_path_.c_str(),
      topo_.nodes.size());
  }
}

void NetPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s", name_.c_str());
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&NetPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void NetPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s", name_.c_str());
  dyn_params_handler_.reset();
}

void NetPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s", name_.c_str());
  topo_ = TopoMap{};
}

bool NetPlanner::getRobotPose(geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = robot_base_frame_;
  p.header.stamp.sec = 0;
  p.header.stamp.nanosec = 0u;
  p.pose.orientation.w = 1.0;

  try {
    geometry_msgs::msg::PoseStamped out;
    tf_->transform(p, out, global_frame_, tf2::durationFromSec(transform_tolerance_));
    pose = out;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(logger_, "NetPlanner: TF map pose failed: %s", e.what());
    return false;
  }
}

nav_msgs::msg::Path NetPlanner::buildPath(
  const std::vector<int> & node_ids,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::unordered_map<int, TopoMap::Node> * node_overlay) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  std::unordered_map<int, TopoMap::Node> id2n;
  for (const auto & n : topo_.nodes) {
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
      if (dx * dx + dy * dy > 1e-6) {
        yaw = std::atan2(dy, dx);
      } else {
        yaw = tf2::getYaw(goal.pose.orientation);
      }
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }

  return path;
}

nav_msgs::msg::Path NetPlanner::applyDenseSampling(const nav_msgs::msg::Path & sparse) const
{
  if (dense_path_step_ <= 0.0 || sparse.poses.size() < 2) {
    return sparse;
  }
  return densifyPolyline(sparse, dense_path_step_, max_dense_path_length_);
}

nav_msgs::msg::Path NetPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  if (topo_.nodes.empty()) {
    RCLCPP_WARN(logger_, "NetPlanner: empty topology");
    return path;
  }

  geometry_msgs::msg::PoseStamped robot = start;
  if (use_tf_robot_pose_) {
    if (!getRobotPose(robot)) {
      RCLCPP_WARN(logger_, "NetPlanner: using start from planner_server (TF failed)");
      robot = start;
    }
  }

  std::unordered_map<int, TopoMap::Node> extra_nodes;
  std::unordered_map<int, std::vector<std::pair<int, float>>> extra_adj;

  std::vector<int> ids = topo_.planNodeIds(
      robot.pose.position.x, robot.pose.position.y, robot.pose.position.z,
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      max_snap_distance_, edge_snap_enabled_, extra_nodes, extra_adj);

  if (ids.empty()) {
    RCLCPP_WARN(
        logger_, "NetPlanner: snap or path failed (max_dist=%.2f, edge_snap=%d)",
        max_snap_distance_, edge_snap_enabled_);
    return path;
  }

  const std::unordered_map<int, TopoMap::Node> * overlay =
    extra_nodes.empty() ? nullptr : &extra_nodes;

  return applyDenseSampling(buildPath(ids, goal, overlay));
}

rcl_interfaces::msg::SetParametersResult NetPlanner::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & pname = parameter.get_name();
    if (type == ParameterType::PARAMETER_STRING && pname == name_ + ".topo_map_path") {
      topo_map_path_ = parameter.as_string();
      if (!topo_map_path_.empty() && topo_.loadTopoMap(topo_map_path_)) {
        RCLCPP_INFO(logger_, "Reloaded topology: %s", topo_map_path_.c_str());
      }
    } else if (type == ParameterType::PARAMETER_BOOL && pname == name_ + ".use_tf_robot_pose") {
      use_tf_robot_pose_ = parameter.as_bool();
    } else if (type == ParameterType::PARAMETER_STRING && pname == name_ + ".robot_base_frame") {
      robot_base_frame_ = parameter.as_string();
    } else if (type == ParameterType::PARAMETER_DOUBLE && pname == name_ + ".max_snap_distance") {
      max_snap_distance_ = parameter.as_double();
    } else if (type == ParameterType::PARAMETER_BOOL && pname == name_ + ".edge_snap_enabled") {
      edge_snap_enabled_ = parameter.as_bool();
    } else if (type == ParameterType::PARAMETER_DOUBLE && pname == name_ + ".transform_tolerance") {
      transform_tolerance_ = parameter.as_double();
    } else if (type == ParameterType::PARAMETER_DOUBLE && pname == name_ + ".dense_path_step") {
      dense_path_step_ = parameter.as_double();
    } else if (type == ParameterType::PARAMETER_DOUBLE && pname == name_ + ".max_dense_path_length") {
      max_dense_path_length_ = parameter.as_double();
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_net_planner

PLUGINLIB_EXPORT_CLASS(nav2_net_planner::NetPlanner, nav2_core::GlobalPlanner)
