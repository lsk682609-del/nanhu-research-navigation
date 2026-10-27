// Copyright 2026
#ifndef NAV2_NET_PLANNER__NET_PLANNER_HPP_
#define NAV2_NET_PLANNER__NET_PLANNER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_net_planner/topo_map.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_net_planner
{

class NetPlanner : public nav2_core::GlobalPlanner
{
public:
  NetPlanner();
  ~NetPlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;
  nav_msgs::msg::Path buildPath(
    const std::vector<int> & node_ids,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::unordered_map<int, TopoMap::Node> * node_overlay = nullptr) const;

  /** Apply polyline densify + max arc-length cap (uses dense_path_step_, max_dense_path_length_). */
  nav_msgs::msg::Path applyDenseSampling(const nav_msgs::msg::Path & sparse) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string global_frame_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("net_planner")};

  TopoMap topo_;
  std::string topo_map_path_;
  bool use_tf_robot_pose_{true};
  std::string robot_base_frame_{"base_link"};
  double max_snap_distance_{15.0};
  bool edge_snap_enabled_{false};
  double transform_tolerance_{0.5};
  /** Arc-length spacing between dense poses along the graph polyline (m). */
  double dense_path_step_{0.1};
  /** If > 0, truncate dense path to this arc length from the first pose (m); <= 0 means full path. */
  double max_dense_path_length_{0.0};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_net_planner

#endif  // NAV2_NET_PLANNER__NET_PLANNER_HPP_
