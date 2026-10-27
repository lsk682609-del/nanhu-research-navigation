#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <limits>
#include <string>

namespace nav2_scan_planner {
template<class T>
T parameter(rclcpp_lifecycle::LifecycleNode &node, const std::string &name, T value) {
  if (!node.has_parameter(name)) node.declare_parameter<T>(name, value);
  node.get_parameter(name, value);
  return value;
}
} // namespace nav2_scan_planner
