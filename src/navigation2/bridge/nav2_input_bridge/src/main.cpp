// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#include <memory>

#include "nav2_input_bridge/input_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_input_bridge::InputBridgeNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
