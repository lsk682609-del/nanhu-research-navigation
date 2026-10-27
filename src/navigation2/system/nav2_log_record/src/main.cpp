// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#include "nav2_log_record/log_server_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_log_record::LogServerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}