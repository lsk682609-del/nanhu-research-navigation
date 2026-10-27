// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#ifndef NAV2_LOG_RECORD__LOG_SERVER_NODE_HPP_
#define NAV2_LOG_RECORD__LOG_SERVER_NODE_HPP_

#include <memory>
#include <string>
#include <spdlog/spdlog.h>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/log.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_log_record
{

class LogServerNode : public nav2_util::LifecycleNode
{
public:
  explicit LogServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void onLog(const nav2_msgs::msg::Log::SharedPtr msg);
  void onRecord(const std_msgs::msg::String::SharedPtr msg);
  void setup();

  std::shared_ptr<spdlog::logger> event_logger_;
  std::shared_ptr<spdlog::logger> record_logger_;
  rclcpp::Subscription<nav2_msgs::msg::Log>::SharedPtr log_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_sub_;
  std::string log_event_topic_;
  std::string log_record_topic_;
};

}  // namespace nav2_log_record

#endif  // NAV2_LOG_SERVER__LOG_SERVER_NODE_HPP_