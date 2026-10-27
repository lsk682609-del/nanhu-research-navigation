// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#include "nav2_log_record/log_server_node.hpp"

#include <filesystem>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace nav2_log_record
{

LogServerNode::LogServerNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("nav2_log_record", "", options)
{
  declare_parameter("log_output", std::string("both"));
  declare_parameter("log_level", std::string("info"));
  declare_parameter("log_event_topic", std::string("log/event"));
  declare_parameter("log_record_topic", std::string("log/record"));
}

nav2_util::CallbackReturn
LogServerNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  setup();

  log_event_topic_ = get_parameter("log_event_topic").as_string();
  log_record_topic_ = get_parameter("log_record_topic").as_string();

  log_sub_ = create_subscription<nav2_msgs::msg::Log>(
    log_event_topic_, rclcpp::QoS(10),
    std::bind(&LogServerNode::onLog, this, std::placeholders::_1));

  record_sub_ = create_subscription<std_msgs::msg::String>(
    log_record_topic_, rclcpp::QoS(10),
    std::bind(&LogServerNode::onRecord, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "LogServerNode configured");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LogServerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  createBond();
  RCLCPP_INFO(get_logger(), "LogServerNode activated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LogServerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  destroyBond();
  RCLCPP_INFO(get_logger(), "LogServerNode deactivated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LogServerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  log_sub_.reset();
  record_sub_.reset();
  RCLCPP_INFO(get_logger(), "LogServerNode cleaned up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LogServerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  event_logger_.reset();
  record_logger_.reset();
  RCLCPP_INFO(get_logger(), "LogServerNode shutdown");
  return nav2_util::CallbackReturn::SUCCESS;
}

void LogServerNode::setup()
{
  std::string output_mode = "both";
  std::string log_level_str = "info";

  get_parameter("log_output", output_mode);
  get_parameter("log_level", log_level_str);

  auto log_level = spdlog::level::info;
  if (log_level_str == "debug") log_level = spdlog::level::debug;
  else if (log_level_str == "warn") log_level = spdlog::level::warn;
  else if (log_level_str == "error") log_level = spdlog::level::err;

  std::filesystem::create_directory("./logs");

  // Event log
  {
    std::shared_ptr<spdlog::logger> logger;
    spdlog::level::level_enum lvl = log_level;

    if (output_mode == "console") {
      auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      logger = std::make_shared<spdlog::logger>("nav2", sink);
    }
    else if (output_mode == "both") {
      auto f_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          "./logs/navigation.log", 10 * 1024 * 1024, 3);
      auto c_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      spdlog::sinks_init_list sinks = {f_sink, c_sink};
      logger = std::make_shared<spdlog::logger>("nav2", sinks.begin(), sinks.end());
    }
    else {  // file
      auto f_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          "./logs/navigation.log", 10 * 1024 * 1024, 3);
      logger = std::make_shared<spdlog::logger>("nav2", f_sink);
    }

    logger->set_level(lvl);
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
    // 每条事件立即刷盘，避免进程异常退出时缓冲区丢失日志。
    logger->flush_on(spdlog::level::trace);
    event_logger_ = logger;
  }

  // Record log: always file only, rotating 50MB x 5
  {
    auto rec_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "./logs/recorder.csv", 100 * 1024 * 1024, 20);
    record_logger_ = std::make_shared<spdlog::logger>("recorder", rec_sink);
    record_logger_->set_level(spdlog::level::info);
    record_logger_->set_pattern("%Y-%m-%d %H:%M:%S.%e,%v");
    // 每条记录立即刷盘，确保 CSV 数据实时落盘。
    record_logger_->flush_on(spdlog::level::trace);

  }
}

void LogServerNode::onLog(const nav2_msgs::msg::Log::SharedPtr msg)
{
  auto level = spdlog::level::info;
  if (msg->level == "debug") level = spdlog::level::debug;
  else if (msg->level == "warn") level = spdlog::level::warn;
  else if (msg->level == "error") level = spdlog::level::err;

  event_logger_->log(level, "[{}] {}", msg->node_name, msg->message);
}

void LogServerNode::onRecord(const std_msgs::msg::String::SharedPtr msg)
{
  record_logger_->info(msg->data);
}

}  // namespace nav2_log_record