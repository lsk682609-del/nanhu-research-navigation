// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#include "nav2_input_bridge/input_bridge_node.hpp"

#include <cmath>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"

#include <fmt/format.h>
#include "nav2_msgs/msg/log.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace nav2_input_bridge
{

InputBridgeNode::InputBridgeNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("nav2_input_bridge", "", options)
{
  declare_parameter("local_odom_topic", std::string("/car1/odom"));
  declare_parameter("goal_input_topic", std::string("/gp_goal"));
  declare_parameter("uav_goal_input_topic", std::string("/uav/target_gps"));
  declare_parameter("goal_output_topic", std::string("/goal_pose"));
  declare_parameter("goal_output_frame", std::string("map"));
  declare_parameter("require_nav_sat_fix", true);
  declare_parameter("follow_path_action_topic", std::string("/follow_path"));

  declare_parameter("reference_latitude", 0.0);
  declare_parameter("reference_longitude", 0.0);
  declare_parameter("reference_altitude", 0.0);

  declare_parameter("gp_path_input_topic", std::string("/navigate_track_gp_path"));
  declare_parameter("gp_path_output_topic", std::string("/navigate_track_path"));

  declare_parameter("lidar_pcl_topic", std::string("/rslidar_points"));

  declare_parameter("body_offset_x", 0.0);
  declare_parameter("body_offset_y", 0.0);
  declare_parameter("body_offset_z", 0.0);

  // 激光雷达的无效回波（Livox 等驱动把无回波点填成 (0,0,0)）会落在传感器自身原点。
  // 这类点进入代价地图后会把车体所在格直接标记成障碍，使车辆“自带”一圈致命膨胀区，
  // 表现为无法移动、无法重规划。这里按到传感器原点的距离把它们剔除。
  declare_parameter("min_range", 0.15);

  declare_parameter("crop_enabled", false);
  declare_parameter("crop_x_min", -1.0);
  declare_parameter("crop_x_max", 0.0);
  declare_parameter("crop_y_min", -0.3);
  declare_parameter("crop_y_max", 0.3);
  declare_parameter("crop_z_min", -0.1);
  declare_parameter("crop_z_max", 1.0);

  declare_parameter("odom_output_topic", std::string("odom_local"));
  declare_parameter("lidar_output_topic", std::string("cur_lid"));
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("lidar_frame", std::string("lidar"));

  declare_parameter("log_event_topic", std::string("log/event"));
  declare_parameter("log_record_topic", std::string("log/record"));
}

nav2_util::CallbackReturn
InputBridgeNode::on_configure(const rclcpp_lifecycle::State &)
{
  local_odom_topic_ = get_parameter("local_odom_topic").as_string();
  goal_input_topic_ = get_parameter("goal_input_topic").as_string();
  uav_goal_input_topic_ = get_parameter("uav_goal_input_topic").as_string();
  goal_output_topic_ = get_parameter("goal_output_topic").as_string();
  goal_output_frame_ = get_parameter("goal_output_frame").as_string();
  require_goal_fix_ = get_parameter("require_nav_sat_fix").as_bool();
  follow_path_action_topic_ = get_parameter("follow_path_action_topic").as_string();

  origin_lat_ = get_parameter("reference_latitude").as_double();
  origin_lon_ = get_parameter("reference_longitude").as_double();
  origin_alt_ = get_parameter("reference_altitude").as_double();

  gp_path_input_topic_ = get_parameter("gp_path_input_topic").as_string();
  gp_path_output_topic_ = get_parameter("gp_path_output_topic").as_string();

  lidar_pcl_topic_ = get_parameter("lidar_pcl_topic").as_string();

  body_offset_x_ = get_parameter("body_offset_x").as_double();
  body_offset_y_ = get_parameter("body_offset_y").as_double();
  body_offset_z_ = get_parameter("body_offset_z").as_double();

  min_range_ = get_parameter("min_range").as_double();

  crop_enabled_ = get_parameter("crop_enabled").as_bool();
  crop_x_min_ = get_parameter("crop_x_min").as_double();
  crop_x_max_ = get_parameter("crop_x_max").as_double();
  crop_y_min_ = get_parameter("crop_y_min").as_double();
  crop_y_max_ = get_parameter("crop_y_max").as_double();
  crop_z_min_ = get_parameter("crop_z_min").as_double();
  crop_z_max_ = get_parameter("crop_z_max").as_double();

  odom_output_topic_ = get_parameter("odom_output_topic").as_string();
  lidar_output_topic_ = get_parameter("lidar_output_topic").as_string();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();

  log_event_topic_ = get_parameter("log_event_topic").as_string();
  log_record_topic_ = get_parameter("log_record_topic").as_string();

  log_pub_ = create_publisher<nav2_msgs::msg::Log>(log_event_topic_, 10);
  record_pub_ = create_publisher<std_msgs::msg::String>(log_record_topic_, 10);

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_pcl_topic_, rclcpp::SensorDataQoS(),
    std::bind(&InputBridgeNode::onLidarMsg, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "InputBridge: lidar input topic=%s", lidar_pcl_topic_.c_str());
  lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_output_topic_, 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  rclcpp::QoS odom_qos(10);
  odom_qos.best_effort();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    local_odom_topic_, odom_qos,
    std::bind(&InputBridgeNode::onOdom, this, std::placeholders::_1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic_, 10);

  rclcpp::QoS goal_qos(10);
  goal_qos.best_effort();
  goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    goal_input_topic_, goal_qos,
    std::bind(&InputBridgeNode::onGoalFix, this, std::placeholders::_1));

  uav_goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    uav_goal_input_topic_, goal_qos,
    std::bind(&InputBridgeNode::onUavGoalFix, this, std::placeholders::_1));

  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_output_topic_, 10);

  gp_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    gp_path_input_topic_, rclcpp::QoS(10).transient_local(),
    std::bind(&InputBridgeNode::onGpPathFix, this, std::placeholders::_1));

  gp_path_pub_ = create_publisher<nav_msgs::msg::Path>(gp_path_output_topic_, 10);

  set_gp_origin_pub_ = create_publisher<geographic_msgs::msg::GeoPointStamped>(
    "/mavros/global_position/set_gp_origin", 10);
  gp_origin_sub_ = create_subscription<geographic_msgs::msg::GeoPointStamped>(
    "/mavros/global_position/gp_origin", rclcpp::QoS(10).best_effort(),
    std::bind(&InputBridgeNode::onGpOrigin, this, std::placeholders::_1));

  declare_parameter("lidar_timeout_s", 2.0);
  declare_parameter("odom_timeout_s", 2.0);
  lidar_timeout_s_ = get_parameter("lidar_timeout_s").as_double();
  odom_timeout_s_ = get_parameter("odom_timeout_s").as_double();

  set_gp_origin_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&InputBridgeNode::publishSetGpOrigin, this));

  check_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&InputBridgeNode::checkDataStatus, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InputBridgeNode::on_activate(const rclcpp_lifecycle::State &)
{
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InputBridgeNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InputBridgeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  odom_sub_.reset();
  odom_pub_.reset();
  lidar_sub_.reset();
  lidar_pub_.reset();
  goal_sub_.reset();
  uav_goal_sub_.reset();
  goal_pub_.reset();
  gp_path_sub_.reset();
  gp_path_pub_.reset();
  set_gp_origin_pub_.reset();
  gp_origin_sub_.reset();
  set_gp_origin_timer_.reset();
  check_timer_.reset();
  log_pub_.reset();
  record_pub_.reset();
  tf_broadcaster_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
InputBridgeNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void InputBridgeNode::publishLog(const std::string& level, const std::string& msg)
{
  auto m = std::make_shared<nav2_msgs::msg::Log>();
  m->level = level;
  m->node_name = "input_bridge";
  m->message = msg;
  log_pub_->publish(*m);
}

void InputBridgeNode::publishRecord(const std::string& data)
{
  auto m = std_msgs::msg::String();
  m.data = data;
  record_pub_->publish(m);
}

void InputBridgeNode::checkDataStatus()
{
  if (!log_pub_) return;
  auto now_t = now();
  if (last_odom_time_.nanoseconds() != 0) {
    double dt = (now_t - last_odom_time_).seconds();
    if (dt > odom_timeout_s_) {
      publishLog("error", fmt::format("odom_timeout {:.1f}s", dt));
    }
  }
  if (last_lidar_time_.nanoseconds() != 0) {
    double dt = (now_t - last_lidar_time_).seconds();
    if (dt > lidar_timeout_s_) {
      publishLog("error", fmt::format("lidar_timeout {:.1f}s", dt));
    } else if (!lidar_has_points_) {
      publishLog("error", "lidar_empty");
    }
  }
}

void InputBridgeNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_time_ = now();

  // Compute twist from pose difference (SLAM odom has no velocity)
  nav_msgs::msg::Odometry out = *msg;

  // Remap frames: world→map, car1/base_link→base_link
  out.header.frame_id = global_frame_;
  out.child_frame_id = robot_base_frame_;

  // Apply body offset: SLAM odom = IMU pose → subtract rotated offset to get robot center
  Eigen::Quaterniond q_odom(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
  Eigen::Vector3d offset_body(body_offset_x_, body_offset_y_, body_offset_z_);
  Eigen::Vector3d offset_world = q_odom * offset_body;
  out.pose.pose.position.x -= offset_world.x();
  out.pose.pose.position.y -= offset_world.y();

  // 2D navigation: force z=0
  out.pose.pose.position.z = 0.0;

  if (has_prev_odom_) {
    double dt = (rclcpp::Time(msg->header.stamp).seconds() -
                 rclcpp::Time(prev_odom_.header.stamp).seconds());
    if (dt > 1e-6) {
      // Linear velocity from position difference
      out.twist.twist.linear.x = (msg->pose.pose.position.x -
                                  prev_odom_.pose.pose.position.x) / dt;
      out.twist.twist.linear.y = (msg->pose.pose.position.y -
                                  prev_odom_.pose.pose.position.y) / dt;
      out.twist.twist.linear.z = (msg->pose.pose.position.z -
                                  prev_odom_.pose.pose.position.z) / dt;

      // Angular velocity from quaternion difference
      Eigen::Quaterniond q_cur(msg->pose.pose.orientation.w,
                               msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z);
      Eigen::Quaterniond q_prev(prev_odom_.pose.pose.orientation.w,
                                prev_odom_.pose.pose.orientation.x,
                                prev_odom_.pose.pose.orientation.y,
                                prev_odom_.pose.pose.orientation.z);
      Eigen::Quaterniond dq = q_cur * q_prev.inverse();
      Eigen::AngleAxisd aa(dq);
      double angle = aa.angle();
      if (angle > M_PI) angle -= 2.0 * M_PI;
      out.twist.twist.angular.x = aa.axis().x() * angle / dt;
      out.twist.twist.angular.y = aa.axis().y() * angle / dt;
      out.twist.twist.angular.z = aa.axis().z() * angle / dt;
    }
  }
  prev_odom_ = *msg;
  has_prev_odom_ = true;

  odom_pub_->publish(out);

  // Publish corrected TF (body_offset applied) to override SLAM's uncorrected TF
  if (!out.header.frame_id.empty() && !out.child_frame_id.empty()) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = out.header.stamp;
    t.header.frame_id = out.header.frame_id;
    t.child_frame_id = out.child_frame_id;
    t.transform.translation.x = out.pose.pose.position.x;
    t.transform.translation.y = out.pose.pose.position.y;
    t.transform.translation.z = 0.0;  // 2D navigation
    t.transform.rotation = out.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }
}

void InputBridgeNode::llaToEnu(
  double lat_deg, double lon_deg, double alt_m,
  double lat0_deg, double lon0_deg, double alt0_m,
  double * east_m, double * north_m, double * up_m) const
{
  const double R = WGS84_R_;
  const double lat_r = lat_deg * M_PI / 180.0;
  const double lon_r = lon_deg * M_PI / 180.0;
  const double lat0_r = lat0_deg * M_PI / 180.0;
  const double lon0_r = lon0_deg * M_PI / 180.0;
  const double sin_lat = sin(lat_r);
  const double cos_lat = cos(lat_r);
  const double sin_lat0 = sin(lat0_r);
  const double cos_lat0 = cos(lat0_r);
  const double dlon = lon_r - lon0_r;
  const double cos_dlon = cos(dlon);

  double arg = sin_lat0 * sin_lat + cos_lat0 * cos_lat * cos_dlon;
  arg = std::min(1.0, std::max(-1.0, arg));
  const double c = acos(arg);
  double k = 1.0;
  if (fabs(c) > 1e-6) {
    k = c / sin(c);
  }
  const double north = k * (cos_lat0 * sin_lat - sin_lat0 * cos_lat * cos_dlon) * R;
  const double east  = k * cos_lat * sin(dlon) * R;

  *east_m  = east;
  *north_m = north;
  *up_m    = alt_m - alt0_m;
}

void InputBridgeNode::processGoalFix(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg,
  const std::string & source)
{
  if (!gp_origin_verified_) { return; }

  if (require_goal_fix_ &&
    msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
  {
    publishLog("warn", "goal_no_fix source=" + source);
    return;
  }

  double east = 0.0, north = 0.0, up = 0.0;
  llaToEnu(
    msg->latitude, msg->longitude, msg->altitude,
    origin_lat_, origin_lon_, origin_alt_,
    &east, &north, &up);

  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = goal_output_frame_;
  out.pose.position.x = east;
  out.pose.position.y = north;
  out.pose.position.z = up;
  out.pose.orientation.w = 1.0;

  goal_pub_->publish(out);

  publishLog("info", fmt::format("gp_goal source={} lat={:.6f} lon={:.6f} alt={:.1f}",
    source, msg->latitude, msg->longitude, msg->altitude));

  publishRecord(fmt::format("{:.6f},gp_goal,{:.6f},{:.6f},{:.1f}",
    now().seconds(), msg->latitude, msg->longitude, msg->altitude));
}

void InputBridgeNode::onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  processGoalFix(msg, "manual");
}

void InputBridgeNode::onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  processGoalFix(msg, "uav");
}

void InputBridgeNode::onGpPathFix(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!gp_origin_verified_) { return; }

  if (msg->poses.empty()) {
    publishLog("warn", "gp_path_empty");
    return;
  }

  const auto & last_gps = msg->poses.back().pose.position;
  publishLog("info", fmt::format("gp_path_received points={} last_lat={:.6f} last_lon={:.6f} last_alt={:.1f}",
    msg->poses.size(), last_gps.x, last_gps.y, last_gps.z));

  nav_msgs::msg::Path out_path;
  out_path.header = msg->header;
  out_path.header.frame_id = goal_output_frame_;

  for (const auto & gps_pose : msg->poses) {
    double east = 0.0, north = 0.0, up = 0.0;
    llaToEnu(
      gps_pose.pose.position.x,
      gps_pose.pose.position.y,
      gps_pose.pose.position.z,
      origin_lat_, origin_lon_, origin_alt_,
      &east, &north, &up);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = out_path.header;
    ps.pose.position.x = east;
    ps.pose.position.y = north;
    ps.pose.position.z = up;
    ps.pose.orientation = gps_pose.pose.orientation;
    out_path.poses.push_back(ps);
  }

  gp_path_pub_->publish(out_path);

  publishRecord(fmt::format("{:.6f},gp_path,{:.6f},{:.6f},{:.1f}",
    now().seconds(), last_gps.x, last_gps.y, last_gps.z));
}

void InputBridgeNode::publishSetGpOrigin()
{
  static bool first_publish = true;
  if (first_publish) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);
    oss << "gp_origin_set_started lat=" << origin_lat_
        << " lon=" << origin_lon_ << " alt=" << origin_alt_;
    publishLog("info", oss.str());
    first_publish = false;
  }

  geographic_msgs::msg::GeoPointStamped geo_msg;
  geo_msg.header.stamp = now();
  geo_msg.header.frame_id = "map";
  geo_msg.position.latitude = origin_lat_;
  geo_msg.position.longitude = origin_lon_;
  geo_msg.position.altitude = origin_alt_;
  set_gp_origin_pub_->publish(geo_msg);
}

void InputBridgeNode::onGpOrigin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg)
{
  if (gp_origin_verified_) {
    gp_origin_sub_.reset();
    set_gp_origin_timer_->cancel();
    return;
  }

  if (std::abs(msg->position.latitude - origin_lat_) < gp_origin_tolerance_latlon_ &&
      std::abs(msg->position.longitude - origin_lon_) < gp_origin_tolerance_latlon_ &&
      std::abs(msg->position.altitude - origin_alt_) < gp_origin_tolerance_alt_)
  {
    gp_origin_verified_ = true;
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);
    oss << "gp_origin_verified lat=" << origin_lat_
        << " lon=" << origin_lon_ << " alt=" << origin_alt_;
    publishLog("info", oss.str());
    gp_origin_sub_.reset();
    set_gp_origin_timer_->cancel();
  }
}

void InputBridgeNode::onLidarMsg(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_lidar_time_ = now();
  lidar_has_points_ = (msg->width * msg->height > 0);
  bool has_intensity = false;
  for (const auto & f : msg->fields) {
    if (f.name == "intensity") { has_intensity = true; break; }
  }

  // 有效点判据：剔除 NaN/Inf 与落在传感器自身原点附近的无效回波（Livox 用 (0,0,0)
  // 表示“无回波”），再应用可选的裁剪盒。
  const double min_range2 = min_range_ * min_range_;
  const auto keep_point = [&](float px, float py, float pz) {
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {return false;}
    if (px * px + py * py + pz * pz < min_range2) {return false;}
    if (crop_enabled_ &&
      px > crop_x_min_ && px < crop_x_max_ &&
      py > crop_y_min_ && py < crop_y_max_ &&
      pz > crop_z_min_ && pz < crop_z_max_) return false;
    return true;
  };

  // First pass: count valid points (after crop)
  uint32_t valid = 0;
  size_t total_points = msg->width * msg->height;
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

  for (size_t i = 0; i < total_points; ++i, ++it_x, ++it_y, ++it_z) {
    if (!keep_point(*it_x, *it_y, *it_z)) continue;
    ++valid;
  }

  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
  out->header = msg->header;
  out->header.frame_id = lidar_frame_;
  out->height = 1;
  out->is_dense = msg->is_dense;
  out->is_bigendian = msg->is_bigendian;

  sensor_msgs::PointCloud2Modifier mod(*out);
  mod.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(valid);
  out->width = valid;

  // Second pass: copy valid points
  sensor_msgs::PointCloud2Iterator<float> ox(*out, "x");
  sensor_msgs::PointCloud2Iterator<float> oy(*out, "y");
  sensor_msgs::PointCloud2Iterator<float> oz(*out, "z");
  sensor_msgs::PointCloud2Iterator<float> oi(*out, "intensity");

  it_x = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
  it_y = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y");
  it_z = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z");
  auto it_i = has_intensity
    ? sensor_msgs::PointCloud2ConstIterator<float>(*msg, "intensity")
    : sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x"); // dummy

  for (size_t i = 0; i < total_points; ++i, ++it_x, ++it_y, ++it_z) {
    const bool keep = keep_point(*it_x, *it_y, *it_z);
    const float intensity = has_intensity ? *it_i : 0.0f;
    if (has_intensity) ++it_i;
    if (!keep) continue;
    *ox = *it_x; ++ox;
    *oy = *it_y; ++oy;
    *oz = *it_z; ++oz;
    *oi = intensity; ++oi;
  }

  lidar_pub_->publish(*out);
}

}  // namespace nav2_input_bridge