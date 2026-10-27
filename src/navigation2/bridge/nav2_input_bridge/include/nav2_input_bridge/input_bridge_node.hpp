// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#ifndef NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
#define NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_msgs/msg/log.hpp"
#include "std_msgs/msg/string.hpp"
#include <Eigen/Geometry>

#include "nav2_util/lifecycle_node.hpp"

namespace nav2_input_bridge
{

class InputBridgeNode : public nav2_util::LifecycleNode
{
public:
  explicit InputBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  using FollowPath = nav2_msgs::action::FollowPath;

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onGpPathFix(const nav_msgs::msg::Path::SharedPtr msg);
  void onLidarMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void processGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg, const std::string & source);

  void publishLog(const std::string& level, const std::string& msg);
  void publishRecord(const std::string& data);
  void checkDataStatus();

  void onGpOrigin(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg);
  void publishSetGpOrigin();
  void llaToEnu(
    double lat_deg, double lon_deg, double alt_m,
    double lat0_deg, double lon0_deg, double alt0_m,
    double * east_m, double * north_m, double * up_m) const;

  std::string local_odom_topic_;
  std::string goal_input_topic_;
  std::string uav_goal_input_topic_;
  std::string follow_path_action_topic_;
  std::string goal_output_topic_;
  std::string goal_output_frame_;
  bool require_goal_fix_;
  std::string gp_path_input_topic_;
  std::string gp_path_output_topic_;
  std::string odom_output_topic_;
  std::string lidar_output_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string log_event_topic_;
  std::string log_record_topic_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr uav_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr gp_path_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_origin_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gp_path_pub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr set_gp_origin_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Log>::SharedPtr log_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Odom twist interpolation
  nav_msgs::msg::Odometry prev_odom_;
  bool has_prev_odom_{false};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr set_gp_origin_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::Time last_lidar_time_;
  rclcpp::Time last_odom_time_;
  bool lidar_has_points_{false};
  double lidar_timeout_s_{2.0};
  double odom_timeout_s_{2.0};

  bool gp_origin_verified_{false};
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};
  double gp_origin_tolerance_latlon_{1e-6};
  double gp_origin_tolerance_alt_{1.0};

  // Body offset: IMU/LiDAR → robot center (body frame, x-forward y-left z-up)
  double body_offset_x_{0.0};
  double body_offset_y_{0.0};
  double body_offset_z_{0.0};

  // Lidar crop box (livox_frame)
  double min_range_{0.15};
  bool crop_enabled_{false};
  double crop_x_min_{-1.0};
  double crop_x_max_{0.0};
  double crop_y_min_{-0.3};
  double crop_y_max_{0.3};
  double crop_z_min_{-0.1};
  double crop_z_max_{1.0};

  std::string lidar_pcl_topic_;

  static constexpr double WGS84_R_{6378137.0};
};

}  // namespace nav2_input_bridge

#endif  // NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
