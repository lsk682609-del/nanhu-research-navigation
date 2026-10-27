#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include "nav2_scan_planner/vendor/grid_map.h"
#include "nav2_scan_planner/vendor/bspline_optimizer.h"

namespace nav2_scan_planner {
// Owns native GridMap perception. It never consumes a Nav2 costmap.
class ScanBackend {
public:
  ScanBackend(rclcpp_lifecycle::LifecycleNode &parent, const std::string &prefix,
    std::shared_ptr<tf2_ros::Buffer> tf, const std::string &frame,
    const std::string &base_frame);
  ~ScanBackend();
  void activate();
  void deactivate();
  bool ready() const;
  bool collision(double x, double y, double yaw) const;
  bool segmentFree(double x0, double y0, double x1, double y1) const;
  bool inside(double x, double y) const;
  double resolution() const { return resolution_; }
  double horizon() const { return horizon_; }
  double radius() const { return radius_; }
  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::Twist &speed, const nav_msgs::msg::Path &reference,
    const geometry_msgs::msg::PoseStamped &goal);
  // The same core entry used by the controller and deterministic offline tests.
  static nav_msgs::msg::Path optimize(nav2_scan_core::GridMap::Ptr map,
    nav2_scan_core::BsplineOptimizer &optimizer,
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::Twist &speed, const nav_msgs::msg::Path &reference,
    const geometry_msgs::msg::PoseStamped &goal, double z, double max_vel,
    double max_acc, double spacing, double budget_ms);
private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void publishMap();
  bool readyLocked() const;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string frame_, base_frame_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  mutable std::mutex mutex_;
  nav2_scan_core::GridMap::Ptr map_;
  nav2_scan_core::BsplineOptimizer optimizer_;
  std::atomic<bool> active_{false};
  bool received_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_received_;
  double resolution_, horizon_, radius_, body_z_, timeout_, budget_ms_;
  double max_vel_, max_acc_, spacing_, min_range_;
  size_t max_points_;
};
} // namespace nav2_scan_planner
