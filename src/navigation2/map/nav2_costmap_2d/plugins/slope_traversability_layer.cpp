#include "nav2_costmap_2d/slope_traversability_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SlopeTraversabilityLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

namespace
{
struct HeightCell
{
  float low{std::numeric_limits<float>::infinity()};
  float high{-std::numeric_limits<float>::infinity()};
  unsigned int count{0};
};
}  // namespace

void SlopeTraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("SlopeTraversabilityLayer: node expired");}
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("pointcloud_topic", rclcpp::ParameterValue(std::string("cur_lid")));
  declareParameter("max_slope_deg", rclcpp::ParameterValue(15.0));
  declareParameter("max_step_height", rclcpp::ParameterValue(0.10));
  declareParameter("max_ground_residual", rclcpp::ParameterValue(0.06));
  declareParameter("obstacle_height", rclcpp::ParameterValue(0.12));
  declareParameter("max_range", rclcpp::ParameterValue(3.0));
  declareParameter("neighborhood_radius", rclcpp::ParameterValue(0.45));
  declareParameter("cloud_timeout", rclcpp::ParameterValue(0.5));
  declareParameter("min_neighbors", rclcpp::ParameterValue(4));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".pointcloud_topic", topic_);
  node->get_parameter(name_ + ".max_slope_deg", max_slope_deg_);
  node->get_parameter(name_ + ".max_step_height", max_step_height_);
  node->get_parameter(name_ + ".max_ground_residual", max_ground_residual_);
  node->get_parameter(name_ + ".obstacle_height", obstacle_height_);
  node->get_parameter(name_ + ".max_range", max_range_);
  node->get_parameter(name_ + ".neighborhood_radius", neighborhood_radius_);
  node->get_parameter(name_ + ".cloud_timeout", cloud_timeout_);
  node->get_parameter(name_ + ".min_neighbors", min_neighbors_);
  max_slope_deg_ = std::clamp(max_slope_deg_, 0.0, 89.0);
  max_step_height_ = std::max(0.01, max_step_height_);
  neighborhood_radius_ = std::max(0.1, neighborhood_radius_);
  matchSize();
  current_ = false;
  activate();
}

void SlopeTraversabilityLayer::activate()
{
  auto node = node_.lock();
  if (!node || cloud_sub_) {return;}
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&SlopeTraversabilityLayer::cloudCallback, this, std::placeholders::_1), options);
}

void SlopeTraversabilityLayer::deactivate()
{
  cloud_sub_.reset();
}

void SlopeTraversabilityLayer::reset()
{
  resetMaps();
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_.reset();
  current_ = false;
}

void SlopeTraversabilityLayer::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = std::move(cloud);
}

void SlopeTraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }
  if (!enabled_) {return;}

  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = latest_cloud_;
  }
  resetMaps();  // Only our own layer; stale LiDAR marks disappear on the next update.
  current_ = false;
  if (cloud && rclcpp::Time(cloud->header.stamp).nanoseconds() > 0 &&
    (clock_->now() - rclcpp::Time(cloud->header.stamp)).seconds() <= cloud_timeout_)
  {
    try {
      classify(*cloud);
      current_ = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Slope layer TF unavailable: %s", e.what());
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Slope layer cloud rejected: %s", e.what());
    }
  }
  // Full bounds are needed to remove marks left by the previous cloud.
  *min_x = std::min(*min_x, getOriginX());
  *min_y = std::min(*min_y, getOriginY());
  *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
  *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
}

void SlopeTraversabilityLayer::classify(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.width == 0 || cloud.height == 0 || cloud.header.frame_id.empty()) {return;}
  const auto transform = tf_->lookupTransform(
    layered_costmap_->getGlobalFrameID(), cloud.header.frame_id, cloud.header.stamp,
    rclcpp::Duration::from_seconds(0.05));
  sensor_msgs::msg::PointCloud2 global_cloud;
  tf2::doTransform(cloud, global_cloud, transform);
  const double sx = transform.transform.translation.x;
  const double sy = transform.transform.translation.y;
  const auto nx = getSizeInCellsX();
  const auto ny = getSizeInCellsY();
  std::vector<HeightCell> heights(static_cast<size_t>(nx) * ny);
  sensor_msgs::PointCloud2ConstIterator<float> x(global_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(global_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(global_cloud, "z");
  const double range2 = max_range_ * max_range_;
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    if (std::pow(*x - sx, 2) + std::pow(*y - sy, 2) > range2) {continue;}
    unsigned int mx, my;
    if (!worldToMap(*x, *y, mx, my)) {continue;}
    auto & cell = heights[static_cast<size_t>(my) * nx + mx];
    cell.low = std::min(cell.low, *z);
    cell.high = std::max(cell.high, *z);
    ++cell.count;
  }

  const int radius = static_cast<int>(std::ceil(neighborhood_radius_ / getResolution()));
  const double radius2 = neighborhood_radius_ * neighborhood_radius_;
  const double max_gradient = std::tan(max_slope_deg_ * M_PI / 180.0);
  for (unsigned int my = 0; my < ny; ++my) {
    for (unsigned int mx = 0; mx < nx; ++mx) {
      const auto & cell = heights[static_cast<size_t>(my) * nx + mx];
      if (!cell.count) {continue;}
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      int neighbors = 0;
      double short_jump = 0.0;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if ((dx * dx + dy * dy) * getResolution() * getResolution() > radius2) {continue;}
          const int ix = static_cast<int>(mx) + dx;
          const int iy = static_cast<int>(my) + dy;
          if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx) || iy >= static_cast<int>(ny)) {
            continue;
          }
          const auto & other = heights[static_cast<size_t>(iy) * nx + ix];
          if (!other.count) {continue;}
          const Eigen::Vector3d v(dx * getResolution(), dy * getResolution(), 1.0);
          normal.noalias() += v * v.transpose();
          rhs.noalias() += v * other.low;
          if (std::hypot(dx, dy) * getResolution() <= 0.20) {
            short_jump = std::max(
              short_jump, std::abs(static_cast<double>(other.low) - cell.low));
          }
          ++neighbors;
        }
      }

      bool lethal = (cell.high - cell.low) > obstacle_height_;
      if (neighbors >= min_neighbors_ && normal.determinant() > 1e-9) {
        const Eigen::Vector3d plane = normal.ldlt().solve(rhs);
        const double slope = std::hypot(plane.x(), plane.y());
        // A stair riser or obstacle top may fit an inclined plane locally;
        // reject it if the measured heights are discontinuous around that plane.
        double max_residual = 0.0;
        double local_jump = 0.0;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            if ((dx * dx + dy * dy) * getResolution() * getResolution() > radius2) {continue;}
            const int ix = static_cast<int>(mx) + dx;
            const int iy = static_cast<int>(my) + dy;
            if (ix < 0 || iy < 0 || ix >= static_cast<int>(nx) || iy >= static_cast<int>(ny)) {
              continue;
            }
            const auto & other = heights[static_cast<size_t>(iy) * nx + ix];
            if (!other.count) {continue;}
            const double planar = plane.x() * dx * getResolution() +
              plane.y() * dy * getResolution() + plane.z();
            max_residual = std::max(max_residual, std::abs(other.low - planar));
            const double allowed_slope_delta = max_gradient *
              std::hypot(dx, dy) * getResolution();
            local_jump = std::max(
              local_jump,
              std::abs(other.low - cell.low) - allowed_slope_delta);
          }
        }
        lethal = lethal || slope > max_gradient ||
          (max_residual > max_ground_residual_ && local_jump > max_step_height_);
      } else if (short_jump > max_step_height_) {
        // Sparse clouds still expose an abrupt nearby riser, but a whole
        // inclined neighborhood must not be mistaken for a step.
        lethal = true;
      }
      if (lethal) {setCost(mx, my, LETHAL_OBSTACLE);}
    }
  }
}

void SlopeTraversabilityLayer::updateCosts(
  Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  if (enabled_) {updateWithMax(master, min_i, min_j, max_i, max_j);}
}

}  // namespace nav2_costmap_2d
