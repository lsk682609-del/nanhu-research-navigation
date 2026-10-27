#ifndef NAV2_COSTMAP_2D__SLOPE_TRAVERSABILITY_LAYER_HPP_
#define NAV2_COSTMAP_2D__SLOPE_TRAVERSABILITY_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nav2_costmap_2d
{

// A local LiDAR obstacle layer which leaves continuous, climbable ground unmarked.
class SlopeTraversabilityLayer : public CostmapLayer
{
public:
  SlopeTraversabilityLayer() {costmap_ = nullptr;}
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override;

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void classify(const sensor_msgs::msg::PointCloud2 & cloud);

  std::string topic_;
  double max_slope_deg_{15.0};
  double max_step_height_{0.10};
  double max_ground_residual_{0.06};
  double obstacle_height_{0.12};
  double max_range_{3.0};
  double neighborhood_radius_{0.45};
  double cloud_timeout_{0.5};
  int min_neighbors_{4};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__SLOPE_TRAVERSABILITY_LAYER_HPP_
