#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace traversability_obstacle_cloud
{

void writeBinaryPcd(
  const std::string & path, const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("cannot open PCD file");
  }
  stream << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F F\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << cloud.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << cloud.size() << "\n"
         << "DATA binary\n";
  for (const auto & point : cloud) {
    const float values[4] = {point.x, point.y, point.z, point.intensity};
    stream.write(
      reinterpret_cast<const char *>(values),
      static_cast<std::streamsize>(sizeof(values)));
  }
  if (!stream) {
    throw std::runtime_error("failed while writing PCD file");
  }
}

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct BoundaryTrack
{
  std::vector<double> y;
  std::vector<int> missing_frames;
  std::vector<std::uint8_t> valid;

  void resize(std::size_t size)
  {
    y.assign(size, 0.0);
    missing_frames.assign(size, 0);
    valid.assign(size, 0U);
  }

  void clear()
  {
    y.clear();
    missing_frames.clear();
    valid.clear();
  }
};

struct GroundSurface
{
  std::int64_t stamp_ns = 0;
  std::string frame_id;
  pcl::PointCloud<pcl::PointXYZI> points;
  std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> buckets;
};

class CameraCostmapCloudNode : public rclcpp::Node
{
public:
  CameraCostmapCloudNode()
  : Node("camera_costmap_cloud_node")
  {
    loadParameters();

    const auto input_qos = rclcpp::SensorDataQoS().keep_last(2);
    const auto output_qos = rclcpp::SensorDataQoS().keep_last(1);
    if (relative_ground_enabled_) {
      ground_grid_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        ground_grid_input_topic_, input_qos,
        std::bind(&CameraCostmapCloudNode::groundGridCallback, this, std::placeholders::_1));
    }
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_input_topic_, input_qos,
      std::bind(&CameraCostmapCloudNode::obstacleCallback, this, std::placeholders::_1));
    boundary_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      boundary_input_topic_, input_qos,
      std::bind(&CameraCostmapCloudNode::boundaryCallback, this, std::placeholders::_1));
    if (pothole_enabled_) {
      pothole_boundary_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pothole_boundary_input_topic_, input_qos,
        std::bind(
          &CameraCostmapCloudNode::potholeBoundaryCallback, this, std::placeholders::_1));
    }
    depth_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(depth_output_topic_, output_qos);
    boundary_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(boundary_output_topic_, output_qos);
    if (pothole_enabled_) {
      pothole_boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pothole_boundary_output_topic_, output_qos);
      pothole_watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CameraCostmapCloudNode::potholeWatchdog, this));
    }
    if (publish_rejected_depth_) {
      rejected_depth_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(rejected_depth_topic_, output_qos);
    }
    if (publish_ground_rejected_depth_) {
      ground_rejected_depth_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_rejected_depth_topic_, output_qos);
    }

    RCLCPP_INFO(
      get_logger(),
      "camera costmap adapter: depth %s -> %s, boundary %s -> %s, output_frame=%s",
      obstacle_input_topic_.c_str(), depth_output_topic_.c_str(),
      boundary_input_topic_.c_str(), boundary_output_topic_.c_str(),
      output_frame_.empty() ? "<input frame>" : output_frame_.c_str());
    if (pothole_enabled_) {
      RCLCPP_INFO(
        get_logger(), "pothole costmap adapter: %s -> %s spacing=%.2f stale=%.2fs",
        pothole_boundary_input_topic_.c_str(), pothole_boundary_output_topic_.c_str(),
        pothole_spacing_, pothole_stale_timeout_);
    }
    RCLCPP_INFO(
      get_logger(),
      "depth filter: x=[%.2f, %.2f] |y|<=%.2f z=[%.2f, %.2f] range<=%.2f voxel=%.2f",
      depth_x_min_, depth_x_max_, depth_abs_y_max_, depth_z_min_, depth_z_max_,
      depth_max_range_, depth_voxel_size_);
    RCLCPP_INFO(
      get_logger(), "depth denoise: %s radius=%.2f min_neighbors=%d rejected_topic=%s",
      depth_denoise_enabled_ ? "on" : "off", depth_denoise_radius_,
      depth_denoise_min_neighbors_,
      publish_rejected_depth_ ? rejected_depth_topic_.c_str() : "<disabled>");
    RCLCPP_INFO(
      get_logger(),
      "relative ground filter: %s topic=%s min_height=%.2f radius=%.2f "
      "min_neighbors=%d max_rmse=%.2f max_slope=%.1fdeg max_age=%.2fs no_ground=%s "
      "rejected_topic=%s",
      relative_ground_enabled_ ? "on" : "off", ground_grid_input_topic_.c_str(),
      relative_ground_min_height_, relative_ground_search_radius_,
      relative_ground_min_neighbors_, relative_ground_max_plane_rmse_,
      relative_ground_max_slope_deg_, relative_ground_max_age_,
      relative_ground_keep_without_ground_ ? "keep" : "reject",
      publish_ground_rejected_depth_ ? ground_rejected_depth_topic_.c_str() : "<disabled>");
    RCLCPP_INFO(
      get_logger(),
      "boundary constraint: sides=%s%s%s z=%.2f spacing=%.2f max_segment=%.2f",
      include_left_ ? "left " : "", include_right_ ? "right " : "",
      include_front_ ? "front" : "", boundary_marking_height_, boundary_spacing_,
      boundary_max_segment_length_);
    RCLCPP_INFO(
      get_logger(),
      "boundary smoothing: %s median=%d max_jump=%.2f spatial=%.2f/%d "
      "resample=%.2f temporal=%.2f..%.2f hold=%d",
      boundary_smoothing_enabled_ ? "on" : "off", boundary_median_window_,
      boundary_max_lateral_jump_, boundary_spatial_smoothing_,
      boundary_spatial_iterations_, boundary_resample_spacing_, boundary_temporal_alpha_,
      boundary_adaptive_alpha_max_, boundary_max_hold_frames_);
  }

private:
  void loadParameters()
  {
    obstacle_input_topic_ = declare_parameter<std::string>(
      "input.obstacle_topic", "/obstacle_points");
    ground_grid_input_topic_ = declare_parameter<std::string>(
      "input.ground_grid_topic", "/traversability/ground_grid_points");
    boundary_input_topic_ = declare_parameter<std::string>(
      "input.boundary_topic", "/traversability/classified_boundary_points");
    pothole_boundary_input_topic_ = declare_parameter<std::string>(
      "input.pothole_boundary_topic", "/traversability/pothole_boundary_points");
    boundary_veto_half_width_ = declare_parameter<double>("boundary.fov_cell_size", 0.15) * 0.5;
    if (boundary_veto_half_width_ <= 0.0) {
      throw std::runtime_error("boundary.fov_cell_size must be positive");
    }
    depth_output_topic_ = declare_parameter<std::string>(
      "output.depth_topic", "/traversability/costmap/depth_obstacles");
    boundary_output_topic_ = declare_parameter<std::string>(
      "output.boundary_topic", "/traversability/costmap/boundary_constraints");
    pothole_boundary_output_topic_ = declare_parameter<std::string>(
      "output.pothole_boundary_topic", "/traversability/costmap/pothole_constraints");
    expected_input_frame_ = declare_parameter<std::string>(
      "expected_input_frame", "gravity_aligned_base");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");

    depth_x_min_ = declare_parameter<double>("depth.x_min", 0.30);
    depth_x_max_ = declare_parameter<double>("depth.x_max", 4.0);
    depth_abs_y_max_ = declare_parameter<double>("depth.abs_y_max", 3.0);
    depth_z_min_ = declare_parameter<double>("depth.min_height", 0.10);
    depth_z_max_ = declare_parameter<double>("depth.max_height", 1.20);
    depth_max_range_ = declare_parameter<double>("depth.max_range", 4.0);
    depth_voxel_size_ = declare_parameter<double>("depth.voxel_size", 0.05);
    depth_denoise_enabled_ = declare_parameter<bool>("depth.denoise.enabled", true);
    depth_denoise_radius_ = declare_parameter<double>("depth.denoise.search_radius", 0.12);
    depth_denoise_min_neighbors_ = std::max(
      1, static_cast<int>(declare_parameter<int>("depth.denoise.min_neighbors", 3)));
    publish_rejected_depth_ = declare_parameter<bool>(
      "depth.denoise.publish_rejected_points", true);
    rejected_depth_topic_ = declare_parameter<std::string>(
      "depth.denoise.rejected_topic", "/traversability/costmap/depth_noise");
    relative_ground_enabled_ = declare_parameter<bool>("depth.relative_ground.enabled", true);
    relative_ground_min_height_ = declare_parameter<double>(
      "depth.relative_ground.min_height", 0.10);
    relative_ground_search_radius_ = declare_parameter<double>(
      "depth.relative_ground.search_radius", 0.45);
    relative_ground_min_neighbors_ = std::max(
      3, static_cast<int>(declare_parameter<int>("depth.relative_ground.min_neighbors", 4)));
    relative_ground_max_plane_rmse_ = declare_parameter<double>(
      "depth.relative_ground.max_plane_rmse", 0.08);
    relative_ground_max_slope_deg_ = declare_parameter<double>(
      "depth.relative_ground.max_slope_deg", 25.0);
    relative_ground_max_age_ = declare_parameter<double>(
      "depth.relative_ground.max_age", 0.15);
    relative_ground_keep_without_ground_ = declare_parameter<bool>(
      "depth.relative_ground.keep_without_ground", true);
    publish_ground_rejected_depth_ = declare_parameter<bool>(
      "depth.relative_ground.publish_rejected_points", true);
    ground_rejected_depth_topic_ = declare_parameter<std::string>(
      "depth.relative_ground.rejected_topic",
      "/traversability/costmap/depth_ground_rejected");

    include_left_ = declare_parameter<bool>("boundary.include_left", true);
    include_right_ = declare_parameter<bool>("boundary.include_right", true);
    include_front_ = declare_parameter<bool>("boundary.include_front", false);
    boundary_x_min_ = declare_parameter<double>("boundary.x_min", 0.30);
    boundary_x_max_ = declare_parameter<double>("boundary.x_max", 4.0);
    boundary_abs_y_max_ = declare_parameter<double>("boundary.abs_y_max", 3.0);
    boundary_marking_height_ = declare_parameter<double>("boundary.marking_height", 0.30);
    boundary_spacing_ = declare_parameter<double>("boundary.point_spacing", 0.05);
    boundary_max_segment_length_ = declare_parameter<double>(
      "boundary.max_segment_length", 0.45);
    boundary_smoothing_enabled_ = declare_parameter<bool>("boundary.smoothing.enabled", true);
    boundary_median_window_ = std::max(
      1, static_cast<int>(declare_parameter<int>("boundary.smoothing.median_window", 5)));
    if (boundary_median_window_ % 2 == 0) {
      ++boundary_median_window_;
    }
    boundary_max_lateral_jump_ = declare_parameter<double>(
      "boundary.smoothing.max_lateral_jump", 0.30);
    boundary_min_valid_points_ = std::max(
      2, static_cast<int>(declare_parameter<int>("boundary.smoothing.min_valid_points", 4)));
    boundary_spatial_smoothing_ = std::clamp(
      declare_parameter<double>("boundary.smoothing.spatial_smoothing", 0.35), 0.0, 1.0);
    boundary_spatial_iterations_ = std::max(
      0, static_cast<int>(declare_parameter<int>("boundary.smoothing.spatial_iterations", 3)));
    boundary_resample_spacing_ = declare_parameter<double>(
      "boundary.smoothing.resample_spacing", boundary_spacing_);
    boundary_temporal_alpha_ = std::clamp(
      declare_parameter<double>("boundary.smoothing.temporal_alpha", 0.20), 0.0, 1.0);
    boundary_adaptive_alpha_max_ = std::clamp(
      declare_parameter<double>("boundary.smoothing.adaptive_alpha_max", 0.45),
      boundary_temporal_alpha_, 1.0);
    boundary_max_temporal_jump_ = declare_parameter<double>(
      "boundary.smoothing.max_temporal_jump", 0.40);
    boundary_max_hold_frames_ = std::max(
      0, static_cast<int>(declare_parameter<int>("boundary.smoothing.max_hold_frames", 3)));
    boundary_reset_timeout_ = declare_parameter<double>(
      "boundary.smoothing.reset_timeout", 0.50);
    boundary_min_corridor_width_ = declare_parameter<double>(
      "boundary.smoothing.min_corridor_width", 0.70);

    pothole_enabled_ = declare_parameter<bool>("pothole.enabled", true);
    pothole_x_min_ = declare_parameter<double>("pothole.x_min", 0.30);
    pothole_x_max_ = declare_parameter<double>("pothole.x_max", 4.0);
    pothole_abs_y_max_ = declare_parameter<double>("pothole.abs_y_max", 3.0);
    pothole_marking_height_ = declare_parameter<double>("pothole.marking_height", 0.30);
    pothole_spacing_ = declare_parameter<double>("pothole.point_spacing", 0.05);
    pothole_max_segment_length_ = declare_parameter<double>(
      "pothole.max_segment_length", 0.35);
    pothole_stale_timeout_ = declare_parameter<double>("pothole.stale_timeout", 0.50);
    save_pothole_pcd_ = declare_parameter<bool>("pothole.save.enabled", false);
    pothole_save_directory_ = declare_parameter<std::string>(
      "pothole.save.directory", "/home/cat/nanhu-research-navigation/pics");

    if (depth_x_max_ <= depth_x_min_ || depth_z_max_ < depth_z_min_ ||
      depth_abs_y_max_ <= 0.0 || depth_max_range_ <= 0.0 || depth_voxel_size_ < 0.0)
    {
      throw std::runtime_error("invalid depth costmap filter parameters");
    }
    if (depth_denoise_enabled_ && depth_denoise_radius_ <= 0.0) {
      throw std::runtime_error("depth.denoise.search_radius must be positive when enabled");
    }
    if (publish_rejected_depth_ && rejected_depth_topic_.empty()) {
      throw std::runtime_error("depth.denoise.rejected_topic cannot be empty when publishing");
    }
    if (relative_ground_enabled_ &&
      (ground_grid_input_topic_.empty() || relative_ground_min_height_ < 0.0 ||
      relative_ground_search_radius_ <= 0.0 || relative_ground_max_plane_rmse_ < 0.0 ||
      relative_ground_max_slope_deg_ <= 0.0 || relative_ground_max_slope_deg_ >= 90.0 ||
      relative_ground_max_age_ < 0.0))
    {
      throw std::runtime_error("invalid depth.relative_ground parameters");
    }
    if (publish_ground_rejected_depth_ && ground_rejected_depth_topic_.empty()) {
      throw std::runtime_error(
              "depth.relative_ground.rejected_topic cannot be empty when publishing");
    }
    if (boundary_x_max_ <= boundary_x_min_ || boundary_abs_y_max_ <= 0.0 ||
      boundary_spacing_ <= 0.0 || boundary_max_segment_length_ < boundary_spacing_)
    {
      throw std::runtime_error("invalid boundary costmap parameters");
    }
    if (boundary_smoothing_enabled_ &&
      (boundary_max_lateral_jump_ <= 0.0 || boundary_resample_spacing_ <= 0.0 ||
      boundary_max_segment_length_ < boundary_resample_spacing_ ||
      boundary_max_temporal_jump_ <= 0.0 || boundary_reset_timeout_ < 0.0 ||
      boundary_min_corridor_width_ <= 0.0))
    {
      throw std::runtime_error("invalid boundary smoothing parameters");
    }
    if (pothole_enabled_ &&
      (pothole_boundary_input_topic_.empty() || pothole_boundary_output_topic_.empty() ||
      pothole_x_max_ <= pothole_x_min_ || pothole_abs_y_max_ <= 0.0 ||
      pothole_spacing_ <= 0.0 || pothole_max_segment_length_ < pothole_spacing_ ||
      pothole_stale_timeout_ <= 0.0))
    {
      throw std::runtime_error("invalid pothole costmap parameters");
    }
  }

  void checkInputFrame(const std::string & frame_id)
  {
    if (!expected_input_frame_.empty() && frame_id != expected_input_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "expected input frame '%s' but received '%s'; coordinates are relabeled, not TF-transformed",
        expected_input_frame_.c_str(), frame_id.c_str());
    }
  }

  std::string outputFrame(const std::string & input_frame) const
  {
    return output_frame_.empty() ? input_frame : output_frame_;
  }

  static bool finite(const pcl::PointXYZI & point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  VoxelKey groundBucket(const pcl::PointXYZI & point) const
  {
    return {
      static_cast<int>(std::floor(point.x / relative_ground_search_radius_)),
      static_cast<int>(std::floor(point.y / relative_ground_search_radius_)), 0};
  }

  void groundGridCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    checkInputFrame(message->header.frame_id);
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*message, input);

    auto surface = std::make_shared<GroundSurface>();
    surface->stamp_ns = stampNanoseconds(message->header.stamp);
    surface->frame_id = message->header.frame_id;
    surface->points.reserve(input.size());
    for (const auto & point : input) {
      if (finite(point)) {
        surface->points.push_back(point);
      }
    }
    surface->buckets.reserve(surface->points.size());
    for (std::size_t index = 0; index < surface->points.size(); ++index) {
      surface->buckets[groundBucket(surface->points[index])].push_back(index);
    }

    std::lock_guard<std::mutex> lock(ground_surface_mutex_);
    latest_ground_surface_ = std::move(surface);
  }

  std::shared_ptr<const GroundSurface> matchingGroundSurface(
    const sensor_msgs::msg::PointCloud2 & obstacle_message, double & age_seconds) const
  {
    std::shared_ptr<const GroundSurface> surface;
    {
      std::lock_guard<std::mutex> lock(ground_surface_mutex_);
      surface = latest_ground_surface_;
    }
    age_seconds = 0.0;
    if (!surface || surface->points.empty() ||
      surface->frame_id != obstacle_message.header.frame_id)
    {
      return nullptr;
    }
    const std::int64_t obstacle_stamp_ns = stampNanoseconds(obstacle_message.header.stamp);
    const std::int64_t delta_ns = obstacle_stamp_ns >= surface->stamp_ns ?
      obstacle_stamp_ns - surface->stamp_ns : surface->stamp_ns - obstacle_stamp_ns;
    age_seconds = static_cast<double>(delta_ns) * 1e-9;
    if (age_seconds > relative_ground_max_age_) {
      return nullptr;
    }
    return surface;
  }

  bool estimateGroundHeight(
    const pcl::PointXYZI & point, const GroundSurface & surface,
    double & ground_height, double & plane_rmse) const
  {
    const VoxelKey center = groundBucket(point);
    const double squared_radius =
      relative_ground_search_radius_ * relative_ground_search_radius_;
    std::vector<const pcl::PointXYZI *> neighbors;
    neighbors.reserve(32);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const auto bucket = surface.buckets.find({center.x + dx, center.y + dy, 0});
        if (bucket == surface.buckets.end()) {
          continue;
        }
        for (const std::size_t index : bucket->second) {
          const auto & ground = surface.points[index];
          const double delta_x = static_cast<double>(ground.x - point.x);
          const double delta_y = static_cast<double>(ground.y - point.y);
          if (delta_x * delta_x + delta_y * delta_y <= squared_radius) {
            neighbors.push_back(&ground);
          }
        }
      }
    }
    if (neighbors.size() < static_cast<std::size_t>(relative_ground_min_neighbors_)) {
      return false;
    }

    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d right_hand_side = Eigen::Vector3d::Zero();
    const double weight_floor = 0.15 * 0.15;
    for (const auto * ground : neighbors) {
      const double delta_x = static_cast<double>(ground->x - point.x);
      const double delta_y = static_cast<double>(ground->y - point.y);
      const double weight = 1.0 / (delta_x * delta_x + delta_y * delta_y + weight_floor);
      const Eigen::Vector3d sample(ground->x, ground->y, 1.0);
      normal.noalias() += weight * sample * sample.transpose();
      right_hand_side.noalias() += weight * sample * static_cast<double>(ground->z);
    }
    const Eigen::FullPivLU<Eigen::Matrix3d> decomposition(normal);
    if (decomposition.rank() < 3) {
      return false;
    }
    const Eigen::Vector3d plane = decomposition.solve(right_hand_side);
    if (!plane.allFinite()) {
      return false;
    }
    const double slope_deg =
      std::atan(std::hypot(plane.x(), plane.y())) * 180.0 / std::acos(-1.0);
    if (slope_deg > relative_ground_max_slope_deg_) {
      return false;
    }

    double squared_error_sum = 0.0;
    for (const auto * ground : neighbors) {
      const double prediction =
        plane.x() * static_cast<double>(ground->x) +
        plane.y() * static_cast<double>(ground->y) + plane.z();
      const double error = static_cast<double>(ground->z) - prediction;
      squared_error_sum += error * error;
    }
    plane_rmse = std::sqrt(squared_error_sum / static_cast<double>(neighbors.size()));
    if (!std::isfinite(plane_rmse) || plane_rmse > relative_ground_max_plane_rmse_) {
      return false;
    }
    ground_height = plane.x() * static_cast<double>(point.x) +
      plane.y() * static_cast<double>(point.y) + plane.z();
    return std::isfinite(ground_height);
  }

  void relativeGroundFilter(
    const pcl::PointCloud<pcl::PointXYZI> & input,
    const sensor_msgs::msg::PointCloud2 & input_message,
    pcl::PointCloud<pcl::PointXYZI> & output,
    pcl::PointCloud<pcl::PointXYZI> & rejected,
    std::size_t & points_without_ground,
    double & ground_age_seconds) const
  {
    output.clear();
    rejected.clear();
    points_without_ground = 0U;
    ground_age_seconds = 0.0;
    output.reserve(input.size());
    rejected.reserve(input.size());
    if (!relative_ground_enabled_ || input.empty()) {
      output = input;
      return;
    }

    const auto surface = matchingGroundSurface(input_message, ground_age_seconds);
    if (!surface) {
      points_without_ground = input.size();
      if (relative_ground_keep_without_ground_) {
        output = input;
      } else {
        rejected = input;
      }
      return;
    }

    for (const auto & point : input) {
      double ground_height = 0.0;
      double plane_rmse = 0.0;
      if (!estimateGroundHeight(point, *surface, ground_height, plane_rmse)) {
        ++points_without_ground;
        if (relative_ground_keep_without_ground_) {
          output.push_back(point);
        } else {
          rejected.push_back(point);
        }
        continue;
      }
      if (static_cast<double>(point.z) - ground_height >= relative_ground_min_height_) {
        output.push_back(point);
      } else {
        rejected.push_back(point);
      }
    }
  }

  void publishCloud(
    const pcl::PointCloud<pcl::PointXYZI> & cloud,
    const sensor_msgs::msg::PointCloud2 & input_message,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher) const
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = input_message.header;
    message.header.frame_id = outputFrame(input_message.header.frame_id);
    publisher->publish(message);
  }

  bool insideDepthFilter(const pcl::PointXYZI & point) const
  {
    if (!finite(point) || point.x < depth_x_min_ || point.x > depth_x_max_ ||
      std::abs(point.y) > depth_abs_y_max_ || point.z < depth_z_min_ ||
      point.z > depth_z_max_)
    {
      return false;
    }
    return std::hypot(point.x, point.y) <= depth_max_range_;
  }

  VoxelKey denoiseBucket(const pcl::PointXYZI & point) const
  {
    return {
      static_cast<int>(std::floor(point.x / depth_denoise_radius_)),
      static_cast<int>(std::floor(point.y / depth_denoise_radius_)),
      static_cast<int>(std::floor(point.z / depth_denoise_radius_))};
  }

  void radiusDenoise(
    const pcl::PointCloud<pcl::PointXYZI> & input,
    pcl::PointCloud<pcl::PointXYZI> & output,
    pcl::PointCloud<pcl::PointXYZI> & rejected) const
  {
    output.clear();
    rejected.clear();
    output.reserve(input.size());
    if (!depth_denoise_enabled_ || input.empty()) {
      output = input;
      return;
    }
    rejected.reserve(input.size());

    std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> buckets;
    buckets.reserve(input.size());
    for (std::size_t index = 0; index < input.size(); ++index) {
      buckets[denoiseBucket(input[index])].push_back(index);
    }

    const double squared_radius = depth_denoise_radius_ * depth_denoise_radius_;
    for (std::size_t index = 0; index < input.size(); ++index) {
      const auto & point = input[index];
      const VoxelKey center = denoiseBucket(point);
      int neighbor_count = 0;
      for (int dx = -1; dx <= 1 && neighbor_count < depth_denoise_min_neighbors_; ++dx) {
        for (int dy = -1; dy <= 1 && neighbor_count < depth_denoise_min_neighbors_; ++dy) {
          for (int dz = -1; dz <= 1 && neighbor_count < depth_denoise_min_neighbors_; ++dz) {
            const auto bucket = buckets.find({center.x + dx, center.y + dy, center.z + dz});
            if (bucket == buckets.end()) {
              continue;
            }
            for (const std::size_t other_index : bucket->second) {
              if (other_index == index) {
                continue;
              }
              const auto & other = input[other_index];
              const double delta_x = static_cast<double>(other.x - point.x);
              const double delta_y = static_cast<double>(other.y - point.y);
              const double delta_z = static_cast<double>(other.z - point.z);
              if (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z <= squared_radius) {
                ++neighbor_count;
                if (neighbor_count >= depth_denoise_min_neighbors_) {
                  break;
                }
              }
            }
          }
        }
      }
      if (neighbor_count >= depth_denoise_min_neighbors_) {
        output.push_back(point);
      } else {
        rejected.push_back(point);
      }
    }
  }

  void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    checkInputFrame(message->header.frame_id);
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*message, input);

    pcl::PointCloud<pcl::PointXYZI> filtered;
    filtered.reserve(input.size());
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    if (depth_voxel_size_ > 0.0) {
      occupied_voxels.reserve(input.size());
    }

    for (const auto & point : input) {
      if (!insideDepthFilter(point)) {
        continue;
      }
      if (depth_voxel_size_ > 0.0) {
        const VoxelKey key{
          static_cast<int>(std::floor(point.x / depth_voxel_size_)),
          static_cast<int>(std::floor(point.y / depth_voxel_size_)),
          static_cast<int>(std::floor(point.z / depth_voxel_size_))};
        if (!occupied_voxels.insert(key).second) {
          continue;
        }
      }
      filtered.push_back(point);
    }

    pcl::PointCloud<pcl::PointXYZI> relative_filtered;
    pcl::PointCloud<pcl::PointXYZI> ground_rejected;
    std::size_t points_without_ground = 0U;
    double ground_age_seconds = 0.0;
    relativeGroundFilter(
      filtered, *message, relative_filtered, ground_rejected,
      points_without_ground, ground_age_seconds);

    pcl::PointCloud<pcl::PointXYZI> output;
    pcl::PointCloud<pcl::PointXYZI> rejected;
    radiusDenoise(relative_filtered, output, rejected);
    publishCloud(output, *message, depth_pub_);
    if (rejected_depth_pub_) {
      publishCloud(rejected, *message, rejected_depth_pub_);
    }
    if (ground_rejected_depth_pub_) {
      publishCloud(ground_rejected, *message, ground_rejected_depth_pub_);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "costmap depth points: input=%zu spatial=%zu relative=%zu output=%zu "
      "rejected_ground=%zu rejected_noise=%zu without_ground=%zu ground_age=%.3fs",
      input.size(), filtered.size(), relative_filtered.size(), output.size(),
      ground_rejected.size(), rejected.size(), points_without_ground, ground_age_seconds);
  }

  bool insideBoundaryFilter(const pcl::PointXYZI & point) const
  {
    return finite(point) && point.x >= boundary_x_min_ && point.x <= boundary_x_max_ &&
           std::abs(point.y) <= boundary_abs_y_max_;
  }

  std::size_t boundarySampleCount() const
  {
    return static_cast<std::size_t>(
      std::floor((boundary_x_max_ - boundary_x_min_) / boundary_resample_spacing_ + 1e-6)) + 1U;
  }

  double boundarySampleX(std::size_t index) const
  {
    return boundary_x_min_ + static_cast<double>(index) * boundary_resample_spacing_;
  }

  void resetBoundaryTracks()
  {
    left_boundary_track_.clear();
    right_boundary_track_.clear();
  }

  void resetBoundaryTracksIfNeeded(const builtin_interfaces::msg::Time & stamp)
  {
    const rclcpp::Time current_stamp(stamp);
    if (last_boundary_stamp_.nanoseconds() != 0 && current_stamp.nanoseconds() != 0) {
      const double elapsed = (current_stamp - last_boundary_stamp_).seconds();
      if (elapsed < 0.0 ||
        (boundary_reset_timeout_ > 0.0 && elapsed > boundary_reset_timeout_))
      {
        resetBoundaryTracks();
      }
    }
    last_boundary_stamp_ = current_stamp;
  }

  std::vector<pcl::PointXYZI> spatiallySmoothBoundary(
    const std::vector<pcl::PointXYZI> & group) const
  {
    std::vector<pcl::PointXYZI> sorted = group;
    std::sort(
      sorted.begin(), sorted.end(),
      [](const auto & first, const auto & second) {return first.x < second.x;});
    if (sorted.size() < static_cast<std::size_t>(boundary_min_valid_points_)) {
      return {};
    }

    std::vector<pcl::PointXYZI> robust;
    robust.reserve(sorted.size());
    const std::size_t half_window = static_cast<std::size_t>(boundary_median_window_ / 2);
    for (std::size_t index = 0; index < sorted.size(); ++index) {
      const std::size_t begin = index > half_window ? index - half_window : 0U;
      const std::size_t end = std::min(sorted.size(), index + half_window + 1U);
      std::vector<float> lateral_values;
      lateral_values.reserve(end - begin);
      for (std::size_t neighbor = begin; neighbor < end; ++neighbor) {
        lateral_values.push_back(sorted[neighbor].y);
      }
      const auto middle = lateral_values.begin() + lateral_values.size() / 2U;
      std::nth_element(lateral_values.begin(), middle, lateral_values.end());
      const double median = static_cast<double>(*middle);
      if (std::abs(static_cast<double>(sorted[index].y) - median) <=
        boundary_max_lateral_jump_)
      {
        robust.push_back(sorted[index]);
      }
    }
    if (robust.size() < static_cast<std::size_t>(boundary_min_valid_points_)) {
      return {};
    }

    const std::vector<pcl::PointXYZI> measurements = robust;
    for (int iteration = 0; iteration < boundary_spatial_iterations_; ++iteration) {
      std::vector<pcl::PointXYZI> next = robust;
      for (std::size_t index = 1; index + 1U < robust.size(); ++index) {
        const auto & previous = robust[index - 1U];
        const auto & current = robust[index];
        const auto & following = robust[index + 1U];
        const double previous_gap = static_cast<double>(current.x - previous.x);
        const double following_gap = static_cast<double>(following.x - current.x);
        const double total_gap = static_cast<double>(following.x - previous.x);
        if (previous_gap <= 0.0 || following_gap <= 0.0 ||
          previous_gap > boundary_max_segment_length_ ||
          following_gap > boundary_max_segment_length_ || total_gap <= 0.0)
        {
          continue;
        }
        const double ratio = previous_gap / total_gap;
        const double locally_linear_y =
          static_cast<double>(previous.y) + ratio * static_cast<double>(following.y - previous.y);
        next[index].y = static_cast<float>(
          (1.0 - boundary_spatial_smoothing_) * static_cast<double>(measurements[index].y) +
          boundary_spatial_smoothing_ * locally_linear_y);
      }
      robust.swap(next);
    }
    return robust;
  }

  bool interpolateBoundaryY(
    const std::vector<pcl::PointXYZI> & candidates, double x, double & y) const
  {
    if (candidates.empty()) {
      return false;
    }
    const auto upper = std::lower_bound(
      candidates.begin(), candidates.end(), x,
      [](const auto & point, double value) {return static_cast<double>(point.x) < value;});
    const double endpoint_tolerance = 0.5 * boundary_resample_spacing_ + 1e-6;
    if (upper != candidates.end() &&
      std::abs(static_cast<double>(upper->x) - x) <= endpoint_tolerance)
    {
      y = upper->y;
      return true;
    }
    if (upper == candidates.begin() || upper == candidates.end()) {
      return false;
    }
    const auto lower = std::prev(upper);
    if (std::abs(static_cast<double>(lower->x) - x) <= endpoint_tolerance) {
      y = lower->y;
      return true;
    }
    const double segment = static_cast<double>(upper->x - lower->x);
    if (segment <= 0.0 || segment > boundary_max_segment_length_) {
      return false;
    }
    const double ratio = (x - static_cast<double>(lower->x)) / segment;
    y = static_cast<double>(lower->y) + ratio * static_cast<double>(upper->y - lower->y);
    return true;
  }

  void updateBoundaryTrack(
    const std::vector<pcl::PointXYZI> & group, BoundaryTrack & track,
    const std::vector<pcl::PointXYZI> & vetoes, int label)
  {
    const std::size_t sample_count = boundarySampleCount();
    if (track.y.size() != sample_count) {
      track.resize(sample_count);
    }
    auto sorted = group;
    std::sort(sorted.begin(), sorted.end(), [](const auto & a, const auto & b) {
      return a.x < b.x;
    });
    std::vector<std::vector<pcl::PointXYZI>> segments;
    std::vector<pcl::PointXYZI> segment;
    for (const auto & point : sorted) {
      if (!segment.empty() && std::any_of(vetoes.begin(), vetoes.end(), [&](const auto & v) {
        return static_cast<int>(std::lround(-v.intensity)) == label &&
               v.x >= segment.back().x && v.x <= point.x;
      })) {
        segments.push_back(spatiallySmoothBoundary(segment));
        segment.clear();
      }
      segment.push_back(point);
    }
    segments.push_back(spatiallySmoothBoundary(segment));
    for (std::size_t index = 0; index < sample_count; ++index) {
      double measurement = 0.0;
      const bool has_measurement = std::any_of(segments.begin(), segments.end(),
        [&](const auto & candidates) {
          return interpolateBoundaryY(candidates, boundarySampleX(index), measurement);
        });
      if (has_measurement) {
        if (track.valid[index] != 0U) {
          double difference = measurement - track.y[index];
          difference = std::clamp(
            difference, -boundary_max_temporal_jump_, boundary_max_temporal_jump_);
          const double response = std::min(
            1.0, std::abs(difference) / boundary_max_lateral_jump_);
          const double alpha = boundary_temporal_alpha_ +
            response * (boundary_adaptive_alpha_max_ - boundary_temporal_alpha_);
          track.y[index] += alpha * difference;
        } else {
          track.y[index] = measurement;
        }
        track.valid[index] = 1U;
        track.missing_frames[index] = 0;
      } else if (track.valid[index] != 0U &&
        track.missing_frames[index] < boundary_max_hold_frames_)
      {
        ++track.missing_frames[index];
      } else {
        track.valid[index] = 0U;
        track.missing_frames[index] = 0;
      }
    }
  }

  void appendSmoothedBoundaryTracks(pcl::PointCloud<pcl::PointXYZI> & output) const
  {
    const std::size_t sample_count = boundarySampleCount();
    for (std::size_t index = 0; index < sample_count; ++index) {
      const bool left_valid = include_left_ && left_boundary_track_.valid[index] != 0U;
      const bool right_valid = include_right_ && right_boundary_track_.valid[index] != 0U;
      if (left_valid && right_valid &&
        left_boundary_track_.y[index] - right_boundary_track_.y[index] <
        boundary_min_corridor_width_)
      {
        continue;
      }
      const float x = static_cast<float>(boundarySampleX(index));
      if (left_valid) {
        pcl::PointXYZI point;
        point.x = x;
        point.y = static_cast<float>(left_boundary_track_.y[index]);
        point.z = static_cast<float>(boundary_marking_height_);
        point.intensity = 1.0F;
        output.push_back(point);
      }
      if (right_valid) {
        pcl::PointXYZI point;
        point.x = x;
        point.y = static_cast<float>(right_boundary_track_.y[index]);
        point.z = static_cast<float>(boundary_marking_height_);
        point.intensity = 2.0F;
        output.push_back(point);
      }
    }
  }

  void appendBoundaryGroup(
    const std::vector<pcl::PointXYZI> & group,
    pcl::PointCloud<pcl::PointXYZI> & output) const
  {
    if (group.empty()) {
      return;
    }

    pcl::PointXYZI previous = group.front();
    previous.z = static_cast<float>(boundary_marking_height_);
    output.push_back(previous);
    for (std::size_t index = 1; index < group.size(); ++index) {
      pcl::PointXYZI current = group[index];
      current.z = static_cast<float>(boundary_marking_height_);
      const double dx = static_cast<double>(current.x - previous.x);
      const double dy = static_cast<double>(current.y - previous.y);
      const double distance = std::hypot(dx, dy);
      if (distance <= boundary_max_segment_length_) {
        const int steps = std::max(1, static_cast<int>(std::ceil(distance / boundary_spacing_)));
        for (int step = 1; step <= steps; ++step) {
          const float ratio = static_cast<float>(step) / static_cast<float>(steps);
          pcl::PointXYZI interpolated;
          interpolated.x = previous.x + ratio * (current.x - previous.x);
          interpolated.y = previous.y + ratio * (current.y - previous.y);
          interpolated.z = static_cast<float>(boundary_marking_height_);
          interpolated.intensity = current.intensity;
          output.push_back(interpolated);
        }
      } else {
        // Do not bridge a large perception gap with an invented wall.
        output.push_back(current);
      }
      previous = current;
    }
  }

  void boundaryCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    checkInputFrame(message->header.frame_id);
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*message, input);

    std::vector<pcl::PointXYZI> left;
    std::vector<pcl::PointXYZI> right;
    std::vector<pcl::PointXYZI> front;
    std::vector<pcl::PointXYZI> vetoes;
    left.reserve(input.size());
    right.reserve(input.size());
    front.reserve(input.size());
    for (const auto & point : input) {
      if (!insideBoundaryFilter(point)) {
        continue;
      }
      const int label = static_cast<int>(std::lround(point.intensity));
      if (label == -1 || label == -2) {
        vetoes.push_back(point);
      } else if (label == 1 && include_left_) {
        left.push_back(point);
      } else if (label == 2 && include_right_) {
        right.push_back(point);
      } else if (label == 3 && include_front_) {
        front.push_back(point);
      }
    }

    pcl::PointCloud<pcl::PointXYZI> output;
    const double expansion = std::max(1.0, 0.15 / boundary_spacing_);
    output.reserve(
      static_cast<std::size_t>((left.size() + right.size() + front.size()) * expansion));
    if (boundary_smoothing_enabled_) {
      resetBoundaryTracksIfNeeded(message->header.stamp);
      updateBoundaryTrack(left, left_boundary_track_, vetoes, 1);
      updateBoundaryTrack(right, right_boundary_track_, vetoes, 2);
      appendSmoothedBoundaryTracks(output);
      // The optional front edge is transverse rather than y(x), so retain its original behavior.
      appendBoundaryGroup(front, output);
    } else {
      appendBoundaryGroup(left, output);
      appendBoundaryGroup(right, output);
      appendBoundaryGroup(front, output);
    }
    // A FOV veto overrides interpolation and historical holds, even with smoothing disabled.
    const auto vetoed = [&](double x, int label) {
        return std::any_of(vetoes.begin(), vetoes.end(), [&](const auto & veto) {
          return static_cast<int>(std::lround(-veto.intensity)) == label &&
                 std::abs(x - veto.x) <= boundary_veto_half_width_ + 1e-6;
        });
      };
    if (boundary_smoothing_enabled_) {
      for (std::size_t i = 0; i < boundarySampleCount(); ++i) {
        if (vetoed(boundarySampleX(i), 1)) {left_boundary_track_.valid[i] = 0U;}
        if (vetoed(boundarySampleX(i), 2)) {right_boundary_track_.valid[i] = 0U;}
      }
    }
    output.erase(std::remove_if(output.begin(), output.end(), [&](const auto & point) {
      return vetoed(point.x, static_cast<int>(std::lround(point.intensity)));
    }), output.end());
    publishCloud(output, *message, boundary_pub_);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "costmap boundary points: input=%zu left=%zu right=%zu front=%zu output=%zu",
      input.size(), left.size(), right.size(), front.size(), output.size());
  }

  bool insidePotholeFilter(const pcl::PointXYZI & point) const
  {
    return finite(point) && point.x >= pothole_x_min_ && point.x <= pothole_x_max_ &&
           std::abs(point.y) <= pothole_abs_y_max_;
  }

  bool appendClosedPothole(
    const std::vector<pcl::PointXYZI> & contour,
    pcl::PointCloud<pcl::PointXYZI> & output) const
  {
    if (contour.size() < 3U) {
      return false;
    }
    for (std::size_t index = 0; index < contour.size(); ++index) {
      const auto & current = contour[index];
      const auto & next = contour[(index + 1U) % contour.size()];
      if (std::hypot(next.x - current.x, next.y - current.y) > pothole_max_segment_length_) {
        return false;
      }
    }
    for (std::size_t index = 0; index < contour.size(); ++index) {
      pcl::PointXYZI current = contour[index];
      pcl::PointXYZI next = contour[(index + 1U) % contour.size()];
      current.z = static_cast<float>(pothole_marking_height_);
      next.z = static_cast<float>(pothole_marking_height_);
      if (index == 0U) {
        output.push_back(current);
      }
      const double distance = std::hypot(next.x - current.x, next.y - current.y);
      const int steps = std::max(
        1, static_cast<int>(std::ceil(distance / pothole_spacing_)));
      for (int step = 1; step <= steps; ++step) {
        const float ratio = static_cast<float>(step) / static_cast<float>(steps);
        pcl::PointXYZI point;
        point.x = current.x + ratio * (next.x - current.x);
        point.y = current.y + ratio * (next.y - current.y);
        point.z = static_cast<float>(pothole_marking_height_);
        point.intensity = current.intensity;
        output.push_back(point);
      }
    }
    return true;
  }

  void potholeBoundaryCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    checkInputFrame(message->header.frame_id);
    last_pothole_input_time_ = now();
    received_pothole_input_ = true;
    pothole_stale_published_ = false;

    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*message, input);
    std::map<int, std::vector<pcl::PointXYZI>> contours;
    std::unordered_set<int> rejected;
    for (const auto & point : input) {
      if (!std::isfinite(point.intensity)) {continue;}
      const int id = static_cast<int>(std::lround(point.intensity));
      if (insidePotholeFilter(point)) {
        contours[id].push_back(point);
      } else {
        rejected.insert(id);
      }
    }

    pcl::PointCloud<pcl::PointXYZI> output;
    std::size_t invalid_contours = 0U;
    for (const auto & entry : contours) {
      if (rejected.count(entry.first) || !appendClosedPothole(entry.second, output)) {
        ++invalid_contours;
      }
    }
    publishCloud(output, *message, pothole_boundary_pub_);
    if (save_pothole_pcd_ && !output.empty()) {
      try {
        std::filesystem::create_directories(pothole_save_directory_);
        std::ostringstream stem;
        stem << pothole_save_directory_ << "/pothole_constraints_"
             << message->header.stamp.sec << "_" << std::setw(9) << std::setfill('0')
             << message->header.stamp.nanosec << "_"
             << std::chrono::system_clock::now().time_since_epoch().count();
        writeBinaryPcd(stem.str() + ".pcd", output);
        // PCD does not preserve ROS frame/time metadata; store it alongside.
        std::ofstream metadata(stem.str() + ".txt");
        metadata << "topic " << pothole_boundary_output_topic_ << "\n"
                 << "frame_id " << outputFrame(message->header.frame_id) << "\n"
                 << "stamp_sec " << message->header.stamp.sec << "\n"
                 << "stamp_nanosec " << message->header.stamp.nanosec << "\n"
                 << "points " << output.size() << "\n";
        if (!metadata) {throw std::runtime_error("PCD metadata writer failed");}
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "failed to save pothole PCD: %s", error.what());
      }
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "costmap potholes: input=%zu contours=%zu invalid=%zu output=%zu",
      input.size(), contours.size(), invalid_contours, output.size());
  }

  void publishEmptyPothole()
  {
    pcl::PointCloud<pcl::PointXYZI> empty;
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(empty, message);
    message.header.stamp = now();
    message.header.frame_id = output_frame_.empty() ? expected_input_frame_ : output_frame_;
    pothole_boundary_pub_->publish(message);
  }

  void potholeWatchdog()
  {
    if (!received_pothole_input_ ||
      (now() - last_pothole_input_time_).seconds() < 0.0 ||
      (now() - last_pothole_input_time_).seconds() > pothole_stale_timeout_)
    {
      publishEmptyPothole();
      if (pothole_stale_published_ || !received_pothole_input_) {return;}
      pothole_stale_published_ = true;
      RCLCPP_WARN(
        get_logger(), "pothole input timed out after %.2fs; published an empty cloud",
        pothole_stale_timeout_);
    }
  }

  std::string obstacle_input_topic_;
  std::string ground_grid_input_topic_;
  std::string boundary_input_topic_;
  std::string pothole_boundary_input_topic_;
  std::string depth_output_topic_;
  std::string boundary_output_topic_;
  std::string pothole_boundary_output_topic_;
  std::string expected_input_frame_;
  std::string output_frame_;
  std::string rejected_depth_topic_;
  std::string ground_rejected_depth_topic_;

  double depth_x_min_ = 0.30;
  double depth_x_max_ = 4.0;
  double depth_abs_y_max_ = 3.0;
  double depth_z_min_ = 0.10;
  double depth_z_max_ = 1.20;
  double depth_max_range_ = 4.0;
  double depth_voxel_size_ = 0.05;
  bool depth_denoise_enabled_ = true;
  double depth_denoise_radius_ = 0.12;
  int depth_denoise_min_neighbors_ = 3;
  bool publish_rejected_depth_ = true;
  bool relative_ground_enabled_ = true;
  double relative_ground_min_height_ = 0.10;
  double relative_ground_search_radius_ = 0.45;
  int relative_ground_min_neighbors_ = 4;
  double relative_ground_max_plane_rmse_ = 0.08;
  double relative_ground_max_slope_deg_ = 25.0;
  double relative_ground_max_age_ = 0.15;
  bool relative_ground_keep_without_ground_ = true;
  bool publish_ground_rejected_depth_ = true;

  bool include_left_ = true;
  double boundary_veto_half_width_ = 0.075;
  bool include_right_ = true;
  bool include_front_ = false;
  double boundary_x_min_ = 0.30;
  double boundary_x_max_ = 4.0;
  double boundary_abs_y_max_ = 3.0;
  double boundary_marking_height_ = 0.30;
  double boundary_spacing_ = 0.05;
  double boundary_max_segment_length_ = 0.45;
  bool boundary_smoothing_enabled_ = true;
  int boundary_median_window_ = 5;
  double boundary_max_lateral_jump_ = 0.30;
  int boundary_min_valid_points_ = 4;
  double boundary_spatial_smoothing_ = 0.35;
  int boundary_spatial_iterations_ = 3;
  double boundary_resample_spacing_ = 0.05;
  double boundary_temporal_alpha_ = 0.20;
  double boundary_adaptive_alpha_max_ = 0.45;
  double boundary_max_temporal_jump_ = 0.40;
  int boundary_max_hold_frames_ = 3;
  double boundary_reset_timeout_ = 0.50;
  double boundary_min_corridor_width_ = 0.70;

  bool pothole_enabled_ = true;
  bool save_pothole_pcd_ = false;
  std::string pothole_save_directory_;
  double pothole_x_min_ = 0.30;
  double pothole_x_max_ = 4.0;
  double pothole_abs_y_max_ = 3.0;
  double pothole_marking_height_ = 0.30;
  double pothole_spacing_ = 0.05;
  double pothole_max_segment_length_ = 0.35;
  double pothole_stale_timeout_ = 0.50;
  bool received_pothole_input_ = false;
  bool pothole_stale_published_ = false;
  rclcpp::Time last_pothole_input_time_{0, 0, RCL_ROS_TIME};

  BoundaryTrack left_boundary_track_;
  BoundaryTrack right_boundary_track_;
  rclcpp::Time last_boundary_stamp_{0, 0, RCL_ROS_TIME};

  mutable std::mutex ground_surface_mutex_;
  std::shared_ptr<const GroundSurface> latest_ground_surface_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pothole_boundary_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_rejected_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pothole_boundary_pub_;
  rclcpp::TimerBase::SharedPtr pothole_watchdog_timer_;
};

}  // namespace traversability_obstacle_cloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<traversability_obstacle_cloud::CameraCostmapCloudNode>());
  rclcpp::shutdown();
  return 0;
}
