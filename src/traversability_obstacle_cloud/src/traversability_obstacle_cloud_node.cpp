#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "traversability_obstacle_cloud/msg/detection2_d_array.hpp"

namespace traversability_obstacle_cloud
{

enum class CellClass : std::uint8_t
{
  UNKNOWN = 0,
  GROUND = 1,
  OBSTACLE = 2,
};

struct ElevationCell
{
  std::uint32_t count = 0;
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  double mean_z = 0.0;
  double m2 = 0.0;
  double ground_z = 0.0;
  double slope = 0.0;
  double roughness = 0.0;
  bool candidate = false;
  bool bridge = false;
  CellClass classification = CellClass::UNKNOWN;

  void add(double z)
  {
    ++count;
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);
    const double delta = z - mean_z;
    mean_z += delta / static_cast<double>(count);
    m2 += delta * (z - mean_z);
  }

  double variance() const
  {
    return count > 1 ? m2 / static_cast<double>(count - 1) : 0.0;
  }
};

struct GridSnapshot
{
  builtin_interfaces::msg::Time stamp;
  Eigen::Quaterniond alignment = Eigen::Quaterniond::Identity();
  int size_x = 0;
  int size_y = 0;
  std::vector<ElevationCell> cells;
};

class TraversabilityObstacleCloudNode : public rclcpp::Node
{
public:
  TraversabilityObstacleCloudNode()
  : Node("traversability_obstacle_cloud_node")
  {
    loadParameters();
    buildStaticRotations();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&TraversabilityObstacleCloudNode::cloudCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&TraversabilityObstacleCloudNode::imuCallback, this, std::placeholders::_1));
    if (publish_overlay_) {
      rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&TraversabilityObstacleCloudNode::rgbCallback, this, std::placeholders::_1));
    }
    if (pothole_enabled_) {
      pothole_detection_sub_ = create_subscription<msg::Detection2DArray>(
        pothole_detection_topic_, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(
          &TraversabilityObstacleCloudNode::potholeDetectionCallback, this,
          std::placeholders::_1));
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, qos);
    if (publish_ground_grid_) {
      ground_grid_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(ground_grid_topic_, qos);
    }
    if (publish_boundary_) {
      boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(boundary_topic_, qos);
      boundary_classified_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/traversability/classified_boundary_points", qos);
    }
    if (publish_obstacle_) {
      obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, qos);
    }
    if (publish_unknown_) {
      unknown_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(unknown_topic_, qos);
    }
    if (publish_grid_) {
      grid_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(grid_topic_, qos);
    }
    if (publish_overlay_) {
      overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(overlay_topic_, qos);
    }
    if (pothole_enabled_) {
      pothole_boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pothole_boundary_topic_, qos);
    }

    RCLCPP_INFO(
      get_logger(),
      "camera traversability: cloud=%s imu=%s frame=%s grid=%.2fm ROI x=[%.1f,%.1f] "
      "y=[%.1f,%.1f] z=[%.1f,%.1f] "
      "outputs(ground_grid=%s boundary=%s obstacle=%s unknown=%s grid=%s overlay=%s)",
      input_cloud_topic_.c_str(), imu_topic_.c_str(),
      output_frame_.c_str(), resolution_,
      x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
      publish_ground_grid_ ? "on" : "off", publish_boundary_ ? "on" : "off",
      publish_obstacle_ ? "on" : "off",
      publish_unknown_ ? "on" : "off",
      publish_grid_ ? "on" : "off", publish_overlay_ ? "on" : "off");
    if (pothole_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "pothole fusion: detections=%s boundary=%s class=%d confidence>=%.2f "
        "sync<=%.3fs drop>=%.3fm",
        pothole_detection_topic_.c_str(), pothole_boundary_topic_.c_str(),
        pothole_class_id_, pothole_min_confidence_, pothole_max_time_difference_,
        pothole_min_drop_);
    }
  }

private:
  void loadParameters()
  {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/depth/points");
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/camera/gyro_accel/sample");
    ground_topic_ = declare_parameter<std::string>("ground_topic", "/ground_points");
    ground_grid_topic_ = declare_parameter<std::string>(
      "ground_grid_topic", "/traversability/ground_grid_points");
    boundary_topic_ = declare_parameter<std::string>(
      "boundary_topic", "/traversability/ground_boundary_points");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/obstacle_points");
    unknown_topic_ = declare_parameter<std::string>("unknown_topic", "/unknown_points");
    grid_topic_ = declare_parameter<std::string>("grid_topic", "/traversability/grid");
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    overlay_topic_ = declare_parameter<std::string>(
      "overlay_topic", "/traversability/rgb_overlay");
    output_frame_ = declare_parameter<std::string>("output_frame", "gravity_aligned_base");
    publish_ground_grid_ = declare_parameter<bool>("publish.ground_grid", true);
    publish_boundary_ = declare_parameter<bool>("publish.boundary", true);
    publish_obstacle_ = declare_parameter<bool>("publish.obstacle", true);
    publish_unknown_ = declare_parameter<bool>("publish.unknown", true);
    publish_grid_ = declare_parameter<bool>("publish.grid", true);
    publish_overlay_ = declare_parameter<bool>("publish.rgb_overlay", true);

    pothole_enabled_ = declare_parameter<bool>("pothole.enabled", true);
    pothole_detection_topic_ = declare_parameter<std::string>(
      "pothole.detection_topic", "/traversability/pothole_detections");
    pothole_boundary_topic_ = declare_parameter<std::string>(
      "pothole.boundary_topic", "/traversability/pothole_boundary_points");
    pothole_class_id_ = declare_parameter<int>("pothole.class_id", 0);
    pothole_min_confidence_ = declare_parameter<double>("pothole.min_confidence", 0.35);
    pothole_max_time_difference_ = declare_parameter<double>(
      "pothole.max_time_difference", 0.05);
    pothole_frame_cache_size_ = std::max(
      2, static_cast<int>(declare_parameter<int>("pothole.frame_cache_size", 12)));
    pothole_min_drop_ = declare_parameter<double>("pothole.min_drop", 0.05);
    pothole_ground_search_radius_ = declare_parameter<double>(
      "pothole.ground_search_radius", 0.60);
    pothole_min_ground_neighbors_ = std::max(
      1, static_cast<int>(declare_parameter<int>("pothole.min_ground_neighbors", 3)));
    pothole_min_component_cells_ = std::max(
      1, static_cast<int>(declare_parameter<int>("pothole.min_component_cells", 3)));

    cloud_is_optical_ = declare_parameter<bool>("cloud_is_optical", true);
    imu_is_optical_ = declare_parameter<bool>("imu_is_optical", true);
    specific_force_is_up_ = declare_parameter<bool>("specific_force_is_up", true);
    require_imu_ = declare_parameter<bool>("require_imu", true);
    require_gyro_ = declare_parameter<bool>("imu.require_gyro", true);
    gravity_alpha_ = declare_parameter<double>("imu.gravity_alpha", 0.02);
    gravity_min_norm_ = declare_parameter<double>("imu.min_accel_norm", 7.0);
    gravity_max_norm_ = declare_parameter<double>("imu.max_accel_norm", 12.5);
    max_gyro_norm_ = declare_parameter<double>("imu.max_gyro_norm", 1.0);

    cloud_roll_ = radians(declare_parameter<double>("cloud_extrinsic.roll_deg", 0.0));
    cloud_pitch_ = radians(declare_parameter<double>("cloud_extrinsic.pitch_deg", 0.0));
    cloud_yaw_ = radians(declare_parameter<double>("cloud_extrinsic.yaw_deg", 0.0));
    cloud_tx_ = declare_parameter<double>("cloud_extrinsic.x", 0.0);
    cloud_ty_ = declare_parameter<double>("cloud_extrinsic.y", 0.0);
    cloud_tz_ = declare_parameter<double>("cloud_extrinsic.z", 0.60);
    imu_roll_ = radians(declare_parameter<double>("imu_extrinsic.roll_deg", 0.0));
    imu_pitch_ = radians(declare_parameter<double>("imu_extrinsic.pitch_deg", 0.0));
    imu_yaw_ = radians(declare_parameter<double>("imu_extrinsic.yaw_deg", 0.0));

    color_fx_ = declare_parameter<double>("color_camera.fx", 610.066650390625);
    color_fy_ = declare_parameter<double>("color_camera.fy", 610.0393676757812);
    color_cx_ = declare_parameter<double>("color_camera.cx", 639.6405639648438);
    color_cy_ = declare_parameter<double>("color_camera.cy", 365.00836181640625);
    overlay_radius_ = std::max(
      1, static_cast<int>(declare_parameter<int>("color_camera.overlay_radius", 3)));
    overlay_show_ground_ = declare_parameter<bool>("color_camera.show_ground", true);
    overlay_show_obstacle_ = declare_parameter<bool>("color_camera.show_obstacle", false);
    overlay_show_unknown_ = declare_parameter<bool>("color_camera.show_unknown", false);
    overlay_show_attitude_ = declare_parameter<bool>("color_camera.show_attitude", true);
    overlay_font_scale_ = declare_parameter<double>("color_camera.font_scale", 0.7);
    max_rgb_time_difference_ = declare_parameter<double>(
      "color_camera.max_time_difference", 0.0);
    color_roll_ = radians(declare_parameter<double>("color_extrinsic.roll_deg", 0.0));
    color_pitch_ = radians(declare_parameter<double>("color_extrinsic.pitch_deg", 0.0));
    color_yaw_ = radians(declare_parameter<double>("color_extrinsic.yaw_deg", 0.0));
    color_tx_ = declare_parameter<double>("color_extrinsic.x", 0.0);
    color_ty_ = declare_parameter<double>("color_extrinsic.y", 0.0);
    color_tz_ = declare_parameter<double>("color_extrinsic.z", 0.0);

    x_min_ = declare_parameter<double>("roi.x_min", 0.2);
    x_max_ = declare_parameter<double>("roi.x_max", 12.0);
    y_min_ = declare_parameter<double>("roi.y_min", -7.0);
    y_max_ = declare_parameter<double>("roi.y_max", 7.0);
    z_min_ = declare_parameter<double>("roi.z_min", -1.5);
    z_max_ = declare_parameter<double>("roi.z_max", 1.5);
    max_range_ = declare_parameter<double>("roi.max_range", 12.0);
    point_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int>("point_stride", 1)));

    resolution_ = declare_parameter<double>("grid.resolution", 0.15);
    min_points_ = declare_parameter<int>("grid.min_points", 2);
    max_variance_ = declare_parameter<double>("grid.max_variance", 0.015);
    max_roughness_ = declare_parameter<double>("grid.max_roughness", 0.18);
    max_slope_ = radians(declare_parameter<double>("grid.max_slope_deg", 18.0));
    max_step_ = declare_parameter<double>("grid.max_step", 0.15);
    ground_point_tolerance_ = declare_parameter<double>("grid.ground_point_tolerance", 0.12);

    seed_x_min_ = declare_parameter<double>("seed.x_min", 0.5);
    seed_x_max_ = declare_parameter<double>("seed.x_max", 3.0);
    seed_half_width_ = declare_parameter<double>("seed.half_width", 0.8);
    max_unknown_gap_ = std::max(
      0, static_cast<int>(declare_parameter<int>("max_unknown_gap_cells", 2)));
    boundary_min_column_cells_ = std::max(
      1, static_cast<int>(declare_parameter<int>("boundary.min_column_cells", 2)));
    fov_enabled_ = declare_parameter<bool>("boundary.fov_filter.enabled", true);
    fov_margin_ = declare_parameter<double>("boundary.fov_filter.horizontal_margin_px", 20.0);
    fov_width_ = declare_parameter<int>("boundary.fov_filter.width", 848);
    fov_fx_ = declare_parameter<double>("boundary.fov_filter.fx", 412.5237731933594);
    fov_cx_ = declare_parameter<double>("boundary.fov_filter.cx", 421.3500061035156);
    if (fov_width_ < 2 || fov_fx_ <= 0 || fov_margin_ < 0 ||
      2 * fov_margin_ >= fov_width_) {
      throw std::runtime_error("invalid boundary FOV intrinsics or margin");
    }

    if (resolution_ <= 0.0 || x_max_ <= x_min_ || y_max_ <= y_min_ || z_max_ <= z_min_) {
      throw std::runtime_error("invalid ROI or grid resolution");
    }
    if (pothole_min_confidence_ < 0.0 || pothole_min_confidence_ > 1.0 ||
      pothole_max_time_difference_ <= 0.0 || pothole_min_drop_ <= 0.0 ||
      pothole_ground_search_radius_ <= 0.0)
    {
      throw std::runtime_error("invalid pothole fusion parameters");
    }
    gravity_alpha_ = std::clamp(gravity_alpha_, 0.0, 1.0);
  }

  static double radians(double degrees)
  {
    return degrees * 3.14159265358979323846 / 180.0;
  }

  static Eigen::Matrix3d rpyRotation(double roll, double pitch, double yaw)
  {
    return (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  static Eigen::Vector3d opticalToBody(const Eigen::Vector3d & point)
  {
    return {point.z(), -point.x(), -point.y()};
  }

  static Eigen::Vector3d bodyToOptical(const Eigen::Vector3d & point)
  {
    return {-point.y(), -point.z(), point.x()};
  }

  void buildStaticRotations()
  {
    cloud_extrinsic_rotation_ = rpyRotation(cloud_roll_, cloud_pitch_, cloud_yaw_);
    imu_extrinsic_rotation_ = rpyRotation(imu_roll_, imu_pitch_, imu_yaw_);
    color_from_depth_rotation_ = rpyRotation(color_roll_, color_pitch_, color_yaw_);
    cloud_translation_ = {cloud_tx_, cloud_ty_, cloud_tz_};
    color_from_depth_translation_ = {color_tx_, color_ty_, color_tz_};
  }

  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr message)
  {
    if (message->data.empty() || message->width == 0U || message->height == 0U) {
      return;
    }

    int cv_type = 0;
    int conversion = -1;
    std::size_t input_channels = 0U;
    if (message->encoding == sensor_msgs::image_encodings::BGR8) {
      cv_type = CV_8UC3;
      input_channels = 3U;
    } else if (message->encoding == sensor_msgs::image_encodings::RGB8) {
      cv_type = CV_8UC3;
      conversion = cv::COLOR_RGB2BGR;
      input_channels = 3U;
    } else if (message->encoding == sensor_msgs::image_encodings::BGRA8) {
      cv_type = CV_8UC4;
      conversion = cv::COLOR_BGRA2BGR;
      input_channels = 4U;
    } else if (message->encoding == sensor_msgs::image_encodings::RGBA8) {
      cv_type = CV_8UC4;
      conversion = cv::COLOR_RGBA2BGR;
      input_channels = 4U;
    } else if (message->encoding == sensor_msgs::image_encodings::MONO8) {
      cv_type = CV_8UC1;
      conversion = cv::COLOR_GRAY2BGR;
      input_channels = 1U;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "RGB overlay does not support image encoding '%s'", message->encoding.c_str());
      return;
    }
    if (message->step < message->width * input_channels ||
      message->data.size() < static_cast<std::size_t>(message->step) * message->height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "RGB image has an invalid step or truncated data");
      return;
    }

    const cv::Mat source(
      static_cast<int>(message->height), static_cast<int>(message->width), cv_type,
      const_cast<std::uint8_t *>(message->data.data()), message->step);
    cv::Mat bgr;
    if (conversion >= 0) {
      cv::cvtColor(source, bgr, conversion);
    } else {
      bgr = source.clone();
    }
    if (!bgr.isContinuous()) {
      bgr = bgr.clone();
    }

    auto normalized = std::make_shared<sensor_msgs::msg::Image>();
    normalized->header = message->header;
    normalized->height = message->height;
    normalized->width = message->width;
    normalized->encoding = sensor_msgs::image_encodings::BGR8;
    normalized->is_bigendian = false;
    normalized->step = message->width * 3U;
    const std::size_t byte_count =
      static_cast<std::size_t>(normalized->step) * normalized->height;
    normalized->data.resize(byte_count);
    std::memcpy(normalized->data.data(), bgr.data, byte_count);
    latest_rgb_ = std::move(normalized);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    Eigen::Vector3d angular_velocity(
      message->angular_velocity.x,
      message->angular_velocity.y,
      message->angular_velocity.z);
    if (!angular_velocity.allFinite()) {
      return;
    }
    if (imu_is_optical_) {
      angular_velocity = opticalToBody(angular_velocity);
    }
    angular_velocity = imu_extrinsic_rotation_ * angular_velocity;
    latest_gyro_norm_ = angular_velocity.norm();
    gyro_ready_ = true;

    Eigen::Vector3d acceleration(
      message->linear_acceleration.x,
      message->linear_acceleration.y,
      message->linear_acceleration.z);
    if (!acceleration.allFinite()) {
      return;
    }
    if (imu_is_optical_) {
      acceleration = opticalToBody(acceleration);
    }
    acceleration = imu_extrinsic_rotation_ * acceleration;
    const double norm = acceleration.norm();
    if (norm < gravity_min_norm_ || norm > gravity_max_norm_ ||
      (require_gyro_ && (!gyro_ready_ || latest_gyro_norm_ > max_gyro_norm_)))
    {
      return;
    }
    Eigen::Vector3d up = acceleration / norm;
    if (!specific_force_is_up_) {
      up = -up;
    }
    if (!gravity_ready_) {
      filtered_up_ = up;
      gravity_ready_ = true;
    } else {
      filtered_up_ = ((1.0 - gravity_alpha_) * filtered_up_ + gravity_alpha_ * up).normalized();
    }
  }

  Eigen::Quaterniond gravityAlignment() const
  {
    if (!gravity_ready_) {
      return Eigen::Quaterniond::Identity();
    }
    // 最短旋转将重力反方向对齐到+Z，不引入绕Z轴的偏航校正。
    return Eigen::Quaterniond::FromTwoVectors(filtered_up_, Eigen::Vector3d::UnitZ());
  }

  bool insideRoi(const Eigen::Vector3d & point) const
  {
    if (point.x() < x_min_ || point.x() >= x_max_ ||
      point.y() < y_min_ || point.y() >= y_max_ ||
      point.z() < z_min_ || point.z() >= z_max_)
    {
      return false;
    }
    return max_range_ <= 0.0 || point.head<2>().squaredNorm() <= max_range_ * max_range_;
  }

  std::size_t index(int x, int y) const
  {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(size_x_) +
           static_cast<std::size_t>(x);
  }

  bool valid(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < size_x_ && y < size_y_;
  }

  void buildGrid(const pcl::PointCloud<pcl::PointXYZI> & points)
  {
    size_x_ = static_cast<int>(std::ceil((x_max_ - x_min_) / resolution_));
    size_y_ = static_cast<int>(std::ceil((y_max_ - y_min_) / resolution_));
    cells_.assign(static_cast<std::size_t>(size_x_ * size_y_), ElevationCell{});
    point_cells_.assign(points.size(), -1);
    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto & point = points[i];
      const int gx = static_cast<int>(std::floor((point.x - x_min_) / resolution_));
      const int gy = static_cast<int>(std::floor((point.y - y_min_) / resolution_));
      if (!valid(gx, gy)) {
        continue;
      }
      const auto cell_index = index(gx, gy);
      cells_[cell_index].add(point.z);
      point_cells_[i] = static_cast<int>(cell_index);
    }
  }

  void classifyCandidates()
  {
    static constexpr int neighbors[8][2] = {
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    for (int gy = 0; gy < size_y_; ++gy) {
      for (int gx = 0; gx < size_x_; ++gx) {
        auto & cell = cells_[index(gx, gy)];
        if (cell.count < static_cast<std::uint32_t>(min_points_)) {
          continue;
        }
        cell.ground_z = cell.min_z;
        cell.roughness = cell.max_z - cell.min_z;
        double max_slope = 0.0;
        double max_step = 0.0;
        int neighbor_count = 0;
        for (const auto & offset : neighbors) {
          const int nx = gx + offset[0];
          const int ny = gy + offset[1];
          if (!valid(nx, ny)) {
            continue;
          }
          const auto & neighbor = cells_[index(nx, ny)];
          if (neighbor.count < static_cast<std::uint32_t>(min_points_)) {
            continue;
          }
          const double dz = std::abs(cell.min_z - neighbor.min_z);
          const double distance = resolution_ * std::hypot(offset[0], offset[1]);
          max_step = std::max(max_step, dz);
          max_slope = std::max(max_slope, std::atan2(dz, distance));
          ++neighbor_count;
        }
        cell.slope = max_slope;
        cell.candidate = neighbor_count > 0 &&
          cell.roughness <= max_roughness_ &&
          cell.variance() <= max_variance_ &&
          cell.slope <= max_slope_ && max_step <= max_step_;
        cell.classification = cell.candidate ? CellClass::UNKNOWN : CellClass::OBSTACLE;
      }
    }
  }

  void bridgeShortUnknownGaps()
  {
    if (max_unknown_gap_ == 0) {
      return;
    }
    // 仅桥接同一行/列中两端均为候选地面的短暂无数据区。
    for (int gy = 0; gy < size_y_; ++gy) {
      int gx = 0;
      while (gx < size_x_) {
        while (gx < size_x_ && cells_[index(gx, gy)].count > 0) {
          ++gx;
        }
        const int begin = gx;
        while (gx < size_x_ && cells_[index(gx, gy)].count == 0) {
          ++gx;
        }
        const int gap = gx - begin;
        if (gap > 0 && gap <= max_unknown_gap_ && begin > 0 && gx < size_x_ &&
          cells_[index(begin - 1, gy)].candidate && cells_[index(gx, gy)].candidate &&
          std::abs(cells_[index(begin - 1, gy)].ground_z - cells_[index(gx, gy)].ground_z) <= max_step_)
        {
          for (int x = begin; x < gx; ++x) {
            cells_[index(x, gy)].bridge = true;
            cells_[index(x, gy)].ground_z = 0.5 *
              (cells_[index(begin - 1, gy)].ground_z + cells_[index(gx, gy)].ground_z);
          }
        }
      }
    }
  }

  void growReachableGround()
  {
    std::deque<std::pair<int, int>> queue;
    for (int gy = 0; gy < size_y_; ++gy) {
      const double y = y_min_ + (static_cast<double>(gy) + 0.5) * resolution_;
      if (std::abs(y) > seed_half_width_) {
        continue;
      }
      for (int gx = 0; gx < size_x_; ++gx) {
        const double x = x_min_ + (static_cast<double>(gx) + 0.5) * resolution_;
        auto & cell = cells_[index(gx, gy)];
        if (x >= seed_x_min_ && x <= seed_x_max_ && cell.candidate) {
          cell.classification = CellClass::GROUND;
          queue.emplace_back(gx, gy);
        }
      }
    }
    static constexpr int neighbors[8][2] = {
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    while (!queue.empty()) {
      const auto [gx, gy] = queue.front();
      queue.pop_front();
      const auto & current = cells_[index(gx, gy)];
      for (const auto & offset : neighbors) {
        const int nx = gx + offset[0];
        const int ny = gy + offset[1];
        if (!valid(nx, ny)) {
          continue;
        }
        auto & next = cells_[index(nx, ny)];
        if (next.classification == CellClass::GROUND || (!next.candidate && !next.bridge)) {
          continue;
        }
        const double distance = resolution_ * std::hypot(offset[0], offset[1]);
        const double dz = std::abs(next.ground_z - current.ground_z);
        if (dz <= max_step_ && std::atan2(dz, distance) <= max_slope_) {
          next.classification = CellClass::GROUND;
          queue.emplace_back(nx, ny);
        }
      }
    }
    for (auto & cell : cells_) {
      // 有足够观测但未与种子连通的候选区域按障碍处理；点数不足的稀疏格
      // 保持UNKNOWN，避免把信息不足误报为确定障碍。
      if (cell.count >= static_cast<std::uint32_t>(min_points_) &&
        cell.classification == CellClass::UNKNOWN)
      {
        cell.classification = CellClass::OBSTACLE;
      }
    }
  }

  void publishCloud(
    const pcl::PointCloud<pcl::PointXYZI> & cloud,
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher) const
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.stamp = stamp;
    message.header.frame_id = output_frame_;
    publisher->publish(message);
  }

  static double stampSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void publishRgbOverlay(
    const builtin_interfaces::msg::Time & cloud_stamp,
    const Eigen::Quaterniond & alignment)
  {
    if (!latest_rgb_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for RGB image on '%s'; overlay is not published yet", rgb_topic_.c_str());
      return;
    }
    const double time_difference = std::abs(
      stampSeconds(cloud_stamp) - stampSeconds(latest_rgb_->header.stamp));
    if (max_rgb_time_difference_ > 0.0 && time_difference > max_rgb_time_difference_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "RGB/depth time difference %.3fs exceeds %.3fs", time_difference,
        max_rgb_time_difference_);
      return;
    }
    const std::size_t expected_size =
      static_cast<std::size_t>(latest_rgb_->step) * latest_rgb_->height;
    if (latest_rgb_->data.size() < expected_size || latest_rgb_->step < latest_rgb_->width * 3U) {
      return;
    }

    const cv::Mat source(
      static_cast<int>(latest_rgb_->height), static_cast<int>(latest_rgb_->width), CV_8UC3,
      const_cast<std::uint8_t *>(latest_rgb_->data.data()), latest_rgb_->step);
    cv::Mat painted = source.clone();
    for (int gy = 0; gy < size_y_; ++gy) {
      for (int gx = 0; gx < size_x_; ++gx) {
        const auto & cell = cells_[index(gx, gy)];
        if (cell.count == 0 && !cell.bridge) {
          continue;
        }
        const Eigen::Vector3d aligned_point(
          x_min_ + (static_cast<double>(gx) + 0.5) * resolution_,
          y_min_ + (static_cast<double>(gy) + 0.5) * resolution_,
          cell.ground_z);
        const Eigen::Vector3d mounted_body = alignment.conjugate() * aligned_point;
        const Eigen::Vector3d nominal_body = cloud_extrinsic_rotation_.transpose() *
          (mounted_body - cloud_translation_);
        const Eigen::Vector3d depth_optical = bodyToOptical(nominal_body);
        const Eigen::Vector3d color_optical =
          color_from_depth_rotation_ * depth_optical + color_from_depth_translation_;
        if (color_optical.z() <= 0.05) {
          continue;
        }
        const int u = static_cast<int>(std::lround(
          color_fx_ * color_optical.x() / color_optical.z() + color_cx_));
        const int v = static_cast<int>(std::lround(
          color_fy_ * color_optical.y() / color_optical.z() + color_cy_));
        if (u < 0 || v < 0 || u >= painted.cols || v >= painted.rows) {
          continue;
        }
        cv::Scalar color;
        if (cell.classification == CellClass::GROUND) {
          if (!overlay_show_ground_) {
            continue;
          }
          color = cv::Scalar(0, 255, 0);
        } else if (cell.classification == CellClass::OBSTACLE) {
          if (!overlay_show_obstacle_) {
            continue;
          }
          color = cv::Scalar(0, 0, 255);
        } else {
          if (!overlay_show_unknown_) {
            continue;
          }
          color = cv::Scalar(0, 255, 255);
        }
        cv::circle(painted, cv::Point(u, v), overlay_radius_, color, cv::FILLED, cv::LINE_AA);
      }
    }
    cv::addWeighted(painted, 0.65, source, 0.35, 0.0, painted);

    if (overlay_show_attitude_) {
      const double roll = std::atan2(filtered_up_.y(), filtered_up_.z());
      const double pitch = std::atan2(
        -filtered_up_.x(),
        std::hypot(filtered_up_.y(), filtered_up_.z()));
      char roll_text[64];
      char pitch_text[64];
      std::snprintf(roll_text, sizeof(roll_text), "Roll: %+6.2f deg", roll * 180.0 / M_PI);
      std::snprintf(pitch_text, sizeof(pitch_text), "Pitch: %+6.2f deg", pitch * 180.0 / M_PI);
      const int thickness = 2;
      int baseline = 0;
      const cv::Size roll_size = cv::getTextSize(
        roll_text, cv::FONT_HERSHEY_SIMPLEX, overlay_font_scale_, thickness, &baseline);
      const cv::Size pitch_size = cv::getTextSize(
        pitch_text, cv::FONT_HERSHEY_SIMPLEX, overlay_font_scale_, thickness, &baseline);
      const int box_width = std::max(roll_size.width, pitch_size.width) + 24;
      const int line_height = std::max(roll_size.height, pitch_size.height) + 12;
      cv::rectangle(
        painted, cv::Point(8, 8), cv::Point(8 + box_width, 16 + 2 * line_height),
        cv::Scalar(0, 0, 0), cv::FILLED);
      cv::putText(
        painted, roll_text, cv::Point(18, 12 + line_height), cv::FONT_HERSHEY_SIMPLEX,
        overlay_font_scale_, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
      cv::putText(
        painted, pitch_text, cv::Point(18, 12 + 2 * line_height), cv::FONT_HERSHEY_SIMPLEX,
        overlay_font_scale_, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
    }

    sensor_msgs::msg::Image output = *latest_rgb_;
    output.header.stamp = cloud_stamp;
    output.data.resize(expected_size);
    if (painted.isContinuous() && painted.step == output.step) {
      std::memcpy(output.data.data(), painted.data, expected_size);
    } else {
      for (std::uint32_t row = 0; row < output.height; ++row) {
        std::memcpy(
          output.data.data() + static_cast<std::size_t>(row) * output.step,
          painted.ptr(static_cast<int>(row)), output.step);
      }
    }
    overlay_pub_->publish(output);
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  bool projectCellToColor(
    const GridSnapshot & snapshot, int gx, int gy, double & u, double & v) const
  {
    const auto cell_index = static_cast<std::size_t>(gy) *
      static_cast<std::size_t>(snapshot.size_x) + static_cast<std::size_t>(gx);
    const auto & cell = snapshot.cells[cell_index];
    if (cell.count == 0U || !std::isfinite(cell.ground_z)) {
      return false;
    }
    const Eigen::Vector3d aligned_point(
      x_min_ + (static_cast<double>(gx) + 0.5) * resolution_,
      y_min_ + (static_cast<double>(gy) + 0.5) * resolution_, cell.ground_z);
    const Eigen::Vector3d mounted_body = snapshot.alignment.conjugate() * aligned_point;
    const Eigen::Vector3d nominal_body = cloud_extrinsic_rotation_.transpose() *
      (mounted_body - cloud_translation_);
    const Eigen::Vector3d depth_optical = bodyToOptical(nominal_body);
    const Eigen::Vector3d color_optical =
      color_from_depth_rotation_ * depth_optical + color_from_depth_translation_;
    if (!color_optical.allFinite() || color_optical.z() <= 0.05) {
      return false;
    }
    u = color_fx_ * color_optical.x() / color_optical.z() + color_cx_;
    v = color_fy_ * color_optical.y() / color_optical.z() + color_cy_;
    return std::isfinite(u) && std::isfinite(v);
  }

  bool localGroundHeight(
    const GridSnapshot & snapshot, int gx, int gy, double & ground_z) const
  {
    const int radius = std::max(
      1, static_cast<int>(std::ceil(pothole_ground_search_radius_ / resolution_)));
    std::vector<Eigen::Vector3d> samples;
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius) {
          continue;
        }
        const int nx = gx + dx;
        const int ny = gy + dy;
        if (nx < 0 || ny < 0 || nx >= snapshot.size_x || ny >= snapshot.size_y) {
          continue;
        }
        const auto & neighbor = snapshot.cells[
          static_cast<std::size_t>(ny) * static_cast<std::size_t>(snapshot.size_x) +
          static_cast<std::size_t>(nx)];
        if (neighbor.classification == CellClass::GROUND &&
          neighbor.count >= static_cast<std::uint32_t>(min_points_) &&
          std::isfinite(neighbor.ground_z))
        {
          samples.emplace_back(dx * resolution_, dy * resolution_, neighbor.ground_z);
        }
      }
    }
    if (samples.size() < static_cast<std::size_t>(std::max(3, pothole_min_ground_neighbors_))) {
      return false;
    }
    Eigen::MatrixXd design(samples.size(), 3);
    Eigen::VectorXd heights(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      design.row(i) << samples[i].x(), samples[i].y(), 1.0;
      heights[i] = samples[i].z();
    }
    const auto qr = design.colPivHouseholderQr();
    if (qr.rank() < 3) {return false;}
    const Eigen::Vector3d plane = qr.solve(heights);
    if (!plane.allFinite() || std::hypot(plane.x(), plane.y()) > std::tan(radians(25.0)) ||
      (design * plane - heights).norm() / std::sqrt(samples.size()) > 0.03)
    {return false;}
    ground_z = plane.z();
    return true;
  }

  void publishPotholeBoundary(const msg::Detection2DArray & detections)
  {
    bool has_target = false;
    for (const auto & detection : detections.detections) {
      has_target = has_target ||
        (detection.class_id == pothole_class_id_ &&
        detection.confidence >= pothole_min_confidence_);
    }
    if (!has_target) {
      pcl::PointCloud<pcl::PointXYZI> empty;
      publishCloud(empty, detections.header.stamp, pothole_boundary_pub_);
      last_pothole_detection_stamp_ns_ = stampNanoseconds(detections.header.stamp);
      return;
    }

    const GridSnapshot * best_snapshot = nullptr;
    double best_difference = std::numeric_limits<double>::infinity();
    for (const auto & snapshot : pothole_frame_cache_) {
      const double difference = std::abs(
        stampSeconds(snapshot.stamp) - stampSeconds(detections.header.stamp));
      if (difference < best_difference) {
        best_difference = difference;
        best_snapshot = &snapshot;
      }
    }
    if (best_snapshot == nullptr || best_difference > pothole_max_time_difference_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no depth grid matches pothole detections: nearest difference=%.3fs limit=%.3fs",
        best_difference, pothole_max_time_difference_);
      pcl::PointCloud<pcl::PointXYZI> empty;
      publishCloud(empty, detections.header.stamp, pothole_boundary_pub_);
      return;
    }

    cv::Mat risk_mask(best_snapshot->size_y, best_snapshot->size_x, CV_8UC1, cv::Scalar(0));
    for (const auto & detection : detections.detections) {
      if (detection.class_id != pothole_class_id_ ||
        detection.confidence < pothole_min_confidence_)
      {
        continue;
      }
      for (int gy = 0; gy < best_snapshot->size_y; ++gy) {
        for (int gx = 0; gx < best_snapshot->size_x; ++gx) {
          const auto & cell = best_snapshot->cells[
            static_cast<std::size_t>(gy) * static_cast<std::size_t>(best_snapshot->size_x) +
            static_cast<std::size_t>(gx)];
          if (cell.count < static_cast<std::uint32_t>(min_points_) ||
            cell.classification == CellClass::GROUND)
          {
            continue;
          }
          double u = 0.0;
          double v = 0.0;
          if (!projectCellToColor(*best_snapshot, gx, gy, u, v) ||
            u < detection.left || u > detection.right ||
            v < detection.top || v > detection.bottom)
          {
            continue;
          }
          double local_ground_z = 0.0;
          if (localGroundHeight(*best_snapshot, gx, gy, local_ground_z) &&
            local_ground_z - cell.min_z >= pothole_min_drop_)
          {
            risk_mask.at<std::uint8_t>(gy, gx) = 255U;
          }
        }
      }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count = cv::connectedComponentsWithStats(
      risk_mask, labels, stats, centroids, 8, CV_32S);
    cv::Mat cleaned(risk_mask.size(), CV_8UC1, cv::Scalar(0));
    for (int label = 1; label < component_count; ++label) {
      if (stats.at<int>(label, cv::CC_STAT_AREA) < pothole_min_component_cells_) {
        continue;
      }
      cleaned.setTo(255U, labels == label);
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cleaned, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    pcl::PointCloud<pcl::PointXYZI> boundary;
    int contour_id = 1;
    for (const auto & contour : contours) {
      if (contour.size() < 3U) {
        continue;
      }
      for (const auto & pixel : contour) {
        const int gx = pixel.x;
        const int gy = pixel.y;
        double ground_z = best_snapshot->cells[
          static_cast<std::size_t>(gy) * static_cast<std::size_t>(best_snapshot->size_x) +
          static_cast<std::size_t>(gx)].ground_z;
        (void)localGroundHeight(*best_snapshot, gx, gy, ground_z);
        pcl::PointXYZI point;
        point.x = static_cast<float>(x_min_ + (static_cast<double>(gx) + 0.5) * resolution_);
        point.y = static_cast<float>(y_min_ + (static_cast<double>(gy) + 0.5) * resolution_);
        point.z = static_cast<float>(ground_z);
        point.intensity = static_cast<float>(contour_id);
        boundary.push_back(point);
      }
      ++contour_id;
    }
    publishCloud(boundary, best_snapshot->stamp, pothole_boundary_pub_);
    last_pothole_detection_stamp_ns_ = stampNanoseconds(detections.header.stamp);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "pothole fusion: detections=%zu components=%d boundary_points=%zu sync=%.3fs",
      detections.detections.size(), std::max(0, contour_id - 1), boundary.size(), best_difference);
  }

  void potholeDetectionCallback(const msg::Detection2DArray::SharedPtr detections)
  {
    latest_pothole_detections_ = detections;
    publishPotholeBoundary(*detections);
  }

  void cachePotholeFrame(
    const builtin_interfaces::msg::Time & stamp, const Eigen::Quaterniond & alignment)
  {
    if (!pothole_enabled_) {
      return;
    }
    if (!pothole_frame_cache_.empty() &&
      stampNanoseconds(stamp) < stampNanoseconds(pothole_frame_cache_.back().stamp))
    {
      pothole_frame_cache_.clear();
      latest_pothole_detections_.reset();
      last_pothole_detection_stamp_ns_ = -1;
    }
    GridSnapshot snapshot;
    snapshot.stamp = stamp;
    snapshot.alignment = alignment;
    snapshot.size_x = size_x_;
    snapshot.size_y = size_y_;
    snapshot.cells = cells_;
    pothole_frame_cache_.push_back(std::move(snapshot));
    while (pothole_frame_cache_.size() > static_cast<std::size_t>(pothole_frame_cache_size_)) {
      pothole_frame_cache_.pop_front();
    }
    if (latest_pothole_detections_ &&
      stampNanoseconds(latest_pothole_detections_->header.stamp) !=
      last_pothole_detection_stamp_ns_)
    {
      publishPotholeBoundary(*latest_pothole_detections_);
    }
  }

  void publishResults(
    const pcl::PointCloud<pcl::PointXYZI> & points,
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Quaterniond & alignment)
  {
    pcl::PointCloud<pcl::PointXYZI> ground;
    pcl::PointCloud<pcl::PointXYZI> obstacles;
    pcl::PointCloud<pcl::PointXYZI> unknown;
    ground.reserve(points.size());
    if (publish_obstacle_) {
      obstacles.reserve(points.size());
    }
    if (publish_unknown_) {
      unknown.reserve(points.size());
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
      const int cell_index = point_cells_[i];
      if (cell_index < 0) {
        if (publish_unknown_) {
          unknown.push_back(points[i]);
        }
        continue;
      }
      const auto & cell = cells_[static_cast<std::size_t>(cell_index)];
      if (cell.classification == CellClass::GROUND &&
        points[i].z <= cell.ground_z + ground_point_tolerance_)
      {
        ground.push_back(points[i]);
      } else if (cell.classification == CellClass::OBSTACLE ||
        points[i].z > cell.ground_z + ground_point_tolerance_)
      {
        if (publish_obstacle_) {
          obstacles.push_back(points[i]);
        }
      } else if (publish_unknown_) {
        unknown.push_back(points[i]);
      }
    }

    pcl::PointCloud<pcl::PointXYZI> grid;
    pcl::PointCloud<pcl::PointXYZI> ground_grid;
    if (publish_grid_ || publish_ground_grid_) {
      for (int gy = 0; gy < size_y_; ++gy) {
        for (int gx = 0; gx < size_x_; ++gx) {
          const auto & cell = cells_[index(gx, gy)];
          if (cell.count == 0 && !cell.bridge) {
            continue;
          }
          pcl::PointXYZI point;
          point.x = static_cast<float>(x_min_ + (gx + 0.5) * resolution_);
          point.y = static_cast<float>(y_min_ + (gy + 0.5) * resolution_);
          point.z = static_cast<float>(cell.ground_z);
          if (publish_ground_grid_ && cell.classification == CellClass::GROUND) {
            // 规则栅格点云包含真实观测及短距离桥接的可通行单元。
            point.intensity = 1.0F;
            ground_grid.push_back(point);
          }
          if (publish_grid_) {
            point.intensity = static_cast<float>(
              cell.classification == CellClass::GROUND ? 1 :
              (cell.classification == CellClass::OBSTACLE ? 2 : 0));
            grid.push_back(point);
          }
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZI> boundary;
    pcl::PointCloud<pcl::PointXYZI> classified_boundary;
    if (publish_boundary_) {
      struct ColumnLimits
      {
        int gx;
        int right_gy;
        int left_gy;
      };
      std::vector<ColumnLimits> limits;
      limits.reserve(static_cast<std::size_t>(size_x_));
      for (int gx = 0; gx < size_x_; ++gx) {
        int right_gy = size_y_;
        int left_gy = -1;
        int ground_cells = 0;
        for (int gy = 0; gy < size_y_; ++gy) {
          if (cells_[index(gx, gy)].classification != CellClass::GROUND) {
            continue;
          }
          right_gy = std::min(right_gy, gy);
          left_gy = std::max(left_gy, gy);
          ++ground_cells;
        }
        if (ground_cells >= boundary_min_column_cells_) {
          limits.push_back({gx, right_gy, left_gy});
        }
      }

      const auto append_boundary_point = [this, &boundary, &classified_boundary, &alignment](int gx, int gy, float label) {
          const auto & cell = cells_[index(gx, gy)];
          pcl::PointXYZI point;
          point.x = static_cast<float>(x_min_ + (gx + 0.5) * resolution_);
          point.y = static_cast<float>(y_min_ + (gy + 0.5) * resolution_);
          point.z = static_cast<float>(cell.ground_z);
          point.intensity = label;
          boundary.push_back(point);
          bool touches_fov = false;
          // Project all cell corners: the published boundary is a quantized cell center.
          for (double dx : {-0.5, 0.5}) {
            for (double dy : {-0.5, 0.5}) {
              const Eigen::Vector3d p(point.x + dx * resolution_,
                point.y + dy * resolution_, point.z);
              const Eigen::Vector3d nominal = cloud_extrinsic_rotation_.transpose() *
                (alignment.inverse() * p - cloud_translation_);
              const Eigen::Vector3d optical = bodyToOptical(nominal);
              if (optical.z() <= 0.0) {
                touches_fov = true;
              } else {
                const double u = fov_fx_ * optical.x() / optical.z() + fov_cx_;
                touches_fov = touches_fov || u <= fov_margin_ ||
                  u >= fov_width_ - 1 - fov_margin_;
              }
            }
          }
          bool physical_evidence = false;
          const int outside_y = gy + (label == 1.0F ? 1 : -1);
          if (valid(gx, outside_y)) {
            const auto & outside = cells_[index(gx, outside_y)];
            // Disconnected Ground alone is not evidence of a physical obstacle.
            physical_evidence = outside.count >= static_cast<std::uint32_t>(min_points_) &&
              (outside.roughness > max_roughness_ ||
              std::abs(outside.min_z - cell.ground_z) > max_step_);
          }
          if (fov_enabled_ && label != 3.0F && touches_fov && !physical_evidence) {
            point.intensity = -label;  // Explicit veto, not a missing observation.
          }
          classified_boundary.push_back(point);
        };

      // Ordered U shape: left side near->far, front left->right, right side far->near.
      // There is deliberately no rear edge near the camera.
      for (const auto & column : limits) {
        append_boundary_point(column.gx, column.left_gy, 1.0F);
      }
      if (!limits.empty()) {
        const auto & front = limits.back();
        for (int gy = front.left_gy - 1; gy > front.right_gy; --gy) {
          if (cells_[index(front.gx, gy)].classification == CellClass::GROUND) {
            append_boundary_point(front.gx, gy, 3.0F);
          }
        }
      }
      for (auto it = limits.rbegin(); it != limits.rend(); ++it) {
        append_boundary_point(it->gx, it->right_gy, 2.0F);
      }
    }
    publishCloud(ground, stamp, ground_pub_);
    if (publish_ground_grid_) {
      publishCloud(ground_grid, stamp, ground_grid_pub_);
    }
    if (publish_boundary_) {
      publishCloud(boundary, stamp, boundary_pub_);
      publishCloud(classified_boundary, stamp, boundary_classified_pub_);
    }
    if (publish_obstacle_) {
      publishCloud(obstacles, stamp, obstacle_pub_);
    }
    if (publish_unknown_) {
      publishCloud(unknown, stamp, unknown_pub_);
    }
    if (publish_grid_) {
      publishCloud(grid, stamp, grid_pub_);
    }
    if (publish_overlay_) {
      publishRgbOverlay(stamp, alignment);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "classified points: ground=%zu ground_grid=%zu boundary=%zu obstacle=%zu unknown=%zu grid=%zu",
      ground.size(), ground_grid.size(), boundary.size(), obstacles.size(), unknown.size(), grid.size());
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (require_imu_ && !gravity_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for a valid IMU gravity estimate");
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*message, input);
    const Eigen::Quaterniond alignment = gravityAlignment();
    pcl::PointCloud<pcl::PointXYZI> points;
    points.reserve(input.size() / static_cast<std::size_t>(point_stride_) + 1);
    for (std::size_t i = 0; i < input.size(); i += static_cast<std::size_t>(point_stride_)) {
      const auto & source = input[i];
      if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
        continue;
      }
      Eigen::Vector3d point(source.x, source.y, source.z);
      if (cloud_is_optical_) {
        point = opticalToBody(point);
      }
      point = alignment * (cloud_extrinsic_rotation_ * point + cloud_translation_);
      if (!insideRoi(point)) {
        continue;
      }
      pcl::PointXYZI output;
      output.x = static_cast<float>(point.x());
      output.y = static_cast<float>(point.y());
      output.z = static_cast<float>(point.z());
      output.intensity = source.intensity;
      points.push_back(output);
    }
    buildGrid(points);
    classifyCandidates();
    bridgeShortUnknownGaps();
    growReachableGround();
    cachePotholeFrame(message->header.stamp, alignment);
    publishResults(points, message->header.stamp, alignment);
  }

  std::string input_cloud_topic_, imu_topic_;
  std::string ground_topic_, ground_grid_topic_, boundary_topic_, obstacle_topic_, unknown_topic_;
  std::string grid_topic_, rgb_topic_, overlay_topic_, output_frame_;
  std::string pothole_detection_topic_, pothole_boundary_topic_;
  bool cloud_is_optical_ = true, imu_is_optical_ = true, specific_force_is_up_ = true;
  bool require_imu_ = true, require_gyro_ = true, gravity_ready_ = false, gyro_ready_ = false;
  bool publish_ground_grid_ = true, publish_boundary_ = true;
  bool publish_obstacle_ = true, publish_unknown_ = true;
  bool publish_grid_ = true, publish_overlay_ = true;
  bool pothole_enabled_ = true;
  int pothole_class_id_ = 0;
  int pothole_frame_cache_size_ = 12;
  int pothole_min_ground_neighbors_ = 3;
  int pothole_min_component_cells_ = 3;
  double pothole_min_confidence_ = 0.35;
  double pothole_max_time_difference_ = 0.05;
  double pothole_min_drop_ = 0.05;
  double pothole_ground_search_radius_ = 0.60;
  double gravity_alpha_ = 0.02, gravity_min_norm_ = 7.0, gravity_max_norm_ = 12.5;
  double max_gyro_norm_ = 1.0;
  double latest_gyro_norm_ = 0.0;
  double cloud_roll_ = 0.0, cloud_pitch_ = 0.0, cloud_yaw_ = 0.0;
  double imu_roll_ = 0.0, imu_pitch_ = 0.0, imu_yaw_ = 0.0;
  double cloud_tx_ = 0.0, cloud_ty_ = 0.0, cloud_tz_ = 0.6;
  double color_fx_ = 610.0667, color_fy_ = 610.0394;
  double color_cx_ = 639.6406, color_cy_ = 365.0084;
  double color_roll_ = 0.0, color_pitch_ = 0.0, color_yaw_ = 0.0;
  double color_tx_ = 0.0, color_ty_ = 0.0, color_tz_ = 0.0;
  double max_rgb_time_difference_ = 0.0;
  int overlay_radius_ = 3;
  bool overlay_show_ground_ = true;
  bool overlay_show_obstacle_ = false;
  bool overlay_show_unknown_ = false;
  bool overlay_show_attitude_ = true;
  double overlay_font_scale_ = 0.7;
  double x_min_ = 0.2, x_max_ = 12.0, y_min_ = -7.0, y_max_ = 7.0;
  double z_min_ = -1.5, z_max_ = 1.5, max_range_ = 12.0;
  int point_stride_ = 1, size_x_ = 0, size_y_ = 0, min_points_ = 2;
  double resolution_ = 0.15, max_variance_ = 0.015, max_roughness_ = 0.18;
  double max_slope_ = 0.314, max_step_ = 0.15, ground_point_tolerance_ = 0.12;
  double seed_x_min_ = 0.5, seed_x_max_ = 3.0, seed_half_width_ = 0.8;
  int max_unknown_gap_ = 2;
  int boundary_min_column_cells_ = 2;
  bool fov_enabled_ = true;
  double fov_margin_ = 20.0, fov_fx_ = 412.5237731933594, fov_cx_ = 421.3500061035156;
  int fov_width_ = 848;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_classified_pub_;
  Eigen::Vector3d filtered_up_{0.0, 0.0, 1.0};
  Eigen::Matrix3d cloud_extrinsic_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d imu_extrinsic_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d color_from_depth_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d cloud_translation_{0.0, 0.0, 0.6};
  Eigen::Vector3d color_from_depth_translation_{0.0, 0.0, 0.0};
  std::vector<ElevationCell> cells_;
  std::vector<int> point_cells_;
  std::deque<GridSnapshot> pothole_frame_cache_;
  std::int64_t last_pothole_detection_stamp_ns_ = -1;
  msg::Detection2DArray::SharedPtr latest_pothole_detections_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<msg::Detection2DArray>::SharedPtr pothole_detection_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pothole_boundary_pub_;
  sensor_msgs::msg::Image::SharedPtr latest_rgb_;
};

}  // namespace traversability_obstacle_cloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<traversability_obstacle_cloud::TraversabilityObstacleCloudNode>());
  rclcpp::shutdown();
  return 0;
}
