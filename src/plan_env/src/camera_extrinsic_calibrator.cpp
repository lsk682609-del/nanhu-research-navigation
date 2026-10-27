#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CameraExtrinsicCalibrator : public rclcpp::Node
{
public:
  CameraExtrinsicCalibrator() : Node("camera_extrinsic_calibrator")
  {
    enabled_ = declare_parameter("calibration.enabled", false);
    use_imu_ = declare_parameter("calibration.use_imu", true);
    use_ground_ = declare_parameter("calibration.use_ground_plane_refinement", false);
    output_path_ = declare_parameter("calibration.output_yaml", std::string("/tmp/gemini330l_extrinsic.yaml"));
    translation_x_ = declare_parameter("calibration.translation_x", 0.0);
    translation_y_ = declare_parameter("calibration.translation_y", 0.0);
    camera_height_ = declare_parameter("calibration.camera_height", 0.60);
    roll_ = declare_parameter("calibration.roll", 0.0);
    pitch_ = declare_parameter("calibration.pitch", 0.0);
    yaw_ = declare_parameter("calibration.yaw", 0.0);
    imu_duration_ = declare_parameter("calibration.imu_sample_duration", 5.0);
    max_angular_velocity_ = declare_parameter("calibration.max_angular_velocity", 0.05);
    gravity_tolerance_ = declare_parameter("calibration.gravity_tolerance", 0.5);
    min_imu_samples_ = declare_parameter("calibration.min_imu_samples", 100);
    depth_scale_ = declare_parameter("calibration.depth_scaling_factor", 1000.0);
    depth_min_ = declare_parameter("calibration.depth_min", 0.30);
    depth_max_ = declare_parameter("calibration.depth_max", 5.0);
    ground_distance_threshold_ = declare_parameter("calibration.ground_max_distance", 0.05);
    ground_min_points_ = declare_parameter("calibration.ground_min_points", 500);
    ground_max_tilt_deg_ = declare_parameter("calibration.ground_max_tilt_deg", 20.0);
    base_frame_ = declare_parameter("calibration.base_frame", std::string("base_link"));

    // Nominal optical convention: x_base=z_optical, y_base=-x_optical, z_base=-y_optical.
    nominal_rotation_ << 0.0, 0.0, 1.0,
                        -1.0, 0.0, 0.0,
                         0.0, -1.0, 0.0;
    const auto nominal = declare_parameter<std::vector<double>>(
        "calibration.nominal_optical_rotation",
        {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0});
    if (nominal.size() != 9)
      throw std::invalid_argument("calibration.nominal_optical_rotation must contain 9 values");
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        nominal_rotation_(row, col) = nominal[row * 3 + col];

    rotation_ = configuredRotation();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        std::bind(&CameraExtrinsicCalibrator::imuCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth", rclcpp::SensorDataQoS(),
        std::bind(&CameraExtrinsicCalibrator::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        std::bind(&CameraExtrinsicCalibrator::cameraInfoCallback, this, std::placeholders::_1));
    raw_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("calibration/depth_cloud_raw", 1);
    base_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("calibration/depth_cloud_base", 1);
    ground_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("calibration/ground_points", 1);

    if (!enabled_)
      RCLCPP_WARN(get_logger(), "calibration.enabled=false; no samples will be processed");
    else if (!use_imu_ && !use_ground_)
    {
      RCLCPP_WARN(get_logger(), "both calibration sources are disabled; writing configured extrinsic only");
      writeYaml("configured values (no calibration source enabled)");
    }
    else
      RCLCPP_INFO(get_logger(), "keep the robot stationary on level ground during calibration");
  }

private:
  Eigen::Matrix3d configuredRotation() const
  {
    return (Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())).toRotationMatrix() *
           nominal_rotation_;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0)
      return;
    fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
    has_camera_info_ = true;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (!enabled_ || !use_imu_ || imu_ready_ || wrote_yaml_)
      return;
    const Eigen::Vector3d angular(msg->angular_velocity.x, msg->angular_velocity.y,
                                  msg->angular_velocity.z);
    const Eigen::Vector3d acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                       msg->linear_acceleration.z);
    if (!angular.allFinite() || !acceleration.allFinite() ||
        angular.norm() > max_angular_velocity_ ||
        std::abs(acceleration.norm() - 9.80665) > gravity_tolerance_)
      return;

    if (imu_samples_ == 0)
      imu_started_ = now();
    acceleration_sum_ += acceleration;
    ++imu_samples_;
    if ((now() - imu_started_).seconds() < imu_duration_ ||
        imu_samples_ < static_cast<size_t>(min_imu_samples_))
      return;

    const Eigen::Vector3d acceleration_camera = acceleration_sum_.normalized();
    const Eigen::Vector3d acceleration_nominal = nominal_rotation_ * acceleration_camera;
    const Eigen::Quaterniond leveling = Eigen::Quaterniond::FromTwoVectors(
        acceleration_nominal, Eigen::Vector3d::UnitZ());
    rotation_ = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                leveling.toRotationMatrix() * nominal_rotation_;
    imu_ready_ = true;
    RCLCPP_INFO(get_logger(), "IMU leveling complete with %zu stationary samples", imu_samples_);
    if (!use_ground_)
      writeYaml("IMU gravity leveling");
  }

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!enabled_ || wrote_yaml_ || !use_ground_ || (use_imu_ && !imu_ready_))
      return;
    if (!has_camera_info_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for aligned-depth CameraInfo");
      return;
    }
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
        msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
      return;

    const auto image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr preliminary(new pcl::PointCloud<pcl::PointXYZ>);
    raw->reserve((image.rows / 2) * (image.cols / 4));
    preliminary->reserve((image.rows / 2) * (image.cols / 4));
    for (int v = image.rows / 2; v < image.rows; v += 2)
      for (int u = 0; u < image.cols; u += 2)
      {
        double depth = 0.0;
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
          depth = image.at<uint16_t>(v, u) / depth_scale_;
        else
          depth = image.at<float>(v, u);
        if (!std::isfinite(depth) || depth < depth_min_ || depth > depth_max_)
          continue;
        const Eigen::Vector3d camera((u - cx_) * depth / fx_, (v - cy_) * depth / fy_, depth);
        const Eigen::Vector3d base = rotation_ * camera;
        raw->emplace_back(camera.x(), camera.y(), camera.z());
        preliminary->emplace_back(base.x(), base.y(), base.z());
      }
    if (static_cast<int>(preliminary->size()) < ground_min_points_)
      return;

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(ground_distance_threshold_);
    segmentation.setMaxIterations(200);
    segmentation.setInputCloud(preliminary);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(inliers, coefficients);
    if (static_cast<int>(inliers.indices.size()) < ground_min_points_ || coefficients.values.size() < 4)
    {
      RCLCPP_WARN(get_logger(), "ground plane fit rejected: only %zu inliers", inliers.indices.size());
      return;
    }

    Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    double plane_d = coefficients.values[3];
    const double normal_norm = normal.norm();
    if (normal_norm < 1e-6)
      return;
    normal /= normal_norm;
    plane_d /= normal_norm;
    if (normal.z() < 0.0)
    {
      normal = -normal;
      plane_d = -plane_d;
    }
    const double tilt_deg = std::acos(std::clamp(normal.z(), -1.0, 1.0)) * 180.0 / M_PI;
    if (tilt_deg > ground_max_tilt_deg_)
    {
      RCLCPP_WARN(get_logger(), "ground plane tilt %.2f deg exceeds configured limit", tilt_deg);
      return;
    }

    const Eigen::Matrix3d correction = Eigen::Quaterniond::FromTwoVectors(
        normal, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    rotation_ = correction * rotation_;
    camera_height_ = plane_d;

    pcl::PointCloud<pcl::PointXYZ> base_cloud;
    pcl::PointCloud<pcl::PointXYZ> ground_cloud;
    base_cloud.reserve(raw->size());
    ground_cloud.reserve(inliers.indices.size());
    const Eigen::Vector3d translation(translation_x_, translation_y_, camera_height_);
    for (size_t index = 0; index < raw->size(); ++index)
    {
      const auto& point = raw->points[index];
      const Eigen::Vector3d transformed = rotation_ * Eigen::Vector3d(point.x, point.y, point.z) + translation;
      base_cloud.emplace_back(transformed.x(), transformed.y(), transformed.z());
    }
    for (const int index : inliers.indices)
      ground_cloud.push_back(base_cloud.points[index]);
    publishCloud(*raw, msg->header.frame_id, msg->header.stamp, raw_cloud_pub_);
    publishCloud(base_cloud, base_frame_, msg->header.stamp, base_cloud_pub_);
    publishCloud(ground_cloud, base_frame_, msg->header.stamp, ground_cloud_pub_);
    writeYaml("IMU leveling and depth ground-plane refinement");
  }

  void publishCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const std::string& frame,
                    const builtin_interfaces::msg::Time& stamp,
                    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
  {
    cloud.width = cloud.size(); cloud.height = 1; cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = frame;
    message.header.stamp = stamp;
    publisher->publish(message);
  }

  void writeYaml(const std::string& method)
  {
    const std::filesystem::path path(output_path_);
    if (!path.parent_path().empty())
      std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output)
      throw std::runtime_error("cannot open calibration output: " + output_path_);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation_;
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(
        translation_x_, translation_y_, camera_height_);
    const Eigen::Matrix3d mounting_adjustment = rotation_ * nominal_rotation_.transpose();
    const Eigen::Vector3d euler = mounting_adjustment.eulerAngles(2, 1, 0);
    output << std::fixed << std::setprecision(9);
    output << "# Gemini 330L extrinsic generated by camera_extrinsic_calibrator\n";
    output << "# Method: " << method << "\n";
    output << "grid_map_node:\n  ros__parameters:\n";
    output << "    calibration.translation_x: " << translation_x_ << "\n";
    output << "    calibration.translation_y: " << translation_y_ << "\n";
    output << "    calibration.camera_height: " << camera_height_ << "\n";
    output << "    calibration.roll: " << euler.z() << "\n";
    output << "    calibration.pitch: " << euler.y() << "\n";
    output << "    calibration.yaw: " << yaw_ << "\n";
    output << "    grid_map.need_extrinsic: true\n";
    output << "    grid_map.depth_extrinsic: [";
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
      {
        if (row != 0 || col != 0) output << ", ";
        output << transform(row, col);
      }
    output << "]\n";
    output.close();
    wrote_yaml_ = true;
    RCLCPP_INFO(get_logger(), "calibration YAML written to %s", output_path_.c_str());
  }

  bool enabled_{false}, use_imu_{true}, use_ground_{false};
  bool has_camera_info_{false}, imu_ready_{false}, wrote_yaml_{false};
  std::string output_path_, base_frame_;
  double translation_x_{0.0}, translation_y_{0.0}, camera_height_{0.60};
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  double imu_duration_{5.0}, max_angular_velocity_{0.05}, gravity_tolerance_{0.5};
  double depth_scale_{1000.0}, depth_min_{0.3}, depth_max_{5.0};
  double ground_distance_threshold_{0.05}, ground_max_tilt_deg_{20.0};
  int min_imu_samples_{100}, ground_min_points_{500};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  size_t imu_samples_{0};
  rclcpp::Time imu_started_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d acceleration_sum_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d nominal_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr base_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<CameraExtrinsicCalibrator>());
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("camera_extrinsic_calibrator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
