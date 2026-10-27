#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace traversability_obstacle_cloud
{

class DepthCloudProjectorNode : public rclcpp::Node
{
public:
  DepthCloudProjectorNode()
  : Node("depth_cloud_projector_node")
  {
    input_mode_ = declare_parameter<std::string>("input_mode", "raw");
    raw_input_topic_ = declare_parameter<std::string>(
      "input_depth_topic", "/camera/depth/image_raw");
    compressed_input_topic_ = declare_parameter<std::string>(
      "input_compressed_depth_topic", "/camera/depth/image_raw/compressedDepth");
    output_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/depth/points");
    output_frame_ = declare_parameter<std::string>(
      "output_frame", "camera_depth_optical_frame");

    width_ = declare_parameter<int>("camera.width", 848);
    height_ = declare_parameter<int>("camera.height", 480);
    fx_ = declare_parameter<double>("camera.fx", 412.5237731933594);
    fy_ = declare_parameter<double>("camera.fy", 412.5237731933594);
    cx_ = declare_parameter<double>("camera.cx", 421.3500061035156);
    cy_ = declare_parameter<double>("camera.cy", 241.98751831054688);
    depth_scale_ = declare_parameter<double>("camera.depth_scale", 0.001);

    min_depth_ = declare_parameter<double>("filter.min_depth", 0.2);
    max_depth_ = declare_parameter<double>("filter.max_depth", 12.0);
    pixel_step_ = declare_parameter<int>("filter.pixel_step", 1);

    process_once_ = declare_parameter<bool>("process_once", false);

    if (width_ <= 0 || height_ <= 0 || fx_ <= 0.0 || fy_ <= 0.0 || depth_scale_ <= 0.0) {
      throw std::runtime_error("invalid camera intrinsics or depth scale");
    }
    if (min_depth_ < 0.0 || max_depth_ <= min_depth_) {
      throw std::runtime_error("filter.max_depth must be greater than filter.min_depth");
    }
    pixel_step_ = std::max(1, pixel_step_);
    // 单帧模式只发布一次。使用 transient-local 缓存最后一帧，使稍后启动的
    // RViz/ros2 topic echo 仍能收到点云，而不必严格抢在第一帧之前订阅。
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    output_qos.reliable().transient_local();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);
    if (input_mode_ == "raw") {
      raw_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        raw_input_topic_, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&DepthCloudProjectorNode::depthCallback, this, std::placeholders::_1));
    } else if (input_mode_ == "compressed_depth") {
      compressed_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        compressed_input_topic_, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(
          &DepthCloudProjectorNode::compressedDepthCallback, this, std::placeholders::_1));
    } else {
      throw std::runtime_error("input_mode must be 'raw' or 'compressed_depth'");
    }

    RCLCPP_INFO(
      get_logger(),
      "depth projector: mode=%s input=%s output=%s frame=%s intrinsics=%dx%d "
      "fx=%.3f fy=%.3f cx=%.3f cy=%.3f depth_scale=%.6f once=%s",
      input_mode_.c_str(),
      (input_mode_ == "raw" ? raw_input_topic_ : compressed_input_topic_).c_str(),
      output_topic_.c_str(), output_frame_.c_str(), width_, height_,
      fx_, fy_, cx_, cy_, depth_scale_, process_once_ ? "true" : "false");
  }

private:
  static std::uint16_t byteSwap16(std::uint16_t value)
  {
    return static_cast<std::uint16_t>((value >> 8U) | (value << 8U));
  }

  static std::uint32_t byteSwap32(std::uint32_t value)
  {
    return ((value & 0x000000ffU) << 24U) |
           ((value & 0x0000ff00U) << 8U) |
           ((value & 0x00ff0000U) >> 8U) |
           ((value & 0xff000000U) >> 24U);
  }

  double depthAt(const sensor_msgs::msg::Image & message, int u, int v, bool is_float) const
  {
    const std::size_t offset = static_cast<std::size_t>(v) * message.step +
      static_cast<std::size_t>(u) * (is_float ? sizeof(float) : sizeof(std::uint16_t));
    if (is_float) {
      std::uint32_t bits = 0U;
      std::memcpy(&bits, message.data.data() + offset, sizeof(bits));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      if (message.is_bigendian) {
        bits = byteSwap32(bits);
      }
#else
      if (!message.is_bigendian) {
        bits = byteSwap32(bits);
      }
#endif
      float value = 0.0F;
      std::memcpy(&value, &bits, sizeof(value));
      return static_cast<double>(value);
    }

    std::uint16_t value = 0U;
    std::memcpy(&value, message.data.data() + offset, sizeof(value));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if (message.is_bigendian) {
      value = byteSwap16(value);
    }
#else
    if (!message.is_bigendian) {
      value = byteSwap16(value);
    }
#endif
    return static_cast<double>(value) * depth_scale_;
  }

  pcl::PointXYZI toOpticalPoint(int u, int v, double depth) const
  {
    const double optical_x = (static_cast<double>(u) - cx_) * depth / fx_;
    const double optical_y = (static_cast<double>(v) - cy_) * depth / fy_;
    pcl::PointXYZI point;
    point.x = static_cast<float>(optical_x);
    point.y = static_cast<float>(optical_y);
    point.z = static_cast<float>(depth);
    point.intensity = static_cast<float>(depth);
    return point;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr message)
  {
    if (processed_ && process_once_) {
      return;
    }
    const bool is_uint16 = message->encoding == "16UC1" || message->encoding == "mono16";
    const bool is_float = message->encoding == "32FC1";
    if (!is_uint16 && !is_float) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "unsupported raw depth encoding '%s'; expected 16UC1, mono16, or 32FC1",
        message->encoding.c_str());
      return;
    }
    if (static_cast<int>(message->width) != width_ || static_cast<int>(message->height) != height_) {
      RCLCPP_ERROR(
        get_logger(), "depth image size %dx%d does not match calibration %dx%d",
        message->width, message->height, width_, height_);
      return;
    }
    const std::size_t bytes_per_pixel = is_float ? sizeof(float) : sizeof(std::uint16_t);
    if (message->step < message->width * bytes_per_pixel ||
      message->data.size() < static_cast<std::size_t>(message->step) * message->height)
    {
      RCLCPP_ERROR(get_logger(), "raw depth image has an invalid step or truncated data");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(static_cast<std::size_t>(width_ / pixel_step_ + 1) *
      static_cast<std::size_t>(height_ / pixel_step_ + 1));
    for (int v = 0; v < height_; v += pixel_step_) {
      for (int u = 0; u < width_; u += pixel_step_) {
        const double depth = depthAt(*message, u, v, is_float);
        if (!std::isfinite(depth) || depth <= 0.0 || depth < min_depth_ || depth > max_depth_) {
          continue;
        }
        cloud.push_back(toOpticalPoint(u, v, depth));
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.stamp = message->header.stamp;
    output.header.frame_id = output_frame_;
    publisher_->publish(output);
    processed_ = true;
  }

  void compressedDepthCallback(const sensor_msgs::msg::CompressedImage::SharedPtr message)
  {
    if (processed_ && process_once_) {
      return;
    }
    const std::string source_encoding = message->format.substr(0, message->format.find(';'));
    if (source_encoding.find("16UC1") == std::string::npos) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "unsupported compressedDepth format '%s'; expected 16UC1", message->format.c_str());
      return;
    }

    static constexpr std::uint8_t png_signature[] = {
      0x89U, 0x50U, 0x4eU, 0x47U, 0x0dU, 0x0aU, 0x1aU, 0x0aU};
    const auto png_begin = std::search(
      message->data.begin(), message->data.end(),
      std::begin(png_signature), std::end(png_signature));
    if (png_begin == message->data.end()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "compressedDepth payload has no PNG stream");
      return;
    }
    const std::vector<std::uint8_t> png(png_begin, message->data.end());
    const cv::Mat depth = cv::imdecode(png, cv::IMREAD_UNCHANGED);
    if (depth.empty() || depth.type() != CV_16UC1) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "could not decode compressedDepth as a 16UC1 PNG");
      return;
    }

    auto decoded = std::make_shared<sensor_msgs::msg::Image>();
    decoded->header = message->header;
    decoded->height = static_cast<std::uint32_t>(depth.rows);
    decoded->width = static_cast<std::uint32_t>(depth.cols);
    decoded->encoding = "16UC1";
    decoded->is_bigendian = false;
    decoded->step = decoded->width * sizeof(std::uint16_t);
    decoded->data.resize(static_cast<std::size_t>(decoded->step) * decoded->height);
    if (depth.isContinuous()) {
      std::memcpy(decoded->data.data(), depth.data, decoded->data.size());
    } else {
      for (int row = 0; row < depth.rows; ++row) {
        std::memcpy(
          decoded->data.data() + static_cast<std::size_t>(row) * decoded->step,
          depth.ptr(row), decoded->step);
      }
    }
    depthCallback(std::move(decoded));
  }

  std::string input_mode_;
  std::string raw_input_topic_;
  std::string compressed_input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  int width_ = 0;
  int height_ = 0;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  double depth_scale_ = 0.001;
  double min_depth_ = 0.2;
  double max_depth_ = 12.0;
  int pixel_step_ = 1;
  bool process_once_ = false;
  bool processed_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
};

}  // namespace traversability_obstacle_cloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<traversability_obstacle_cloud::DepthCloudProjectorNode>());
  rclcpp::shutdown();
  return 0;
}
