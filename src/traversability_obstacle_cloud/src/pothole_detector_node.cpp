#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "traversability_obstacle_cloud/msg/detection2_d.hpp"
#include "traversability_obstacle_cloud/msg/detection2_d_array.hpp"
#include "traversability_obstacle_cloud/sbp_yolo_engine.hpp"

namespace traversability_obstacle_cloud
{

class PotholeDetectorNode : public rclcpp::Node
{
public:
  PotholeDetectorNode()
  : Node("pothole_detector_node")
  {
    const std::string package_share =
      ament_index_cpp::get_package_share_directory("traversability_obstacle_cloud");
    model_path_ = resolvePath(
      package_share,
      declare_parameter<std::string>("model_path", "models/sbp-yolo-rkopt-fp16.rknn"));
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/color/image_raw");
    detection_topic_ = declare_parameter<std::string>(
      "detection_topic", "/traversability/pothole_detections");
    confidence_threshold_ = static_cast<float>(
      declare_parameter<double>("confidence_threshold", 0.25));
    nms_threshold_ = static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    process_every_n_ = std::max<int64_t>(
      1, declare_parameter<int64_t>("process_every_n", 1));
    log_every_n_ = std::max<int64_t>(1, declare_parameter<int64_t>("log_every_n", 10));
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{"Pothole", "SpeedBump"});
    save_enabled_ = declare_parameter<bool>("save.enabled", false);
    save_directory_ = declare_parameter<std::string>(
      "save.directory", "/home/cat/nanhu-research-navigation/pics");
    save_metadata_ = declare_parameter<bool>("save.write_metadata", true);
    const auto qos_depth = std::max<int64_t>(1, declare_parameter<int64_t>("qos_depth", 2));
    const std::string qos_reliability =
      declare_parameter<std::string>("qos_reliability", "best_effort");
    const std::string npu_core_mask =
      declare_parameter<std::string>("npu_core_mask", "core0_1_2");

    if (confidence_threshold_ < 0.0F || confidence_threshold_ > 1.0F ||
      nms_threshold_ < 0.0F || nms_threshold_ > 1.0F)
    {
      throw std::invalid_argument("confidence_threshold and nms_threshold must be in [0, 1]");
    }
    if (save_enabled_) {
      std::error_code error;
      std::filesystem::create_directories(save_directory_, error);
      if (error) {
        throw std::runtime_error(
                "failed to create save.directory '" + save_directory_ + "': " + error.message());
      }
    }

    std::string error;
    if (!engine_.initialize(model_path_, npu_core_mask, error)) {
      throw std::runtime_error(error);
    }

    rclcpp::QoS image_qos(rclcpp::KeepLast(static_cast<size_t>(qos_depth)));
    if (qos_reliability == "best_effort") {
      image_qos.best_effort();
    } else if (qos_reliability == "reliable") {
      image_qos.reliable();
    } else {
      throw std::invalid_argument("qos_reliability must be 'best_effort' or 'reliable'");
    }
    detection_pub_ = create_publisher<msg::Detection2DArray>(
      detection_topic_, rclcpp::SensorDataQoS().keep_last(2));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos,
      std::bind(&PotholeDetectorNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "pothole detector: model=%s input=%dx%d image=%s detections=%s save=%s directory=%s",
      model_path_.c_str(), engine_.model_width(), engine_.model_height(), image_topic_.c_str(),
      detection_topic_.c_str(), save_enabled_ ? "on" : "off", save_directory_.c_str());
  }

private:
  static std::string resolvePath(const std::string & package_share, const std::string & value)
  {
    const std::filesystem::path path(value);
    return path.is_absolute() ? path.string() : (std::filesystem::path(package_share) / path).string();
  }

  std::string className(int class_id) const
  {
    if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
      return class_names_[static_cast<std::size_t>(class_id)];
    }
    return "class_" + std::to_string(class_id);
  }

  std::string resultStem(
    const sensor_msgs::msg::Image & image, std::uint64_t inference_index) const
  {
    std::ostringstream stem;
    stem << save_directory_ << "/detection_" << image.header.stamp.sec << "_"
         << std::setw(9) << std::setfill('0') << image.header.stamp.nanosec
         << "_" << inference_index;
    return stem.str();
  }

  void saveResult(
    const sensor_msgs::msg::Image & image, const cv::Mat & source,
    const std::vector<Detection> & detections)
  {
    if (!save_enabled_ || detections.empty()) {
      return;
    }
    cv::Mat annotated = source.clone();
    for (const auto & detection : detections) {
      const int left = std::clamp(static_cast<int>(std::lround(detection.left)), 0, annotated.cols - 1);
      const int top = std::clamp(static_cast<int>(std::lround(detection.top)), 0, annotated.rows - 1);
      const int right = std::clamp(static_cast<int>(std::lround(detection.right)), 0, annotated.cols - 1);
      const int bottom = std::clamp(static_cast<int>(std::lround(detection.bottom)), 0, annotated.rows - 1);
      if (right <= left || bottom <= top) {
        continue;
      }
      const cv::Scalar color = detection.class_id == 0 ?
        cv::Scalar(0, 0, 255) : cv::Scalar(0, 165, 255);
      cv::rectangle(
        annotated, cv::Point(left, top), cv::Point(right, bottom), color, 2, cv::LINE_AA);
      std::ostringstream label;
      label << className(detection.class_id) << " " << std::fixed << std::setprecision(2)
            << detection.confidence;
      cv::putText(
        annotated, label.str(), cv::Point(left, std::max(20, top - 6)),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
    }

    const std::string stem = resultStem(image, inference_count_);
    try {
      if (!cv::imwrite(stem + ".jpg", annotated)) {
        RCLCPP_ERROR(get_logger(), "failed to save annotated detection image: %s.jpg", stem.c_str());
        return;
      }
      if (save_metadata_) {
        std::ofstream metadata(stem + ".txt");
        metadata << "stamp_sec " << image.header.stamp.sec << "\n"
                 << "stamp_nanosec " << image.header.stamp.nanosec << "\n"
                 << "image_width " << image.width << "\n"
                 << "image_height " << image.height << "\n";
        metadata << std::fixed << std::setprecision(6);
        for (const auto & detection : detections) {
          metadata << className(detection.class_id) << " " << detection.class_id << " "
                   << detection.confidence << " " << detection.left << " " << detection.top
                   << " " << detection.right << " " << detection.bottom << "\n";
        }
        if (!metadata) {
          RCLCPP_ERROR(get_logger(), "failed to save detection metadata: %s.txt", stem.c_str());
        }
      }
      RCLCPP_INFO(get_logger(), "saved detection result: %s.jpg", stem.c_str());
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR(get_logger(), "failed to save detection image: %s", exception.what());
    }
  }

  void publishDetections(
    const sensor_msgs::msg::Image & image, const std::vector<Detection> & detections)
  {
    msg::Detection2DArray output;
    output.header = image.header;
    output.image_width = image.width;
    output.image_height = image.height;
    output.detections.reserve(detections.size());
    for (const auto & detection : detections) {
      msg::Detection2D item;
      item.class_id = detection.class_id;
      item.confidence = detection.confidence;
      item.left = detection.left;
      item.top = detection.top;
      item.right = detection.right;
      item.bottom = detection.bottom;
      output.detections.push_back(item);
    }
    detection_pub_->publish(output);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image)
  {
    ++received_count_;
    if (received_count_ % static_cast<std::uint64_t>(process_every_n_) != 0U) {
      return;
    }
    cv_bridge::CvImagePtr converted;
    try {
      converted = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_ERROR(get_logger(), "image conversion failed: %s", exception.what());
      return;
    }

    ++inference_count_;
    std::vector<Detection> detections;
    double inference_ms = 0.0;
    std::string error;
    if (!engine_.infer(
        converted->image, confidence_threshold_, nms_threshold_, detections, inference_ms, error))
    {
      RCLCPP_ERROR(get_logger(), "inference failed: %s", error.c_str());
      return;
    }

    // The bundled model has two classes; retain its class ordering, but expose
    // only Pothole (class 0) to ROS, logs and the saved annotations.
    detections.erase(std::remove_if(detections.begin(), detections.end(),
      [](const Detection & detection) {return detection.class_id != 0;}), detections.end());
    publishDetections(*image, detections);
    saveResult(*image, converted->image, detections);
    if (inference_count_ % static_cast<std::uint64_t>(log_every_n_) == 0U) {
      RCLCPP_INFO(
        get_logger(), "inference #%llu: npu=%.3f ms detections=%zu",
        static_cast<unsigned long long>(inference_count_), inference_ms, detections.size());
    }
  }

  SbpYoloEngine engine_;
  std::string model_path_;
  std::string image_topic_;
  std::string detection_topic_;
  std::string save_directory_;
  std::vector<std::string> class_names_;
  float confidence_threshold_{0.25F};
  float nms_threshold_{0.45F};
  int64_t process_every_n_{1};
  int64_t log_every_n_{10};
  bool save_enabled_{false};
  bool save_metadata_{true};
  std::uint64_t received_count_{0};
  std::uint64_t inference_count_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<msg::Detection2DArray>::SharedPtr detection_pub_;
};

}  // namespace traversability_obstacle_cloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<traversability_obstacle_cloud::PotholeDetectorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pothole_detector_node"), "%s", exception.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
