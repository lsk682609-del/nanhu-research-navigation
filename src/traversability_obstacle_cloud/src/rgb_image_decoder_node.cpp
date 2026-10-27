#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace traversability_obstacle_cloud
{

class RgbImageDecoderNode : public rclcpp::Node
{
public:
  RgbImageDecoderNode()
  : Node("rgb_image_decoder_node")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_rgb_topic", "/camera/color/image_raw/compressed");
    output_topic_ = declare_parameter<std::string>(
      "output_rgb_topic", "/camera/color/image_raw");
    output_frame_ = declare_parameter<std::string>("output_frame", "");

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    output_qos.best_effort().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&RgbImageDecoderNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "RGB decoder: input=%s output=%s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr message)
  {
    if (message->data.empty()) {
      return;
    }
    const cv::Mat encoded(
      1, static_cast<int>(message->data.size()), CV_8UC1,
      const_cast<std::uint8_t *>(message->data.data()));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty() || image.type() != CV_8UC3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to decode RGB compressed image (format='%s')", message->format.c_str());
      return;
    }
    if (!image.isContinuous()) {
      image = image.clone();
    }

    sensor_msgs::msg::Image output;
    output.header = message->header;
    if (!output_frame_.empty()) {
      output.header.frame_id = output_frame_;
    }
    output.height = static_cast<std::uint32_t>(image.rows);
    output.width = static_cast<std::uint32_t>(image.cols);
    output.encoding = sensor_msgs::image_encodings::BGR8;
    output.is_bigendian = false;
    output.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * image.elemSize());
    const std::size_t byte_count = static_cast<std::size_t>(output.step) * output.height;
    output.data.resize(byte_count);
    std::memcpy(output.data.data(), image.data, byte_count);
    publisher_->publish(output);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "decoded RGB images: %ux%u encoding=%s",
      output.width, output.height, output.encoding.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

}  // namespace traversability_obstacle_cloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<traversability_obstacle_cloud::RgbImageDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
