#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rknn_api.h>

namespace traversability_obstacle_cloud
{

struct Detection
{
  float left;
  float top;
  float right;
  float bottom;
  float confidence;
  int class_id;
};

class SbpYoloEngine
{
public:
  SbpYoloEngine() = default;
  ~SbpYoloEngine();

  SbpYoloEngine(const SbpYoloEngine &) = delete;
  SbpYoloEngine & operator=(const SbpYoloEngine &) = delete;

  bool initialize(
    const std::string & model_path,
    const std::string & npu_core_mask,
    std::string & error);

  bool infer(
    const cv::Mat & bgr_image,
    float confidence_threshold,
    float nms_threshold,
    std::vector<Detection> & detections,
    double & inference_ms,
    std::string & error);

  int model_width() const {return model_width_;}
  int model_height() const {return model_height_;}

private:
  void release();

  rknn_context context_{0};
  rknn_input_output_num io_num_{};
  std::vector<rknn_tensor_attr> input_attrs_;
  std::vector<rknn_tensor_attr> output_attrs_;
  int model_width_{0};
  int model_height_{0};
  int model_channels_{0};
};

}  // namespace traversability_obstacle_cloud
