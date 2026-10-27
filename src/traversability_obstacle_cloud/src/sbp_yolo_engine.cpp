#include "traversability_obstacle_cloud/sbp_yolo_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace traversability_obstacle_cloud
{
namespace
{

constexpr int kBranchCount = 4;
constexpr int kOutputsPerBranch = 3;
constexpr int kDflLength = 16;

struct LetterboxInfo
{
  float scale;
  int pad_x;
  int pad_y;
  int original_width;
  int original_height;
};

struct Candidate
{
  float left;
  float top;
  float right;
  float bottom;
  float score;
  int class_id;
};

bool read_binary_file(const std::string & path, std::vector<uint8_t> & data)
{
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    return false;
  }
  const auto size = stream.tellg();
  if (size <= 0) {
    return false;
  }
  data.resize(static_cast<size_t>(size));
  stream.seekg(0, std::ios::beg);
  return static_cast<bool>(stream.read(
      reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(size)));
}

bool parse_core_mask(const std::string & value, rknn_core_mask & mask)
{
  if (value == "core0") {
    mask = RKNN_NPU_CORE_0;
  } else if (value == "core1") {
    mask = RKNN_NPU_CORE_1;
  } else if (value == "core2") {
    mask = RKNN_NPU_CORE_2;
  } else if (value == "core0_1_2" || value == "all") {
    mask = RKNN_NPU_CORE_0_1_2;
  } else {
    return false;
  }
  return true;
}

cv::Mat letterbox_rgb(
  const cv::Mat & bgr, int target_width, int target_height, LetterboxInfo & info)
{
  info.original_width = bgr.cols;
  info.original_height = bgr.rows;
  info.scale = std::min(
    static_cast<float>(target_width) / static_cast<float>(bgr.cols),
    static_cast<float>(target_height) / static_cast<float>(bgr.rows));

  const int resized_width = std::max(1, static_cast<int>(std::round(bgr.cols * info.scale)));
  const int resized_height = std::max(1, static_cast<int>(std::round(bgr.rows * info.scale)));
  info.pad_x = (target_width - resized_width) / 2;
  info.pad_y = (target_height - resized_height) / 2;

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);
  cv::Mat canvas(target_height, target_width, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(info.pad_x, info.pad_y, resized_width, resized_height)));

  cv::Mat rgb;
  cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);
  return rgb.isContinuous() ? rgb : rgb.clone();
}

float iou(const Candidate & a, const Candidate & b)
{
  const float left = std::max(a.left, b.left);
  const float top = std::max(a.top, b.top);
  const float right = std::min(a.right, b.right);
  const float bottom = std::min(a.bottom, b.bottom);
  const float intersection = std::max(0.0F, right - left) * std::max(0.0F, bottom - top);
  const float area_a = std::max(0.0F, a.right - a.left) * std::max(0.0F, a.bottom - a.top);
  const float area_b = std::max(0.0F, b.right - b.left) * std::max(0.0F, b.bottom - b.top);
  const float union_area = area_a + area_b - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

float decode_dfl(const float * box, int side, int grid_index, int grid_size)
{
  float maximum = -std::numeric_limits<float>::infinity();
  for (int bin = 0; bin < kDflLength; ++bin) {
    maximum = std::max(maximum, box[(side * kDflLength + bin) * grid_size + grid_index]);
  }

  float weighted_sum = 0.0F;
  float exponential_sum = 0.0F;
  for (int bin = 0; bin < kDflLength; ++bin) {
    const float value = std::exp(
      box[(side * kDflLength + bin) * grid_size + grid_index] - maximum);
    exponential_sum += value;
    weighted_sum += value * static_cast<float>(bin);
  }
  return weighted_sum / exponential_sum;
}

std::vector<Detection> decode_outputs(
  const std::vector<rknn_output> & outputs,
  const std::vector<rknn_tensor_attr> & attrs,
  int model_height,
  const LetterboxInfo & letterbox,
  float confidence_threshold,
  float nms_threshold)
{
  std::vector<Candidate> candidates;

  for (int branch = 0; branch < kBranchCount; ++branch) {
    const int box_index = branch * kOutputsPerBranch;
    const int score_index = box_index + 1;
    const int sum_index = box_index + 2;
    const auto & box_attr = attrs[box_index];
    const auto & score_attr = attrs[score_index];
    if (box_attr.fmt != RKNN_TENSOR_NCHW || score_attr.fmt != RKNN_TENSOR_NCHW) {
      continue;
    }

    const int grid_height = box_attr.dims[2];
    const int grid_width = box_attr.dims[3];
    const int grid_size = grid_height * grid_width;
    const int class_count = score_attr.dims[1];
    const int stride = model_height / grid_height;
    const auto * box = static_cast<const float *>(outputs[box_index].buf);
    const auto * scores = static_cast<const float *>(outputs[score_index].buf);
    const auto * score_sum = static_cast<const float *>(outputs[sum_index].buf);

    for (int y = 0; y < grid_height; ++y) {
      for (int x = 0; x < grid_width; ++x) {
        const int grid_index = y * grid_width + x;
        if (score_sum != nullptr && score_sum[grid_index] < confidence_threshold) {
          continue;
        }

        int class_id = 0;
        float best_score = scores[grid_index];
        for (int cls = 1; cls < class_count; ++cls) {
          const float score = scores[cls * grid_size + grid_index];
          if (score > best_score) {
            best_score = score;
            class_id = cls;
          }
        }
        if (best_score < confidence_threshold) {
          continue;
        }

        const float left_distance = decode_dfl(box, 0, grid_index, grid_size);
        const float top_distance = decode_dfl(box, 1, grid_index, grid_size);
        const float right_distance = decode_dfl(box, 2, grid_index, grid_size);
        const float bottom_distance = decode_dfl(box, 3, grid_index, grid_size);
        Candidate candidate;
        candidate.left = (static_cast<float>(x) + 0.5F - left_distance) * stride;
        candidate.top = (static_cast<float>(y) + 0.5F - top_distance) * stride;
        candidate.right = (static_cast<float>(x) + 0.5F + right_distance) * stride;
        candidate.bottom = (static_cast<float>(y) + 0.5F + bottom_distance) * stride;
        candidate.score = best_score;
        candidate.class_id = class_id;
        candidates.push_back(candidate);
      }
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.score > b.score;});
  std::vector<Candidate> kept;
  for (const auto & candidate : candidates) {
    bool suppressed = false;
    for (const auto & previous : kept) {
      if (candidate.class_id == previous.class_id && iou(candidate, previous) > nms_threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(candidate);
    }
  }

  std::vector<Detection> detections;
  detections.reserve(kept.size());
  for (const auto & candidate : kept) {
    Detection detection;
    detection.left = std::clamp(
      (candidate.left - letterbox.pad_x) / letterbox.scale,
      0.0F, static_cast<float>(letterbox.original_width - 1));
    detection.top = std::clamp(
      (candidate.top - letterbox.pad_y) / letterbox.scale,
      0.0F, static_cast<float>(letterbox.original_height - 1));
    detection.right = std::clamp(
      (candidate.right - letterbox.pad_x) / letterbox.scale,
      0.0F, static_cast<float>(letterbox.original_width - 1));
    detection.bottom = std::clamp(
      (candidate.bottom - letterbox.pad_y) / letterbox.scale,
      0.0F, static_cast<float>(letterbox.original_height - 1));
    detection.confidence = candidate.score;
    detection.class_id = candidate.class_id;
    detections.push_back(detection);
  }
  return detections;
}

}  // namespace

SbpYoloEngine::~SbpYoloEngine()
{
  release();
}

void SbpYoloEngine::release()
{
  if (context_ != 0) {
    rknn_destroy(context_);
    context_ = 0;
  }
  input_attrs_.clear();
  output_attrs_.clear();
}

bool SbpYoloEngine::initialize(
  const std::string & model_path,
  const std::string & npu_core_mask,
  std::string & error)
{
  release();
  std::vector<uint8_t> model;
  if (!read_binary_file(model_path, model)) {
    error = "unable to read RKNN model: " + model_path;
    return false;
  }

  int ret = rknn_init(&context_, model.data(), static_cast<uint32_t>(model.size()), 0, nullptr);
  if (ret != RKNN_SUCC) {
    error = "rknn_init failed: " + std::to_string(ret);
    context_ = 0;
    return false;
  }

  if (npu_core_mask != "auto") {
    rknn_core_mask mask{};
    if (!parse_core_mask(npu_core_mask, mask)) {
      error = "invalid npu_core_mask: " + npu_core_mask;
      release();
      return false;
    }
    ret = rknn_set_core_mask(context_, mask);
    if (ret != RKNN_SUCC) {
      error = "rknn_set_core_mask failed: " + std::to_string(ret);
      release();
      return false;
    }
  }

  ret = rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
  if (ret != RKNN_SUCC || io_num_.n_input != 1 || io_num_.n_output != 12) {
    error = "unexpected RKNN input/output count";
    release();
    return false;
  }

  input_attrs_.resize(io_num_.n_input);
  for (uint32_t i = 0; i < io_num_.n_input; ++i) {
    std::memset(&input_attrs_[i], 0, sizeof(rknn_tensor_attr));
    input_attrs_[i].index = i;
    ret = rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attrs_[i], sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      error = "RKNN_QUERY_INPUT_ATTR failed: " + std::to_string(ret);
      release();
      return false;
    }
  }

  output_attrs_.resize(io_num_.n_output);
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    std::memset(&output_attrs_[i], 0, sizeof(rknn_tensor_attr));
    output_attrs_[i].index = i;
    ret = rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      error = "RKNN_QUERY_OUTPUT_ATTR failed: " + std::to_string(ret);
      release();
      return false;
    }
  }

  const auto & input = input_attrs_.front();
  if (input.fmt == RKNN_TENSOR_NHWC) {
    model_height_ = input.dims[1];
    model_width_ = input.dims[2];
    model_channels_ = input.dims[3];
  } else {
    model_channels_ = input.dims[1];
    model_height_ = input.dims[2];
    model_width_ = input.dims[3];
  }
  if (model_width_ <= 0 || model_height_ <= 0 || model_channels_ != 3) {
    error = "unsupported RKNN input shape";
    release();
    return false;
  }
  return true;
}

bool SbpYoloEngine::infer(
  const cv::Mat & bgr_image,
  float confidence_threshold,
  float nms_threshold,
  std::vector<Detection> & detections,
  double & inference_ms,
  std::string & error)
{
  detections.clear();
  if (context_ == 0 || bgr_image.empty()) {
    error = "engine is not initialized or input image is empty";
    return false;
  }

  LetterboxInfo letterbox{};
  cv::Mat rgb = letterbox_rgb(bgr_image, model_width_, model_height_, letterbox);
  rknn_input input{};
  input.index = 0;
  input.type = RKNN_TENSOR_UINT8;
  input.fmt = RKNN_TENSOR_NHWC;
  input.size = static_cast<uint32_t>(rgb.total() * rgb.elemSize());
  input.buf = rgb.data;

  int ret = rknn_inputs_set(context_, 1, &input);
  if (ret != RKNN_SUCC) {
    error = "rknn_inputs_set failed: " + std::to_string(ret);
    return false;
  }

  const auto start = std::chrono::steady_clock::now();
  ret = rknn_run(context_, nullptr);
  const auto finish = std::chrono::steady_clock::now();
  inference_ms = std::chrono::duration<double, std::milli>(finish - start).count();
  if (ret != RKNN_SUCC) {
    error = "rknn_run failed: " + std::to_string(ret);
    return false;
  }

  std::vector<rknn_output> outputs(io_num_.n_output);
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    std::memset(&outputs[i], 0, sizeof(rknn_output));
    outputs[i].index = i;
    outputs[i].want_float = 1;
  }
  ret = rknn_outputs_get(context_, io_num_.n_output, outputs.data(), nullptr);
  if (ret != RKNN_SUCC) {
    error = "rknn_outputs_get failed: " + std::to_string(ret);
    return false;
  }

  detections = decode_outputs(
    outputs, output_attrs_, model_height_, letterbox,
    confidence_threshold, nms_threshold);
  rknn_outputs_release(context_, io_num_.n_output, outputs.data());
  return true;
}

}  // namespace traversability_obstacle_cloud
