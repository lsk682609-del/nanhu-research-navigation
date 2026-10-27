#include "camera_boundary_navigation/boundary_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>

namespace camera_boundary_navigation
{

BoundaryModel::BoundaryModel(BoundaryModelConfig config)
: config_(std::move(config))
{
}

BoundaryCurve BoundaryModel::solve(const std::vector<Point2d> & points) const
{
  BoundaryCurve curve;
  if (points.size() < 3U) {
    return curve;
  }

  Eigen::MatrixXd design(static_cast<Eigen::Index>(points.size()), 3);
  Eigen::VectorXd values(static_cast<Eigen::Index>(points.size()));
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double x = points[i].x;
    design(static_cast<Eigen::Index>(i), 0) = 1.0;
    design(static_cast<Eigen::Index>(i), 1) = x;
    design(static_cast<Eigen::Index>(i), 2) = x * x;
    values(static_cast<Eigen::Index>(i)) = points[i].y;
  }
  const Eigen::Vector3d coefficients = design.colPivHouseholderQr().solve(values);
  if (!coefficients.allFinite()) {
    return curve;
  }

  curve.a0 = coefficients[0];
  curve.a1 = coefficients[1];
  curve.a2 = coefficients[2];
  curve.x_min = points.front().x;
  curve.x_max = points.front().x;
  double squared_error = 0.0;
  for (const auto & point : points) {
    curve.x_min = std::min(curve.x_min, point.x);
    curve.x_max = std::max(curve.x_max, point.x);
    const double error = point.y - curve.y(point.x);
    squared_error += error * error;
  }
  curve.rms_residual = std::sqrt(squared_error / static_cast<double>(points.size()));
  curve.inlier_count = points.size();
  return curve;
}

BoundaryCurve BoundaryModel::fit(const std::vector<Point2d> & input)
{
  std::vector<Point2d> points;
  points.reserve(input.size());
  for (const auto & point : input) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
      point.x >= config_.fit_x_min && point.x <= config_.fit_x_max)
    {
      points.push_back(point);
    }
  }
  std::sort(points.begin(), points.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.x < rhs.x;
    });
  if (points.size() < static_cast<std::size_t>(config_.minimum_points)) {
    return BoundaryCurve{};
  }

  // 使用确定性的三点随机采样一致性算法生成初始模型。如果直接从最小二乘开始，
  // 单个极端障碍点可能把曲线拉偏，继而导致正确的道路边界点被误判为离群点。
  std::vector<std::size_t> samples;
  const std::size_t maximum_samples = 30U;
  if (points.size() <= maximum_samples) {
    for (std::size_t i = 0; i < points.size(); ++i) {
      samples.push_back(i);
    }
  } else {
    for (std::size_t i = 0; i < maximum_samples; ++i) {
      samples.push_back(i * (points.size() - 1U) / (maximum_samples - 1U));
    }
  }
  std::size_t best_count = 0U;
  double best_error = std::numeric_limits<double>::infinity();
  BoundaryCurve best_seed;
  for (std::size_t a = 0; a + 2U < samples.size(); ++a) {
    for (std::size_t b = a + 1U; b + 1U < samples.size(); ++b) {
      for (std::size_t c = b + 1U; c < samples.size(); ++c) {
        const Point2d & first = points[samples[a]];
        const Point2d & second = points[samples[b]];
        const Point2d & third = points[samples[c]];
        if (second.x - first.x < 0.05 || third.x - second.x < 0.05) {
          continue;
        }
        const BoundaryCurve seed = solve({first, second, third});
        std::size_t count = 0U;
        double squared_error = 0.0;
        for (const auto & point : points) {
          const double error = point.y - seed.y(point.x);
          if (std::abs(error) <= config_.outlier_threshold) {
            ++count;
            squared_error += error * error;
          }
        }
        if (count > best_count || (count == best_count && squared_error < best_error)) {
          best_count = count;
          best_error = squared_error;
          best_seed = seed;
        }
      }
    }
  }
  if (best_count < static_cast<std::size_t>(config_.minimum_points)) {
    return BoundaryCurve{};
  }
  std::vector<Point2d> ransac_inliers;
  ransac_inliers.reserve(best_count);
  for (const auto & point : points) {
    if (std::abs(point.y - best_seed.y(point.x)) <= config_.outlier_threshold) {
      ransac_inliers.push_back(point);
    }
  }
  points = std::move(ransac_inliers);

  for (int iteration = 0; iteration < config_.robust_iterations; ++iteration) {
    const BoundaryCurve estimate = solve(points);
    if (estimate.inlier_count == 0U) {
      return BoundaryCurve{};
    }
    std::vector<Point2d> inliers;
    inliers.reserve(points.size());
    for (const auto & point : points) {
      if (std::abs(point.y - estimate.y(point.x)) <= config_.outlier_threshold) {
        inliers.push_back(point);
      }
    }
    if (inliers.size() == points.size()) {
      break;
    }
    if (inliers.size() < static_cast<std::size_t>(config_.minimum_points)) {
      return BoundaryCurve{};
    }
    points = std::move(inliers);
  }

  BoundaryCurve curve = solve(points);
  const double span = curve.x_max - curve.x_min;
  curve.valid = curve.inlier_count >= static_cast<std::size_t>(config_.minimum_points) &&
    span >= config_.minimum_x_span &&
    curve.rms_residual <= config_.maximum_rms_residual;
  if (!curve.valid) {
    return curve;
  }

  const double count_score = std::min(
    1.0, static_cast<double>(curve.inlier_count) /
    static_cast<double>(std::max(1, 2 * config_.minimum_points)));
  const double span_score = std::min(1.0, span / std::max(0.01, 2.0 * config_.minimum_x_span));
  const double residual_score = std::max(
    0.0, 1.0 - curve.rms_residual / std::max(1e-6, config_.maximum_rms_residual));
  curve.confidence = count_score * span_score * residual_score;

  if (!filter_ready_) {
    filtered_curve_ = curve;
    filter_ready_ = true;
  } else {
    const double alpha = std::clamp(config_.filter_alpha, 0.0, 1.0);
    filtered_curve_.a0 = (1.0 - alpha) * filtered_curve_.a0 + alpha * curve.a0;
    filtered_curve_.a1 = (1.0 - alpha) * filtered_curve_.a1 + alpha * curve.a1;
    filtered_curve_.a2 = (1.0 - alpha) * filtered_curve_.a2 + alpha * curve.a2;
    filtered_curve_.x_min = curve.x_min;
    filtered_curve_.x_max = curve.x_max;
    filtered_curve_.rms_residual = curve.rms_residual;
    filtered_curve_.confidence = curve.confidence;
    filtered_curve_.inlier_count = curve.inlier_count;
    filtered_curve_.valid = true;
  }
  return filtered_curve_;
}

void BoundaryModel::reset()
{
  filtered_curve_ = BoundaryCurve{};
  filter_ready_ = false;
}

}  // camera_boundary_navigation命名空间
