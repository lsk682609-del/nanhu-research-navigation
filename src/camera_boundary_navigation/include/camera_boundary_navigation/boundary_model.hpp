#ifndef CAMERA_BOUNDARY_NAVIGATION__BOUNDARY_MODEL_HPP_
#define CAMERA_BOUNDARY_NAVIGATION__BOUNDARY_MODEL_HPP_

#include <vector>

#include "camera_boundary_navigation/types.hpp"

namespace camera_boundary_navigation
{

struct BoundaryModelConfig
{
  double fit_x_min = 0.6;
  double fit_x_max = 3.5;
  int minimum_points = 8;
  double minimum_x_span = 1.0;
  double maximum_rms_residual = 0.12;
  double outlier_threshold = 0.18;
  int robust_iterations = 3;
  double filter_alpha = 0.3;
};

class BoundaryModel
{
public:
  explicit BoundaryModel(BoundaryModelConfig config);

  BoundaryCurve fit(const std::vector<Point2d> & points);
  void reset();

private:
  BoundaryCurve solve(const std::vector<Point2d> & points) const;

  BoundaryModelConfig config_;
  BoundaryCurve filtered_curve_;
  bool filter_ready_ = false;
};

}  // camera_boundary_navigation命名空间

#endif  // 头文件保护宏结束
