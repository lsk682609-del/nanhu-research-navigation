#ifndef CAMERA_BOUNDARY_NAVIGATION__BOUNDARY_CONTROLLER_HPP_
#define CAMERA_BOUNDARY_NAVIGATION__BOUNDARY_CONTROLLER_HPP_

#include <limits>
#include "camera_boundary_navigation/types.hpp"

namespace camera_boundary_navigation
{

struct BoundaryControllerConfig
{
  ReferenceSide side = ReferenceSide::LEFT;
  double boundary_offset = 0.65;
  double path_sample_step = 0.10;
  double bezier_handle_ratio = 0.35;
  double lookahead_distance = 1.5;
  double nominal_linear_velocity = 0.25;
  double minimum_linear_velocity = 0.10;
  double maximum_linear_velocity = 0.30;
  double maximum_angular_velocity = 0.35;
  double maximum_angular_acceleration = 0.50;
  double maximum_abs_curvature = 1.50;
  double curvature_slowdown_gain = 1.50;
  double command_filter_alpha = 0.30;
};

class BoundaryController
{
public:
  explicit BoundaryController(BoundaryControllerConfig config);

  ReferencePath buildReferencePath(const BoundaryCurve & curve) const;
  ReferencePath buildConstantCurvaturePath(double curvature, double horizon) const;
  double targetCurvature(const ReferencePath & path) const;
  ControlCommand computeCommand(const ReferencePath & path, double dt,
    double speed_limit = std::numeric_limits<double>::infinity());
  void reset();

private:
  BoundaryControllerConfig config_;
  ControlCommand previous_command_;
  bool command_ready_ = false;
};

}  // camera_boundary_navigation命名空间

#endif  // 头文件保护宏结束
