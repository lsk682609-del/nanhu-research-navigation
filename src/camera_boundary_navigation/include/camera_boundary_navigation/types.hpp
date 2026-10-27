#ifndef CAMERA_BOUNDARY_NAVIGATION__TYPES_HPP_
#define CAMERA_BOUNDARY_NAVIGATION__TYPES_HPP_

#include <cstddef>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace camera_boundary_navigation
{

struct Point2d
{
  double x = 0.0;
  double y = 0.0;
};

enum class ReferenceSide
{
  LEFT,
  RIGHT,
};

struct BoundaryCurve
{
  double a0 = 0.0;
  double a1 = 0.0;
  double a2 = 0.0;
  double x_min = 0.0;
  double x_max = 0.0;
  double rms_residual = 0.0;
  double confidence = 0.0;
  std::size_t inlier_count = 0U;
  bool valid = false;

  double y(double x) const {return a0 + a1 * x + a2 * x * x;}
  double derivative(double x) const {return a1 + 2.0 * a2 * x;}
};

struct ReferencePath
{
  std::vector<Point2d> points;
  Point2d target;
  double lookahead_distance = 0.0;
  bool valid = false;
};

struct ControlCommand
{
  double linear = 0.0;
  double angular = 0.0;
  double curvature = 0.0;
  bool valid = false;
};

struct BoundaryCorridor
{
  BoundaryCurve left, right;
  double margin = 0.0;
  bool valid = false;
  double xMin() const {return std::max(left.x_min, right.x_min);}
  double xMax() const {return std::min(left.x_max, right.x_max);}
  double lower(double x) const {return right.y(x) + margin * std::hypot(1.0, right.derivative(x));}
  double upper(double x) const {return left.y(x) - margin * std::hypot(1.0, left.derivative(x));}
  bool usable() const {
    if (!valid || xMax() <= xMin()) {return false;}
    const int count = std::max(1, static_cast<int>(std::ceil((xMax() - xMin()) / 0.02)));
    for (int i = 0; i <= count; ++i) {
      const double x = xMin() + (xMax() - xMin()) * i / count;
      if (lower(x) >= upper(x)) {return false;}
    }
    return true;
  }
  bool contains(const ReferencePath & path) const {
    if (!valid) {return true;}
    if (!usable() || !path.valid) {return false;}
    for (const auto & p : path.points) {
      if (p.x < xMin() || p.x > xMax()) {continue;}
      if (p.y < lower(p.x) || p.y > upper(p.x)) {return false;}
    }
    return true;
  }
};

enum class SafetyState
{
  DISABLED,
  PAUSED,
  WAITING_FOR_INPUT,
  BOUNDARY_INVALID,
  FRONT_TOO_CLOSE,
  PATH_NOT_ON_GROUND,
  PATH_BLOCKED,
  TRACKING,
};

std::string toString(SafetyState state);

}  // camera_boundary_navigation命名空间

#endif  // 头文件保护宏结束
