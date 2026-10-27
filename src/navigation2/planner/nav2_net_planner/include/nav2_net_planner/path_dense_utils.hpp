// Copyright 2026
#ifndef NAV2_NET_PLANNER__PATH_DENSE_UTILS_HPP_
#define NAV2_NET_PLANNER__PATH_DENSE_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_net_planner
{

inline double yawFromOrientation(const geometry_msgs::msg::Quaternion & o)
{
  tf2::Quaternion q;
  tf2::fromMsg(o, q);
  return tf2::getYaw(q);
}

/**
 * @brief Resample a polyline path at fixed arc-length spacing, optionally truncating total length.
 * @param sparse Path with at least consecutive vertices (typically graph nodes).
 * @param step_m Distance between dense samples along the polyline (clamped to >= 1e-3).
 * @param max_path_length_m If > 0, stop after this arc length from the first pose; if <= 0, use full path.
 */
inline nav_msgs::msg::Path densifyPolyline(
  const nav_msgs::msg::Path & sparse,
  double step_m,
  double max_path_length_m)
{
  nav_msgs::msg::Path out;
  out.header = sparse.header;
  if (sparse.poses.empty()) {
    return out;
  }
  if (sparse.poses.size() == 1) {
    out.poses.push_back(sparse.poses[0]);
    return out;
  }

  const double step = std::max(step_m, 1e-3);
  const size_t n = sparse.poses.size();
  std::vector<double> cum(n, 0.0);
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & a = sparse.poses[i].pose.position;
    const auto & b = sparse.poses[i + 1].pose.position;
    cum[i + 1] = cum[i] + std::hypot(b.x - a.x, b.y - a.y);
  }
  const double total_len = cum.back();
  const double max_s =
    (max_path_length_m > 1e-6) ? std::min(total_len, max_path_length_m) : total_len;

  auto interp = [&](double s, double & x, double & y, double & yaw) {
    size_t k = 0;
    while (k + 1 < n && cum[k + 1] < s - 1e-9) {
      ++k;
    }
    if (k + 1 >= n) {
      const auto & p = sparse.poses.back().pose;
      x = p.position.x;
      y = p.position.y;
      yaw = yawFromOrientation(p.orientation);
      return;
    }
    const auto & pa = sparse.poses[k].pose.position;
    const auto & pb = sparse.poses[k + 1].pose.position;
    const double seg_len = std::hypot(pb.x - pa.x, pb.y - pa.y);
    const double ds = s - cum[k];
    const double t = (seg_len > 1e-9) ? std::clamp(ds / seg_len, 0.0, 1.0) : 0.0;
    x = pa.x + t * (pb.x - pa.x);
    y = pa.y + t * (pb.y - pa.y);
    yaw = (seg_len > 1e-9) ? std::atan2(pb.y - pa.y, pb.x - pa.x)
                          : yawFromOrientation(sparse.poses[k].pose.orientation);
  };

  const double z0 = sparse.poses.front().pose.position.z;
  for (double s = 0.0; s <= max_s + 1e-9; ) {
    double x = 0.0, y = 0.0, yaw = 0.0;
    interp(s, x, y, yaw);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = out.header;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    ps.pose.orientation = tf2::toMsg(q);
    out.poses.push_back(ps);
    if (s >= max_s - 1e-9) {
      break;
    }
    const double next_s = std::min(s + step, max_s);
    if (next_s <= s + 1e-12) {
      break;
    }
    s = next_s;
  }

  if (max_s >= total_len - 1e-6 && !sparse.poses.empty()) {
    const auto & pe = sparse.poses.back().pose;
    if (out.poses.empty() ||
      std::hypot(
        out.poses.back().pose.position.x - pe.position.x,
        out.poses.back().pose.position.y - pe.position.y) > step * 0.25)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose = pe;
      out.poses.push_back(ps);
    }
  }

  return out;
}

}  // namespace nav2_net_planner

#endif  // NAV2_NET_PLANNER__PATH_DENSE_UTILS_HPP_
