// Copyright 2026

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "nav2_a_star_rpp/costmap_path_smoother.hpp"

namespace
{

nav_msgs::msg::Path makeRightAnglePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.1 * i;
    path.poses.push_back(pose);
  }
  for (int i = 1; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 0.1 * i;
    path.poses.push_back(pose);
  }
  return path;
}

double maxHeadingChange(const nav_msgs::msg::Path & path)
{
  double result = 0.0;
  for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const auto & c = path.poses[i + 1].pose.position;
    const double first = std::atan2(b.y - a.y, b.x - a.x);
    const double second = std::atan2(c.y - b.y, c.x - b.x);
    result = std::max(result, std::abs(std::remainder(second - first, 2.0 * M_PI)));
  }
  return result;
}

}  // namespace

TEST(CostmapPathSmoother, RoundsCornerAndKeepsEndpointsAndCorridor)
{
  nav2_a_star_rpp::CostmapPathSmootherParameters parameters;
  parameters.max_iterations = 200;
  parameters.max_deviation = 0.30;
  parameters.collision_check_step = 0.02;
  nav2_a_star_rpp::CostmapPathSmoother smoother(parameters);
  const auto input = makeRightAnglePath();
  nav_msgs::msg::Path output;

  ASSERT_TRUE(smoother.smooth(input, output, [](double, double, double) {return true;}));
  ASSERT_EQ(output.poses.size(), input.poses.size());
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.x, input.poses.front().pose.position.x);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.y, input.poses.front().pose.position.y);
  EXPECT_DOUBLE_EQ(output.poses.back().pose.position.x, input.poses.back().pose.position.x);
  EXPECT_DOUBLE_EQ(output.poses.back().pose.position.y, input.poses.back().pose.position.y);
  EXPECT_LT(maxHeadingChange(output), maxHeadingChange(input));

  for (size_t i = 0; i < input.poses.size(); ++i) {
    const auto & before = input.poses[i].pose.position;
    const auto & after = output.poses[i].pose.position;
    EXPECT_LE(std::hypot(after.x - before.x, after.y - before.y), 0.300001);
    EXPECT_NEAR(
      std::hypot(output.poses[i].pose.orientation.z, output.poses[i].pose.orientation.w),
      1.0, 1.0e-12);
  }
}

TEST(CostmapPathSmoother, RejectsUnsafeOutputAndPreservesInputForFallback)
{
  nav2_a_star_rpp::CostmapPathSmoother smoother;
  const auto input = makeRightAnglePath();
  nav_msgs::msg::Path output;

  EXPECT_FALSE(
    smoother.smooth(
      input, output, [](double, double, double) {return false;}));
  ASSERT_EQ(output.poses.size(), input.poses.size());
  for (size_t i = 0; i < input.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(output.poses[i].pose.position.x, input.poses[i].pose.position.x);
    EXPECT_DOUBLE_EQ(output.poses[i].pose.position.y, input.poses[i].pose.position.y);
  }
}

TEST(CostmapPathSmoother, SamplesBetweenPathPosesDuringValidation)
{
  nav_msgs::msg::Path input;
  input.poses.resize(3);
  input.poses[0].pose.position.x = 0.0;
  input.poses[1].pose.position.x = 1.0;
  input.poses[2].pose.position.x = 2.0;
  nav2_a_star_rpp::CostmapPathSmootherParameters parameters;
  parameters.collision_check_step = 0.05;
  nav2_a_star_rpp::CostmapPathSmoother smoother(parameters);
  nav_msgs::msg::Path output;

  const auto thin_obstacle = [](double x, double, double) {
      return x < 0.47 || x > 0.53;
    };
  nav2_a_star_rpp::SmoothingDiagnostics diagnostics;
  EXPECT_FALSE(smoother.smooth(input, output, thin_obstacle, &diagnostics));
  EXPECT_TRUE(diagnostics.has_failed_pose);
  EXPECT_EQ(diagnostics.failed_segment, 0u);
  EXPECT_GE(diagnostics.x, 0.47);
  EXPECT_LE(diagnostics.x, 0.53);
  EXPECT_DOUBLE_EQ(diagnostics.y, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.yaw, 0.0);
  EXPECT_EQ(diagnostics.backtracking_attempts, parameters.backtracking_steps);
  EXPECT_EQ(diagnostics.rejected_points, 0);  // 碰撞位于线段内部，而非候选点。
  EXPECT_EQ(output, input);  // 诊断不改变失败后返回原路径的行为。

  // 同一个诊断对象再次使用时必须清除上次的碰撞信息。
  ASSERT_TRUE(smoother.smooth(input, output,
    [](double, double, double) {return true;}, &diagnostics));
  EXPECT_FALSE(diagnostics.has_failed_pose);
  EXPECT_EQ(diagnostics.backtracking_attempts, 0);
}

TEST(CostmapPathSmoother, ReportsShortPathWithoutInventingCollision)
{
  nav2_a_star_rpp::CostmapPathSmoother smoother;
  nav_msgs::msg::Path input, output;
  input.poses.resize(2);
  nav2_a_star_rpp::SmoothingDiagnostics diagnostics;
  EXPECT_FALSE(smoother.smooth(input, output,
    [](double, double, double) {return true;}, &diagnostics));
  EXPECT_EQ(diagnostics.iterations, 0);
  EXPECT_FALSE(diagnostics.has_failed_pose);
  EXPECT_EQ(output, input);
}
