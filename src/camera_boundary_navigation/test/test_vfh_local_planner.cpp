#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "camera_boundary_navigation/vfh_local_planner.hpp"

namespace camera_boundary_navigation
{
namespace
{
std::vector<Point2d> ground(double min_y = -2.0, double max_y = 2.0)
{
  std::vector<Point2d> result;
  for (int x = 0; x < 45; ++x) {
    for (int y = -25; y < 25; ++y) {
      const Point2d p{(x + 0.5) * 0.1, (y + 0.5) * 0.1};
      if (p.y >= min_y && p.y <= max_y) {result.push_back(p);}
    }
  }
  return result;
}
std::vector<Point2d> groundUntil(double max_x)
{
  auto result = ground();
  result.erase(
    std::remove_if(
      result.begin(), result.end(),
      [max_x](const Point2d & point) {return point.x > max_x;}),
    result.end());
  return result;
}
double pathLength(const ReferencePath & path)
{
  double result = 0.0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    result += std::hypot(
      path.points[i].x - path.points[i - 1].x,
      path.points[i].y - path.points[i - 1].y);
  }
  return result;
}
SafetyConfig safetyConfig()
{
  SafetyConfig c;
  c.grid_resolution = 0.1;
  c.minimum_obstacle_points_per_cell = 1;
  c.path_clearance = 0.3;
  c.ground_half_width = 0.25;
  c.directional_front_check = true;
  c.recovery_valid_frames = 1;
  return c;
}
VfhConfig plannerConfig()
{
  VfhConfig c;
  c.inflation_radius = 0.371;
  c.return_confirmation_frames = 3;
  return c;
}
}
class VfhTest : public ::testing::Test
{
protected:
  BoundaryControllerConfig control_config;
  BoundaryController controller{control_config};
  SafetyGate safety{safetyConfig()};
  VfhLocalPlanner planner{plannerConfig(), control_config};
  ReferencePath straight = controller.buildConstantCurvaturePath(0.0, 2.0);
  void SetUp() override {safety.updateGroundCells(ground()); safety.updateObstacleCells({});}
};

TEST_F(VfhTest, ClearScenePreservesNominalPath)
{
  const auto d = planner.plan(straight, straight, 2.0, safety);
  EXPECT_EQ(d.mode, "FOLLOW");
  EXPECT_EQ(d.path.points.size(), straight.points.size());
  EXPECT_DOUBLE_EQ(d.curvature, 0.0);
  EXPECT_GT(d.speed_limit, 0.0);
}

TEST_F(VfhTest, LowClearanceStraightActivelyMovesAwayFromRightSide)
{
  std::vector<Point2d> wall;
  for (int i = 10; i <= 22; ++i) {wall.push_back({i * 0.1 + 0.05, -0.45});}
  safety.updateObstacleCells(wall);
  ASSERT_EQ(safety.checkPathGeometry(straight), SafetyState::TRACKING);
  const double old_clearance = planner.pathClearance(straight, safety);
  ASSERT_LT(old_clearance, plannerConfig().preferred_clearance);
  const auto d = planner.plan(straight, straight, 2.0, safety);
  ASSERT_EQ(d.mode, "AVOID");
  EXPECT_EQ(d.reason, "nominal_clearance_low");
  EXPECT_GT(d.curvature, 0.0);
  EXPECT_GT(d.clearance, old_clearance);
  EXPECT_EQ(safety.checkPathGeometry(d.path), SafetyState::TRACKING);
}

TEST_F(VfhTest, CorridorRejectsTooNarrowPassageEvenWithoutObstaclePoints)
{
  BoundaryCorridor c;
  c.valid = true;
  c.margin = plannerConfig().inflation_radius;
  c.left.valid = c.right.valid = true;
  c.left.x_min = c.right.x_min = 0.6;
  c.left.x_max = c.right.x_max = 2.0;
  c.left.a0 = 0.35;
  c.right.a0 = -0.35;
  const auto d = planner.plan(straight, straight, 2.0, safety, c);
  EXPECT_EQ(d.mode, "BLOCKED");
  EXPECT_EQ(d.reason, "corridor_too_narrow");
  EXPECT_FALSE(d.path.valid);
}

TEST_F(VfhTest, CorridorKeepsCandidatesWithinBothBoundaryMargins)
{
  BoundaryCorridor c;
  c.valid = true;
  c.margin = plannerConfig().inflation_radius;
  c.left.valid = c.right.valid = true;
  c.left.x_min = c.right.x_min = 0.6;
  c.left.x_max = c.right.x_max = 2.0;
  c.left.a0 = 0.8;
  c.right.a0 = -0.45;
  auto center_config = control_config;
  center_config.boundary_offset = 0.0;
  BoundaryCurve center = c.left;
  center.a0 = (c.left.a0 + c.right.a0) * 0.5;
  const auto target = BoundaryController(center_config).buildReferencePath(center);
  const auto d = planner.plan(straight, target, 2.0, safety, c);
  ASSERT_TRUE(d.path.valid);
  EXPECT_GT(d.curvature, 0.0);
  EXPECT_TRUE(c.contains(d.path));
  for (const auto & candidate : d.candidates) {
    if (candidate.safe) {EXPECT_TRUE(c.contains(candidate.path));}
  }
}

TEST_F(VfhTest, CentralObstacleProducesSafeArcAndKeepsSide)
{
  safety.updateObstacleCells({{1.35, 0.05}});
  const auto d = planner.plan(straight, straight, 2.0, safety);
  ASSERT_EQ(d.mode, "AVOID");
  ASSERT_TRUE(d.path.valid);
  EXPECT_GT(std::abs(d.curvature), 0.1);
  EXPECT_EQ(safety.checkPathGeometry(d.path), SafetyState::TRACKING);
  EXPECT_GT(d.speed_limit, 0.0);
  const auto next = planner.plan(straight, straight, 2.0, safety);
  EXPECT_GT(next.curvature * d.curvature, 0.0);
}

TEST_F(VfhTest, RoadEdgeForcesInwardBypass)
{
  safety.updateGroundCells(ground(-0.35, 2.0));
  safety.updateObstacleCells({{1.35, 0.05}});
  const auto d = planner.plan(straight, straight, 2.0, safety);
  ASSERT_EQ(d.mode, "AVOID");
  EXPECT_GT(d.curvature, 0.0);
}

TEST_F(VfhTest, WallAndNarrowRoadBlockInsteadOfFallingBackToTarget)
{
  std::vector<Point2d> wall;
  for (int i = -20; i <= 20; ++i) {wall.push_back({0.75, i * 0.1 + 0.05});}
  safety.updateObstacleCells(wall);
  auto d = planner.plan(straight, straight, 2.0, safety);
  EXPECT_EQ(d.mode, "BLOCKED");
  EXPECT_FALSE(d.path.valid);
  EXPECT_DOUBLE_EQ(d.speed_limit, 0.0);
  safety.updateGroundCells(ground(-0.4, 0.4));
  safety.updateObstacleCells({{0.75, 0.05}});
  d = planner.plan(straight, straight, 2.0, safety);
  EXPECT_EQ(d.mode, "BLOCKED");
}

TEST_F(VfhTest, NearObstacleStopsAllCandidates)
{
  safety.updateObstacleCells({{0.4, 0.0}});
  EXPECT_EQ(planner.plan(straight, straight, 2.0, safety).mode, "BLOCKED");
}

TEST_F(VfhTest, EmptyObstacleCloudDoesNotMakeUnknownGroundFree)
{
  safety.updateGroundCells({});
  EXPECT_EQ(planner.plan(straight, straight, 2.0, safety).mode, "BLOCKED");
  safety.updateGroundCells(straight.points);
  EXPECT_EQ(planner.plan(straight, straight, 2.0, safety).mode, "BLOCKED");
}

TEST_F(VfhTest, DensityDoesNotChangeSelectedArc)
{
  safety.updateObstacleCells({{1.35, 0.05}});
  const auto first = planner.plan(straight, straight, 2.0, safety);
  planner.reset();
  safety.updateObstacleCells(std::vector<Point2d>(1000, {1.35, 0.05}));
  const auto dense = planner.plan(straight, straight, 2.0, safety);
  EXPECT_EQ(first.histogram, dense.histogram);
  EXPECT_DOUBLE_EQ(first.curvature, dense.curvature);
}

TEST_F(VfhTest, ReturnsAfterClearFramesAndConverges)
{
  safety.updateObstacleCells({{1.35, 0.05}});
  ASSERT_EQ(planner.plan(straight, straight, 2.0, safety).mode, "AVOID");
  safety.updateObstacleCells({});
  EXPECT_EQ(planner.plan(straight, straight, 2.0, safety).mode, "AVOID");
  EXPECT_EQ(planner.plan(straight, straight, 2.0, safety).mode, "AVOID");
  bool returned = false;
  for (int i = 0; i < 50; ++i) {
    auto d = planner.plan(straight, straight, 2.0, safety);
    ASSERT_TRUE(d.path.valid);
    if (d.mode == "FOLLOW") {returned = true; break;}
  }
  EXPECT_TRUE(returned);
}

TEST_F(VfhTest, ShortVisibleRangeAndInvalidBoundaryStop)
{
  EXPECT_EQ(planner.plan(straight, straight, 0.8, safety).mode, "BLOCKED");
  EXPECT_EQ(planner.plan(straight, ReferencePath{}, 2.0, safety).mode, "BLOCKED");
}

TEST_F(VfhTest, UsesLongestSafePrefixWhenFullHorizonLeavesVisibleGround)
{
  // The footprint can be verified for roughly 1.2 m, but not for the full 2 m
  // planning horizon.  Adaptive candidates must keep moving on a safe prefix.
  safety.updateGroundCells(groundUntil(1.55));
  const auto d = planner.plan(straight, straight, 2.0, safety);
  ASSERT_EQ(d.mode, "AVOID");
  ASSERT_TRUE(d.path.valid);
  EXPECT_EQ(safety.checkPathGeometry(d.path), SafetyState::TRACKING);
  EXPECT_GE(pathLength(d.path) + 1e-6, plannerConfig().minimum_path_length);
  EXPECT_LT(pathLength(d.path), plannerConfig().horizon - 0.1);
  EXPECT_GT(d.speed_limit, 0.0);
  EXPECT_LT(d.speed_limit, control_config.maximum_linear_velocity);
}

TEST_F(VfhTest, RejectsSafePrefixShorterThanConfiguredMinimum)
{
  safety.updateGroundCells(groundUntil(0.75));
  const auto d = planner.plan(straight, straight, 2.0, safety);
  EXPECT_EQ(d.mode, "BLOCKED");
  EXPECT_FALSE(d.path.valid);
  EXPECT_DOUBLE_EQ(d.speed_limit, 0.0);
}

TEST_F(VfhTest, DirectionalGroundCheckAllowsSidePassDespiteShortGlobalFront)
{
  safety.setFrontRange(0.5);
  safety.setInputsFresh(true, true, true);
  safety.markNewFrame();
  BoundaryCurve curve;
  curve.valid = true;
  EXPECT_EQ(safety.evaluate(true, false, curve, straight), SafetyState::TRACKING);
  safety.setInputsFresh(true, true, false);
  EXPECT_EQ(safety.evaluate(true, false, curve, straight), SafetyState::WAITING_FOR_INPUT);
}

TEST_F(VfhTest, SafetySpeedCapOverridesMinimumAndFilter)
{
  controller.computeCommand(straight, 0.05);
  auto cmd = controller.computeCommand(straight, 0.05, 0.02);
  ASSERT_TRUE(cmd.valid);
  EXPECT_LE(cmd.linear, 0.02);
  cmd = controller.computeCommand(straight, 0.05, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
}

TEST_F(VfhTest, ActualCommandPredictionRejectsCollisionAndReverse)
{
  ControlCommand cmd;
  cmd.valid = true;
  cmd.linear = 0.3;
  EXPECT_TRUE(planner.commandIsSafe(cmd, safety));
  cmd.linear = 0.02;
  cmd.angular = 0.01;
  EXPECT_TRUE(planner.commandIsSafe(cmd, safety));
  cmd.linear = 0.3;
  cmd.angular = 0.0;
  safety.updateObstacleCells({{0.4, 0.0}});
  EXPECT_FALSE(planner.commandIsSafe(cmd, safety));
  safety.updateObstacleCells({});
  cmd.linear = -0.1;
  EXPECT_FALSE(planner.commandIsSafe(cmd, safety));
  cmd.linear = 0.0;
  cmd.angular = 0.2;
  EXPECT_FALSE(planner.commandIsSafe(cmd, safety));
}

TEST(VfhParameters, RejectsInvalidBrakingAndSampling)
{
  VfhConfig config;
  config.braking_deceleration = 0.0;
  EXPECT_THROW(VfhLocalPlanner(config, BoundaryControllerConfig{}), std::invalid_argument);
  config = VfhConfig{};
  config.angular_bins = 1;
  EXPECT_THROW(VfhLocalPlanner(config, BoundaryControllerConfig{}), std::invalid_argument);
}
}  // namespace camera_boundary_navigation
