#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "camera_boundary_navigation/boundary_controller.hpp"
#include "camera_boundary_navigation/boundary_model.hpp"
#include "camera_boundary_navigation/safety_gate.hpp"
#include "camera_boundary_navigation/stable_local_planner.hpp"

namespace camera_boundary_navigation
{

TEST(BoundaryModel, FitsQuadraticAndRejectsOutlier)
{
  BoundaryModelConfig config;
  config.fit_x_min = 0.5;
  config.fit_x_max = 4.0;
  config.minimum_points = 8;
  config.minimum_x_span = 1.0;
  config.maximum_rms_residual = 0.05;
  config.outlier_threshold = 0.12;
  config.filter_alpha = 1.0;
  BoundaryModel model(config);

  std::vector<Point2d> points;
  for (int i = 0; i < 30; ++i) {
    const double x = 0.5 + 0.1 * i;
    points.push_back({x, 1.0 + 0.1 * x + 0.02 * x * x});
  }
  points.push_back({2.0, -2.0});

  const BoundaryCurve curve = model.fit(points);
  ASSERT_TRUE(curve.valid);
  EXPECT_NEAR(curve.a0, 1.0, 1e-6);
  EXPECT_NEAR(curve.a1, 0.1, 1e-6);
  EXPECT_NEAR(curve.a2, 0.02, 1e-6);
  EXPECT_EQ(curve.inlier_count, 30U);
}

TEST(BoundaryController, SteersTowardLeftAndRightOffsetPaths)
{
  BoundaryCurve left_curve;
  left_curve.a0 = 1.0;
  left_curve.x_min = 0.6;
  left_curve.x_max = 3.5;
  left_curve.valid = true;

  BoundaryControllerConfig left_config;
  left_config.side = ReferenceSide::LEFT;
  left_config.boundary_offset = 0.6;
  left_config.command_filter_alpha = 1.0;
  BoundaryController left_controller(left_config);
  const ReferencePath left_path = left_controller.buildReferencePath(left_curve);
  const ControlCommand left_command = left_controller.computeCommand(left_path, 0.05);
  ASSERT_TRUE(left_command.valid);
  EXPECT_GT(left_path.target.y, 0.0);
  EXPECT_GT(left_command.angular, 0.0);

  BoundaryCurve right_curve = left_curve;
  right_curve.a0 = -1.0;
  BoundaryControllerConfig right_config = left_config;
  right_config.side = ReferenceSide::RIGHT;
  BoundaryController right_controller(right_config);
  const ReferencePath right_path = right_controller.buildReferencePath(right_curve);
  const ControlCommand right_command = right_controller.computeCommand(right_path, 0.05);
  ASSERT_TRUE(right_command.valid);
  EXPECT_LT(right_path.target.y, 0.0);
  EXPECT_LT(right_command.angular, 0.0);
}

TEST(BoundaryController, BezierConnectorIsTangentToOffsetBoundary)
{
  BoundaryCurve curve;
  curve.a0 = 1.0;
  curve.x_min = 0.6;
  curve.x_max = 2.0;
  curve.valid = true;

  BoundaryControllerConfig config;
  config.side = ReferenceSide::LEFT;
  config.boundary_offset = 0.6;
  config.path_sample_step = 0.1;
  config.bezier_handle_ratio = 0.35;
  BoundaryController controller(config);

  const ReferencePath path = controller.buildReferencePath(curve);
  ASSERT_TRUE(path.valid);
  ASSERT_GT(path.points.size(), 5U);
  EXPECT_NEAR(path.points.front().x, 0.0, 1e-12);
  EXPECT_NEAR(path.points.front().y, 0.0, 1e-12);

  // 对于这条水平左边界，第一个偏移点为(0.6, 0.4)。连接点前的Bé塞尔线段
  // 与连接点后的边界线段应具有近似相同的航向，不能再出现原来的明显折角。
  std::size_t join_index = path.points.size();
  for (std::size_t i = 1; i + 1U < path.points.size(); ++i) {
    if (std::abs(path.points[i].x - 0.6) < 1e-9 &&
      std::abs(path.points[i].y - 0.4) < 1e-9)
    {
      join_index = i;
      break;
    }
  }
  ASSERT_LT(join_index + 1U, path.points.size());
  const Point2d incoming{
    path.points[join_index].x - path.points[join_index - 1U].x,
    path.points[join_index].y - path.points[join_index - 1U].y};
  const Point2d outgoing{
    path.points[join_index + 1U].x - path.points[join_index].x,
    path.points[join_index + 1U].y - path.points[join_index].y};
  const double heading_difference = std::abs(
    std::atan2(incoming.y, incoming.x) - std::atan2(outgoing.y, outgoing.x));
  EXPECT_LT(heading_difference, 0.10);
}

TEST(StableLocalPlanner, DeadbandForcesStraightDespiteBoundaryCurvature)
{
  StablePlannerConfig config;
  config.preferred_boundary_distance = 0.65;
  config.boundary_distance_deadband = 0.20;
  config.turn_confirmation_frames = 3;
  StableLocalPlanner planner(config, ReferenceSide::RIGHT);

  for (double distance : {0.52, 0.78, 0.61, 0.83}) {
    const StablePlannerDecision decision = planner.update(true, distance, 0.25);
    EXPECT_TRUE(decision.select_straight);
    EXPECT_FALSE(decision.turning);
    EXPECT_DOUBLE_EQ(decision.selected_curvature, 0.0);
  }
}

TEST(StableLocalPlanner, RequiresConsecutiveFramesAndPenalizesCurvatureChange)
{
  StablePlannerConfig config;
  config.preferred_boundary_distance = 0.65;
  config.boundary_distance_deadband = 0.10;
  config.turn_confirmation_frames = 3;
  config.straight_confirmation_frames = 2;
  config.minimum_turn_curvature = 0.05;
  config.curvature_tracking_weight = 1.0;
  config.curvature_change_weight = 3.0;
  config.maximum_curvature_change_per_frame = 0.10;
  StableLocalPlanner planner(config, ReferenceSide::RIGHT);

  // 右边界距离过近，需要向左转；前两帧仍保持直行。
  EXPECT_TRUE(planner.update(true, 0.40, 0.40).select_straight);
  EXPECT_TRUE(planner.update(true, 0.40, 0.40).select_straight);
  const StablePlannerDecision turn = planner.update(true, 0.40, 0.40);
  EXPECT_FALSE(turn.select_straight);
  EXPECT_TRUE(turn.turning);
  EXPECT_GT(turn.selected_curvature, 0.0);
  EXPECT_LE(turn.selected_curvature, 0.10);
  EXPECT_LT(turn.selected_curvature, 0.40);

  // 回到死区后仍需连续两帧确认，不能立即从转弯跳回直线。
  EXPECT_FALSE(planner.update(true, 0.65, 0.0).select_straight);
  EXPECT_TRUE(planner.update(true, 0.65, 0.0).select_straight);
}

TEST(SafetyGate, RequiresRecoveryThenRejectsObstacle)
{
  SafetyConfig config;
  config.grid_resolution = 0.1;
  config.path_clearance = 0.25;
  config.minimum_front_range = 1.0;
  config.minimum_obstacle_points_per_cell = 1;
  config.recovery_valid_frames = 2;
  SafetyGate gate(config);

  BoundaryCurve curve;
  curve.valid = true;
  ReferencePath path;
  path.valid = true;
  for (int i = 0; i <= 20; ++i) {
    path.points.push_back({0.1 * i, 0.4});
  }
  path.target = {1.5, 0.4};
  path.lookahead_distance = std::hypot(path.target.x, path.target.y);
  gate.updateGroundCells(path.points);
  gate.updateObstacleCells({});
  gate.setFrontRange(3.0);
  gate.setInputsFresh(true, true, true);

  gate.markNewFrame();
  EXPECT_EQ(
    gate.evaluate(true, false, curve, path), SafetyState::WAITING_FOR_INPUT);
  gate.markNewFrame();
  EXPECT_EQ(gate.evaluate(true, false, curve, path), SafetyState::TRACKING);

  gate.updateObstacleCells({{1.0, 0.45}});
  EXPECT_EQ(gate.evaluate(true, false, curve, path), SafetyState::PATH_BLOCKED);
}

TEST(SafetyGate, GroundBlindZoneDoesNotDisableNearObstacleCheck)
{
  SafetyConfig config;
  config.grid_resolution = 0.1;
  config.minimum_ground_check_x = 0.6;
  config.path_clearance = 0.2;
  config.minimum_obstacle_points_per_cell = 1;
  SafetyGate gate(config);

  ReferencePath path;
  path.valid = true;
  for (int i = 0; i <= 10; ++i) {
    path.points.push_back({0.1 * i, 0.0});
  }
  std::vector<Point2d> visible_ground;
  for (int i = 6; i <= 10; ++i) {
    visible_ground.push_back({0.1 * i, 0.0});
  }
  gate.updateGroundCells(visible_ground);
  gate.updateObstacleCells({});
  EXPECT_EQ(gate.checkPathGeometry(path), SafetyState::TRACKING);

  // Ground盲区可以跳过，但同一区域的障碍物仍必须触发停车。
  gate.updateObstacleCells({{0.3, 0.0}});
  EXPECT_EQ(gate.checkPathGeometry(path), SafetyState::PATH_BLOCKED);
}

}  // camera_boundary_navigation命名空间
