// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_A_STAR_RPP__COSTMAP_PATH_SMOOTHER_HPP_
#define NAV2_A_STAR_RPP__COSTMAP_PATH_SMOOTHER_HPP_

#include <functional>

#include "nav_msgs/msg/path.hpp"

namespace nav2_a_star_rpp
{

struct CostmapPathSmootherParameters
{
  int max_iterations{100};                 // 最大迭代次数
  double tolerance{1.0e-4};                // 单轮路径点累计变化量的收敛阈值
  double data_weight{0.20};                 // 原始 Theta* 路径的保持权重
  double smooth_weight{0.30};               // 相邻路径点的平滑权重
  double max_deviation{0.50};               // 每个点相对原路径对应点的最大偏移（米）
  double collision_check_step{0.025};       // 连续碰撞复检的采样间距（米）
  int backtracking_steps{6};                // 平滑路径碰撞时向原路径回退的次数
};

// 仅记录本次平滑过程，不参与路径选择与碰撞判定。
struct SmoothingDiagnostics
{
  const char * reason{"未执行"};
  int iterations{0};
  int rejected_points{0};
  int backtracking_attempts{0};
  bool has_failed_pose{false};
  size_t failed_segment{0};  // 失败线段的起点索引，从零开始
  double x{0.0}, y{0.0}, yaw{0.0};  // 最近一次完整路径复检的首个失败位姿
};

/**
 * 对加密后的折线路径执行迭代平滑，并对平滑结果进行采样碰撞复检。
 * 首尾位置保持不变。失败时返回原路径，由调用方直接采用原始 Theta* 路径。
 */
class CostmapPathSmoother
{
public:
  using PoseIsFree = std::function<bool (double, double, double)>;

  explicit CostmapPathSmoother(
    const CostmapPathSmootherParameters & parameters = CostmapPathSmootherParameters());

  void setParameters(const CostmapPathSmootherParameters & parameters);

  bool smooth(
    const nav_msgs::msg::Path & input,
    nav_msgs::msg::Path & output,
    const PoseIsFree & pose_is_free,
    SmoothingDiagnostics * diagnostics = nullptr) const;

private:
  bool validate(const nav_msgs::msg::Path & path, const PoseIsFree & pose_is_free,
    SmoothingDiagnostics * diagnostics) const;
  static void setPathOrientations(nav_msgs::msg::Path & path);

  CostmapPathSmootherParameters parameters_;
};

}  // namespace nav2_a_star_rpp

#endif  // NAV2_A_STAR_RPP__COSTMAP_PATH_SMOOTHER_HPP_
