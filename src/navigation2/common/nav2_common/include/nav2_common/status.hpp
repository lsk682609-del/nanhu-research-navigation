// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#ifndef NAV2_COMMON__STATUS_HPP_
#define NAV2_COMMON__STATUS_HPP_

namespace nav2_common
{

enum class ExecStatus : int {
  // 通用
  OK = 0,

  // waypoint_navigator (50-99)
  WPT_NO_ROBOT_POS     = 51,  // TF 获取机器人位姿失败
  WPT_FOLLOW_NOT_READY = 53,  // FollowPath action 服务未就绪
  WPT_SLICE_TOO_SHORT  = 55,  // 路径切片不足2个点
  WPT_OFFSET_TOO_LARGE = 56,  // 机器人偏离切片起点过远

  // net_navigator (100-149)
  NET_NO_ROBOT_POS      = 101, // TF 获取机器人位姿失败
  NET_FOLLOW_NOT_READY  = 103, // FollowPath action 服务未就绪
  NET_SLICE_TOO_SHORT   = 105, // 路径切片不足2个点
  NET_PLANNING_TIMEOUT  = 106, // 规划请求超时
  NET_COMPUTE_FAIL      = 107, // ComputePathToPose 失败

  // input_bridge (150-199)
  BRIDGE_NO_GPS_FIX        = 150, // GPS 无固定解
  BRIDGE_ORIGIN_UNVERIFIED = 151, // GPS 原点未验证
};

}  // namespace nav2_common

#endif  // NAV2_COMMON__STATUS_HPP_