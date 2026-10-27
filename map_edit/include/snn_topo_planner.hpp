/**
******************************************************************************
*
* @file       snn_topo_planner.hpp
* @brief      基于首次脉冲时间的静态路网 SNN 波前规划器。
*
*****************************************************************************/

#pragma once

#include <cstddef>
#include <vector>

class TopoMap;

/**
 * 从 Python 栅格版本迁移而来的连续时间、事件驱动显式神经元参数。
 *
 * 默认值对应：静息膜电位 u=0、阈值 1、单位输入电流 I=1、放电后
 * u=beta=-10；未放电时膜电位以每仿真时间单位 1 的速度恢复到 0。
 * 它保留 Python 的阈值/复位语义，但不逐整数 tick 扫描整个网络。
 */
struct SnnDynamicsParameters
{
    double u_rest = 0.0;    // 静息膜电位
    double beta = -10.0;    // 放电后的复位膜电位
    double theta = 1.0;     // 放电阈值
    double I_spike = 1.0;   // 单个突触脉冲注入的电流
    double du_dt = 1.0;     // 未放电时 u 向 u_rest 恢复的速率
    double delay_scale = 1.0; // 一个 edge.cost 对应的连续仿真时间
};

/** 一次显式神经元更新记录，可用于检查或后续可视化 I、u、v。 */
struct SnnNeuronUpdate
{
    double t = 0.0;
    int neuronId = 0;
    int sourceNeuronId = 0;
    bool externalStimulus = false;
    double I = 0.0;          // 本次输入电流
    double u_before = 0.0;   // 输入前膜电位
    double u_plus_I = 0.0;   // 阈值判定量 u+I
    double u_after = 0.0;    // 放电时为 beta，否则为恢复后的 u
    bool v = false;          // 本次是否发放脉冲
};

struct SnnPlanResult
{
    enum class Status {
        Success,
        InvalidStart,
        InvalidGoal,
        InvalidDynamics,
        Unreachable,
    };

    Status status = Status::Unreachable;
    std::vector<int> nodeIds;             // Success 时为 start 到 goal 的节点序列。
    double totalDelay = 0.0;              // 路径上原始 edge.cost 之和。
    double startSpikeTime = 0.0;           // 起点神经元的首次放电仿真时间。
    std::size_t firedNeuronCount = 0;      // 本次波前中真正发放过脉冲的节点数。
    std::size_t processedSynapticEventCount = 0; // 已交付的突触脉冲数量。
    std::size_t ignoredEdgeCount = 0;      // 端点、类型或延迟无效而被跳过的边数。
    std::vector<SnnNeuronUpdate> dynamicsTrace;  // 目标刺激及每次神经元事件更新。

    bool success() const { return status == Status::Success; }
};

/**
 * 每个路网节点视为一个神经元，每条有向边的 cost 视为突触延迟。
 * 目标节点在 t=0 接受外部电流，经 u+I>=threshold 判定后放电；波前
 * 沿原图的反向邻接传播。每个节点显式更新输入电流、膜电位、放电和
 * beta 复位。为保证 first-spike 最短路性质，C++ 规划器增加了一次性
 * 放电门控：每个节点只有首次放电会继续传播，不复现 Python 的重复放电。
 *
 * 支持有限的非负延迟（包括零延迟）。端点不存在、类型非法或延迟为
 * 负数/NaN/Inf 的边会被跳过并计入 ignoredEdgeCount。等时脉冲按节点
 * ID 排序，因此相同输入会得到确定的路径。
 */
class SnnTopoPlanner final
{
public:
    SnnTopoPlanner() = default;
    explicit SnnTopoPlanner(const SnnDynamicsParameters &parameters)
        : parameters_(parameters) {}

    SnnPlanResult plan(const TopoMap &topo, int startId, int goalId) const;

    const SnnDynamicsParameters &parameters() const { return parameters_; }

private:
    SnnDynamicsParameters parameters_;
};
