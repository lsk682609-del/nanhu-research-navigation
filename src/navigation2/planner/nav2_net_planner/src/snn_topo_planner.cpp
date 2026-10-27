/**
******************************************************************************
*
* @file       snn_topo_planner.cpp
* @brief      显式 I/u/v 神经元动力学的路网 SNN 首次脉冲规划器。
*
*****************************************************************************/

#include "nav2_net_planner/snn_topo_planner.hpp"

#include "nav2_net_planner/topo_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace snn_topo_detail {

// 原道路 u->v 在反向波网络中成为 v->u，D 仍取原道路 cost。
struct ReverseSynapse
{
    int to = 0;
    double D = 0.0;
};

// 一个在途脉冲。t-now 对应 Python 中突触倒计时 d。
struct SpikeEvent
{
    double t = 0.0;
    int from = 0;
    int to = 0;
    double I = 0.0;
};

struct LaterEventFirst
{
    bool operator()(const SpikeEvent &a, const SpikeEvent &b) const
    {
        if (a.t != b.t)
            return a.t > b.t;
        if (a.to != b.to)
            return a.to > b.to;
        return a.from > b.from;
    }
};

// 与 Python/论文符号直接对应的单个神经元状态。
struct NeuronState
{
    double I = 0.0; // 输入电流
    double u = 0.0; // 膜电位
    bool v = false; // 本次更新是否放电

    bool fired = false; // 是否已经完成首次放电
    double t_last = 0.0;
    double t_fire = std::numeric_limits<double>::infinity();
};

using NeuronTable = std::unordered_map<int, NeuronState>;
using ReverseNetwork = std::unordered_map<int, std::vector<ReverseSynapse>>;
using NextHopTable = std::unordered_map<int, int>;
using EventQueue = std::priority_queue<SpikeEvent, std::vector<SpikeEvent>, LaterEventFirst>;

} // namespace snn_topo_detail

// 这个快照与 TopoMap 的生命周期无关；规划器绝不保存原始 topo 的引用。
struct SnnTopoPlanner::CachedTopology
{
    std::vector<int> nodeIds;
    std::unordered_set<int> nodeIdSet;
    snn_topo_detail::ReverseNetwork network;
    std::size_t ignoredEdgeCount = 0;
};

namespace snn_topo_detail {

// -----------------------------------------------------------------------------
// 1. 参数与路网突触
// -----------------------------------------------------------------------------

bool validDynamics(const SnnDynamicsParameters &p)
{
    const bool allFinite = std::isfinite(p.u_rest)
                        && std::isfinite(p.beta)
                        && std::isfinite(p.theta)
                        && std::isfinite(p.I_spike)
                        && std::isfinite(p.du_dt)
                        && std::isfinite(p.delay_scale);
    if (!allFinite)
        return false;

    // 单个静息神经元收到一个突触脉冲后必须越阈，才能保证首次放电
    // 时间严格等于到目标的最小累计边代价。
    return p.theta > p.u_rest
        && p.beta <= p.u_rest
        && p.I_spike > 0.0
        && p.u_rest + p.I_spike >= p.theta
        && p.du_dt > 0.0
        && p.delay_scale > 0.0;
}

std::vector<int> collectNodeIds(const TopoMap &topo, std::unordered_set<int> &nodeIdSet)
{
    std::vector<int> nodeIds;
    nodeIds.reserve(topo.nodes.size());
    nodeIdSet.clear();
    nodeIdSet.reserve(topo.nodes.size());

    for (const auto &node : topo.nodes) {
        // 与旧版 unordered_map::emplace 的语义一致：重复 ID 只保留第一次。
        if (nodeIdSet.insert(node.id).second)
            nodeIds.push_back(node.id);
    }
    return nodeIds;
}

NeuronTable makeNeurons(const std::vector<int> &nodeIds, const SnnDynamicsParameters &p)
{
    NeuronTable neurons;
    neurons.reserve(nodeIds.size());
    for (int nodeId : nodeIds) {
        NeuronState neuron;
        neuron.u = p.u_rest;
        neurons.emplace(nodeId, neuron);
    }
    return neurons;
}

ReverseNetwork buildReverseNetwork(const TopoMap &topo,
                                   const std::unordered_set<int> &nodeIdSet,
                                   const SnnDynamicsParameters &p,
                                   std::size_t &ignoredEdgeCount)
{
    ReverseNetwork network;
    network.reserve(nodeIdSet.size());

    for (const auto &edge : topo.edges) {
        const bool validType = edge.type == TopoMap::EdgeType::Bidirectional
                            || edge.type == TopoMap::EdgeType::Unidirectional;
        const double edgeCost = static_cast<double>(edge.cost);
        const double D = edgeCost * p.delay_scale;

        if (!validType || !nodeIdSet.count(edge.nodeId1) || !nodeIdSet.count(edge.nodeId2)
            || !std::isfinite(edgeCost) || edgeCost < 0.0 || !std::isfinite(D)) {
            ++ignoredEdgeCount;
            continue;
        }

        // 原道路 nodeId1->nodeId2，计算波由 nodeId2 反向传给 nodeId1。
        network[edge.nodeId2].push_back({edge.nodeId1, D});
        if (edge.type == TopoMap::EdgeType::Bidirectional)
            network[edge.nodeId1].push_back({edge.nodeId2, D});
    }

    return network;
}

// -----------------------------------------------------------------------------
// 2. 显式 I/u/v 神经元更新
// -----------------------------------------------------------------------------

SnnNeuronUpdate updateIuv(NeuronState &neuron,
                          int neuronId,
                          int sourceNeuronId,
                          bool externalStimulus,
                          double t,
                          double incomingI,
                          const SnnDynamicsParameters &p)
{
    // Python 每个空 tick 执行 u=min(u+1,0)。事件驱动版本根据 elapsed
    // 一次完成连续时间恢复，不逐个扫描没有脉冲到达的时钟步。
    const double elapsed = std::max(0.0, t - neuron.t_last);
    if (neuron.u < p.u_rest)
        neuron.u = std::min(p.u_rest, neuron.u + p.du_dt * elapsed);
    else if (neuron.u > p.u_rest)
        neuron.u = p.u_rest;

    SnnNeuronUpdate update;
    update.t = t;
    update.neuronId = neuronId;
    update.sourceNeuronId = sourceNeuronId;
    update.externalStimulus = externalStimulus;

    neuron.I = incomingI;
    update.I = neuron.I;
    update.u_before = neuron.u;
    update.u_plus_I = neuron.u + neuron.I;

    // 核心放电公式：v = 1[u+I >= theta]。
    // fired 是路网 first-spike 规划增加的一次性门控。
    neuron.v = !neuron.fired && update.u_plus_I >= p.theta;
    if (neuron.v) {
        neuron.fired = true;
        neuron.t_fire = t;
        neuron.u = p.beta;
    }

    update.u_after = neuron.u;
    update.v = neuron.v;

    // I 和 v 都是本次事件的瞬时状态；长期状态由 fired/t_fire 保存。
    neuron.I = 0.0;
    neuron.v = false;
    neuron.t_last = t;
    return update;
}

// -----------------------------------------------------------------------------
// 3. 目标刺激与反向脉冲波前
// -----------------------------------------------------------------------------

void emitSpike(int from,
               double t,
               const ReverseNetwork &network,
               const SnnDynamicsParameters &p,
               EventQueue &events)
{
    const auto outgoingIt = network.find(from);
    if (outgoingIt == network.end())
        return;

    for (const ReverseSynapse &synapse : outgoingIt->second) {
        const double arrivalTime = t + synapse.D;
        if (std::isfinite(arrivalTime))
            events.push({arrivalTime, from, synapse.to, p.I_spike});
    }
}

bool runWavefront(int startId,
                  int goalId,
                  const ReverseNetwork &network,
                  const SnnDynamicsParameters &p,
                  NeuronTable &neurons,
                  NextHopTable &nextHop,
                  SnnPlanResult &result)
{
    EventQueue events;

    // 目标在 t=0 接受外部电流，并通过同一个 u+I 阈值公式产生 v。
    NeuronState &goal = neurons.at(goalId);
    const SnnNeuronUpdate goalUpdate = updateIuv(
        goal, goalId, goalId, true, 0.0, p.I_spike, p);
    result.dynamicsTrace.push_back(goalUpdate);
    if (!goalUpdate.v) {
        result.status = SnnPlanResult::Status::InvalidDynamics;
        return false;
    }

    result.firedNeuronCount = 1;
    if (startId == goalId)
        return true;
    emitSpike(goalId, 0.0, network, p, events);

    while (!events.empty()) {
        const SpikeEvent first = events.top();
        events.pop();
        ++result.processedSynapticEventCount;

        // 对应 Python 同一 tick 的 I+=1：合并同一时刻到达同一神经元的电流。
        double I = first.I;
        while (!events.empty() && events.top().t == first.t && events.top().to == first.to) {
            I += events.top().I;
            events.pop();
            ++result.processedSynapticEventCount;
        }

        NeuronState &neuron = neurons.at(first.to);
        const SnnNeuronUpdate update = updateIuv(
            neuron, first.to, first.from, false, first.t, I, p);
        result.dynamicsTrace.push_back(update);
        if (!update.v)
            continue;

        ++result.firedNeuronCount;
        // 波由 first.from 到达 first.to，因此机器人正向下一跳为 from。
        nextHop[first.to] = first.from;

        if (first.to == startId)
            return true;
        emitSpike(first.to, first.t, network, p, events);
    }

    return false;
}

// -----------------------------------------------------------------------------
// 4. 沿首次脉冲来源恢复正向路径
// -----------------------------------------------------------------------------

bool reconstructPath(int startId,
                     int goalId,
                     const NextHopTable &nextHop,
                     std::size_t nodeCount,
                     std::vector<int> &path)
{
    path.clear();
    path.reserve(nodeCount);

    std::unordered_set<int> visited;
    visited.reserve(nodeCount);
    int current = startId;
    path.push_back(current);
    visited.insert(current);

    while (current != goalId) {
        const auto hopIt = nextHop.find(current);
        if (hopIt == nextHop.end() || !visited.insert(hopIt->second).second) {
            path.clear();
            return false;
        }
        current = hopIt->second;
        path.push_back(current);
    }
    return true;
}

} // namespace snn_topo_detail

void SnnTopoPlanner::setTopology(const TopoMap &topo)
{
    auto cached = std::make_shared<CachedTopology>();
    cached->nodeIds = snn_topo_detail::collectNodeIds(topo, cached->nodeIdSet);
    cached->network = snn_topo_detail::buildReverseNetwork(
        topo, cached->nodeIdSet, parameters_, cached->ignoredEdgeCount);
    cachedTopology_ = std::move(cached);
}

void SnnTopoPlanner::clearTopology() noexcept
{
    cachedTopology_.reset();
}

bool SnnTopoPlanner::hasTopology() const noexcept
{
    return static_cast<bool>(cachedTopology_);
}

SnnPlanResult SnnTopoPlanner::plan(const TopoMap &topo, int startId, int goalId) const
{
    // 保留旧接口的行为：该重载只服务本次调用，不污染已有缓存。
    SnnTopoPlanner oneShotPlanner(parameters_);
    oneShotPlanner.setTopology(topo);
    return oneShotPlanner.plan(startId, goalId);
}

SnnPlanResult SnnTopoPlanner::plan(int startId, int goalId) const
{
    SnnPlanResult result;
    const std::shared_ptr<const CachedTopology> cached = cachedTopology_;
    if (!cached) {
        result.status = SnnPlanResult::Status::TopologyNotCached;
        return result;
    }

    // 静态网络已缓存；I/u/v、首次放电时间和事件队列仍须对每次查询重置。
    snn_topo_detail::NeuronTable neurons = snn_topo_detail::makeNeurons(cached->nodeIds, parameters_);

    if (!neurons.count(startId)) {
        result.status = SnnPlanResult::Status::InvalidStart;
        return result;
    }
    if (!neurons.count(goalId)) {
        result.status = SnnPlanResult::Status::InvalidGoal;
        return result;
    }
    if (!snn_topo_detail::validDynamics(parameters_)) {
        result.status = SnnPlanResult::Status::InvalidDynamics;
        return result;
    }

    result.ignoredEdgeCount = cached->ignoredEdgeCount;
    snn_topo_detail::NextHopTable nextHop;
    nextHop.reserve(neurons.size());
    result.dynamicsTrace.reserve(neurons.size());

    if (!snn_topo_detail::runWavefront(
            startId, goalId, cached->network, parameters_, neurons, nextHop, result)) {
        if (result.status != SnnPlanResult::Status::InvalidDynamics)
            result.status = SnnPlanResult::Status::Unreachable;
        return result;
    }

    const snn_topo_detail::NeuronState &start = neurons.at(startId);
    if (!start.fired || !std::isfinite(start.t_fire)
        || !snn_topo_detail::reconstructPath(startId, goalId, nextHop, neurons.size(), result.nodeIds)) {
        result.status = SnnPlanResult::Status::Unreachable;
        return result;
    }

    result.status = SnnPlanResult::Status::Success;
    result.startSpikeTime = start.t_fire;
    result.totalDelay = result.startSpikeTime / parameters_.delay_scale;
    return result;
}
