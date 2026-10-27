/**
******************************************************************************
*
* @file       topo_map.hpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      路网拓扑图（map/node/edge 文本段）解析、序列化与图算法辅助。
*
*****************************************************************************/

#include "nav2_net_planner/snn_topo_planner.hpp"

#include <cctype>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iomanip>

class TopoMap
{
public:
    /// WGS84 椭球参数，与定位模块 LocalCartesian 保持一致。
    static constexpr double kWgs84SemiMajorM = 6378137.0;
    static constexpr double kWgs84Flattening = 1.0 / 298.257223563;
    static constexpr double kWgs84EccentricitySquared =
        kWgs84Flattening * (2.0 - kWgs84Flattening);
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kDegToRad = kPi / 180.0;
    static constexpr double kRadToDeg = 180.0 / kPi;

    static void llaToEcef(double latDeg, double lonDeg, double altM,
                          double *xM, double *yM, double *zM)
    {
        const double lat = latDeg * kDegToRad;
        const double lon = lonDeg * kDegToRad;
        const double sinLat = std::sin(lat);
        const double cosLat = std::cos(lat);
        const double sinLon = std::sin(lon);
        const double cosLon = std::cos(lon);
        const double primeVerticalRadius = kWgs84SemiMajorM /
            std::sqrt(1.0 - kWgs84EccentricitySquared * sinLat * sinLat);
        *xM = (primeVerticalRadius + altM) * cosLat * cosLon;
        *yM = (primeVerticalRadius + altM) * cosLat * sinLon;
        *zM = (primeVerticalRadius * (1.0 - kWgs84EccentricitySquared) + altM) * sinLat;
    }

    static void ecefToLla(double xM, double yM, double zM,
                          double *latDeg, double *lonDeg, double *altM)
    {
        const double lon = std::atan2(yM, xM);
        const double horizontal = std::hypot(xM, yM);
        double lat = std::atan2(
            zM, horizontal * (1.0 - kWgs84EccentricitySquared));
        for (int i = 0; i < 10; ++i) {
            const double sinLat = std::sin(lat);
            const double primeVerticalRadius = kWgs84SemiMajorM /
                std::sqrt(1.0 - kWgs84EccentricitySquared * sinLat * sinLat);
            const double next = std::atan2(
                zM + kWgs84EccentricitySquared * primeVerticalRadius * sinLat,
                horizontal);
            if (std::abs(next - lat) < 1.0e-13) {
                lat = next;
                break;
            }
            lat = next;
        }

        const double sinLat = std::sin(lat);
        const double cosLat = std::cos(lat);
        const double primeVerticalRadius = kWgs84SemiMajorM /
            std::sqrt(1.0 - kWgs84EccentricitySquared * sinLat * sinLat);
        *altM = std::abs(cosLat) > 1.0e-12
            ? horizontal / cosLat - primeVerticalRadius
            : zM / sinLat - primeVerticalRadius * (1.0 - kWgs84EccentricitySquared);
        *latDeg = lat * kRadToDeg;
        *lonDeg = lon * kRadToDeg;
    }

    static void llaToEnu(double latDeg, double lonDeg, double altM, double lat0Deg, double lon0Deg,
                         double alt0M, double *eastM, double *northM, double *upM)
    {
        double x0, y0, z0, x, y, z;
        llaToEcef(lat0Deg, lon0Deg, alt0M, &x0, &y0, &z0);
        llaToEcef(latDeg, lonDeg, altM, &x, &y, &z);

        const double lat0 = lat0Deg * kDegToRad;
        const double lon0 = lon0Deg * kDegToRad;
        const double sinLat0 = std::sin(lat0);
        const double cosLat0 = std::cos(lat0);
        const double sinLon0 = std::sin(lon0);
        const double cosLon0 = std::cos(lon0);
        const double dx = x - x0;
        const double dy = y - y0;
        const double dz = z - z0;

        *eastM = -sinLon0 * dx + cosLon0 * dy;
        *northM = -sinLat0 * cosLon0 * dx - sinLat0 * sinLon0 * dy + cosLat0 * dz;
        *upM = cosLat0 * cosLon0 * dx + cosLat0 * sinLon0 * dy + sinLat0 * dz;
    }

    /// `llaToEnu` 的逆：ENU 先旋转回 ECEF，再转换为 WGS84 经纬高。
    static void enuToLla(double eastM, double northM, double upM, double lat0Deg, double lon0Deg, double alt0M,
                         double *latDeg, double *lonDeg, double *altM)
    {
        double x0, y0, z0;
        llaToEcef(lat0Deg, lon0Deg, alt0M, &x0, &y0, &z0);

        const double lat0 = lat0Deg * kDegToRad;
        const double lon0 = lon0Deg * kDegToRad;
        const double sinLat0 = std::sin(lat0);
        const double cosLat0 = std::cos(lat0);
        const double sinLon0 = std::sin(lon0);
        const double cosLon0 = std::cos(lon0);

        const double x = x0 - sinLon0 * eastM - sinLat0 * cosLon0 * northM +
            cosLat0 * cosLon0 * upM;
        const double y = y0 + cosLon0 * eastM - sinLat0 * sinLon0 * northM +
            cosLat0 * sinLon0 * upM;
        const double z = z0 + cosLat0 * northM + sinLat0 * upM;
        ecefToLla(x, y, z, latDeg, lonDeg, altM);
    }

    enum class NodeType {
        Waypoint = 0,
        Elevator = 1,
        Stair = 2,
        Entrance = 3,
    };

    enum class EdgeType {
        Bidirectional = 0,
        Unidirectional = 1
    };

    struct Node {
        int id;
        NodeType type;
        int mapId;
        float x, y, z;
        /// WGS84 纬度/经度（度）、椭球高（米）；无效时为 NaN（旧文件仅 xyz 时保持 NaN）
        double latDeg = std::numeric_limits<double>::quiet_NaN();
        double lonDeg = std::numeric_limits<double>::quiet_NaN();
        double altM = std::numeric_limits<double>::quiet_NaN();
    };

    struct Edge {
        int id = -1;                     // 默认无效id
        EdgeType type = EdgeType::Bidirectional;
        int mapId = -1;
        int nodeId1 = -1;
        int nodeId2 = -1;
        int obt = 0;
        float cost = 0.0f;

        Edge() = default;

        Edge(int id_, EdgeType type_, int mapId_, int n1, int n2, int obt_, float cost_)
            : id(id_), type(type_), mapId(mapId_), nodeId1(n1), nodeId2(n2), obt(obt_), cost(cost_) {}
    };

    struct SubMap {
        int id;
        std::string name;
    };
    
    std::vector<SubMap> subMaps;
    std::vector<Node>   nodes;
    std::vector<Edge>   edges;

    /// 是否启用经纬度 + ENU 工作流（写入 lla_mode:；关闭时仅用手动平面坐标，不弹原点、编辑里不显示经纬高）
    bool useLlaCoordinates = false;

    /// 局部 ENU 原点（与 origin: 段一致）；仅在 useLlaCoordinates 为真时参与保存与换算
    bool refLlaValid = false;
    double refLatDeg = 0.0;
    double refLonDeg = 0.0;
    double refAltM = 0.0;

    /// 若原点有效且节点经纬高均为有限值，则写入 n.x/y/z（东、北、上，米）
    bool applyEnuFromLla(Node &n) const
    {
        if (!useLlaCoordinates || !refLlaValid)
            return false;
        if (!std::isfinite(n.latDeg) || !std::isfinite(n.lonDeg) || !std::isfinite(n.altM))
            return false;
        double east, north, up;
        llaToEnu(n.latDeg, n.lonDeg, n.altM, refLatDeg, refLonDeg, refAltM, &east, &north, &up);
        n.x = static_cast<float>(east);
        n.y = static_cast<float>(north);
        n.z = static_cast<float>(up);
        return true;
    }

    /// 由 ENU（`n.x` 东、`n.y` 北、`n.z` 上，米）反算并写入 `latDeg/lonDeg/altM`；需已启用经纬度模式且原点有效
    bool applyLlaFromEnu(Node &n) const
    {
        if (!useLlaCoordinates || !refLlaValid)
            return false;
        double la, lo, al;
        enuToLla(static_cast<double>(n.x), static_cast<double>(n.y), static_cast<double>(n.z), refLatDeg, refLonDeg,
                 refAltM, &la, &lo, &al);
        n.latDeg = la;
        n.lonDeg = lo;
        n.altM = al;
        return true;
    }

    /// 按无向图统计连通分量个数（节点为顶点，边连接两端；无边相连的孤立节点各算一个分量）。无节点时为 0。
    int countUndirectedTopoComponents() const
    {
        const std::size_t n = nodes.size();
        if (n == 0)
            return 0;
        std::unordered_map<int, std::size_t> idToIdx;
        idToIdx.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
            idToIdx[nodes[i].id] = i;
        std::vector<std::vector<std::size_t>> adj(n);
        for (const auto &e : edges) {
            const auto it1 = idToIdx.find(e.nodeId1);
            const auto it2 = idToIdx.find(e.nodeId2);
            if (it1 == idToIdx.end() || it2 == idToIdx.end())
                continue;
            const std::size_t a = it1->second;
            const std::size_t b = it2->second;
            if (a == b)
                continue;
            adj[a].push_back(b);
            adj[b].push_back(a);
        }
        std::vector<bool> vis(n, false);
        int comp = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (vis[i])
                continue;
            ++comp;
            std::queue<std::size_t> q;
            q.push(i);
            vis[i] = true;
            while (!q.empty()) {
                const std::size_t u = q.front();
                q.pop();
                for (std::size_t v : adj[u]) {
                    if (!vis[v]) {
                        vis[v] = true;
                        q.push(v);
                    }
                }
            }
        }
        return comp;
    }

    /// 无向意义下是否全图单一连通（无节点时视为无需检查）
    bool isUndirectedTopoConnected() const { return nodes.empty() || countUndirectedTopoComponents() <= 1; }

    static void trimTopoLine(std::string &line)
    {
        while (!line.empty() && std::isspace(static_cast<unsigned char>(line.front())))
            line.erase(line.begin());
        while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back())))
            line.pop_back();
    }

    template <typename Stream>
    bool parseTopoStream(Stream &in)
    {
        subMaps.clear();
        nodes.clear();
        edges.clear();
        useLlaCoordinates = false;
        refLlaValid = false;
        refLatDeg = refLonDeg = refAltM = 0.0;
        bool llaModeParsed = false;

        std::string line;
        enum Section { NONE, MAP, LLA_MODE, ORIGIN, NODE, EDGE };
        Section current = NONE;

        while (std::getline(in, line)) {
            const auto hash = line.find('#');
            if (hash != std::string::npos)
                line = line.substr(0, hash);
            trimTopoLine(line);
            if (line.empty())
                continue;
            if (line.rfind("map:", 0) == 0) {
                current = MAP;
                continue;
            }
            if (line.rfind("lla_mode:", 0) == 0) {
                current = LLA_MODE;
                continue;
            }
            if (line.rfind("origin:", 0) == 0) {
                current = ORIGIN;
                continue;
            }
            if (line.rfind("node:", 0) == 0) {
                current = NODE;
                continue;
            }
            if (line.rfind("edge:", 0) == 0) {
                current = EDGE;
                continue;
            }

            std::istringstream iss(line);
            if (current == MAP) {
                int id;
                std::string name;
                if (iss >> id >> name) {
                    subMaps.push_back({id, name});
                }
            } else if (current == LLA_MODE) {
                int v;
                if (iss >> v) {
                    useLlaCoordinates = (v != 0);
                    llaModeParsed = true;
                }
            } else if (current == ORIGIN) {
                double la, lo, al;
                if (iss >> la >> lo >> al) {
                    refLatDeg = la;
                    refLonDeg = lo;
                    refAltM = al;
                    refLlaValid = true;
                }
            } else if (current == NODE) {
                int id, type, mapId;
                float x, y, z;
                if (iss >> id >> type >> mapId >> x >> y >> z) {
                    Node n{id, static_cast<NodeType>(type), mapId, x, y, z};
                    double la, lo, al;
                    if (iss >> la >> lo >> al) {
                        n.latDeg = la;
                        n.lonDeg = lo;
                        n.altM = al;
                    }
                    nodes.push_back(n);
                }
            } else if (current == EDGE) {
                int id, type, mapId, from, to, obt;
                if (iss >> id >> type >> mapId >> from >> to >> obt) {
                    Edge tmp = {id, static_cast<EdgeType>(type), mapId, from, to, obt, 0.0f};
                    edges.push_back(tmp);
                }
            }
        }

        if (!llaModeParsed && refLlaValid)
            useLlaCoordinates = true;

        // 经纬度模式下以 LLA 为真值，加载时通过 WGS84 椭球的
        // LLA→ECEF→ENU 链路重新生成 xyz，修正旧文件中的平面近似坐标。
        std::size_t reprojectedNodeCount = 0;
        if (useLlaCoordinates && refLlaValid) {
            for (auto &node : nodes) {
                if (applyEnuFromLla(node))
                    ++reprojectedNodeCount;
            }
        }
        // 边权依赖 ENU 坐标，必须在节点重新投影后计算。
        for (auto &edge : edges)
            edgeCost(edge);

        std::cout << "Loaded " << subMaps.size() << " maps, "
                  << nodes.size() << " nodes, "
                  << edges.size() << " edges";
        if (reprojectedNodeCount > 0)
            std::cout << ", reprojected " << reprojectedNodeCount << " LLA nodes with WGS84";
        std::cout << '\n';
        return true;
    }

    bool loadTopoMapFromString(const std::string &text)
    {
        std::istringstream iss(text);
        return parseTopoStream(iss);
    }

    bool loadTopoMap(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open map file: " << path << std::endl;
            return false;
        }
        return parseTopoStream(file);
    }

    std::string serializeTopoText() const
    {
        std::ostringstream oss;
        oss << std::setprecision(15);
        oss << "map:\n";
        for (const auto &sm : subMaps)
            oss << sm.id << ' ' << sm.name << '\n';
        oss << "lla_mode:\n" << (useLlaCoordinates ? 1 : 0) << '\n';
        if (useLlaCoordinates && refLlaValid) {
            oss << "origin:\n";
            oss << refLatDeg << ' ' << refLonDeg << ' ' << refAltM << '\n';
        }
        oss << "node:\n";
        for (const auto &n : nodes) {
            oss << n.id << ' ' << static_cast<int>(n.type) << ' ' << n.mapId << ' ' << n.x << ' ' << n.y << ' '
                << n.z;
            if (std::isfinite(n.latDeg) && std::isfinite(n.lonDeg) && std::isfinite(n.altM))
                oss << ' ' << n.latDeg << ' ' << n.lonDeg << ' ' << n.altM;
            else
                oss << " nan nan nan";
            oss << '\n';
        }
        oss << "edge:\n";
        for (const auto &e : edges)
            oss << e.id << ' ' << static_cast<int>(e.type) << ' ' << e.mapId << ' ' << e.nodeId1 << ' '
                << e.nodeId2 << ' ' << e.obt << '\n';
        return oss.str();
    }

    bool empty() const { return subMaps.empty() && nodes.empty() && edges.empty(); }

    // 返回对应edge，找不到返回默认构造的edge(id==-1表示无效)
    Edge getEdgeByNodeId(int nodeId1, int nodeId2) const 
    {
        for (const auto& edge : edges) {
            if ((edge.nodeId1 == nodeId1 && edge.nodeId2 == nodeId2) ||
                (edge.nodeId1 == nodeId2 && edge.nodeId2 == nodeId1)) {
                return edge;
            }
        }
        return Edge{};  // 默认构造的edge
    }

    void edgeCost(Edge &e)
    {
        float dis = nodeDis(e.nodeId1, e.nodeId2);
        e.cost = dis;
    }

    float nodeDis(int id1, int id2) const
    {
        const Node *pa = nullptr;
        const Node *pb = nullptr;
        for (const auto &n : nodes) {
            if (n.id == id1)
                pa = &n;
            if (n.id == id2)
                pb = &n;
        }
        if (!pa || !pb)
            return 0.0f;
        float dx = pa->x - pb->x;
        float dy = pa->y - pb->y;
        float dz = pa->z - pb->z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool nearestNode(double x, double y, double z, Node &nearest, double &minDist) const
    {
        if (nodes.empty()) {
            return false;
        }

        minDist = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto &node : nodes) {
            double dx = node.x - x;
            double dy = node.y - y;
            double dz = node.z - z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < minDist) {
                minDist = dist;
                nearest = node;
                found = true;
            }
        }

        return found;
    }

    /// 规划用临时虚拟节点 id（与文件内正 id 区分）；勿写入拓扑文件。
    static constexpr int kSnapVirtualStartId = -971000001;
    static constexpr int kSnapVirtualGoalId = -971000002;

    /** 点到线段 AB 的投影（t∈[0,1] 从 A 沿向 B），返回 false 若 AB 退化。 */
    static bool projectPointToSegment(
        double px, double py, double pz,
        double ax, double ay, double az,
        double bx, double by, double bz,
        double & t, double & qx, double & qy, double & qz)
    {
        const double abx = bx - ax;
        const double aby = by - ay;
        const double abz = bz - az;
        const double ab2 = abx * abx + aby * aby + abz * abz;
        if (ab2 < 1e-18) {
            return false;
        }
        t = (abx * (px - ax) + aby * (py - ay) + abz * (pz - az)) / ab2;
        t = std::clamp(t, 0.0, 1.0);
        qx = ax + t * abx;
        qy = ay + t * aby;
        qz = az + t * abz;
        return true;
    }

    /** 到最近边的垂距与投影；无有效边时返回 false。 */
    bool nearestEdgeProjection(
        double x, double y, double z,
        double & outDist,
        Edge & outEdge,
        double & outT,
        double & outQx, double & outQy, double & outQz) const
    {
        outDist = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto & e : edges) {
            const Node *na = nullptr;
            const Node *nb = nullptr;
            for (const auto & n : nodes) {
                if (n.id == e.nodeId1) {
                    na = &n;
                }
                if (n.id == e.nodeId2) {
                    nb = &n;
                }
            }
            if (!na || !nb) {
                continue;
            }
            const double ax = static_cast<double>(na->x);
            const double ay = static_cast<double>(na->y);
            const double az = static_cast<double>(na->z);
            const double bx = static_cast<double>(nb->x);
            const double by = static_cast<double>(nb->y);
            const double bz = static_cast<double>(nb->z);
            double t = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
            if (!projectPointToSegment(x, y, z, ax, ay, az, bx, by, bz, t, qx, qy, qz)) {
                continue;
            }
            const double dx = x - qx;
            const double dy = y - qy;
            const double dz = z - qz;
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d < outDist) {
                outDist = d;
                outEdge = e;
                outT = t;
                outQx = qx;
                outQy = qy;
                outQz = qz;
                found = true;
            }
        }
        return found;
    }

    /**
     * 将 (x,y,z) 吸附到路网：若最近节点更近或等距则用节点；否则用最近边上的投影点作为虚拟节点。
     * virtualId 为本次规划用的临时 id；isStartSnap 决定虚拟边方向（起点：从虚拟连向端点；终点：从端点连向虚拟）。
     * extraAdj 仅写入与虚拟点相关的附加邻接（不修改 edges）。
     */
    bool snapPointForPlanning(
        double x, double y, double z, double maxDistance,
        int virtualId,
        bool isStartSnap,
        bool edgeSnap,
        int & graphId,
        Node & snapGeom,
        std::unordered_map<int, std::vector<std::pair<int, float>>> & extraAdj) const
    {
        extraAdj.clear();
        Node nn{};
        double dn = 0.0;
        if (!nearestNode(x, y, z, nn, dn)) {
            return false;
        }

        Edge bestE{};
        double de = std::numeric_limits<double>::max();
        double t_geom = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
        const bool have_edge = edgeSnap ? nearestEdgeProjection(x, y, z, de, bestE, t_geom, qx, qy, qz) : false;

        if (std::min(dn, have_edge ? de : std::numeric_limits<double>::max()) > maxDistance) {
            return false;
        }

        if (!have_edge || dn <= de) {
            graphId = nn.id;
            snapGeom = nn;
            return true;
        }

        const Node *na = nullptr;
        const Node *nb = nullptr;
        for (const auto & n : nodes) {
            if (n.id == bestE.nodeId1) {
                na = &n;
            }
            if (n.id == bestE.nodeId2) {
                nb = &n;
            }
        }
        if (!na || !nb) {
            graphId = nn.id;
            snapGeom = nn;
            return true;
        }

        const double Ld = static_cast<double>(nodeDis(bestE.nodeId1, bestE.nodeId2));
        if (Ld < 1e-9) {
            graphId = nn.id;
            snapGeom = nn;
            return true;
        }

        graphId = virtualId;
        snapGeom.id = virtualId;
        snapGeom.type = na->type;
        snapGeom.mapId = bestE.mapId;
        snapGeom.x = static_cast<float>(qx);
        snapGeom.y = static_cast<float>(qy);
        snapGeom.z = static_cast<float>(qz);

        const float t_f = static_cast<float>(t_geom);
        const float Lf = static_cast<float>(Ld);
        const float dist_to_n1 = t_f * Lf;
        const float dist_to_n2 = (1.0f - t_f) * Lf;

        if (isStartSnap) {
            if (bestE.type == EdgeType::Bidirectional) {
                extraAdj[virtualId].emplace_back(bestE.nodeId1, dist_to_n1);
                extraAdj[virtualId].emplace_back(bestE.nodeId2, dist_to_n2);
            } else {
                extraAdj[virtualId].emplace_back(bestE.nodeId2, dist_to_n2);
            }
        } else {
            if (bestE.type == EdgeType::Bidirectional) {
                extraAdj[bestE.nodeId1].emplace_back(virtualId, dist_to_n1);
                extraAdj[bestE.nodeId2].emplace_back(virtualId, dist_to_n2);
            } else {
                extraAdj[bestE.nodeId1].emplace_back(virtualId, dist_to_n1);
            }
        }
        return true;
    }

    std::vector<int> astarExtended(
        int startId, int goalId,
        const std::unordered_map<int, Node> * extraNodes,
        const std::unordered_map<int, std::vector<std::pair<int, float>>> * extraAdj) const
    {
        std::unordered_map<int, std::vector<std::pair<int, float>>> adj;
        buildAdjacency(edges, adj);
        if (extraAdj) {
            for (const auto & kv : *extraAdj) {
                auto & dst = adj[kv.first];
                for (const auto & pr : kv.second) {
                    dst.push_back(pr);
                }
            }
        }

        std::unordered_map<int, Node> nodeMap;
        for (const auto & node : nodes) {
            nodeMap[node.id] = node;
        }
        if (extraNodes) {
            for (const auto & kv : *extraNodes) {
                nodeMap[kv.first] = kv.second;
            }
        }

        if (!nodeMap.count(startId) || !nodeMap.count(goalId)) {
            return {};
        }

        std::unordered_set<int> allIds;
        for (const auto & kv : nodeMap) {
            allIds.insert(kv.first);
        }
        for (const auto & kv : adj) {
            allIds.insert(kv.first);
            for (const auto & pr : kv.second) {
                allIds.insert(pr.first);
            }
        }

        std::unordered_map<int, float> gScore;
        std::unordered_map<int, float> fScore;
        std::unordered_map<int, int> cameFrom;

        for (int id : allIds) {
            gScore[id] = std::numeric_limits<float>::infinity();
            fScore[id] = std::numeric_limits<float>::infinity();
        }

        auto cmp = [&](int a, int b) { return fScore[a] > fScore[b]; };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> openSet(cmp);

        gScore[startId] = 0.0f;
        fScore[startId] = heuristic(nodeMap[startId], nodeMap[goalId]);
        openSet.push(startId);

        std::unordered_set<int> openSetLookup{startId};

        while (!openSet.empty()) {
            int current = openSet.top();
            openSet.pop();
            openSetLookup.erase(current);

            if (current == goalId) {
                std::vector<int> path;
                while (cameFrom.count(current)) {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                path.push_back(startId);
                std::reverse(path.begin(), path.end());
                return path;
            }

            const auto it_adj = adj.find(current);
            if (it_adj == adj.end()) {
                continue;
            }
            for (const auto & [neighbor, cost] : it_adj->second) {
                if (!nodeMap.count(neighbor)) {
                    continue;
                }
                float tentativeG = gScore[current] + cost;
                if (tentativeG < gScore[neighbor]) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + heuristic(nodeMap[neighbor], nodeMap[goalId]);
                    if (!openSetLookup.count(neighbor)) {
                        openSet.push(neighbor);
                        openSetLookup.insert(neighbor);
                    }
                }
            }
        }

        return {};
    }

    std::vector<int> astar(int startId, int goalId) const
    {
        return astarExtended(startId, goalId, nullptr, nullptr);
    }

    /**
     * Snap start/goal + SNN 波前规划 → 返回节点 ID 序列（失败返回空）。
     * snn_result_out 非空时输出本次 SNN 规划的完整结果（状态、耗时统计、
     * 路径节点等），供调用方做算法测试日志；为 nullptr 时行为与原来一致。
     */
    std::vector<int> planNodeIds(
        double sx, double sy, double sz,
        double gx, double gy, double gz,
        double maxSnapDist, bool edgeSnap,
        std::unordered_map<int, Node> & extra_nodes_out,
        std::unordered_map<int, std::vector<std::pair<int, float>>> & extra_adj_out,
        SnnPlanResult * snn_result_out = nullptr) const
    {
        extra_nodes_out.clear();
        extra_adj_out.clear();

        int start_id = 0;
        Node start_geom{};
        std::unordered_map<int, std::vector<std::pair<int, float>>> start_adj;
        if (!snapPointForPlanning(sx, sy, sz, maxSnapDist,
                kSnapVirtualStartId, true, edgeSnap,
                start_id, start_geom, start_adj))
            return {};
        for (const auto & kv : start_adj) {
            auto & dst = extra_adj_out[kv.first];
            dst.insert(dst.end(), kv.second.begin(), kv.second.end());
        }
        if (start_id == kSnapVirtualStartId)
            extra_nodes_out[start_id] = start_geom;

        int goal_id = 0;
        Node goal_geom{};
        std::unordered_map<int, std::vector<std::pair<int, float>>> goal_adj;
        if (!snapPointForPlanning(gx, gy, gz, maxSnapDist,
                kSnapVirtualGoalId, false, edgeSnap,
                goal_id, goal_geom, goal_adj))
            return {};
        for (const auto & kv : goal_adj) {
            auto & dst = extra_adj_out[kv.first];
            dst.insert(dst.end(), kv.second.begin(), kv.second.end());
        }
        if (goal_id == kSnapVirtualGoalId)
            extra_nodes_out[goal_id] = goal_geom;

        if (start_id == goal_id)
            return {start_id};

        // SNN 波前规划器替换 A*
        SnnTopoPlanner snn;
        auto result = snn.plan(*this, start_id, goal_id);
        // 记录本次 SNN 结果，供调用方在 SNN 调用点前后打测试日志。
        if (snn_result_out) {
            *snn_result_out = result;
        }
        if (result.success())
            return result.nodeIds;
        return {};
    }

private:
    void buildAdjacency(const std::vector<Edge>& edges,
                        std::unordered_map<int, std::vector<std::pair<int, float>>>& adj) const
    {
        adj.clear();
        for (const auto& edge : edges) {
            adj[edge.nodeId1].emplace_back(edge.nodeId2, edge.cost);
            if (edge.type == EdgeType::Bidirectional) {
                adj[edge.nodeId2].emplace_back(edge.nodeId1, edge.cost);
            }
        }
    }

    float heuristic(const Node& a, const Node& b) const
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};
