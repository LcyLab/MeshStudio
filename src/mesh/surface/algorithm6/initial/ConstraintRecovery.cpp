/** @file ConstraintRecovery.cpp
 * @brief 原生严格相交、顶点星追踪和 FIFO 翻边恢复。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "ConstraintRecovery.h"
#include "mesh/surface/algorithm6/data/HalfEdgeTopology.h"
#include "geometry/predicates/GeomPredicates.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <exception>
#include <map>
#include <set>
namespace mesher::gmshReference {
/// 普通单 UV 图表的 BDS localId -> 物理身份 -> 初始顶点池索引转换。
OperationStatus prepareInitialConstraints(const FaceChart& chart, const WalkTriangulation& mesh,
    std::vector<InitialConstraint>& output, const RunControl& control)
{
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"initial constraint mapping cancelled"}}; };
        auto fail = [] { throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid single-copy initial constraint identity"}}; };
        check();
        if(chart.route != FaceChartRoute::General || mesh.vertices.size() < 4 || chart.vertices.size() != mesh.vertices.size()-4)
            throw OperationFailure{{MeshingStatus::InvalidInput,"initial constraint mapping requires ordinary single-copy chart"}};
        if(mesh.vertices.size() > control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"initial constraint mapping node budget exceeded"}};
        std::map<std::uint64_t,int> pool;
        for(std::size_t i = 0; i < mesh.vertices.size(); ++i) {
            check(); const auto physical = mesh.vertices[i].physicalId;
            if(i < 4) { if(physical != 0) fail(); }
            else if(physical == 0 || !pool.emplace(physical,int(i)).second) fail();
        }
        std::map<int,int> local; std::set<std::uint64_t> identities;
        for(const auto& p : chart.vertices) {
            check(); const auto found = pool.find(p.physicalId);
            if(p.localId < 0 || found == pool.end() || !identities.insert(p.physicalId).second || !local.emplace(p.localId,found->second).second) fail();
        }
        std::vector<InitialConstraint> mapped;
        for(const auto& c : chart.constraints) {
            check(); const auto from = local.find(c.from), to = local.find(c.to);
            if(from == local.end() || to == local.end() || c.curveEntity <= 0) fail();
            mapped.push_back({from->second,to->second,c.curveEntity});
        }
        check(); output = std::move(mapped); return {};
    } catch(const OperationFailure& error) { return error.status; }
      catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
}
/// 判断约束恢复中的两个 UV 线段是否相交。
bool intersectRecoverySegments(Point2D a, Point2D b, Point2D c, Point2D d)
{
    const double s0 = orient2dValue(a,b,c), s1 = orient2dValue(a,b,d);
    if(!std::isfinite(s0) || !std::isfinite(s1)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite recovery orientation"}};
    if(s0*s1 >= 0.) return false;
    const double t0 = orient2dValue(c,d,a), t1 = orient2dValue(c,d,b);
    if(!std::isfinite(t0) || !std::isfinite(t1)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite recovery orientation"}};
    return !(t0*t1 >= 0.); // 保留数值乘积的负零/无穷语义，不改成符号异号。
}
namespace {
class ConstraintRecoverer {
public:
    WalkTriangulation mesh;
    /// 保存运行控制，准备执行初始剖分约束恢复。
    explicit ConstraintRecoverer(const RunControl& control) : control_(control) {}
    /// 检查取消回调，取消时终止当前算法阶段。
    void checkCancellation() const {
        if(control_.keepGoing && !control_.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"initial constraint recovery cancelled"}};
    }
    /// 检查输入数据是否满足当前算法的拓扑要求。
    void validateInput(const WalkTriangulation& input) const {
        const auto status = validateHalfEdgeTopology(input,control_);
        if(!status.complete()) throw OperationFailure{status};
    }
    /// 判断指定节点组成的两条参数线段是否相交。
    bool intersects(int a, int b, int c, int d) const {
        return intersectRecoverySegments(mesh.vertices[a].uv,mesh.vertices[b].uv,mesh.vertices[c].uv,mesh.vertices[d].uv);
    }
    /// 沿节点星形邻域收集目标约束穿过的半边。
    int collectCrossedEdges(int from, int to, std::vector<int>& crossed) const {
        const int first = mesh.vertices[from].incidentHalfEdge;
        int edge = first;
        do {
            checkCancellation();
            if(edge < 0) throw OperationFailure{{MeshingStatus::InvalidTopology,"native recovery star reached missing boundary opposite"}};
            const int next = mesh.halfEdges[edge].next, last = mesh.halfEdges[next].next;
            const int v1 = mesh.halfEdges[next].origin, v2 = mesh.halfEdges[last].origin;
            if(v1 == to || v2 == to) return 0;
            if(intersects(from,to,v1,v2)) { crossed.push_back(next); break; }
            edge = mesh.halfEdges[last].opposite;
        } while(edge != first);
        if(crossed.empty()) return -1;
        for(;;) {
            checkCancellation(); edge = mesh.halfEdges[crossed.back()].opposite;
            if(edge < 0) return -2;
            edge = mesh.halfEdges[edge].next;
            int next = mesh.halfEdges[edge].next;
            if(mesh.halfEdges[next].origin == to) break;
            if(intersects(from,to,mesh.halfEdges[edge].origin,mesh.halfEdges[next].origin)) {
                if(std::find(crossed.begin(),crossed.end(),edge) != crossed.end())
                    throw OperationFailure{{MeshingStatus::NonConverged,"native recovery crossing path repeats"}};
                crossed.push_back(edge);
            }
            else {
                edge = next; next = mesh.halfEdges[edge].next;
                if(mesh.halfEdges[next].origin == to) break;
                if(intersects(from,to,mesh.halfEdges[edge].origin,mesh.halfEdges[next].origin)) {
                    if(std::find(crossed.begin(),crossed.end(),edge) != crossed.end())
                        throw OperationFailure{{MeshingStatus::NonConverged,"native recovery crossing path repeats"}};
                    crossed.push_back(edge);
                }
                else return -3;
            }
        }
        return int(crossed.size());
    }
    /// 在节点邻域查找具有指定方向的半边。
    int findDirectedHalfEdge(int from, int to) const {
        const int first = mesh.vertices[from].incidentHalfEdge;
        int edge = first;
        do {
            checkCancellation();
            if(mesh.halfEdges[mesh.halfEdges[edge].next].origin == to) return edge;
            edge = mesh.halfEdges[edge].opposite;
            if(edge < 0) return -1;
            edge = mesh.halfEdges[edge].next;
        } while(edge != first);
        return -1;
    }
    /// 通过局部换边恢复一条约束，并记录恢复过程。
    InitialConstraintDecision recoverOneConstraint(InitialConstraint constraint) {
        InitialConstraintDecision decision; decision.constraint = constraint;
        const int from = constraint.fromVertex, to = constraint.toVertex;
        decision.nativeResult = collectCrossedEdges(from,to,decision.crossedEdges);
        if(decision.nativeResult < 0) return decision;
        std::deque<int> queue(decision.crossedEdges.begin(),decision.crossedEdges.end());
        std::size_t withoutSwap = 0;
        while(!queue.empty()) {
            checkCancellation(); const int edge = queue.front(); queue.pop_front();
            const int opposite = mesh.halfEdges[edge].opposite;
            if(opposite < 0) throw OperationFailure{{MeshingStatus::InvalidTopology,"native recovery flip has no opposite"}};
            const int e1 = mesh.halfEdges[edge].next, e2 = mesh.halfEdges[e1].next;
            const int o1 = mesh.halfEdges[opposite].next, o2 = mesh.halfEdges[o1].next;
            if(intersects(mesh.halfEdges[edge].origin,mesh.halfEdges[e1].origin,mesh.halfEdges[e2].origin,mesh.halfEdges[o2].origin)) {
                const bool stillIntersect = intersects(from,to,mesh.halfEdges[e2].origin,mesh.halfEdges[o2].origin);
                swapHalfEdgeDiagonal(mesh,edge); withoutSwap = 0;
                if(stillIntersect) queue.push_back(edge);
            }
            else {
                queue.push_back(edge);
                if(++withoutSwap >= queue.size())
                    throw OperationFailure{{MeshingStatus::NonConverged,"native recovery queue has no flippable diagonal"}};
            }
        }
        decision.recoveredHalfEdge = findDirectedHalfEdge(from,to);
        if(decision.recoveredHalfEdge >= 0) {
            auto& edge = mesh.halfEdges[decision.recoveredHalfEdge]; edge.curveEntity = constraint.curveEntity;
            if(edge.opposite >= 0) mesh.halfEdges[edge.opposite].curveEntity = constraint.curveEntity;
        }
        return decision;
    }
private:
    const RunControl& control_;
};
}
/// 对初始剖分逐条恢复参数约束边。
ConstraintRecoveryResult recoverConstraintEdges(const WalkTriangulation& input,
    const std::vector<InitialConstraint>& constraints, WalkTriangulation& output, const RunControl& control)
{
    ConstraintRecoveryResult result;
    try {
        ConstraintRecoverer recoverer(control); recoverer.checkCancellation();
        if(input.vertices.size() > control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"initial recovery node budget exceeded"}};
        recoverer.validateInput(input);
        for(const auto& c : constraints) if(c.fromVertex < 0 || c.toVertex < 0 || c.curveEntity <= 0 ||
            std::size_t(c.fromVertex) >= input.vertices.size() || std::size_t(c.toVertex) >= input.vertices.size())
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid initial recovery constraint"}};
        recoverer.mesh = input;
        for(const auto& c : constraints) { recoverer.checkCancellation(); result.decisions.push_back(recoverer.recoverOneConstraint(c)); }
        recoverer.checkCancellation(); output = std::move(recoverer.mesh);
    } catch(const OperationFailure& error) { result.status = error.status; result.decisions.clear(); }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; result.decisions.clear(); }
    return result;
}
}
