/** @file BdsConstraintRecovery.cpp
 * @brief BDS恢复的数值求交和原池扫描；连接交换交给BdsTopology。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsConstraintRecovery.h"
#include "geometry/predicates/GeomPredicates.h"
#include <algorithm>
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
/// BDS::Intersect_Edges_2d的原始double sys2x2及闭区间[0,1]。
BdsSegmentIntersection intersectBdsRecoverySegments(Point2D a,Point2D b,Point2D c,Point2D d)
{
    const double m00 = b.x-a.x, m01 = -(d.x-c.x), m10 = b.y-a.y, m11 = -(d.y-c.y);
    const double r0 = c.x-a.x, r1 = c.y-a.y;
    const double norm = std::pow(m00,2)+std::pow(m11,2)+std::pow(m01,2)+std::pow(m10,2);
    const double det = m00*m11-m10*m01;
    BdsSegmentIntersection result;
    if(norm == 0. || std::abs(det)/norm < 1.e-16) return result;
    const double inverse = 1./det;
    result.parameters.x = (r0*m11-m01*r1)*inverse;
    result.parameters.y = (m00*r1-m10*r0)*inverse;
    result.systemSolved = true;
    result.intersects = result.parameters.x >= 0. && result.parameters.x <= 1. && result.parameters.y >= 0. && result.parameters.y <= 1.;
    return result;
}
/// 原生swap_edge公共保护和BDS_SwapEdgeTestRecover的面积数值乘积>0。
bool canSwapBdsRecoveryEdge(const BdsMesh& mesh,int edge,const BdsEdgeNeighborhood& n,bool force)
{
    if(edge < 0 || std::size_t(edge) >= mesh.edges.size()) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid BDS recovery edge"}};
    const auto& e = mesh.edges[edge];
    if(e.deleted || e.faces.size() != 2 || e.classification.dimension == 1) return false;
    const auto& first = mesh.points.at(n.first); const auto& second = mesh.points.at(n.second);
    const auto& op0 = mesh.points.at(n.oppositeNodes[0]); const auto& op1 = mesh.points.at(n.oppositeNodes[1]);
    if(!force && !first.configurationModified && !second.configurationModified && !op0.configurationModified && !op1.configurationModified) return false;
    const double a = orient2dValue(op0.uv,first.uv,op1.uv), b = orient2dValue(op0.uv,op1.uv,second.uv);
    if(!(a*b > 0.)) return false;
    return n.sideEdges[0] != n.sideEdges[2] && n.sideEdges[1] != n.sideEdges[3];
}
/// 在 BDS 网格中用换边恢复单条指定约束。
BdsRecoveryResult recoverBdsConstraintEdge(const BdsMesh& input,int first,int second,BdsMesh& output,
    const std::vector<BdsRecoveryConstraint>& registered,const RunControl& control)
{
    BdsRecoveryResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"BDS recovery cancelled"}}; };
        check(); const auto valid = validateBdsTopology(input,control); if(!valid.complete()) throw OperationFailure{valid};
        if(first == second || input.points.find(first) == input.points.end() || input.points.find(second) == input.points.end())
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid BDS recovery endpoint identities"}};
        auto key = [](int a,int b) { return std::make_pair(std::min(a,b),std::max(a,b)); };
        std::map<std::pair<int,int>,BdsRecoveryConstraint> requests,failed;
        for(auto request : registered) { const auto pair = key(request.first,request.second); request.first = pair.first; request.second = pair.second; requests.emplace(pair,request); }
        BdsMesh mesh = input;
        const auto existing = findBdsEdge(mesh,first,second);
        if(existing >= 0) result.recoveredEdge = existing;
        else {
            const auto from = mesh.points.at(first).uv, to = mesh.points.at(second).uv;
            int iteration = 0;
            while(true) {
                check(); BdsRecoveryScan scan; bool selfIntersection = false;
                for(std::size_t i = 0; i < mesh.edges.size(); ++i) {
                    check(); const auto& e = mesh.edges[i];
                    if(e.deleted || e.first == first || e.first == second || e.second == first || e.second == second) continue;
                    if(!intersectBdsRecoverySegments(mesh.points.at(e.first).uv,mesh.points.at(e.second).uv,from,to).intersects) continue;
                    const auto collision = requests.find(key(e.first,e.second));
                    if(collision != requests.end()) {
                        const auto current = requests.find(key(first,second));
                        if(current == requests.end()) throw OperationFailure{{MeshingStatus::InvalidInput,"BDS self-intersection target absent from registered constraints"}};
                        failed.emplace(current->first,current->second); failed.emplace(collision->first,collision->second); selfIntersection = true;
                    }
                    scan.intersectedEdges.push_back(int(i));
                }
                result.scans.push_back(std::move(scan)); auto& currentScan = result.scans.back();
                if(selfIntersection) break;
                if(currentScan.intersectedEdges.empty() || iteration > 300) {
                    result.recoveredEdge = findBdsEdge(mesh,first,second); result.nativeFatal = result.recoveredEdge < 0; break;
                }
                bool swapped = false;
                for(int edge : currentScan.intersectedEdges) {
                    check(); const auto neighborhood = getBdsEdgeNeighborhood(mesh,edge);
                    const bool allowed = neighborhood && canSwapBdsRecoveryEdge(mesh,edge,*neighborhood);
                    currentScan.attempts.emplace_back(edge,allowed);
                    if(allowed) { swapBdsEdgeTopology(mesh,edge,*neighborhood); swapped = true; break; }
                }
                if(!swapped) { result.nativeFatal = true; break; }
                ++iteration;
            }
        }
        for(const auto& entry : failed) result.notRecovered.push_back(entry.second);
        check(); const auto finished = validateBdsTopology(mesh,control); if(!finished.complete()) throw OperationFailure{finished};
        output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
