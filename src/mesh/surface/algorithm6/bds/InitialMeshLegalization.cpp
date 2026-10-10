/** @file InitialMeshLegalization.cpp
 * @brief BDS初始度量合法化。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "InitialMeshLegalization.h"
#include <algorithm>
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
/// 数组重载inCircumCircleAniso：前三点构圆、第四点度量距离；半径平方。
InitialMetricCircleDecision shouldFlipInitialEdge(const std::array<Point2D,4>& points,SurfaceMetric metric) {
    InitialMetricCircleDecision result;
    result.circle = computeMetricCircumcircle({points[0],points[1],points[2]},metric);
    if(!result.circle.status.complete()) { result.status = result.circle.status; return result; }
    const double d0 = result.circle.center.x-points[3].x, d1 = result.circle.center.y-points[3].y;
    result.queryDistanceSquared = d0*d0*metric.gUU+d1*d1*metric.gVV+2.0*d0*d1*metric.gUV;
    if(!std::isfinite(result.queryDistanceSquared)) { result.status = {MeshingStatus::GeometryQueryFailed,"initial metric circle distance is nonfinite"}; return result; }
    result.tolerance = result.circle.radiusSquared <= 1.e3 ? 1.e-12 : result.circle.radiusSquared <= 1.e5 ? 1.e-11 : 1.e-9;
    result.inside = result.queryDistanceSquared < result.circle.radiusSquared-result.tolerance; return result;
}
/// 按曲面度量换边，完成初始 BDS 网格合法化。
InitialMeshLegalizationResult legalizeInitialMesh(const BdsMesh& input,const InitialSurfaceMetricSource& source,BdsMesh& output,const RunControl& control)
{
    InitialMeshLegalizationResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"initial BDS legalization cancelled"}}; };
        auto require = [](OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; };
        check(); require(validateBdsTopology(input,control)); BdsMesh mesh = input;
        while(true) {
            check(); result.scans.push_back({mesh.edges.size(),0,0,{}}); auto& scan = result.scans.back();
            // 原版同轮继续访问新追加边，不能冻结本轮初始池大小。
            for(std::size_t i = 0; i < mesh.edges.size(); ++i) {
                check(); const auto& edge = mesh.edges[i]; if(edge.deleted) continue;
                scan.attempts.push_back({int(i)}); auto& attempt = scan.attempts.back();
                const auto& first = mesh.points.at(edge.first); const auto& second = mesh.points.at(edge.second);
                if(!first.configurationModified && !second.configurationModified) { attempt.decision = InitialEdgeDecision::EndpointsUnmodified; continue; }
                if(edge.faces.size() != 2) { attempt.decision = InitialEdgeDecision::NotTwoFaces; continue; }
                const auto n = *getBdsEdgeNeighborhood(mesh,int(i));
                std::array<int,4> configuration{n.first,n.second,n.oppositeNodes[0],n.oppositeNodes[1]}; std::sort(configuration.begin(),configuration.end());
                attempt.configuration = configuration;
                // 配置在度量/圆内/交换资格前登记，圆外和拒绝项也不再重复查询。
                if(!result.configurations.insert(configuration).second) { attempt.decision = InitialEdgeDecision::ConfigurationVisited; continue; }
                if(result.metricQueries >= control.maximumGeometryEvaluations) throw OperationFailure{{MeshingStatus::BudgetExceeded,"initial BDS metric query budget exceeded"}};
                if(!source) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"initial BDS surface metric capability missing"}};
                attempt.midpoint = {.5*(first.uv.x+second.uv.x),.5*(first.uv.y+second.uv.y)};
                ++result.metricQueries;
                MetricEvaluation evaluated;
                try { evaluated = source(attempt.midpoint); }
                catch(const OperationFailure&) { throw; }
                catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
                require(evaluated.status); attempt.metric = evaluated.metric;
                attempt.circle = shouldFlipInitialEdge({first.uv,second.uv,mesh.points.at(n.oppositeNodes[0]).uv,mesh.points.at(n.oppositeNodes[1]).uv},attempt.metric);
                require(attempt.circle.status);
                if(!attempt.circle.inside) { attempt.decision = InitialEdgeDecision::OutsideCircle; continue; }
                if(!canSwapInitialBdsEdge(mesh,int(i),n)) { attempt.decision = InitialEdgeDecision::SwapRejected; continue; }
                swapBdsEdgeTopology(mesh,int(i),n); attempt.decision = InitialEdgeDecision::Swapped; ++scan.swaps;
            }
            scan.endingEdges = mesh.edges.size(); result.swaps += scan.swaps;
            if(!scan.swaps) break;
        }
        check(); require(validateBdsTopology(mesh,control)); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
