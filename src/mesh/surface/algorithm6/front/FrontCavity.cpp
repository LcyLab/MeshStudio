/** @file FrontCavity.cpp
 * @brief 原前沿固定度量空腔与壳边顺序。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontCavity.h"
#include <algorithm>
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
/// 定位与候选点相关的前沿空腔并标记待替换三角形。
FrontCavityResult buildFrontCavity(FrontMesh& mesh,const NodeSizeState& nodes,const FrontTriangleSet& all,
    int worst,Point2D point,SurfaceMetric metric,bool force,const RunControl& control)
{
    FrontCavityResult result;
    auto restoreEntryState = [&] {
        for(std::size_t i = 0; i < result.triangles.size(); ++i) mesh.triangles[result.triangles[i]].deleted = result.entryDeleted[i];
    };
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"front cavity cancelled"}}; };
        auto triangle = [&](int index)->FrontTriangle& {
            if(index < 0 || std::size_t(index) >= mesh.triangles.size()) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid front cavity triangle"}};
            return mesh.triangles[index];
        };
        auto circleTest = [&](int index) {
            check(); const auto& t = triangle(index); std::array<Point2D,3> parameters;
            for(int i = 0; i < 3; ++i) {
                const auto found = nodes.indices.find(t.vertices[i]);
                if(found == nodes.indices.end() || found->second >= nodes.nodes.size() || nodes.nodes[found->second].vertex != t.vertices[i])
                    throw OperationFailure{{MeshingStatus::InvalidTopology,"front cavity node index mismatch"}};
                parameters[i] = nodes.nodes[found->second].parameters;
            }
            FrontCavityCircle test; test.triangle = index; test.circle = computeMetricCircumcircle(parameters,metric);
            if(!test.circle.status.complete()) throw OperationFailure{test.circle.status};
            const double du = test.circle.center.x-point.x,dv = test.circle.center.y-point.y;
            test.distanceSquared = du*du*metric.gUU+dv*dv*metric.gVV+2.0*du*dv*metric.gUV;
            if(!std::isfinite(test.distanceSquared)) throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite front cavity distance"}};
            // MTriangle重载没有初始BDS/raw数组重载的减容差。
            test.inside = test.distanceSquared < test.circle.radiusSquared;
            result.circleTests.push_back(test); return test.inside;
        };
        check();
        if(!std::isfinite(point.x) || !std::isfinite(point.y)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite front cavity candidate"}};
        if(triangle(worst).deleted) throw OperationFailure{{MeshingStatus::InvalidTopology,"front cavity worst triangle already deleted"}};
        result.worstInsideCircle = circleTest(worst);
        int seed = worst;
        if(!result.worstInsideCircle) {
            result.search = findContainingFrontTriangle(mesh,nodes,all,worst,point,force,control);
            if(!result.search.status.complete()) throw OperationFailure{result.search.status};
            seed = result.search.triangle;
            result.coordinateTests = result.search.coordinateTests;
            result.containingTriangle = seed; result.coordinates = result.search.coordinates;
        }
        if(seed >= 0) {
            struct Frame { int triangle,edge; }; std::vector<Frame> stack;
            auto enter = [&](int index) {
                auto& t = triangle(index);
                result.entryDeleted.push_back(t.deleted);
                result.triangles.push_back(index); t.deleted = true; stack.push_back({index,0});
            };
            enter(seed);
            while(!stack.empty()) {
                check(); auto& frame = stack.back();
                if(frame.edge == 3) { stack.pop_back(); continue; }
                const int index = frame.triangle,edge = frame.edge++; const auto& t = triangle(index);
                std::array<std::uint64_t,2> ids{t.vertices[(edge+2)%3],t.vertices[edge]};
                if(ids[1] < ids[0]) std::swap(ids[0],ids[1]);
                const int neighbor = t.neighbors[edge];
                if(neighbor == -1 || nodes.internalEdges.find(ids) != nodes.internalEdges.end()) result.shell.push_back({index,edge});
                else if(!triangle(neighbor).deleted) {
                    if(circleTest(neighbor)) enter(neighbor); else result.shell.push_back({index,edge});
                }
            }
            if(result.worstInsideCircle) for(int index : result.triangles) {
                check(); result.coordinates = mapFrontPointToTriangle(triangle(index),nodes,point);
                if(!result.coordinates.status.complete()) throw OperationFailure{result.coordinates.status};
                result.coordinateTests.push_back({index,result.coordinates});
                if(result.coordinates.inside) { result.containingTriangle = index; break; }
            }
        }
        check();
    } catch(const OperationFailure& error) {
        restoreEntryState();
        result.status = error.status;
    } catch(const std::exception& error) {
        restoreEntryState(); result.status = {MeshingStatus::InvalidTopology,error.what()};
    } catch(...) {
        restoreEntryState(); result.status = {MeshingStatus::InvalidTopology,"unknown front cavity failure"};
    }
    return result;
}
/// 撤销空腔构造期间的临时删除标记。
OperationStatus rollbackFrontCavity(FrontMesh& mesh,const FrontCavityResult& cavity)
{
    for(int index : cavity.triangles) if(index < 0 || std::size_t(index) >= mesh.triangles.size())
        return {MeshingStatus::InvalidTopology,"invalid front cavity rollback index"};
    for(int index : cavity.triangles) mesh.triangles[index].deleted = false;
    return {};
}
}
