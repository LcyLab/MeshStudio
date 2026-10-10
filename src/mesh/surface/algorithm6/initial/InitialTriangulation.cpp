/** @file InitialTriangulation.cpp
 * @brief 初始剖分编排；保留图表的独立几何分类登记。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "InitialTriangulation.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <set>
namespace mesher::gmshReference {
/// 按普通面图表的原节点顺序构造初始剖分。
OperationStatus buildInitialTriangulation(const FaceChart& chart, OrdinaryInitialTriangulation& output, const RunControl& control)
{
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing())
            throw OperationFailure{{MeshingStatus::Cancelled,"ordinary initial entry cancelled"}}; };
        check(); OrdinaryInitialTriangulation initial; initial.route = chart.route;
        if(chart.route == FaceChartRoute::SkipInsufficientNodes) { output = std::move(initial); return {}; }
        if(chart.route == FaceChartRoute::DirectTriangle) {
            if(chart.vertices.size() != 3) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid ordinary direct triangle count"}};
            std::array<int,3> ids{chart.vertices[0].localId,chart.vertices[1].localId,chart.vertices[2].localId};
            std::sort(ids.begin(),ids.end());
            if(ids[0] < 0 || ids[0] == ids[1] || ids[1] == ids[2])
                throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid ordinary direct BDS identities"}};
            initial.directTriangles.push_back(ids); check(); output = std::move(initial); return {};
        }
        if(chart.route != FaceChartRoute::General || chart.initialVertices.size() < 4 || chart.initialVertices.size() != chart.vertices.size())
            throw OperationFailure{{MeshingStatus::InvalidInput,"ordinary initial entry requires complete single-copy chart"}};
        std::set<std::uint64_t> ids;
        for(const auto& p : chart.vertices) if(p.physicalId == 0 || !ids.insert(p.physicalId).second)
            throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid ordinary chart physical identity"}};
        Point2D lower{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()}, upper{-lower.x,-lower.y};
        for(const auto& p : chart.initialVertices) {
            check();
            if(!std::isfinite(p.uv.x) || !std::isfinite(p.uv.y) || ids.erase(p.physicalId) != 1)
                throw OperationFailure{{MeshingStatus::InvalidInput,"invalid ordinary initial copy record"}};
            lower.x = std::min(lower.x,p.uv.x); lower.y = std::min(lower.y,p.uv.y);
            upper.x = std::max(upper.x,p.uv.x); upper.y = std::max(upper.y,p.uv.y);
        }
        const Point2D center{(lower.x+upper.x)*.5,(lower.y+upper.y)*.5};
        initial.outerUpper = {(upper.x-center.x)*1.1+center.x,(upper.y-center.y)*1.1+center.y};
        initial.outerLower = {(lower.x-center.x)*1.1+center.x,(lower.y-center.y)*1.1+center.y};
        auto status = buildWalkTriangulation(initial.outerLower,initial.outerUpper,chart.initialVertices,initial.mesh,control);
        if(!status.complete()) return status;
        check(); output = std::move(initial); return {};
    } catch(const OperationFailure& error) { return error.status; }
      catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
}
/// 构造周期图表的初始分治剖分。
InitialTriangulationResult buildPeriodicInitialTriangulation(const PeriodicFaceChart& chart,
    const PeriodicFaceBoundaryInput& geometry, InitialTriangulation& output, const RunControl& control)
{
    InitialTriangulationResult result;
    try {
        auto check = [&] {
            if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"periodic initial triangulation cancelled"}};
        };
        auto finite = [](Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); };
        auto fail = [](const char* message) { throw OperationFailure{{MeshingStatus::InvalidTopology,message}}; };
        check(); InitialTriangulation mesh; mesh.route = chart.boundary.route; mesh.vertices = chart.boundary.vertices;
        mesh.geometry = chart.embedded.geometry;
        if(mesh.route == FaceChartRoute::SkipInsufficientNodes) { output = std::move(mesh); return result; }
        if(mesh.route == FaceChartRoute::DirectTriangle) {
            if(mesh.vertices.size() != 3 || chart.boundary.loopPositionCount != 3) fail("invalid native direct triangle count");
            std::sort(mesh.vertices.begin(),mesh.vertices.end(),[](const auto& a,const auto& b) { return a.vertex.localId < b.vertex.localId; });
            mesh.triangles.push_back({mesh.vertices[0].vertex.localId,mesh.vertices[1].vertex.localId,mesh.vertices[2].vertex.localId});
            check(); output = std::move(mesh); return result;
        }
        if(mesh.route != FaceChartRoute::General || !chart.boundary.hasBounds || geometry.entity != chart.boundary.entity ||
           !geometry.surfacePoint || !finite(geometry.parameterLower) || !finite(geometry.parameterUpper))
            throw OperationFailure{{MeshingStatus::InvalidInput,"missing periodic initial CAD chart capability"}};
        mesh.vertices.insert(mesh.vertices.end(),chart.embedded.vertices.begin(),chart.embedded.vertices.end());
        if(chart.embedded.initialPointCountMismatch || chart.initialPoints.size() != chart.expectedInitialPointCount || chart.initialPoints.size() < 4)
            fail("periodic initial record count differs from native allocation");
        if(control.maximumNodes < 4 || mesh.vertices.size() > control.maximumNodes-4 || chart.initialPoints.size() > control.maximumNodes-4)
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic initial outer-point budget exceeded"}};
        std::set<int> ids;
        for(const auto& p : mesh.vertices) if(p.vertex.localId < 0 || !ids.insert(p.vertex.localId).second) fail("invalid initial BDS identity");
        std::vector<InitialPoint> initial;
        for(const auto& p : chart.initialPoints) {
            if(!ids.count(p.localId)) fail("unknown periodic initial record BDS identity");
            initial.push_back({p.localId,p.coordinates});
        }
        auto lower = chart.boundary.lower, upper = chart.boundary.upper;
        if(!finite(lower) || !finite(upper) || lower.x > upper.x || lower.y > upper.y) fail("invalid raw boundary bounding box");
        const double du = geometry.parameterUpper.x-geometry.parameterLower.x, dv = geometry.parameterUpper.y-geometry.parameterLower.y;
        // makeCube 的半边长来自原始 raw 盒对角线，保留先 ±1 后 scale 的舍入次序。
        auto scale = [&](double sx,double sy) {
            const Point2D center{(lower.x+upper.x)*.5,(lower.y+upper.y)*.5};
            upper = {(upper.x-center.x)*sx+center.x,(upper.y-center.y)*sy+center.y};
            lower = {(lower.x-center.x)*sx+center.x,(lower.y-center.y)*sy+center.y};
        };
        if(du/dv < 1200. && dv/du < 1200.) {
            mesh.cubicBounds = true;
            const double dx = upper.x-lower.x, dy = upper.y-lower.y;
            const Point2D center{(lower.x+upper.x)*.5,(lower.y+upper.y)*.5};
            upper = {center.x+1.,center.y+1.}; lower = {center.x-1.,center.y-1.};
            const double half = std::sqrt(dx*dx+dy*dy)*.5; scale(half,half);
        }
        scale(3.5,3.5); mesh.outerLower = lower; mesh.outerUpper = upper;
        if(!finite(lower) || !finite(upper) || !(lower.x < upper.x) || !(lower.y < upper.y)) fail("degenerate periodic outer bounding box");
        const Point2D corners[4]{{lower.x,lower.y},{lower.x,upper.y},{upper.x,lower.y},{upper.x,upper.y}};
        for(int i = 0; i < 4; ++i) {
            check();
            if(result.geometryEvaluations >= control.maximumGeometryEvaluations)
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic outer-point CAD query budget exceeded"}};
            ++result.geometryEvaluations; Point3D point;
            try { point = geometry.surfacePoint(corners[i]); }
            catch(const OperationFailure&) { throw; }
            catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
            catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"periodic outer-point CAD exception"}}; }
            if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite periodic outer-point CAD value"}};
            mesh.vertices.push_back({{-i-1,0,2,geometry.entity,corners[i],point},{1.e22,1.e22},-1});
            initial.push_back({-i-1,corners[i]});
        }
        result.status = buildDivideAndConquerTriangulation(initial,mesh.triangulation,control);
        if(!result.status.complete()) return result;
        for(const auto& triangle : mesh.triangulation.triangles)
            mesh.triangles.push_back({mesh.triangulation.sortedPoints[triangle[0]].localId,
                                      mesh.triangulation.sortedPoints[triangle[1]].localId,
                                      mesh.triangulation.sortedPoints[triangle[2]].localId});
        check(); output = std::move(mesh);
    }
    catch(const OperationFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    catch(...) { result.status = {MeshingStatus::InvalidTopology,"unknown periodic initial triangulation failure"}; }
    return result;
}
}
