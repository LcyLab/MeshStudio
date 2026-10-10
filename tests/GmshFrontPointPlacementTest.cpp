/** @file GmshFrontPointPlacementTest.cpp
 * @brief 前沿度量候选/尺寸均值/普通CAD圆修正与失败回退、取消预算原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontPointPlacement.h"
#include <cmath>
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
}
int main() {
    FrontTriangle triangle; triangle.vertices = {1,2,3}; NodeSizeState nodes; FaceMesh mesh;
    const Point2D uv[] = {{0.,0.},{2.,0.},{1.,2.}};
    for(int i = 0; i < 3; ++i) {
        nodes.indices[i+1] = i; nodes.nodes.push_back({std::uint64_t(i+1),uv[i],{.5,2.}});
        FaceVertex v; v.id = v.physicalId = i+1; v.point = {uv[i].x,uv[i].y,0.}; mesh.vertices.emplace(v.id,v);
    }
    int queries = 0; Point2D query;
    FrontMetricSource metric = [&](Point2D p) { ++queries; query = p; return SurfaceMetric{1.,0.,1.}; };
    FrontPointCandidate fast;
    auto result = computeFrontPoint(triangle,1,nodes,true,metric,fast);
    check(result.status.complete() && queries == 1 && query.x == 1. && query.y == 2./3. && fast.geometryEvaluations == 1,"UV centroid uses exact ONE_THIRD and one metric query");
    check(result.status.complete() && fast.planarCenter.x == 1. && fast.planarCenter.y == .75 && fast.metricCenter.x == 1. && fast.metricCenter.y == .75,"planar and metric centers preserved separately");
    check(result.status.complete() && fast.edgeMeanSizes.meshSize == .5 && fast.edgeMeanSizes.backgroundSize == 2. && fast.targetSize == .5 && fast.distanceToCenter == .75 && fast.height == .5*std::sqrt(3.)*.5,"endpoint dual means and min(d,q) native order");
    check(result.status.complete() && fast.fastPoint.x == 1. && fast.fastPoint.y == fast.height && fast.point.y == fast.fastPoint.y,"edge1 endpoints0/1 midpoint and candidate toward circumcenter");
    FrontPointCandidate corrected;
    SurfaceFunction plane = [](Point2D p) { return Point3D{p.x,p.y,0.}; };
    if(result.status.complete()) {
        result = correctFrontPointOnSurface(triangle,1,mesh,fast,plane,corrected);
        check(result.status.complete() && corrected.correctionAttempted && corrected.intersectionFound && corrected.point.x == fast.point.x && corrected.point.y == fast.point.y,"plane correction initial residual success preserves exact seed");
        check(result.status.complete() && corrected.circle.cosineAxis.y == 1. && corrected.circle.sineAxis.z == 1. && corrected.circle.center.x == 1. && corrected.circle.radius == fast.height && corrected.geometryEvaluations == 2,"physical circle uses cosine transverse/sine normal and cumulative budget");
        SurfaceFunction unreachable = [](Point2D p) { return Point3D{p.x,p.y,5.}; };
        result = correctFrontPointOnSurface(triangle,1,mesh,fast,unreachable,corrected);
        check(result.status.complete() && !corrected.intersectionFound && corrected.retainedFastPoint && corrected.point.x == fast.point.x && corrected.point.y == fast.point.y && result.intersection.has_value(),"ordinary CAD nonconvergence explicitly retains candidate");
        auto degenerate = mesh; degenerate.vertices.at(3).point = {1.,1.e-14,0.}; int surfaceQueries = 0;
        result = correctFrontPointOnSurface(triangle,1,degenerate,fast,[&](Point2D p) { ++surfaceQueries; return plane(p); },corrected);
        check(result.status.complete() && corrected.skippedDegenerateNormal && !corrected.correctionAttempted && surfaceQueries == 0,"raw physical normal threshold skips solve without a surface query");
        FrontPointCandidate unchanged; unchanged.point = {77.,88.};
        result = correctFrontPointOnSurface(triangle,1,mesh,fast,[](Point2D)->Point3D { throw OperationFailure{{MeshingStatus::Cancelled,"cancelled CAD query"}}; },unchanged);
        check(result.status.code == MeshingStatus::Cancelled && unchanged.point.x == 77.,"CAD callback cancellation is propagated, never normal failure fallback");
        int cancellationChecks = 0; RunControl lateCancellation; lateCancellation.keepGoing = [&] { return ++cancellationChecks == 1; };
        result = correctFrontPointOnSurface(triangle,1,degenerate,fast,plane,unchanged,lateCancellation);
        check(result.status.code == MeshingStatus::Cancelled && unchanged.point.x == 77.,"degenerate correction checks cancellation before publishing");
    }
    result = computeFrontPoint(triangle,1,nodes,false,metric,corrected);
    check(result.status.complete() && corrected.targetSize == 2. && corrected.height == .75 && corrected.point.y == .75,"extension off uses background size and metric circumcenter cap");
    nodes.nodes[0].sizes.backgroundSize = .1; nodes.nodes[1].sizes.backgroundSize = .1;
    result = computeFrontPoint(triangle,1,nodes,true,metric,corrected);
    check(result.status.complete() && corrected.targetSize == .1,"extension min can select background endpoint mean");
    FrontPointCandidate guard; guard.point = {77.,88.}; RunControl control; control.keepGoing = [] { return false; };
    const int before = queries; result = computeFrontPoint(triangle,1,nodes,true,metric,guard,control);
    check(result.status.code == MeshingStatus::Cancelled && queries == before && guard.point.x == 77.,"cancel atomic before metric query");
    control = {}; control.maximumGeometryEvaluations = 0;
    result = computeFrontPoint(triangle,1,nodes,true,metric,guard,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && queries == before && guard.point.x == 77.,"metric query budget exact and atomic");
    control.maximumGeometryEvaluations = 1;
    result = correctFrontPointOnSurface(triangle,1,mesh,fast,plane,guard,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && guard.point.x == 77.,"correction counts prior metric query, no silent budget fallback");
    auto broken = nodes; broken.indices.at(1) = 999;
    result = computeFrontPoint(triangle,1,broken,true,metric,guard);
    check(result.status.code == MeshingStatus::InvalidTopology && guard.point.x == 77.,"invalid node index explicit");
    result = computeFrontPoint(triangle,3,nodes,true,metric,guard);
    check(result.status.code == MeshingStatus::InvalidInput && guard.point.x == 77.,"invalid edge explicit");
    result = computeFrontPoint(triangle,1,nodes,true,[](Point2D)->SurfaceMetric { throw std::runtime_error("CAD derivative unavailable"); },guard);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && guard.point.x == 77.,"failed metric query explicit and atomic");
    result = computeFrontPoint(triangle,1,nodes,true,[](Point2D) { return SurfaceMetric{4.,0.,1.}; },guard);
    check(result.status.code == MeshingStatus::NonConverged && guard.point.x == 77.,"zero direction norm from metric right triangle explicit nonfinite path");
    return failures ? 1 : 0;
}
