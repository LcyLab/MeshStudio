/** @file GmshInitialMeshLegalizationTest.cpp
 * @brief 原生平方半径容差/Quality(false)保护/动态池去重与失败原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/InitialMeshLegalization.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok){ ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh badDiagonal() {
    BdsMesh mesh; const std::array<Point2D,4> uv{{{0.,0.},{2.,0.},{2.,1.},{0.,2.}}};
    for(int i = 0; i < 4; ++i) { BdsPoint p; p.id = i; p.uv = uv[i]; p.point = {uv[i].x,uv[i].y,0.}; insertBdsPoint(mesh,p); }
    appendBdsTriangle(mesh,{0,1,3}); appendBdsTriangle(mesh,{1,2,3}); return mesh;
}
MetricEvaluation identity(Point2D) { MetricEvaluation result; result.metric = {1.,0.,1.}; return result; }
}
int main() {
    const std::array<Point2D,4> inside{{{0.,0.},{2.,0.},{0.,2.},{1.,1.}}};
    auto circle = shouldFlipInitialEdge(inside,{1.,0.,1.});
    check(circle.status.complete() && circle.inside && circle.circle.radiusSquared == 2. && circle.queryDistanceSquared == 0. && circle.tolerance == 1.e-12,"metric circle uses physical squared radius and fixed absolute tolerance");
    auto equal = inside; equal[3] = {2.,2.};
    check(!shouldFlipInitialEdge(equal,{1.,0.,1.}).inside,"exact cocircular query is outside strict tolerance");
    check(shouldFlipInitialEdge(inside,{500.,0.,500.}).tolerance == 1.e-12 && shouldFlipInitialEdge(inside,{501.,0.,501.}).tolerance == 1.e-11 && shouldFlipInitialEdge(inside,{50001.,0.,50001.}).tolerance == 1.e-9,"native tolerance thresholds apply to radius squared");
    circle = shouldFlipInitialEdge({Point2D{5.,5.},{6.,5.},{7.,5.},{0.,0.}},{1.,0.,1.});
    check(circle.status.complete() && circle.circle.singularLinearSystem && circle.inside && circle.circle.radiusSquared == 50.,"native singular origin center remains a decision with explicit flag");
    BdsPoint a,b,c; a.uv = {0.,0.}; b.uv = {2.,1.}; c.uv = {4.,3.};
    check(computeBdsParameterTriangleArea(a,b,c) == 1.,"ordinary area is original raw half cross product");
    a.degenerated = 1; check(computeBdsParameterTriangleArea(a,b,c) == 4.,"axis1 degenerate area formula differs from raw UV orientation");
    a.degenerated = 2; check(computeBdsParameterTriangleArea(a,b,c) == 6.,"axis2 degenerate area formula");
    b.degenerated = 1; check(computeBdsParameterTriangleArea(a,b,c) == 0.,"multiple degenerate triangle points have zero area");
    auto mesh = badDiagonal(); const int diagonal = findBdsEdge(mesh,1,3); const auto n = *getBdsEdgeNeighborhood(mesh,diagonal);
    check(canSwapInitialBdsEdge(mesh,diagonal,n),"unclassified endpoints permit valid swap despite small valence");
    auto small = mesh; small.geometry.insert({1,2}); small.points.at(1).classification = {1,2};
    check(!canSwapInitialBdsEdge(small,diagonal,n),"Quality(false) still has low-degree point protection");
    auto seam = mesh; seam.points.at(0).periodicCounterpart = 2; seam.points.at(2).periodicCounterpart = 0;
    check(!canSwapInitialBdsEdge(seam,diagonal,n),"both new diagonal counterpart pointers block quality swap");
    auto pole = mesh; pole.points.at(0).degenerated = 1; pole.points.at(2).degenerated = 2;
    check(!canSwapInitialBdsEdge(pole,diagonal,n),"both new diagonal degenerate points block swap");
    BdsMesh output; std::vector<Point2D> queries;
    auto result = legalizeInitialMesh(mesh,[&](Point2D uv) { queries.push_back(uv); return identity(uv); },output);
    check(result.status.complete() && result.swaps == 1 && result.metricQueries == 1 && result.configurations.size() == 1 && result.scans.size() == 2,"one original swap with shared configuration set and final zero-swap scan");
    if(result.status.complete()) {
        check(findBdsEdge(output,0,2) == 5 && output.edges[diagonal].deleted && output.edges.size() == 6 && output.triangles.size() == 4 && validateBdsTopology(output).complete(),"deleted records retained and new diagonal/triangles appended");
        check(queries.size() == 1 && queries[0].x == 1. && queries[0].y == 1. && result.scans[0].endingEdges == 6 && result.scans[0].attempts.back().edge == 5 && result.scans[0].attempts.back().decision == InitialEdgeDecision::ConfigurationVisited,"same scan visits newly appended edge and rejects duplicate configuration without CAD query");
    }
    auto locked = mesh; locked.geometry.insert({11,1}); locked.edges[diagonal].classification = {11,1};
    result = legalizeInitialMesh(locked,identity,output);
    check(result.status.complete() && result.swaps == 0 && result.metricQueries == 1 && result.configurations.size() == 1,"curve protection occurs after configuration/metric/circle stage");
    auto unchanged = mesh; unchanged.points.at(1).configurationModified = false; unchanged.points.at(3).configurationModified = false;
    result = legalizeInitialMesh(unchanged,identity,output);
    check(result.status.complete() && result.metricQueries == 0 && result.configurations.empty(),"outer test checks only edge endpoints, even modified opposite points");
    BdsMesh sentinel; sentinel.maximumPointId = 888; RunControl cancel; cancel.keepGoing = [] { return false; };
    check(legalizeInitialMesh(mesh,identity,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 888,"cancel retains output");
    RunControl budget; budget.maximumGeometryEvaluations = 0;
    check(legalizeInitialMesh(mesh,identity,sentinel,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 888,"metric budget retains output");
    auto failed = [](Point2D) { MetricEvaluation m; m.status = {MeshingStatus::GeometryQueryFailed,"CAD derivative unavailable"}; return m; };
    check(legalizeInitialMesh(mesh,failed,sentinel).status.code == MeshingStatus::GeometryQueryFailed && sentinel.maximumPointId == 888,"geometry failure preserves output");
    return failures ? 1 : 0;
}
