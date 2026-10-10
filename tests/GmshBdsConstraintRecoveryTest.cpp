/** @file GmshBdsConstraintRecoveryTest.cpp
 * @brief BDS数值求交、原邻接翻边/标签继承、恢复与CAD自交失败。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsConstraintRecovery.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok){ ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh square(bool reversed = false) {
    BdsMesh mesh; int id = 0;
    for(Point2D uv : {Point2D{0.,0.},Point2D{1.,0.},Point2D{1.,1.},Point2D{0.,1.}}) {
        BdsPoint p; p.id = id++; p.uv = uv; p.point = {uv.x,uv.y,0.}; insertBdsPoint(mesh,p);
    }
    if(reversed) { appendBdsTriangle(mesh,{2,1,0}); appendBdsTriangle(mesh,{3,2,0}); }
    else { appendBdsTriangle(mesh,{0,1,2}); appendBdsTriangle(mesh,{0,2,3}); }
    return mesh;
}
}
int main()
{
    auto crossing = intersectBdsRecoverySegments({0.,0.},{1.,1.},{0.,1.},{1.,0.});
    check(crossing.systemSolved && crossing.intersects && crossing.parameters.x == .5 && crossing.parameters.y == .5,"closed numerical segment intersection");
    check(intersectBdsRecoverySegments({0.,0.},{1.,0.},{1.,0.},{1.,1.}).intersects,"endpoint contact included");
    check(!intersectBdsRecoverySegments({0.,0.},{1.,0.},{0.,1.},{1.,1.}).systemSolved,"singular parallel solve");
    for(bool reversed : {false,true}) {
        auto mesh = square(reversed); const int diagonal = findBdsEdge(mesh,0,2);
        mesh.geometry.insert({4,2}); mesh.geometry.insert({5,2}); mesh.geometry.insert({6,2});
        mesh.triangles[0].classification = {4,2}; mesh.triangles[1].classification = {5,2}; mesh.edges[diagonal].classification = {6,2};
        const auto neighborhood = getBdsEdgeNeighborhood(mesh,diagonal);
        check(bool(neighborhood),"native two-triangle neighborhood");
        if(neighborhood) {
            check(canSwapBdsRecoveryEdge(mesh,diagonal,*neighborhood),"convex diagonal permitted");
            auto protectedMesh = mesh; protectedMesh.geometry.insert({77,1}); protectedMesh.edges[diagonal].classification = {77,1};
            check(!canSwapBdsRecoveryEdge(protectedMesh,diagonal,*neighborhood,true),"force does not bypass curve protection");
            for(auto& entry : protectedMesh.points) entry.second.configurationModified = false;
            protectedMesh.edges[diagonal].classification = {};
            check(!canSwapBdsRecoveryEdge(protectedMesh,diagonal,*neighborhood) && canSwapBdsRecoveryEdge(protectedMesh,diagonal,*neighborhood,true),"config_modified gate and force");
            swapBdsEdgeTopology(mesh,diagonal,*neighborhood);
            check(mesh.edges.size() == 6 && mesh.triangles.size() == 4 && mesh.edges[diagonal].deleted && mesh.edges[diagonal].faces.empty() && mesh.triangles[0].deleted && mesh.triangles[1].deleted,"stable old pools and new edge/faces appended");
            if(mesh.edges.size() == 6 && mesh.triangles.size() == 4) {
                check(findBdsEdge(mesh,1,3) == 5 && mesh.edges[5].classification.tag == 6 && mesh.triangles[2].classification.tag == 4 && mesh.triangles[3].classification.tag == 5,"classification inherits by native face order");
                check(validateBdsTopology(mesh).complete(),"swapped bidirectional adjacency valid");
            }
        }
        BdsMesh recovered; const auto result = recoverBdsConstraintEdge(square(reversed),1,3,recovered);
        check(result.status.complete() && result.recoveredEdge == 5 && !result.nativeFatal && result.scans.size() == 2,"native recovery swap then empty scan");
        if(result.scans.size() == 2) check(result.scans[0].attempts == std::vector<std::pair<int,bool>>{{diagonal,true}} && result.scans[1].intersectedEdges.empty(),"first eligible scan edge ordering");
    }
    auto existing = square(); BdsMesh output;
    auto result = recoverBdsConstraintEdge(existing,0,2,output);
    check(result.status.complete() && result.recoveredEdge == 2 && result.scans.empty(),"existing edge avoids scans");
    result = recoverBdsConstraintEdge(existing,1,3,output,{{0,2,11},{1,3,12}});
    check(result.status.complete() && !result.nativeFatal && result.recoveredEdge == -1 && result.notRecovered.size() == 2 && output.edges.size() == 5,"registered CAD self intersection nonfatal with both requests");
    existing.geometry.insert({9,1}); existing.edges[2].classification = {9,1};
    result = recoverBdsConstraintEdge(existing,1,3,output);
    check(result.status.complete() && result.nativeFatal && result.recoveredEdge == -1 && output.edges.size() == 5,"protected crossing edge fails fatal without registry");
    BdsMesh fan; int id = 0;
    for(Point2D uv : {Point2D{2.,0.},Point2D{1.,1.},Point2D{0.,2.},Point2D{-1.,1.},Point2D{-2.,0.},Point2D{-1.,-1.},Point2D{0.,-2.},Point2D{1.,-1.}}) {
        BdsPoint p; p.id = id++; p.uv = uv; p.point = {uv.x,uv.y,0.}; insertBdsPoint(fan,p);
    }
    for(auto nodes : {std::array<int,3>{0,1,2},{0,4,5},{0,3,4},{0,2,3},{0,6,7},{0,5,6}}) appendBdsTriangle(fan,nodes);
    fan.geometry.insert({9,1}); fan.edges[findBdsEdge(fan,0,5)].classification = {9,1};
    result = recoverBdsConstraintEdge(fan,2,6,output);
    check(result.status.complete() && result.nativeFatal && result.recoveredEdge == -1 && output.edges.size() > fan.edges.size() && validateBdsTopology(output).complete(),"native failure publishes earlier successful swaps");
    BdsMesh sentinel; sentinel.maximumPointId = 888;
    RunControl cancelled; cancelled.keepGoing = [] { return false; };
    check(recoverBdsConstraintEdge(existing,1,3,sentinel,{},cancelled).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 888,"cancel atomic output");
    check(recoverBdsConstraintEdge(existing,1,123,sentinel).status.code == MeshingStatus::InvalidInput && sentinel.maximumPointId == 888,"missing endpoint explicit instead of native null dereference");
    RunControl budget; budget.maximumNodes = 3;
    check(recoverBdsConstraintEdge(existing,1,3,sentinel,{},budget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 888,"node budget atomic output");
    return failures ? 1 : 0;
}
