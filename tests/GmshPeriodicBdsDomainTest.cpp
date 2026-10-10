/** @file GmshPeriodicBdsDomainTest.cpp
 * @brief 周期域(3,2)/deleted种子跳过/无负对点补填及共用内域清理。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsDomainClassification.h"
#include "mesh/surface/algorithm6/bds/BdsInteriorPreparation.h"
#include <algorithm>
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh framedSquare() {
    BdsMesh mesh;
    const std::array<Point2D,8> uv{{{-1.,-1.},{2.,-1.},{2.,2.},{-1.,2.},{0.,0.},{1.,0.},{1.,1.},{0.,1.}}};
    for(int i = 0; i < 8; ++i) { BdsPoint p; p.id = i-4; p.physicalId = i+1; p.uv = uv[i]; p.point = {p.uv.x,p.uv.y,0.}; p.sizes = {.4,.8}; insertBdsPoint(mesh,p); }
    appendBdsTriangle(mesh,{-4,-3,-2});
    for(int edge : mesh.triangles[0].edges) { auto& faces = mesh.edges[edge].faces; faces.erase(std::remove(faces.begin(),faces.end(),0),faces.end()); }
    mesh.triangles[0].deleted = true;
    for(auto t : {std::array<int,3>{-4,-3,1},{-4,1,0},{-3,-2,2},{-3,2,1},{-2,-1,3},{-2,3,2},{-1,-4,0},{-1,0,3},{0,1,2},{0,2,3}}) appendBdsTriangle(mesh,t);
    for(auto p : {std::array<int,2>{0,1},{1,2},{2,3},{3,0}}) mesh.edges[findBdsEdge(mesh,p[0],p[1])].classification = {1,1,BdsClassificationOrigin::FaceBoundary};
    return mesh;
}
}
int main() {
    const auto input = framedSquare(); check(validateBdsTopology(input).complete(),"fixture with deleted first negative triangle is valid");
    BdsMesh classified; auto domain = classifyPeriodicBdsDomain(input,classified);
    check(domain.status.complete() && domain.fills.size() == 2 && domain.interiorTriangles == 2 && domain.clearedExteriorTriangles.size() == 8,"periodic has only exterior/inner fills, no negative-opposite fallback");
    if(domain.status.complete() && domain.fills.size() == 2) {
        check(domain.fills[0].seed == 1 && domain.fills[0].classification.tag == 3 && domain.fills[0].classification.dimension == 2 && domain.fills[0].classification.origin == BdsClassificationOrigin::FaceExterior,"deleted initial negative triangle skipped and periodic exterior marker distinct");
        check(classified.triangles[0].classification.dimension == -1 && classified.triangles[0].deleted && classified.points.count(-4),"domain retains deleted record and outer nodes");
        std::size_t redundant = 0; for(const auto& fill : domain.fills) for(auto visit : fill.visits) if(!visit.second) ++redundant;
        check(redundant > 0,"periodic LIFO retains redundant visits");
        BdsMesh prepared; auto interior = prepareBdsInterior(classified,prepared);
        check(interior.status.complete() && prepared.triangles.size() == 2 && prepared.points.size() == 4 && interior.removedOuterPoints == std::vector<int>({-1,-2,-3,-4}),"shared preparation deletes exterior and four outer nodes");
        if(interior.status.complete()) {
            check(prepared.points.at(0).classification.origin == BdsClassificationOrigin::FaceBoundary && prepared.points.at(0).classification.dimension == 1 && prepared.geometry.empty(),"CLASS_E propagates to empty endpoint without registering CAD curve");
            check(prepared.edges[findBdsEdge(prepared,0,2)].classification.origin == BdsClassificationOrigin::FaceInterior && prepared.points.at(0).sizes.meshSize == .4 && prepared.points.at(0).sizes.backgroundSize == .8,"interior diagonal CLASS_F and original dual sizes retained");
            check(std::count(interior.firstCleanup.triangles.begin(),interior.firstCleanup.triangles.end(),-1) == 9 && validateBdsTopology(prepared).complete(),"cleanup removes deleted and unclassified pool entries with valid remapping");
        }
    }
    auto unbounded = input; for(auto& e : unbounded.edges) e.classification = {};
    domain = classifyPeriodicBdsDomain(unbounded,classified);
    check(domain.status.complete() && domain.interiorTriangles == 0 && domain.fills.size() == 1,"periodic missing inner seed stays explicitly empty");
    BdsMesh sentinel; sentinel.maximumPointId = 999; RunControl cancel; cancel.keepGoing = [] { return false; };
    check(classifyPeriodicBdsDomain(input,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 999,"domain cancellation preserves output");
    check(prepareBdsInterior(input,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 999,"shared preparation cancellation preserves output");
    return failures ? 1 : 0;
}
