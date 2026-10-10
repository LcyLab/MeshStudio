/** @file GmshBdsDomainClassificationTest.cpp
 * @brief BDS原序域填充/外域清除/分类对象身份/无种子诊断。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsDomainClassification.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok){ ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh framedSquare() {
    BdsMesh mesh;
    const std::array<Point2D,8> coordinates{{{-1.,-1.},{2.,-1.},{2.,2.},{-1.,2.},{0.,0.},{1.,0.},{1.,1.},{0.,1.}}};
    for(int i = 0; i < 8; ++i) { BdsPoint p; p.id = i-4; p.uv = coordinates[i]; p.point = {p.uv.x,p.uv.y,0.}; insertBdsPoint(mesh,p); }
    for(auto nodes : {std::array<int,3>{-4,-3,1},{-4,1,0},{-3,-2,2},{-3,2,1},{-2,-1,3},{-2,3,2},{-1,-4,0},{-1,0,3},{0,1,2},{0,2,3}}) appendBdsTriangle(mesh,nodes);
    mesh.geometry.insert({11,1});
    for(auto pair : {std::array<int,2>{0,1},{1,2},{2,3},{3,0}}) mesh.edges[findBdsEdge(mesh,pair[0],pair[1])].classification = {11,1};
    return mesh;
}
}
int main() {
    const BdsClassification registry{1,2}, marker{1,2,BdsClassificationOrigin::FaceInterior};
    check(registry < marker || marker < registry,"equal tag/degree registered and temporary objects retain distinct identity");
    auto mesh = framedSquare(); mesh.geometry.insert({77,2}); for(auto& face : mesh.triangles) face.classification = {77,2};
    BdsMesh output = mesh; auto result = classifyOrdinaryBdsDomain(mesh,output);
    check(result.status.complete() && result.interiorTriangles == 2 && result.clearedExteriorTriangles.size() == 8 && result.fills.size() == 3,"exterior/inner/negative-opposite three original fills and two inner triangles");
    check(output.triangles[8].classification.dimension == 2 && output.triangles[8].classification.tag == 1 && output.triangles[8].classification.origin == BdsClassificationOrigin::FaceInterior && output.triangles[0].classification.dimension == -1,"temporary interior source preserved, exterior reset to null");
    check(output.geometry.size() == mesh.geometry.size() && output.geometry.count(marker) == 0 && validateBdsTopology(output).complete(),"temporary classifications are not registered geometry entities");
    if(result.fills.size() == 3) {
        check(result.fills[0].classification.tag == 1 && result.fills[0].classification.dimension == 3 && result.fills[0].classification.origin == BdsClassificationOrigin::FaceExterior,"ordinary exterior marker differs from periodic marker");
        check(result.fills[2].visits.size() == 1 && !result.fills[2].visits[0].second,"negative opposite fallback still called on previously colored seed");
        std::size_t redundant = 0; for(auto visit : result.fills[0].visits) if(!visit.second) ++redundant;
        check(redundant > 0,"LIFO keeps redundant pops instead of marking at push");
    }
    check(output.edges.size() == mesh.edges.size() && output.triangles.size() == mesh.triangles.size() && output.points.count(-4) == 1,"classification does not clean pools or remove outer nodes");
    auto unbounded = mesh; for(auto& edge : unbounded.edges) edge.classification = {};
    output = unbounded; result = classifyOrdinaryBdsDomain(unbounded,output);
    check(result.status.complete() && result.interiorTriangles == 0 && result.fills.size() == 1,"missing boundary seed yields explicit empty inner domain without geometry guess");
    auto temporaryPoint = mesh; temporaryPoint.points.at(0).classification = marker;
    check(validateBdsTopology(temporaryPoint).complete(),"temporary point classification recognized without registration");
    auto malformed = mesh; malformed.geometry.insert(marker);
    check(!validateBdsTopology(malformed).complete(),"registry excludes temporary marker objects");
    BdsMesh sentinel; sentinel.maximumPointId = 888; RunControl cancel; cancel.keepGoing = [] { return false; };
    check(classifyOrdinaryBdsDomain(mesh,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 888,"cancel retains output");
    RunControl budget; budget.maximumNodes = 7;
    check(classifyOrdinaryBdsDomain(mesh,sentinel,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 888,"budget retains output");
    return failures ? 1 : 0;
}
