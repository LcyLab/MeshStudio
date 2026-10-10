/** @file GmshBdsInteriorPreparationTest.cpp
 * @brief 非稳定两次清理/分类降维/邻接映射/外围删除与失败原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsInteriorPreparation.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok){ ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh meshInput() {
    BdsMesh mesh;
    for(int id = -4; id < 9; ++id) { BdsPoint p; p.id = id; p.point = {double(id),0.,0.}; p.classification = {2,3}; p.sizes = {.4,.8}; insertBdsPoint(mesh,p); }
    mesh.maximumPointId = 99; mesh.geometry.insert({100,0}); mesh.points.at(0).classification = {100,0};
    mesh.geometry.insert({11,1});
    for(int i = 1; i < 7; ++i) appendBdsTriangle(mesh,{0,i,i+1});
    for(int i : {1,4,5}) mesh.triangles[i].classification = {1,2,BdsClassificationOrigin::FaceInterior};
    mesh.edges[findBdsEdge(mesh,0,2)].classification = {11,1};
    return mesh;
}
}
int main() {
    const auto input = meshInput(); BdsMesh output;
    auto result = prepareOrdinaryBdsInterior(input,output);
    check(result.status.complete() && output.triangles.size() == 3 && validateBdsTopology(output).complete(),"delete unclassified triangles and retain valid compact topology");
    check(result.unclassifiedTriangles == std::vector<int>({0,2,3}) && result.firstCleanup.triangles == std::vector<int>({-1,1,-1,-1,2,0}),"std::partition reverses tail survivors instead of stable filtering");
    if(output.triangles.size() == 3) {
        check(getBdsTriangleNodes(output,0) == std::array<int,3>{0,6,7} && getBdsTriangleNodes(output,1) == std::array<int,3>{0,2,3},"original triangle directions retained despite reordering");
        check(output.points.at(0).classification.dimension == 0 && output.points.at(2).classification.dimension == 1 && output.points.at(2).classification.tag == 11,"existing 0D retained and higher degree point lowered to curve classification");
        const auto edge = findBdsEdge(output,0,6);
        check(edge >= 0 && output.edges[edge].classification.origin == BdsClassificationOrigin::FaceInterior && output.points.at(6).classification.origin == BdsClassificationOrigin::FaceInterior,"unclassified live edges and high-degree points get temporary interior identity");
        check(output.points.at(6).sizes.meshSize == .4 && output.points.at(6).sizes.backgroundSize == .8 && output.maximumPointId == 99 && output.geometry.size() == input.geometry.size(),"no size changes, renumber or geometry deregistration");
        const auto& old = input.points.at(6).edges; std::vector<int> expected;
        for(int index : old) { const int mapped = result.secondCleanup.edges.at(result.firstCleanup.edges.at(index)); if(mapped >= 0) expected.push_back(mapped); }
        check(output.points.at(6).edges == expected && !result.orphanEdges.empty(),"point incident edge order preserved through both mappings and orphan removal");
    }
    check(result.removedOuterPoints == std::vector<int>({-1,-2,-3,-4}) && output.points.count(-1) == 0 && output.points.count(8) == 1,"only four outer points removed in native order, other isolated points retained");
    auto empty = input; for(auto& f : empty.triangles) f.classification = {};
    result = prepareOrdinaryBdsInterior(empty,output);
    check(result.status.complete() && output.triangles.empty() && output.edges.empty() && output.points.size() == 9,"empty inner domain is retained without guessed fill or deleting ordinary isolated points");
    BdsMesh sentinel; sentinel.maximumPointId = 888; RunControl cancel; cancel.keepGoing = [] { return false; };
    check(prepareOrdinaryBdsInterior(input,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 888,"cancel preserves output");
    RunControl budget; budget.maximumNodes = 12;
    check(prepareOrdinaryBdsInterior(input,sentinel,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 888,"node budget preserves output");
    auto dangling = input; appendBdsTriangle(dangling,{-1,7,8}); dangling.triangles.back().classification = {1,2,BdsClassificationOrigin::FaceInterior};
    check(prepareOrdinaryBdsInterior(dangling,sentinel).status.code == MeshingStatus::InvalidTopology && sentinel.maximumPointId == 888,"live outer adjacency fails explicitly instead of leaving dangling pointers");
    return failures ? 1 : 0;
}
