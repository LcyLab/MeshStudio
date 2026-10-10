/** @file GmshFrontTriangleInitializationTest.cpp
 * @brief 前沿双尺寸均值、R/h排序/并列、重复片、边编号邻接及失败原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontTriangleInitialization.h"
#include <cmath>
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
FaceMesh fixture() {
    FaceMesh mesh; mesh.entity = 1; mesh.maximumVertexId = 4;
    const Point3D points[] = {{0.,0.,0.},{2.,0.,0.},{2.,2.,0.},{0.,2.,0.}};
    for(int i = 0; i < 4; ++i) { FaceVertex v; v.id = v.physicalId = i+1; v.point = points[i]; v.dimension = 0; v.entity = i+1; mesh.vertices.emplace(v.id,v); }
    // 两片都以坐标轴为首边，避免投影舍入不同使数学同半径并非double同值。
    mesh.triangles = {{1,4,3},{1,2,3},{1,4,3}}; return mesh;
}
NodeSizeState sizes() {
    NodeSizeState state;
    for(int id : {3,1,4,2}) { state.indices.emplace(id,state.nodes.size()); state.nodes.push_back({std::uint64_t(id),{id*.2,id*.3},{1.,4.}}); }
    return state;
}
bool sentinel(const FrontMesh& mesh) { return mesh.orderedTriangles == std::vector<int>{888} && mesh.triangles.empty(); }
}
int main() {
    auto mesh = fixture(); auto nodes = sizes(); FrontMesh output;
    auto result = initializeFrontTriangles(mesh,nodes,true,output);
    check(result.status.complete() && output.triangles.size() == 3,"complete triangle pool retained");
    check(output.orderedTriangles == std::vector<int>{1,0} && result.rejectedDuplicateTriangles == std::vector<int>{2},"radius tie uses sorted face IDs and set rejects same-radius same-identity face");
    if(output.triangles.size() == 3) {
        const double mean = .3333333333*3.;
        check(output.triangles[0].meanSizes.meshSize == mean && output.triangles[0].meanSizes.backgroundSize == .3333333333*12. && output.triangles[0].targetSize == mean,"literal native average and boundary extension min");
        check(output.triangles[0].radius == std::sqrt(2.)/mean,"physical circumradius divided by target length");
        check(output.triangles[0].neighbors == std::array<int,3>{1,-1,-1} && output.triangles[1].neighbors == std::array<int,3>{0,-1,-1},"native edge indices join common diagonal bidirectionally");
        check(output.triangles[2].neighbors == std::array<int,3>{-1,-1,-1} && !output.triangles[2].deleted,"duplicate set rejection does not mark native created triangle deleted or connect it");
    }
    result = initializeFrontTriangles(mesh,nodes,false,output);
    check(result.status.complete() && output.triangles.size() == 3 && output.triangles[0].targetSize == .3333333333*12.,"extension off uses background mean");
    for(auto& n : nodes.nodes) n.sizes.backgroundSize = .5;
    result = initializeFrontTriangles(mesh,nodes,true,output);
    check(result.status.complete() && output.triangles.size() == 3 && output.triangles[0].targetSize == .3333333333*1.5,"extension min can select background size");
    mesh.triangles.resize(2); mesh.vertices.at(4).point = {0.,4.,0.}; nodes = sizes();
    result = initializeFrontTriangles(mesh,nodes,true,output);
    check(result.status.complete() && output.orderedTriangles == std::vector<int>{0,1},"larger physical R/h sorts first regardless of input face IDs");
    FrontMesh guard; guard.orderedTriangles = {888}; RunControl control; control.keepGoing = [] { return false; };
    result = initializeFrontTriangles(mesh,nodes,true,guard,control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel(guard),"cancel atomic output");
    control = {}; control.maximumNodes = 3;
    result = initializeFrontTriangles(mesh,nodes,true,guard,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel(guard),"front node budget atomic output");
    auto broken = nodes; broken.indices.at(1) = 999;
    result = initializeFrontTriangles(mesh,broken,true,guard);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel(guard),"invalid node index rejected");
    broken = nodes; broken.nodes[0].sizes.backgroundSize = std::numeric_limits<double>::quiet_NaN();
    result = initializeFrontTriangles(mesh,broken,true,guard);
    check(result.status.code == MeshingStatus::InvalidInput && sentinel(guard),"nonfinite node size rejected");
    broken = nodes;
    for(auto& n : broken.nodes) n.sizes = {std::numeric_limits<double>::max(),std::numeric_limits<double>::max()};
    result = initializeFrontTriangles(mesh,broken,true,guard);
    check(result.status.code == MeshingStatus::NonConverged && sentinel(guard),"finite inputs overflowing native size sums explicit instead of zero radius success");
    auto degenerate = mesh; degenerate.triangles = {{1,2,3}}; degenerate.vertices.at(3).point = {4.,0.,0.};
    result = initializeFrontTriangles(degenerate,nodes,true,output);
    check(result.status.complete() && output.triangles.size() == 1 && output.triangles[0].radius == 99999./(.3333333333*3.),"finite degenerate physical circumcenter sentinel retained");
    for(auto& n : nodes.nodes) n.sizes = {0.,0.};
    result = initializeFrontTriangles(mesh,nodes,true,guard);
    check(result.status.code == MeshingStatus::NonConverged && sentinel(guard),"nonfinite normalized radius explicit instead of inventing target size");
    FaceMesh empty; empty.entity = 1;
    result = initializeFrontTriangles(empty,{},true,output);
    check(result.status.complete() && output.triangles.empty() && output.orderedTriangles.empty(),"empty initializer preserved, front empty-set error belongs to driver");
    return failures ? 1 : 0;
}
