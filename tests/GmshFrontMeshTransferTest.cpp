/** @file GmshFrontMeshTransferTest.cpp
 * @brief 转移顺序、double朝向、单次等价映射、重复诊断和原子发布验证。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/postprocess/FrontMeshTransfer.h"
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
struct Fixture { FaceMesh mesh; NodeSizeState nodes; FrontMesh front; FrontQueueState queue; };
Fixture fixture() {
    Fixture f; f.mesh.entity = 1; f.mesh.maximumVertexId = 99;
    const Point2D uv[] = {{0.,0.},{1.,0.},{0.,1.},{1.,1.},{2.,0.},{2.,1.}};
    for(int i = 0; i < 6; ++i) {
        FaceVertex v; v.id = v.physicalId = i+1; v.dimension = 0; v.entity = i+1;
        v.point = {uv[i].x,uv[i].y,0.}; f.mesh.vertices.emplace(v.id,v);
        f.nodes.indices.emplace(v.id,f.nodes.nodes.size()); f.nodes.nodes.push_back({v.id,uv[i],{1.,1.}});
    }
    f.mesh.interiorVertices = {6}; f.mesh.triangles = {{999,999,999}};
    f.mesh.parametricCoordinates.emplace(1,Point2D{9.,9.}); // 转移朝向使用运行槽，非此旧UV快照。
    f.nodes.nodes.push_back({88,{4.,5.},{2.,3.}}); // 原拒绝ghost不进入live indices。
    return f;
}
void add(Fixture& f,std::array<std::uint64_t,3> ids,double r,bool deleted = false,bool include = true) {
    FrontTriangle t; t.vertices = ids; t.radius = r; t.deleted = deleted;
    const int index = int(f.front.triangles.size()); f.front.triangles.push_back(t);
    if(include) f.queue.allTriangles.insert(makeFrontTriangleKey(t,index));
}
bool sentinel(const FaceMesh& f) { return f.entity == 888 && f.triangles == std::vector<std::array<std::uint64_t,3>>{{77,77,77}}; }
}
int main() {
    auto f = fixture(); add(f,{1,2,3},1.); add(f,{2,3,4},3.); add(f,{1,4,3},2.,true); add(f,{2,5,4},4.,false,false);
    FaceMesh output; FrontMeshTransferOptions options;
    auto result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.consumedTriangles == std::vector<int>{1,2,0},"consume actual AllTris order, ignore unregistered pool triangle");
    check(result.skippedDeletedTriangles == std::vector<int>{2} && result.reversedTriangles == std::vector<int>{0},"skip deleted and reverse against first retained triangle");
    check(output.triangles == std::vector<std::array<std::uint64_t,3>>{{2,3,4},{1,3,2}},"replace initial face snapshot, preserve first vertex in reverse");
    check(output.vertices.size() == f.mesh.vertices.size() && output.maximumVertexId == 99 && output.interiorVertices == std::vector<std::uint64_t>{6} && f.nodes.nodes.size() == 7 && f.queue.allTriangles.size() == 3,"preserve vertex metadata, ghost/max ID and caller runtime ownership");
    check(!result.equivalenceDiagnosticsEvaluated && result.equivalentTriangles.empty(),"null equivalence skips diagnostics");
    f = fixture(); add(f,{1,2,5},3.); add(f,{1,2,3},2.); add(f,{1,3,2},1.);
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.reversedTriangles.empty() && output.triangles.back() == std::array<std::uint64_t,3>{1,3,2},"zero first normal does not select a later nondegenerate reference");
    f = fixture(); add(f,{1,2,3},2.); add(f,{2,3,4},1.);
    f.mesh.vertices.at(4).point = {-1.,0.,0.}; options.usePhysicalNormals = true;
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.reversedTriangles.empty(),"physical normal branch ignores opposite UV orientation");
    options = {}; result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.reversedTriangles == std::vector<int>{1},"normal baseline uses UV rather than physical orientation");
    f = fixture(); add(f,{1,2,3},4.); add(f,{2,1,3},3.); add(f,{4,2,3},2.); add(f,{1,4,5},1.);
    f.nodes.nodes.at(f.nodes.indices.at(4)).parameters = {-1.,0.};
    std::map<std::uint64_t,std::uint64_t> equivalence{{4,1},{1,6}}; options.equivalence = &equivalence;
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.equivalenceDiagnosticsEvaluated && result.equivalentTriangles == std::vector<std::array<int,2>>{{0,1}},"equivalence diagnostics use a single lookup, no recursive mapping");
    check(output.triangles.size() == 4 && output.triangles[2] == std::array<std::uint64_t,3>{1,2,3} && output.triangles[3] == std::array<std::uint64_t,3>{6,1,5},"one-hop mapping retains nondegenerate duplicates and vertex order");
    equivalence = {{4,1}};
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.equivalentTriangles == std::vector<std::array<int,2>>{{0,1},{0,2}} && result.collapsedTriangles == std::vector<int>{3} && output.triangles.size() == 3,"three equivalent faces report first-with-each pairs and collapsed face discarded");
    equivalence.clear(); result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.equivalenceDiagnosticsEvaluated && result.equivalentTriangles == std::vector<std::array<int,2>>{{0,1}} && output.triangles.size() == 4,"nonnull empty map still diagnoses duplicates and retains faces");
    options = {}; f = fixture(); result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && output.triangles.empty() && output.maximumVertexId == 99,"empty runtime replaces original face triangles");
    f = fixture(); add(f,{1,2,3},1.);
    // 原double叉积平方及法线点积下溢：不能用robust符号强制翻转。
    for(auto& node : f.nodes.nodes) { node.parameters.x *= 1.e-100; node.parameters.y *= 1.e-100; }
    add(f,{2,3,4},.5);
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,output);
    check(result.status.complete() && result.reversedTriangles.empty(),"double normal arithmetic underflow retained");
    FaceMesh guard; guard.entity = 888; guard.triangles = {{77,77,77}};
    RunControl control; control.keepGoing = [] { return false; };
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,guard,control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel(guard),"cancel preserves output");
    int calls = 0; control.keepGoing = [&] { return ++calls < 5; };
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,guard,control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel(guard),"mid-transfer cancel does not publish partial faces");
    control = {}; control.maximumNodes = 5;
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,guard,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel(guard),"vertex budget does not publish output");
    f.nodes.indices.at(2) = 999;
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,guard);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel(guard),"invalid runtime UV slot rejected");
    f = fixture(); add(f,{1,2,9},1.);
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,guard);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel(guard),"missing face vertex rejected");
    f = fixture(); add(f,{1,2,3},1.); equivalence = {{1,999}}; options.equivalence = &equivalence;
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,guard);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel(guard),"missing equivalence destination rejected");
    options = {}; f = fixture(); add(f,{1,2,3},1.);
    result = transferFrontMesh(f.front,f.queue,f.mesh,f.nodes,options,f.mesh);
    check(result.status.complete() && f.mesh.triangles == std::vector<std::array<std::uint64_t,3>>{{1,2,3}},"alias output publishes safely");
    return failures ? 1 : 0;
}
