/** @file GmshNodeSizeInitializationTest.cpp
 * @brief 前沿邻边尺寸/embedded/半段/等价边/UV覆盖和失败原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/NodeSizeInitialization.h"
#include <iostream>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
FaceMesh fixture() {
    FaceMesh mesh; mesh.entity = 9; mesh.maximumVertexId = 7;
    for(int id : {1,2,3,6,7}) {
        FaceVertex v; v.id = id; v.physicalId = id; v.entity = 10+id; v.dimension = id == 1 || id == 6 ? 0 : 1;
        v.point = id == 2 ? Point3D{4.,0.,0.} : id == 3 ? Point3D{0.,3.,0.} : id == 6 ? Point3D{10.,0.,0.} : id == 7 ? Point3D{.1,0.,0.} : Point3D{};
        v.storedSize = .01;
        if(id == 7) { v.physicalId = 0; v.origin = FaceVertexOrigin::FaceCreated; v.dimension = 2; v.entity = mesh.entity; }
        mesh.vertices.emplace(id,v);
    }
    for(int id : {4,5}) { auto v = mesh.vertices.at(1); v.id = id; v.origin = FaceVertexOrigin::PeriodicCopy; mesh.vertices.emplace(id,v); mesh.equivalence.emplace(id,1); }
    mesh.triangles = {{1,2,3},{1,7,3}};
    return mesh;
}
bool sentinel(const NodeSizeState& state) { return state.indices == std::map<std::uint64_t,std::size_t>{{999,888}} && state.nodes.empty(); }
}
int main() {
    auto mesh = fixture(); NodeSizeInitializationInput input;
    input.vertexOrder = {3,6,2,1,7}; input.embeddedVertices = {{1,.05},{6,.25}};
    input.boundaryCurves = {{false,{{1,2}}},{true,{{999,998}}}};
    input.embeddedCurves = {{false,{{1,7},{7,1}}},{true,{{997,996}}}};
    mesh.parametricCoordinates[1] = {99.,88.};
    SurfaceMeshingOptions options; options.sizeMax = 100.; options.sizeFromPoints = false;
    GeometrySizeSource source; std::vector<MeshSizeQuery> bgm;
    source.backgroundField = [&](const MeshSizeQuery& q) { bgm.push_back(q); return .75; };
    std::vector<std::uint64_t> calls;
    auto reparam = [&](const FaceVertex& v) { calls.push_back(v.id); return Point2D{v.id*.125,v.id*.25}; };
    NodeSizeState output;
    auto result = initializeNodeSizes(mesh,input,reparam,100.,options,source,output);
    check(result.status.complete() && output.nodes.size() == 5,"successful node initialization");
    check(calls == input.vertexOrder && result.reparameterizations.size() == 5,"explicit identity order and every raw CAD query retained");
    check(result.embeddedQueries.size() == 2 && bgm.size() == 2,"only embedded original nodes query BGM");
    if(bgm.size() == 2) check(bgm[0].dimension == 2 && bgm[0].entity == 9 && bgm[0].point.x == 10. && bgm[1].u == .125 && bgm[1].v == .25,"embedded face entity/original XYZ/raw UV before periodic override");
    if(output.nodes.size() == 5) {
        const auto& v1 = output.nodes.at(output.indices.at(1)); const auto& v6 = output.nodes.at(output.indices.at(6));
        check(v1.sizes.meshSize == 2. && v1.sizes.backgroundSize == .75,"boundary half length overrides embedded prescription, BGM stays independent");
        check(v6.sizes.meshSize == 0. && v6.sizes.backgroundSize == .75,"unreferenced embedded map insertion has native zero initial size");
        check(v1.parameters.x == 99. && v1.parameters.y == 88. && result.reparameterizations[3].parameters.x == .125,"periodic UV override after actual CAD reparameterization");
        check(output.nodes.at(output.indices.at(2)).sizes.meshSize == 4. && output.nodes.at(output.indices.at(2)).sizes.backgroundSize == 4.,"ordinary dual size from shortest physical edge, stored curve size ignored");
        check(output.internalEdges == std::set<std::array<std::uint64_t,2>>{{1,7}},"embedded constraints deduplicated without adding boundary edges or degenerate curves");
    }
    mesh = fixture(); mesh.triangles = {{1,4,2}}; input = {}; input.vertexOrder = {4,1,2};
    result = initializeNodeSizes(mesh,input,reparam,100.,options,source,output);
    check(result.status.complete() && output.nodes.size() == 3 && output.nodes[0].sizes.meshSize == 4. && output.nodes[1].sizes.meshSize == 4.,"fake-original zero edge excluded symmetrically");
    mesh.triangles.push_back({4,5,2}); input.vertexOrder = {5,4,1,2};
    result = initializeNodeSizes(mesh,input,reparam,100.,options,source,output);
    check(result.status.complete() && output.nodes.size() == 4 && output.nodes[0].sizes.meshSize == 0. && output.nodes[1].sizes.meshSize == 0.,"two fakes sharing original are distinct, native zero edge retained");
    NodeSizeState guard; guard.indices.emplace(999,888); RunControl control; control.keepGoing = [] { return false; };
    result = initializeNodeSizes(mesh,input,reparam,100.,options,source,guard,control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel(guard),"cancel atomic output");
    control = {}; control.maximumNodes = 3;
    result = initializeNodeSizes(mesh,input,reparam,100.,options,source,guard,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel(guard),"actual added node budget atomic");
    control = {}; control.maximumGeometryEvaluations = 1;
    result = initializeNodeSizes(mesh,input,reparam,100.,options,source,guard,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel(guard) && result.reparameterizations.size() == 1,"CAD query budget includes all reparameterizations");
    auto invalid = input; invalid.vertexOrder.back() = 5;
    result = initializeNodeSizes(mesh,invalid,reparam,100.,options,source,guard);
    check(result.status.code == MeshingStatus::InvalidInput && sentinel(guard),"duplicate or missing node order rejected");
    result = initializeNodeSizes(mesh,input,[](const FaceVertex&) -> Point2D { throw std::runtime_error("CAD unavailable"); },100.,options,source,guard);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel(guard),"CAD failure atomic and explicit");
    auto broken = mesh; broken.triangles[0][1] = 999;
    result = initializeNodeSizes(broken,input,reparam,100.,options,source,guard);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel(guard),"missing triangle node rejected");
    mesh = fixture(); input = {}; input.vertexOrder = {1,2,3,7}; input.embeddedVertices = {{1,.5}};
    control = {}; control.maximumGeometryEvaluations = 1;
    result = initializeNodeSizes(mesh,input,reparam,100.,options,source,guard,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel(guard) && result.reparameterizations.size() == 1 && result.embeddedQueries.empty(),"embedded BGM consumes a separate geometry evaluation after CAD reparameterization");
    source.backgroundField = [](const MeshSizeQuery&) -> double { throw std::runtime_error("BGM unavailable"); };
    result = initializeNodeSizes(mesh,input,reparam,100.,options,source,guard);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel(guard) && result.embeddedQueries.size() == 1,"embedded BGM failure preserves output and actual failed query");
    bool running = true; control = {}; control.keepGoing = [&] { return running; };
    result = initializeNodeSizes(mesh,input,[&](const FaceVertex&) { running = false; return Point2D{}; },100.,options,source,guard,control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel(guard) && result.reparameterizations.size() == 1 && result.embeddedQueries.empty(),"cancellation between CAD reparameterization and BGM is explicit and atomic");
    FaceMesh empty; empty.entity = 9;
    result = initializeNodeSizes(empty,{}, {},100.,options,{},output);
    check(result.status.complete() && output.nodes.empty() && output.indices.empty() && output.internalEdges.empty(),"empty node stage preserved; front empty-triangle error belongs to driver");
    return failures ? 1 : 0;
}
