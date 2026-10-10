/** @file GmshFrontPointInsertionTest.cpp
 * @brief 实际新节点查询/append/正常失败槽、身份与取消恢复。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontInsertion.h"
#include <iostream>
#include <stdexcept>
using namespace mesher; using namespace mesher::gmshReference;
int main() {
    int failures = 0; auto check = [&](bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } };
    FaceMesh originalMesh; originalMesh.entity = 7; originalMesh.maximumVertexId = 4;
    NodeSizeState originalNodes; const Point2D p[] = {{0.,0.},{2.,0.},{0.,2.},{2.,2.}};
    for(int i = 0; i < 4; ++i) { FaceVertex v; v.id = i+1; v.point = {p[i].x,p[i].y,0.}; v.dimension = 1; originalMesh.vertices[v.id] = v; originalNodes.indices[v.id] = i; originalNodes.nodes.push_back({v.id,p[i],{.125,.25}}); }
    FrontMesh originalFront; originalFront.triangles.resize(2); originalFront.triangles[0].vertices = {1,2,3}; originalFront.triangles[1].vertices = {2,4,3};
    originalFront.triangles[0].neighbors[2] = 1; originalFront.triangles[1].neighbors[0] = 0; originalFront.orderedTriangles = {0,1}; for(auto& t : originalFront.triangles) t.radius = 2.;
    auto mesh = originalMesh; auto nodes = originalNodes; auto front = originalFront; FrontQueueState queue; initializeFrontQueue(front,queue); popActiveTriangle(front,queue);
    std::string calls; FrontInsertionSource source;
    source.surfacePoint = [&](Point2D uv) { calls += 'P'; return FrontSurfacePoint{{uv.x,uv.y,0.},true}; };
    source.backgroundSize = [&](const MeshSizeQuery& q) { calls += 'B'; check(q.dimension == 2 && q.entity == 7 && q.u == .5 && q.v == .5 && q.point.x == .5,"BGM gets actual face/UV/XYZ"); MeshSizeEvaluation e; e.size = 1.; return e; };
    auto result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source);
    check(result.status.complete() && result.inserted && result.nativeCode == 1 && calls == "PB","point then independent BGM before successful insertion");
    check(result.allocatedVertex == 5 && result.parameterIndex == 4 && nodes.nodes.size() == 5 && nodes.indices.at(5) == 4 && nodes.nodes[4].sizes.meshSize == .125 && nodes.nodes[4].sizes.backgroundSize == 1.,"new node uses containing mesh interpolation and appended dual-size index");
    check(mesh.vertices.count(5) && mesh.vertices.at(5).origin == FaceVertexOrigin::FaceCreated && mesh.vertices.at(5).dimension == 2 && mesh.interiorVertices == std::vector<std::uint64_t>{5} && mesh.maximumVertexId == 5,"only successful point published to face with consumed identity");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; initializeFrontQueue(front,queue); popActiveTriangle(front,queue); calls.clear();
    result = insertFrontPoint(front,queue,mesh,nodes,0,{3.,3.},{1.,0.,1.},{});
    check(result.status.complete() && !result.inserted && result.nativeCode == 0 && nodes.nodes.size() == 4 && mesh.maximumVertexId == 4 && front.triangles[0].radius == 0.,"missing containing triangle needs no sources/query/append, sets radius0");
    auto closeNodes = originalNodes; for(auto& n : closeNodes.nodes) n.sizes = {1.,1.};
    source.backgroundSize = [&](const MeshSizeQuery&) { calls += 'B'; MeshSizeEvaluation e; e.size = 1.; return e; };
    mesh = originalMesh; nodes = closeNodes; front = originalFront; initializeFrontQueue(front,queue); popActiveTriangle(front,queue); calls.clear();
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.01,.01},{1.,0.,1.},source);
    check(result.status.complete() && !result.inserted && result.nativeCode == -4 && calls == "PB" && nodes.nodes.size() == 5 && !nodes.indices.count(5) && !mesh.vertices.count(5) && mesh.maximumVertexId == 5 && mesh.interiorVertices.empty(),"normal reject retains slot and identity without publishing live point");
    check(front.triangles[0].radius == -1. && !front.triangles[0].deleted && !front.triangles[1].deleted && front.triangles.size() == 2,"outer rejection restores all cavity marks and radius-1");
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.01,.01},{1.,0.,1.},source);
    check(result.status.complete() && result.nativeCode == -4 && result.allocatedVertex == 6 && result.parameterIndex == 5 && nodes.nodes.size() == 6 && nodes.nodes[4].vertex == 5 && nodes.nodes[5].vertex == 6 && nodes.indices.size() == 4,"repeated rejection preserves prior ghost slot and never reuses its index/identity");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; initializeFrontQueue(front,queue); popActiveTriangle(front,queue); calls.clear();
    source.surfacePoint = [&](Point2D uv) { calls += 'P'; return FrontSurfacePoint{{uv.x,uv.y,0.},false}; };
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source);
    check(result.status.complete() && result.nativeCode == -9 && calls == "PB" && result.nodeAppended && nodes.nodes.size() == 5 && front.triangles[0].radius == -1. && !mesh.vertices.count(5),"failed GPoint still queries/appends before native-9 reject");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; initializeFrontQueue(front,queue); popActiveTriangle(front,queue); front.triangles[1].deleted = true;
    source.surfacePoint = [&](Point2D) -> FrontSurfacePoint { throw OperationFailure{{MeshingStatus::Cancelled,"point callback cancelled"}}; };
    result = insertFrontPoint(front,queue,mesh,nodes,0,{2.,2.},{1.,0.,1.},source);
    check(result.status.code == MeshingStatus::Cancelled && !front.triangles[0].deleted && front.triangles[1].deleted && front.triangles[0].radius == 2. && mesh.maximumVertexId == 4 && nodes.nodes.size() == 4,"cancel after searched predeleted seed restores original flags and counters");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; initializeFrontQueue(front,queue); popActiveTriangle(front,queue); calls.clear();
    source.surfacePoint = [&](Point2D uv) { calls += 'P'; return FrontSurfacePoint{{uv.x,uv.y,0.},true}; };
    source.backgroundSize = [&](const MeshSizeQuery&) -> MeshSizeEvaluation { throw std::runtime_error("BGM callback failed"); };
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && result.status.message == "BGM callback failed" && nodes.nodes.size() == 4 && mesh.maximumVertexId == 4 && !front.triangles[0].deleted && queue.allTriangles.size() == 2,"BGM exception restores allocation/cavity and reports query failure");
    RunControl control; control.maximumNodes = 4; calls.clear();
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source,{},control);
    check(result.status.code == MeshingStatus::BudgetExceeded && calls.empty() && nodes.nodes.size() == 4 && !front.triangles[0].deleted,"node budget is explicit without geometry queries or append");
    source.backgroundSize = [&](const MeshSizeQuery&) { calls += 'B'; MeshSizeEvaluation e; e.size = 1.; return e; };
    control = {}; control.maximumGeometryEvaluations = 1; calls.clear();
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source,{},control);
    check(result.status.code == MeshingStatus::BudgetExceeded && calls == "P" && nodes.nodes.size() == 4 && mesh.maximumVertexId == 4 && front.triangles[0].radius == 2.,"query budget after CAD restores consumed identity before BGM");
    auto restored = [&] { return front.triangles.size() == 2 && !front.triangles[0].deleted && !front.triangles[1].deleted && front.triangles[0].neighbors[2] == 1 && front.triangles[1].neighbors[0] == 0 && front.triangles[0].radius == 2. && queue.allTriangles.size() == 2 && queue.activeTriangles.size() == 1 && nodes.nodes.size() == 4 && nodes.indices.size() == 4 && mesh.maximumVertexId == 4 && mesh.vertices.size() == 4 && mesh.interiorVertices.empty(); };
    control = {}; control.keepGoing = [&] { return nodes.nodes.size() <= 4; };
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source,{},control);
    check(result.status.code == MeshingStatus::Cancelled && restored(),"cancel after slot append restores entry without leaving a ghost");
    control.keepGoing = [&] { return queue.allTriangles.size() <= 2; };
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source,{},control);
    check(result.status.code == MeshingStatus::Cancelled && restored(),"cancel after AllTris registration restores pool/local neighbors/slots/counter");
    control.keepGoing = [&] { return queue.activeTriangles.size() <= 1; };
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source,{},control);
    check(result.status.code == MeshingStatus::Cancelled && restored(),"cancel after activity merge restores entry queue");
    FrontInsertionOptions forced; forced.force = true; calls.clear();
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source,forced);
    check(result.status.code == MeshingStatus::InvalidInput && calls.empty() && restored(),"algorithm6 high-level never forces insertion");
    front.triangles[0].neighbors[2] = -1; front.triangles[1].neighbors[0] = -1;
    result = insertFrontPoint(front,queue,mesh,nodes,0,{.5,.5},{1.,0.,1.},source);
    check(result.status.complete() && result.nativeCode == -1 && nodes.nodes.size() == 5 && mesh.maximumVertexId == 5 && !mesh.vertices.count(5) && !front.triangles[0].deleted,"single-cavity early reject still retains allocated slot and identity");
    return failures ? 1 : 0;
}
