/** @file GmshFrontInsertionTest.cpp
 * @brief 原生插点验收/拒绝、局部连接、陈旧队列和取消原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontInsertion.h"
#include <iostream>
#include <stdexcept>
using namespace mesher; using namespace mesher::gmshReference;
int main() {
    int failures = 0; auto check = [&](bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } };
    FaceMesh mesh; NodeSizeState nodes; const Point2D p[] = {{0.,0.},{2.,0.},{0.,2.},{2.,2.}};
    for(int i = 0; i < 4; ++i) { FaceVertex v; v.id = i+1; v.point = {p[i].x,p[i].y,0.}; v.dimension = 1; mesh.vertices[v.id] = v; nodes.indices[v.id] = i; nodes.nodes.push_back({v.id,p[i],{.125,.25}}); }
    FrontMesh original; original.triangles.resize(2); original.triangles[0].vertices = {1,2,3}; original.triangles[1].vertices = {2,4,3};
    original.triangles[0].neighbors[2] = 1; original.triangles[1].neighbors[0] = 0;
    for(auto& t : original.triangles) t.radius = 2.; original.orderedTriangles = {0,1};
    FrontQueueState queue; initializeFrontQueue(original,queue); popActiveTriangle(original,queue);
    FrontCavityResult cavity; cavity.triangles = {0,1}; cavity.shell = {{0,0},{0,1},{1,1},{1,2}};
    auto front = original; for(auto& t : front.triangles) t.deleted = true;
    FaceVertex vertex; vertex.id = 5; vertex.point = {1.,1.,0.}; vertex.dimension = 2;
    InitializedNodeSize node{5,{1.,1.},{.5,1.}};
    auto validation = validateFrontInsertion(front,mesh,nodes,cavity,vertex,node);
    check(validation.status.complete() && validation.nativeCode == 1 && validation.oldArea == 4. && validation.newArea == 4. && validation.triangles.size() == 4,"square cavity accepted with conserved UV area");
    check(validation.triangles.size() == 4 && validation.triangles[0].triangle.vertices == std::array<std::uint64_t,3>{3,1,5} && validation.triangles[0].triangle.targetSize == .25,"shell original orientation and insertion exact ONE_THIRD dual size");
    auto committed = commitFrontInsertion(front,queue,cavity,validation);
    check(committed.status.complete() && committed.createdTriangles == std::vector<int>{2,3,4,5} && front.triangles.size() == 6 && front.triangles[0].deleted && front.triangles[1].deleted,"commit appends shell sequence and keeps original deleted slots");
    check(queue.allTriangles.size() == 6 && queue.activeTriangles.size() == 5 && front.orderedTriangles == std::vector<int>{0,1},"commit registers new triangles, keeps stale active/all and initial snapshot");
    check(front.triangles.size() == 6 && front.triangles[2].neighbors[1] == -1 && front.triangles[2].neighbors[0] >= 2 && front.triangles[2].neighbors[2] >= 2,"new fan base is original edge1; edges0/2 connect sides");
    auto one = cavity; one.triangles = {0}; validation = validateFrontInsertion(original,mesh,nodes,one,vertex,node);
    check(validation.status.complete() && validation.nativeCode == -1 && validation.triangles.empty(),"native single cavity early return");
    auto badEuler = cavity; badEuler.shell.pop_back(); validation = validateFrontInsertion(original,mesh,nodes,badEuler,vertex,node);
    check(validation.status.complete() && validation.nativeCode == -2 && validation.triangles.empty(),"native Euler early return");
    auto outside = node; outside.parameters = {3.,1.}; validation = validateFrontInsertion(original,mesh,nodes,cavity,vertex,outside);
    check(validation.status.complete() && validation.nativeCode == -3,"strict star area rejects point outside shell");
    FrontInsertionOptions options; options.verifyStarShapeness = false; options.force = true;
    validation = validateFrontInsertion(original,mesh,nodes,cavity,vertex,outside,options);
    check(validation.status.complete() && validation.nativeCode == 1,"native relaxed area control retained separately from force");
    auto closeNodes = nodes; for(auto& n : closeNodes.nodes) n.sizes = {1.,1.}; auto close = node; close.parameters = {.01,.01}; close.sizes = {1.,1.}; auto closeVertex = vertex; closeVertex.point = {.01,.01,0.};
    validation = validateFrontInsertion(original,mesh,closeNodes,cavity,closeVertex,close);
    check(validation.status.complete() && validation.nativeCode == -4 && validation.tooClose,"distance/height rejects close candidate");
    options = {}; options.force = true; validation = validateFrontInsertion(original,mesh,closeNodes,cavity,closeVertex,close,options);
    check(validation.status.complete() && validation.nativeCode == 1,"force bypasses distance rejection");
    auto large = nodes; for(auto& n : large.nodes) { n.parameters.x *= 500000.; n.parameters.y *= 500000.; }
    auto equality = node; equality.parameters = {-1.e-6,500000.}; validation = validateFrontInsertion(original,mesh,large,cavity,vertex,equality);
    check(validation.status.complete() && validation.nativeCode == -5 && validation.oldArea == 1.e12 && validation.newArea-validation.oldArea == 1.,"strict area equality retains native -5 code");
    front = original; for(auto& t : front.triangles) t.deleted = true; queue = {}; initializeFrontQueue(original,queue); popActiveTriangle(original,queue);
    auto rejected = insertFrontTriangles(front,queue,mesh,closeNodes,cavity,closeVertex,close);
    check(rejected.status.complete() && rejected.nativeCode == -4 && !front.triangles[0].deleted && !front.triangles[1].deleted && front.triangles.size() == 2,"normal geometry rejection clears cavity marks without appending triangles");
    front = original; for(auto& t : front.triangles) t.deleted = true; rejected = insertFrontTriangles(front,queue,mesh,nodes,one,vertex,node);
    check(rejected.status.complete() && rejected.nativeCode == -1 && front.triangles[0].deleted && front.triangles[1].deleted,"native early -1 preserves marks for outer caller cleanup");
    RunControl control; control.keepGoing = [] { return false; }; validation = validateFrontInsertion(original,mesh,nodes,cavity,vertex,node,{},control);
    check(validation.status.code == MeshingStatus::Cancelled && validation.triangles.empty(),"cancel before validation returns explicit status");
    validation = validateFrontInsertion(original,mesh,nodes,cavity,vertex,node); const auto beforeAll = queue.allTriangles.size(),beforeActive = queue.activeTriangles.size();
    committed = commitFrontInsertion(front,queue,cavity,validation,control);
    check(committed.status.code == MeshingStatus::Cancelled && front.triangles.size() == 2 && queue.allTriangles.size() == beforeAll && queue.activeTriangles.size() == beforeActive,"cancel before commit leaves pool and queues intact");
    control.keepGoing = [&] { return queue.allTriangles.size() <= beforeAll; };
    committed = commitFrontInsertion(front,queue,cavity,validation,control);
    check(committed.status.code == MeshingStatus::Cancelled && front.triangles.size() == 2 && queue.allTriangles.size() == beforeAll && queue.activeTriangles.size() == beforeActive,"cancel after AllTris registration removes only this call additions");
    control.keepGoing = [&] { return queue.activeTriangles.size() <= beforeActive; };
    committed = commitFrontInsertion(front,queue,cavity,validation,control);
    check(committed.status.code == MeshingStatus::Cancelled && front.triangles.size() == 2 && queue.allTriangles.size() == beforeAll && queue.activeTriangles.size() == beforeActive,"cancel after activity merge restores both queue increments");
    control.keepGoing = [&] { if(front.triangles.size() > 2) throw std::runtime_error("commit callback failure"); return true; };
    committed = commitFrontInsertion(front,queue,cavity,validation,control);
    check(committed.status.code == MeshingStatus::InvalidTopology && committed.status.message == "commit callback failure" && front.triangles.size() == 2 && queue.allTriangles.size() == beforeAll,"commit callback exception restores appended pool");
    auto broken = cavity; broken.shell[0].triangle = 99; committed = commitFrontInsertion(front,queue,broken,validation);
    check(committed.status.code == MeshingStatus::InvalidTopology && front.triangles.size() == 2 && queue.allTriangles.size() == beforeAll,"invalid shell cannot partially commit");
    return failures ? 1 : 0;
}
