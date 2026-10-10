/** @file GmshFrontCavityTest.cpp
 * @brief 前沿无减容差圆/原DFS壳序/内部边/标记恢复和实际包含片。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontCavity.h"
#include <iostream>
#include <stdexcept>
using namespace mesher; using namespace mesher::gmshReference;
int main() {
    int failures = 0; auto check = [&](bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } };
    NodeSizeState nodes; const Point2D p[] = {{0.,0.},{2.,0.},{0.,2.},{2.,2.}};
    for(int i = 0; i < 4; ++i) { nodes.indices[i+1] = i; nodes.nodes.push_back({std::uint64_t(i+1),p[i],{1.,1.}}); }
    FrontMesh original; original.triangles.resize(2); original.triangles[0].vertices = {1,2,3}; original.triangles[1].vertices = {2,4,3};
    original.triangles[0].neighbors[2] = 1; original.triangles[1].neighbors[0] = 0;
    FrontTriangleSet all; for(int i = 0; i < 2; ++i) all.insert(makeFrontTriangleKey(original.triangles[i],i));
    auto mesh = original; auto cavity = buildFrontCavity(mesh,nodes,all,0,{.5,.5},{1.,0.,1.});
    check(cavity.status.complete() && cavity.worstInsideCircle && cavity.triangles == std::vector<int>{0,1} && mesh.triangles[0].deleted && mesh.triangles[1].deleted,"DFS cavity marks both original triangles");
    check(cavity.shell.size() == 4 && cavity.shell[0].triangle == 0 && cavity.shell[0].edge == 0 && cavity.shell[1].triangle == 0 && cavity.shell[1].edge == 1 && cavity.shell[2].triangle == 1 && cavity.shell[2].edge == 1 && cavity.shell[3].triangle == 1 && cavity.shell[3].edge == 2,"shell follows recursive native edge order");
    check(cavity.status.complete() && cavity.containingTriangle == 0 && cavity.coordinates.local.x == .25 && cavity.coordinates.local.y == .25,"first containing triangle in cavity visitation order");
    auto status = rollbackFrontCavity(mesh,cavity);
    check(status.complete() && !mesh.triangles[0].deleted && !mesh.triangles[1].deleted,"rollback restores new cavity marks");
    nodes.internalEdges.insert({2,3}); mesh = original; cavity = buildFrontCavity(mesh,nodes,all,0,{1.5,1.5},{1.,0.,1.});
    check(cavity.status.complete() && cavity.triangles == std::vector<int>{0} && cavity.shell.size() == 3 && cavity.containingTriangle == -1 && !mesh.triangles[1].deleted,"embedded edge blocks expansion even when candidate inside neighbor circle");
    nodes.internalEdges.clear(); mesh = original; cavity = buildFrontCavity(mesh,nodes,all,0,{2.,2.},{1.,0.,1.});
    check(cavity.status.complete() && !cavity.worstInsideCircle && cavity.search.triangle == 1 && cavity.triangles == std::vector<int>{1} && cavity.containingTriangle == 1,"on-circle strict comparison takes search branch then marks containing seed");
    mesh = original; mesh.triangles[1].deleted = true; cavity = buildFrontCavity(mesh,nodes,all,0,{.5,.5},{1.,0.,1.});
    check(cavity.status.complete() && cavity.triangles == std::vector<int>{0} && cavity.shell.size() == 2,"predeleted neighbor contributes no shell edge");
    status = rollbackFrontCavity(mesh,cavity); check(status.complete() && !mesh.triangles[0].deleted && mesh.triangles[1].deleted,"rollback preserves prior deleted neighbor");
    mesh = original; mesh.triangles[1].deleted = true; cavity = buildFrontCavity(mesh,nodes,all,0,{2.,2.},{1.,0.,1.});
    check(cavity.status.complete() && cavity.containingTriangle == 1 && cavity.triangles == std::vector<int>{1},"native search may enter predeleted containing seed");
    status = rollbackFrontCavity(mesh,cavity); check(status.complete() && !mesh.triangles[1].deleted,"normal native rollback clears even predeleted searched seed");
    mesh = original; mesh.triangles[0].neighbors[2] = 99; cavity = buildFrontCavity(mesh,nodes,all,0,{.5,.5},{1.,0.,1.});
    check(cavity.status.code == MeshingStatus::InvalidTopology && !mesh.triangles[0].deleted && !mesh.triangles[1].deleted,"invalid neighbor restores this call marks");
    mesh = original; RunControl control; control.keepGoing = [&] { return !mesh.triangles[0].deleted; };
    cavity = buildFrontCavity(mesh,nodes,all,0,{.5,.5},{1.,0.,1.},false,control);
    check(cavity.status.code == MeshingStatus::Cancelled && !mesh.triangles[0].deleted && !mesh.triangles[1].deleted,"cancel during expansion restores this call marks");
    // 调用取消/异常保留进入前的标记；区别于正常原生失败清除 searched seed。
    mesh = original; mesh.triangles[1].deleted = true;
    control.keepGoing = [&] { return !mesh.triangles[0].deleted; };
    cavity = buildFrontCavity(mesh,nodes,all,0,{.5,.5},{1.,0.,1.},false,control);
    check(cavity.status.code == MeshingStatus::Cancelled && !mesh.triangles[0].deleted && mesh.triangles[1].deleted,"cancel restores both new and prior deleted states");
    mesh = original; mesh.triangles[1].deleted = true; mesh.triangles[1].neighbors[1] = 99;
    cavity = buildFrontCavity(mesh,nodes,all,0,{2.,2.},{1.,0.,1.});
    check(cavity.status.code == MeshingStatus::InvalidTopology && !mesh.triangles[0].deleted && mesh.triangles[1].deleted,"searched predeleted seed survives failed expansion");
    mesh = original; control.keepGoing = [&] { if(mesh.triangles[0].deleted) throw std::runtime_error("callback failure"); return true; };
    cavity = buildFrontCavity(mesh,nodes,all,0,{.5,.5},{1.,0.,1.},false,control);
    check(cavity.status.code == MeshingStatus::InvalidTopology && cavity.status.message == "callback failure" && !mesh.triangles[0].deleted && !mesh.triangles[1].deleted,"callback exception explicitly restores marks");
    mesh = original; mesh.triangles[0].deleted = true; cavity.triangles = {0,99}; status = rollbackFrontCavity(mesh,cavity);
    check(status.code == MeshingStatus::InvalidTopology && mesh.triangles[0].deleted,"invalid rollback list changes no valid marks");
    return failures ? 1 : 0;
}
