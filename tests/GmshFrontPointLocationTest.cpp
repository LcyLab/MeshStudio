/** @file GmshFrontPointLocationTest.cpp
 * @brief 原UV反映射/严格容差/有符号插值/搜索边序和force语义。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontPointLocation.h"
#include <cmath>
#include <iostream>
using namespace mesher; using namespace mesher::gmshReference;
int main() {
    int failures = 0; auto check = [&](bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } };
    NodeSizeState nodes; const Point2D p[] = {{0.,0.},{2.,0.},{0.,2.},{2.,2.}};
    for(int i = 0; i < 4; ++i) { nodes.indices[i+1] = i; nodes.nodes.push_back({std::uint64_t(i+1),p[i],{double(i+1),99.}}); }
    FrontMesh mesh; mesh.triangles.resize(2); mesh.triangles[0].vertices = {1,2,3}; mesh.triangles[1].vertices = {2,4,3};
    mesh.triangles[0].neighbors[2] = 1; mesh.triangles[1].neighbors[0] = 0;
    FrontTriangleSet all; for(int i = 0; i < 2; ++i) all.insert(makeFrontTriangleKey(mesh.triangles[i],i));
    auto mapped = mapFrontPointToTriangle(mesh.triangles[0],nodes,{.5,.5});
    check(mapped.status.complete() && mapped.inside && mapped.local.x == .25 && mapped.local.y == .25,"native UV barycentric coordinates");
    mapped = mapFrontPointToTriangle(mesh.triangles[0],nodes,{2.25,0.},.125);
    check(mapped.status.complete() && !mapped.inside && mapped.local.x == 1.125,"last barycentric tolerance bound is strict");
    mapped = mapFrontPointToTriangle(mesh.triangles[0],nodes,{std::nextafter(2.25,0.),0.},.125);
    check(mapped.status.complete() && mapped.inside,"nextafter inside strict tolerance");
    auto degenerate = nodes; for(auto& n : degenerate.nodes) n.parameters = {};
    mapped = mapFrontPointToTriangle(mesh.triangles[0],degenerate,{100.,100.});
    check(mapped.status.complete() && mapped.singularSystem && mapped.inside && mapped.local.x == 0. && mapped.local.y == 0.,"native ignored singular solver still reports inside with diagnostic");
    double size = 77.; auto status = interpolateNodeMeshSize(mesh.triangles[0],nodes,{.25,.25},size);
    check(status.complete() && size == 1.75,"interpolate only mesh size from actual containing triangle");
    status = interpolateNodeMeshSize(mesh.triangles[0],nodes,{2.,-1.},size);
    check(status.complete() && size == 1.,"weights are not clamped or replaced by background size");
    auto found = findContainingFrontTriangle(mesh,nodes,all,0,{1.5,1.5});
    check(found.status.complete() && found.triangle == 1 && found.visitedTriangles == std::vector<int>{0,1} && found.crossedEdges == std::vector<int>{2} && found.coordinates.local.x == .5 && found.coordinates.local.y == .25,"search crosses original edge2 to actual containing triangle");
    found = findContainingFrontTriangle(mesh,nodes,all,0,{3.,3.});
    check(found.status.complete() && found.triangle == -1 && found.crossedEdges == std::vector<int>{2,1} && found.forceScannedTriangles.empty(),"outside search stops at null neighbor, no implicit global fallback");
    auto unlinked = mesh; unlinked.triangles[0].neighbors[2] = -1;
    found = findContainingFrontTriangle(unlinked,nodes,all,0,{1.5,1.5},true);
    check(found.status.complete() && found.triangle == 1 && found.forceScannedTriangles == std::vector<int>{0,1},"explicit force scans AllTris in original set order");
    check(intersectFrontSearchSegments({0.,0.},{1.,0.},{2.,0.},{3.,0.}),"native collinear disjoint intersection is retained");
    RunControl control; control.keepGoing = [] { return false; };
    found = findContainingFrontTriangle(mesh,nodes,all,0,{.5,.5},false,control);
    check(found.status.code == MeshingStatus::Cancelled && found.visitedTriangles.empty(),"cancel before search");
    auto broken = nodes; broken.indices.at(1) = 999;
    status = interpolateNodeMeshSize(mesh.triangles[0],broken,{0.,0.},size);
    check(status.code == MeshingStatus::InvalidTopology && size == 1.,"invalid node index does not publish size");
    return failures ? 1 : 0;
}
