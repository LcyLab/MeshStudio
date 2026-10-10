#include "mesh/surface/algorithm6/initial/WalkTriangulation.h"
#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
#include "geometry/predicates/GeomPredicates.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok,const char* name) { if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    WalkTriangulation mesh;
    auto status = buildWalkTriangulation({-2.,-3.},{2.,3.},{},mesh);
    check(status.complete() && mesh.vertices.size() == 4 && mesh.faces.size() == 2 && mesh.halfEdges.size() == 6 &&
          mesh.faces[0].halfEdge == 0 && mesh.faces[1].halfEdge == 3 && mesh.halfEdges[0].opposite == 3 &&
          mesh.halfEdges[3].opposite == 0 && mesh.vertices[0].incidentHalfEdge == 4,"native rectangle pool/head identities");
    std::vector<InitialChartVertex> points{{91,{0.,0.}},{37,{.2,.3}},{16,{-.5,.2}}};
    for(int i = 0; i < 50; ++i) points.push_back({std::uint64_t(100+i),{std::sin(i*1.234),std::cos(i*2.345)}});
    status = buildWalkTriangulation({-2.,-3.},{2.,3.},points,mesh);
    bool valid = status.complete() && mesh.vertices.size() == points.size()+4 && mesh.faces.size() == 2*points.size()+2 &&
                 mesh.halfEdges.size() == 6*points.size()+6;
    if(valid) {
        for(std::size_t i = 0; i < points.size(); ++i) valid &= mesh.vertices[i+4].physicalId == points[i].physicalId;
        for(std::size_t i = 0; i < mesh.halfEdges.size(); ++i) {
            const auto& e = mesh.halfEdges[i];
            valid &= e.next >= 0 && e.previous >= 0 && e.origin >= 0 && e.face >= 0 &&
                     mesh.halfEdges[e.next].previous == int(i) && mesh.halfEdges[e.previous].next == int(i);
            if(e.opposite >= 0) valid &= mesh.halfEdges[e.opposite].opposite == int(i) &&
                mesh.halfEdges[e.opposite].origin == mesh.halfEdges[e.next].origin;
        }
        for(const auto& f : mesh.faces) {
            const auto& e = mesh.halfEdges[f.halfEdge]; const auto& b = mesh.halfEdges[e.next]; const auto& c = mesh.halfEdges[b.next];
            const auto aUV = mesh.vertices[e.origin].uv, bUV = mesh.vertices[b.origin].uv, cUV = mesh.vertices[c.origin].uv;
            valid &= orient2d(aUV,bUV,cUV) < 0;
            for(const auto& p : mesh.vertices) valid &= incircle(aUV,bUV,cUV,p.uv) <= 0;
        }
    }
    check(valid,"ordered insertion yields reciprocal clockwise Delaunay topology");
    const auto previousCount = mesh.vertices.size();
    status = buildWalkTriangulation({-2.,-3.},{2.,3.},{{1,{3.,0.}}},mesh);
    check(status.code == MeshingStatus::InvalidInput && mesh.vertices.size() == previousCount,"outside point fails atomically");
    status = buildWalkTriangulation({-2.,-3.},{2.,3.},{{1,{std::numeric_limits<double>::quiet_NaN(),0.}}},mesh);
    check(status.code == MeshingStatus::InvalidInput && mesh.vertices.size() == previousCount,"nonfinite point fails atomically");
    RunControl control; control.maximumNodes = points.size()+3;
    status = buildWalkTriangulation({-2.,-3.},{2.,3.},points,mesh,control);
    check(status.code == MeshingStatus::BudgetExceeded && mesh.vertices.size() == previousCount,"outer vertices count against node budget");
    control = {}; int calls = 0; control.keepGoing = [&] { return ++calls < 25; };
    status = buildWalkTriangulation({-2.,-3.},{2.,3.},points,mesh,control);
    check(status.code == MeshingStatus::Cancelled && mesh.vertices.size() == previousCount,"mid-walk cancellation preserves previous mesh");
    FaceChart chart; chart.initialVertices = {{91,{-1.,-.5}},{37,{1.,-.5}},{16,{1.,.5}},{28,{-1.,.5}}};
    for(std::size_t i = 0; i < chart.initialVertices.size(); ++i) {
        FaceChartVertex v; v.localId = int(i); v.physicalId = chart.initialVertices[i].physicalId; chart.vertices.push_back(v);
    }
    OrdinaryInitialTriangulation initial;
    status = buildInitialTriangulation(chart,initial);
    check(status.complete() && initial.outerLower.x == -1.1 && initial.outerUpper.y == .55 &&
          initial.mesh.vertices.size() == 8 && initial.mesh.vertices[4].physicalId == 91,"ordinary raw box expands 1.1 without sorting or CAD");
    chart.route = FaceChartRoute::DirectTriangle; chart.vertices.pop_back();
    std::swap(chart.vertices[0],chart.vertices[2]);
    status = buildInitialTriangulation(chart,initial);
    check(status.complete() && initial.mesh.vertices.empty() && initial.directTriangles == std::vector<std::array<int,3>>{{0,1,2}},"three-point early route uses BDS numbering without Walk");
    chart.route = FaceChartRoute::SkipInsufficientNodes;
    status = buildInitialTriangulation(chart,initial);
    check(status.complete() && initial.mesh.vertices.empty() && initial.directTriangles.empty(),"insufficient early route has no outer vertices");
    std::cout << "Walk triangulation failures=" << failures << '\n'; return failures ? 1 : 0;
}
