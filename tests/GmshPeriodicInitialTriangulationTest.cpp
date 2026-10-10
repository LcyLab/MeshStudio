#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
#include <cmath>
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok,const char* name) { if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    PeriodicFaceChart chart; auto& boundary = chart.boundary;
    boundary.entity = 7; boundary.hasBounds = true; boundary.lower = {0.,0.}; boundary.upper = {4.,2.};
    boundary.loopPositionCount = 4; boundary.maximumLocalId = 3; boundary.loops = {{0,1,2,3}};
    const std::vector<Point2D> uv{{0.,0.},{4.,0.},{4.,2.},{0.,2.}};
    for(int i = 0; i < 4; ++i) {
        boundary.vertices.push_back({{i,std::uint64_t(i+1),0,i+1,uv[i],{uv[i].x,uv[i].y,0.}},{.2,.2},-1});
        chart.initialPoints.push_back({i,{uv[i].x+(i+1)*1.e-9,uv[i].y+(4-i)*1.e-9}});
    }
    chart.expectedInitialPointCount = 4;
    PeriodicFaceBoundaryInput geometry; geometry.entity = 7; geometry.parameterLower = {0.,0.}; geometry.parameterUpper = {4.,2.};
    int calls = 0; geometry.surfacePoint = [&](Point2D p) { ++calls; return Point3D{p.x,p.y,p.x*p.y}; };
    InitialTriangulation output;
    auto result = buildPeriodicInitialTriangulation(chart,geometry,output);
    check(result.status.complete() && calls == 4 && result.geometryEvaluations == 4 && output.vertices.size() == 8 &&
          output.triangles.size() == 10 && output.cubicBounds,"four native outer points then divide/conquer");
    if(output.vertices.size() == 8) {
        const double half = std::sqrt(20.)*.5;
        double lo = 2.-1., hi = 2.+1.;
        const double center = .5*(lo+hi); lo = (lo-center)*half+center; hi = (hi-center)*half+center;
        const double cubeCenter = .5*(lo+hi); lo = (lo-cubeCenter)*3.5+cubeCenter; hi = (hi-cubeCenter)*3.5+cubeCenter;
        check(output.outerLower.x == lo && output.outerUpper.x == hi && output.outerLower.y == lo-1. &&
              output.vertices[4].vertex.localId == -1 && output.vertices[5].vertex.localId == -2 &&
              output.vertices[7].vertex.localId == -4 && output.vertices[4].sizes.meshSize == 1.e22 &&
              output.vertices[4].vertex.entity == 7 && output.vertices[4].vertex.dimension == 2,
              "cube uses diagonal length, native arithmetic, IDs and untouched size defaults");
        check(output.vertices[0].vertex.uv.x == 0. && output.triangulation.sortedPoints[0].localId < 0,
              "raw BDS UV and perturbed sorted record remain separate");
    }
    const auto prior = output.vertices.size(); RunControl control; control.maximumGeometryEvaluations = 3;
    result = buildPeriodicInitialTriangulation(chart,geometry,output,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 3 && output.vertices.size() == prior,
          "outer CAD query budget is atomic");
    control = {}; control.maximumNodes = 7;
    result = buildPeriodicInitialTriangulation(chart,geometry,output,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && output.vertices.size() == prior,"temporary outer points count toward node budget");
    auto skinny = geometry; skinny.parameterUpper = {5000.,1.};
    result = buildPeriodicInitialTriangulation(chart,skinny,output);
    check(result.status.complete() && !output.cubicBounds && output.outerLower.x == -5. && output.outerUpper.x == 9. &&
          output.outerLower.y == -2.5 && output.outerUpper.y == 4.5,"skinny CAD parameter span bypasses makeCube");
    auto bad = chart; bad.initialPoints[0].localId = 999;
    result = buildPeriodicInitialTriangulation(bad,geometry,output);
    check(result.status.code == MeshingStatus::InvalidTopology && output.vertices.size() == prior,"unknown initial identity rejected");
    auto early = chart; early.boundary.route = FaceChartRoute::DirectTriangle; early.boundary.vertices.resize(3);
    early.boundary.loopPositionCount = 3; early.initialPoints.clear(); early.expectedInitialPointCount = 0;
    calls = 0; result = buildPeriodicInitialTriangulation(early,{},output);
    check(result.status.complete() && calls == 0 && output.triangles == std::vector<std::array<int,3>>{{0,1,2}},"native direct triangle bypasses CAD and DocRecord");
    std::cout << "periodic initial failures=" << failures << '\n'; return failures ? 1 : 0;
}
