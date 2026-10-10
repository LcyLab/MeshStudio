#include "mesh/surface/algorithm6/chart/FaceChart.h"
#include <iostream>
#include <limits>
#include <set>
#include <unordered_map>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool value, const char* name) {
        if(!value) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    BoundaryInput cad;
    cad.vertices = {{1,{0.,0.,0.}},{2,{1.,0.,0.}},{3,{1.,1.,0.}},{4,{0.,1.,0.}},{5,{.5,.5,0.}}};
    for(int entity = 1; entity <= 4; ++entity) {
        BoundaryCurveInput c; c.entity = entity; c.kind = CurveKind::Line;
        c.beginVertex = entity; c.endVertex = entity % 4 + 1;
        const auto a = cad.vertices[c.beginVertex-1].point, b = cad.vertices[c.endVertex-1].point;
        c.point = [=](double t) { return Point3D{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,0.}; };
        c.derivative = [=](double) { return Point3D{b.x-a.x,b.y-a.y,0.}; };
        c.inverseParameter = [](Point3D) { return 0.; }; cad.curves.push_back(c);
    }
    SurfaceMeshingOptions options; options.sizeMax = 10.;
    BoundaryMesh boundary;
    auto built = buildBoundaryMesh(cad,options,{},boundary);
    check(built.status.complete() && boundary.nodes.size() == 5, "prepare shared square boundary and isolated point");
    FaceChartInput input; input.entity = 8;
    input.boundaryCurves = {{1,false},{2,false},{3,false},{4,false}};
    input.embeddedVertices = {5};
    input.nodeCoordinates = [](const BoundaryNode& n) { return Point2D{n.point.x,n.point.y}; };
    input.surfacePoint = [](Point2D p) { return Point3D{p.x,p.y,7.}; };
    FaceChart chart;
    auto result = buildFaceChart(boundary,input,chart);
    check(result.status.complete() && chart.route == FaceChartRoute::General && chart.vertices.size() == 5 &&
          chart.constraints.size() == 4 && chart.initialVertices.size() == 5,
          "ordinary chart includes embedded point without adding an outer constraint");
    if(result.status.complete() && chart.vertices.size() == 5) {
        check(chart.vertices[0].localId == 0 && chart.vertices[0].physicalId == 1 && chart.vertices[4].physicalId == 5 &&
              chart.vertices[4].entity == 5 && chart.vertices[4].surfacePoint.z == 7. && boundary.nodes[4].point.z == 0.,
              "BDS numbering classification and evaluated surface point remain separate from physical point");
        check(chart.constraints[3].from == 3 && chart.constraints[3].to == 0 && !chart.constraints[3].embedded,
              "constraint direction follows physical edge chain");
        std::unordered_map<std::uint64_t,int> nativeOrder;
        for(int edge = 1; edge <= 4; ++edge) {
            std::set<std::uint64_t> endpoints{std::uint64_t(edge),std::uint64_t(edge%4+1)};
            for(auto id : endpoints) nativeOrder.emplace(id,0);
        }
        nativeOrder.emplace(5,0);
        std::vector<std::uint64_t> expected, actual;
        for(const auto& v : nativeOrder) expected.push_back(v.first);
        for(const auto& v : chart.initialVertices) actual.push_back(v.physicalId);
        check(actual == expected, "default initial insertion retains native unordered-map enumeration");
    }
    FaceChart sentinel; sentinel.entity = 123;
    auto invalid = input; invalid.boundaryCurves.pop_back();
    result = buildFaceChart(boundary,invalid,sentinel);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel.entity == 123,
          "open boundary reports parity failure and retains output");
    invalid = input; invalid.boundaryCurves[0].seam = true;
    result = buildFaceChart(boundary,invalid,sentinel);
    check(result.status.code == MeshingStatus::NonConverged && result.route == FaceChartRoute::PeriodicRequired && sentinel.entity == 123,
          "seam requires periodic route without publishing a flattened chart");
    invalid = input; invalid.embeddedCurves = {{1,false}};
    result = buildFaceChart(boundary,invalid,chart);
    check(result.status.complete() && chart.constraints.size() == 5 && chart.constraints.back().embedded &&
          chart.vertices.size() == 5 && chart.boundaryVersions.size() == 5,
          "embedded edge records constraint and version independently of boundary parity");
    invalid.embeddedCurves[0].seam = true;
    result = buildFaceChart(boundary,invalid,sentinel);
    check(result.status.code == MeshingStatus::NonConverged && result.route == FaceChartRoute::PeriodicRequired && sentinel.entity == 123,
          "embedded seam cannot silently lose additional UV copies");
    RunControl limited; limited.maximumGeometryEvaluations = 1;
    result = buildFaceChart(boundary,input,sentinel,limited);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 1 && sentinel.entity == 123,
          "face geometry budget has explicit atomic status");
    limited = {}; limited.maximumNodes = 4;
    result = buildFaceChart(boundary,input,sentinel,limited);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel.entity == 123, "chart node budget retains output");
    limited = {}; limited.keepGoing = [] { return false; };
    result = buildFaceChart(boundary,input,sentinel,limited);
    check(result.status.code == MeshingStatus::Cancelled && sentinel.entity == 123, "cancelled face chart retains output");
    invalid = input; invalid.nodeCoordinates = [](const BoundaryNode&) { return Point2D{std::numeric_limits<double>::quiet_NaN(),0.}; };
    result = buildFaceChart(boundary,invalid,sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel.entity == 123, "nonfinite UV is explicit query failure");
    invalid = input; invalid.boundaryCurves[0].entity = 999;
    result = buildFaceChart(boundary,invalid,sentinel);
    check(result.status.code == MeshingStatus::InvalidTopology, "unknown CAD edge rejected");
    invalid = input; invalid.embeddedVertices = {999};
    result = buildFaceChart(boundary,invalid,sentinel);
    check(result.status.code == MeshingStatus::InvalidTopology, "unknown embedded identity rejected");
    FaceChartInput small; small.entity = 9; small.embeddedVertices = {1,2,3};
    result = buildFaceChart(boundary,small,chart);
    check(result.status.complete() && chart.route == FaceChartRoute::DirectTriangle && chart.vertices.size() == 3 &&
          result.geometryEvaluations == 0 && chart.initialVertices.empty(),
          "exactly three physical nodes follows direct triangle path without UV queries");
    small.embeddedVertices.pop_back();
    result = buildFaceChart(boundary,small,chart);
    check(result.status.complete() && chart.route == FaceChartRoute::SkipInsufficientNodes && chart.vertices.size() == 2,
          "fewer than three physical nodes follows native skip path");
    std::cout << "face chart failures=" << failures << '\n';
    return failures ? 1 : 0;
}
