#include "mesh/surface/algorithm6/boundary/BoundaryRegistry.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool value, const char* name) {
        if(!value) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    BoundaryInput input; input.modelCharacteristicLength = 100.;
    input.vertices = {{9,{0.,0.,0.}},{3,{1.,0.,0.}}};
    BoundaryCurveInput curve; curve.entity = 7; curve.beginVertex = 9; curve.endVertex = 3;
    curve.kind = CurveKind::Other; curve.incidentFaces = {8,4,8};
    curve.point = [](double t) { return Point3D{t*t*t,0.,0.}; };
    curve.derivative = [](double t) { return Point3D{3.*t*t,0.,0.}; };
    curve.inverseParameter = [](Point3D p) { return std::cbrt(p.x); };
    input.curves = {curve};
    SurfaceMeshingOptions options; options.minimumCurvePoints = 2; options.sizeMax = 10.;
    GeometrySizeSource sizes;
    sizes.sizeCallback = [](const MeshSizeQuery& q, double) { return q.dimension ? 8. : q.entity == 9 ? 2. : 4.; };
    BoundaryMesh mesh;
    auto built = buildBoundaryMesh(input,options,sizes,mesh);
    check(built.status.complete() && mesh.nodes.size() == 2, "prepare nonlinear curve with no interior node");
    auto result = refineUnrecoveredEdges(input,options,sizes,{{7,2,1},{7,1,2}},mesh);
    check(result.status.complete() && result.splitSegments == 1 && result.unmatchedRequests == std::vector<std::size_t>({1}) &&
          result.incidentFaces == std::vector<int>({8,4}), "reversed match and explicit stale duplicate with incident faces");
    check(result.status.complete() && mesh.nodes.size() == 3 && mesh.curves[0].version == 1 &&
          mesh.curves[0].samples.size() == 3 && mesh.nodes[2].parameter == .5 && mesh.nodes[2].point.x == .125 &&
          mesh.nodes[2].storedSize == 3., "native parameter midpoint and endpoint BGM size average");
    std::vector<MeshSizeQuery> queries;
    sizes.sizeCallback = [&](const MeshSizeQuery& q, double) { queries.push_back(q); return q.dimension ? 8. : 2.; };
    result = refineUnrecoveredEdges(input,options,sizes,{{7,1,3}},mesh);
    check(result.status.complete() && mesh.nodes.size() == 4 && mesh.nodes[3].parameter == .25 &&
          mesh.nodes[3].storedSize == 2.5 && queries.size() == 1 && queries[0].dimension == 0 && queries[0].u == 0.,
          "stored lc passes through; only unset endpoint queried at parameter zero");
    if(mesh.nodes.size() >= 4) mesh.nodes[3].storedSize = -1.;
    queries.clear();
    result = refineUnrecoveredEdges(input,options,sizes,{{7,4,3}},mesh);
    check(result.status.complete() && mesh.nodes.size() == 5 && mesh.nodes[4].storedSize == 5.5 &&
          queries.size() == 1 && queries[0].dimension == 1 && queries[0].entity == 7 && queries[0].u == 0.,
          "stored lc equal to minus one triggers 1D BGM query, not only for CAD vertices");

    BoundaryInput closed; closed.modelCharacteristicLength = 100.; closed.vertices = {{1,{1.,0.,0.}}};
    auto circle = curve; circle.entity = 1; circle.beginVertex = circle.endVertex = 1;
    circle.kind = CurveKind::Circle; circle.last = 2.*std::acos(-1.);
    circle.point = [](double t) { return Point3D{std::cos(t),std::sin(t),0.}; };
    circle.derivative = [](double t) { return Point3D{-std::sin(t),std::cos(t),0.}; };
    circle.inverseParameter = [](Point3D p) { return std::atan2(p.y,p.x); };
    closed.curves = {circle}; sizes = {};
    built = buildBoundaryMesh(closed,options,sizes,mesh);
    check(built.status.complete() && mesh.curves[0].samples.size() == 8, "prepare closed circle");
    if(built.status.complete()) {
        const auto& samples = mesh.curves[0].samples;
        const auto lastInterior = samples[samples.size()-2].node;
        const double expected = .5*(samples[samples.size()-2].parameter+circle.last);
        result = refineUnrecoveredEdges(closed,options,sizes,{{1,1,lastInterior}},mesh);
        check(result.status.complete() && mesh.nodes.back().parameter == expected && mesh.curves[0].samples.back().node == 1,
              "closed endpoint chooses high branch from other endpoint proximity");
    }
    const auto beforeSize = mesh.nodes.size(); const auto beforeVersion = mesh.curves[0].version;
    auto request = UnrecoveredEdge{1,mesh.curves[0].samples[0].node,mesh.curves[0].samples[1].node};
    RunControl budget; budget.maximumNodes = beforeSize;
    result = refineUnrecoveredEdges(closed,options,sizes,{request},mesh,budget);
    check(result.status.code == MeshingStatus::BudgetExceeded && mesh.nodes.size() == beforeSize &&
          mesh.curves[0].version == beforeVersion && result.incidentFaces.empty() && result.splitSegments == 0,
          "node budget recovery publishes no partial updates or invalidations");
    RunControl cancelled; cancelled.keepGoing = [] { return false; };
    result = refineUnrecoveredEdges(closed,options,sizes,{request},mesh,cancelled);
    check(result.status.code == MeshingStatus::Cancelled && mesh.nodes.size() == beforeSize, "cancelled recovery retains mesh");
    result = refineUnrecoveredEdges(closed,options,sizes,{{999,1,2}},mesh);
    check(result.status.code == MeshingStatus::InvalidInput, "unknown recovery CAD edge is explicit error");
    auto invalid = closed; invalid.curves[0].point = [](double) { return Point3D{std::numeric_limits<double>::quiet_NaN(),0.,0.}; };
    result = refineUnrecoveredEdges(invalid,options,sizes,{request},mesh);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && mesh.nodes.size() == beforeSize,
          "nonfinite recovery CAD point retains node pool and version");
    std::cout << "boundary recovery failures=" << failures << '\n';
    return failures ? 1 : 0;
}
