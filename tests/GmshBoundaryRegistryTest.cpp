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
    BoundaryInput input;
    input.modelCharacteristicLength = 100.;
    input.vertices = {{9,{0.,0.,0.}}, {3,{1.,0.,0.}}, {17,{0.,0.,0.}}, {22,{1.,0.,0.}}};
    BoundaryCurveInput line;
    line.entity = 7; line.beginVertex = 9; line.endVertex = 3; line.kind = CurveKind::Line;
    line.incidentFaces = {8,4,8};
    line.point = [](double t) { return Point3D{t,0.,0.}; };
    line.derivative = [](double) { return Point3D{1.,0.,0.}; };
    line.inverseParameter = [](Point3D p) { return p.x; };
    input.curves = {line};
    auto coincident = line; coincident.entity = 2; coincident.beginVertex = 17; coincident.endVertex = 22;
    input.curves.push_back(coincident);
    SurfaceMeshingOptions options; options.sizeMax = .25;
    GeometrySizeSource sizes;
    BoundaryMesh mesh;
    auto result = buildBoundaryMesh(input, options, sizes, mesh);
    check(result.status.complete() && mesh.curves.size() == 2 && mesh.nodes.size() == 10,
          "two coincident CAD edges remain distinct and ordered");
    if(result.status.complete() && mesh.curves.size() == 2) {
        const auto& a = mesh.curves[0]; const auto& b = mesh.curves[1];
        check(a.samples.size() == 5 && a.beginNode == 1 && a.endNode == 2 &&
              a.samples[1].node == 5 && b.samples[1].node == 8 && a.length == 1.,
              "0D allocation precedes curve nodes; shared edge uses one physical chain");
        check(mesh.nodes[4].dimension == 1 && mesh.nodes[4].entity == 7 &&
              mesh.nodes[4].parameter == .25 && mesh.nodes[4].storedSize == .25,
              "curve node classification parameter and native stored metadata");
    }
    std::vector<int> faces{-9};
    auto status = findIncidentFaces(mesh, 7, faces);
    check(status.complete() && faces == std::vector<int>({8,4}), "incident faces preserve order without duplicates");
    status = findIncidentFaces(mesh, 500, faces);
    check(status.code == MeshingStatus::InvalidInput && faces == std::vector<int>({8,4}),
          "unknown CAD edge retains face output");

    BoundaryInput endpointInput = input; endpointInput.curves.resize(1);
    int vertexQueries = 0;
    sizes.sizeCallback = [&](const MeshSizeQuery& query, double) {
        if(query.dimension == 0) { ++vertexQueries; return .05; }
        return .25;
    };
    result = buildBoundaryMesh(endpointInput, options, sizes, mesh);
    check(result.status.complete() && vertexQueries == 2 && !mesh.curves.empty() &&
          mesh.curves[0].samples.size() > 5, "curve endpoints query both 0D and 1D sizes");

    sizes = {};
    BoundaryInput closed;
    closed.modelCharacteristicLength = 100.; closed.vertices = {{1,{0.,1.,0.}}};
    auto circle = line; circle.entity = 1; circle.beginVertex = circle.endVertex = 1;
    circle.kind = CurveKind::Circle; circle.last = 2.*std::acos(-1.);
    circle.point = [](double t) { return Point3D{std::cos(t),std::sin(t),0.}; };
    circle.derivative = [](double t) { return Point3D{-std::sin(t),std::cos(t),0.}; };
    circle.inverseParameter = [](Point3D p) { return std::atan2(p.y,p.x); };
    closed.curves = {circle}; options.sizeMax = 10.;
    result = buildBoundaryMesh(closed, options, sizes, mesh);
    check(result.status.complete() && mesh.curves.size() == 1 && mesh.curves[0].samples.size() == 8 &&
          mesh.curves[0].samples.front().node == mesh.curves[0].samples.back().node &&
          mesh.nodes[0].point.x == 1. && mesh.nodes[0].point.y == 0. &&
          mesh.curves[0].minimumNodeCountPreventedRemoval && !mesh.curves[0].meshDegenerate,
          "closed endpoints share identity and relocate only after native minimum-gated filtering");
    closed.curves[0].geometryDegenerate = true;
    closed.curves[0].point = {}; closed.curves[0].derivative = {}; closed.curves[0].inverseParameter = {};
    result = buildBoundaryMesh(closed, options, sizes, mesh);
    check(result.status.complete() && mesh.nodes.size() == 1 && mesh.curves.size() == 1 &&
          mesh.curves[0].samples.empty() && mesh.curves[0].geometryDegenerate && mesh.curves[0].meshDegenerate,
          "OCC degenerate edge skipped with physical endpoint retained");

    auto zero = line;
    zero.point = [](double) { return Point3D{0.,0.,0.}; };
    zero.derivative = [](double) { return Point3D{0.,0.,0.}; };
    BoundaryInput zeroInput; zeroInput.vertices = {{9,{0.,0.,0.}},{3,{0.,0.,0.}}}; zeroInput.curves = {zero};
    result = buildBoundaryMesh(zeroInput, options, sizes, mesh);
    check(result.status.complete() && mesh.nodes.size() == 2 && mesh.curves[0].samples.size() == 2 &&
          mesh.curves[0].length == 0. && !mesh.curves[0].tooSmall && !mesh.curves[0].meshDegenerate,
          "zero physical length keeps one segment between distinct CAD identities");

    BoundaryMesh sentinel; sentinel.invalidSizeFallbacks = 123;
    RunControl control; control.maximumNodes = 3;
    result = buildBoundaryMesh(input, options, sizes, sentinel, control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel.invalidSizeFallbacks == 123 && sentinel.nodes.empty(),
          "node budget across whole model retains output");
    control = {}; control.maximumGeometryEvaluations = 200;
    result = buildBoundaryMesh(input, options, sizes, sentinel, control);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 200 && sentinel.nodes.empty(),
          "geometry budget includes length and density across curves");
    options.sizeMax = .25;
    BoundaryMesh budgetProbe;
    auto probe = buildBoundaryMesh(input,options,sizes,budgetProbe);
    control.maximumGeometryEvaluations = probe.geometryEvaluations-1;
    result = buildBoundaryMesh(input,options,sizes,sentinel,control);
    check(probe.status.complete() && result.status.code == MeshingStatus::BudgetExceeded && sentinel.nodes.empty(),
          "budget failure inside filter callback preserves typed status instead of unknown query failure");
    control = {}; control.keepGoing = [] { return false; };
    result = buildBoundaryMesh(input, options, sizes, sentinel, control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel.nodes.empty(), "cancelled build publishes no partial registry");
    auto invalid = input; invalid.curves[0].beginVertex = 999;
    result = buildBoundaryMesh(invalid, options, sizes, sentinel);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel.nodes.empty(), "unknown endpoint is explicit topology failure");
    invalid = input; invalid.vertices.push_back(invalid.vertices[0]);
    result = buildBoundaryMesh(invalid, options, sizes, sentinel);
    check(result.status.code == MeshingStatus::InvalidTopology, "duplicate CAD identities rejected");
    invalid = input; invalid.curves[0].point = [](double) {
        return Point3D{std::numeric_limits<double>::quiet_NaN(),0.,0.};
    };
    result = buildBoundaryMesh(invalid, options, sizes, sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel.nodes.empty(), "nonfinite CAD query is explicit failure");
    invalid.curves[0].point = [](double) -> Point3D { throw std::invalid_argument("CAD evaluator rejected parameter"); };
    result = buildBoundaryMesh(invalid,options,sizes,sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel.nodes.empty(),
          "CAD evaluator invalid_argument is query failure, not configuration rejection");
    invalid = closed;
    invalid.curves[0] = line; invalid.curves[0].beginVertex = invalid.curves[0].endVertex = 1;
    invalid.curves[0].point = [](double) -> Point3D { throw std::invalid_argument("closed CAD line query failed"); };
    result = buildBoundaryMesh(invalid,options,sizes,sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel.nodes.empty(),
          "closed-line pre-integration CAD exception is distinct from invalid options");
    GeometrySizeSource stepSizes;
    stepSizes.sizeCallback = [](const MeshSizeQuery& q, double) { return q.point.x < .5 ? .01 : 10.; };
    options.sizeMax = 10.;
    result = buildBoundaryMesh(input,options,stepSizes,mesh);
    bool erased = false, chainUsesAliveNodes = result.status.complete();
    for(const auto& node : mesh.nodes) erased |= !node.alive;
    for(const auto& chain : mesh.curves) for(const auto& sample : chain.samples)
        chainUsesAliveNodes &= sample.node && sample.node <= mesh.nodes.size() && mesh.nodes[sample.node-1].alive;
    check(result.status.complete() && erased && chainUsesAliveNodes && mesh.curves.size() == 2 &&
          mesh.curves[1].samples[1].node > mesh.curves[0].samples.back().node,
          "abrupt size field filters nodes, retains allocation gaps and keeps all chain references live");
    std::cout << "boundary registry failures=" << failures << '\n';
    return failures ? 1 : 0;
}
