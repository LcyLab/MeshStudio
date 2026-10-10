#include "mesh/surface/algorithm6/chart/PeriodicFaceChart.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok, const char* name) {
        if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    BoundaryMesh boundary;
    for(std::uint64_t id = 1; id <= 5; ++id) {
        BoundaryNode node; node.id = id; node.dimension = id == 5 ? 1 : 0;
        node.entity = int(id)+10; node.parameter = .75;
        node.point = {double(id),2.*double(id),3.}; boundary.nodes.push_back(node);
    }
    PeriodicFaceBoundaryInput input; input.entity = 7;
    input.parameterLower = {0.,0.}; input.parameterUpper = {1.,1.};
    input.modelCharacteristicLength = 10.;
    input.surfacePoint = [](Point2D p) { return Point3D{p.x,p.y,9.}; };
    input.edgeLoops = {{{1,false,{{1,{0.,0.}},{5,{.5,0.}},{2,{1.,0.}}},{}},
                        {2,false,{{2,{1.,0.}},{3,{1.,1.}}},{}},
                        {3,false,{{3,{1.,1.}},{4,{0.,1.}}},{}},
                        {4,false,{{4,{0.,1.}},{1,{0.,0.}}},{}}}};
    SurfaceMeshingOptions options; options.sizeMax = 2.; options.sizeFactor = 1.5;
    GeometrySizeSource sizes; std::vector<MeshSizeQuery> queries;
    sizes.sizeCallback = [&](const MeshSizeQuery& q, double) {
        queries.push_back(q); return q.dimension == 0 ? .4 : .8;
    };
    PeriodicBoundaryChart chart;
    auto result = buildPeriodicBoundaryChart(boundary,input,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 5 && chart.loopPositionCount == 5 &&
          chart.loops.size() == 1 && chart.route == FaceChartRoute::General && result.geometryEvaluations == 10,
          "single loop creates BDS nodes and two size states");
    if(chart.vertices.size() == 5 && queries.size() == 5) {
        check(chart.vertices[0].vertex.localId == 0 && chart.vertices[1].vertex.physicalId == 5 &&
              chart.vertices[1].sizes.meshSize == .8*1.5 && chart.vertices[0].sizes.backgroundSize == .4*1.5,
              "size callback then scaling matches BGM and mesh starts equal");
        check(queries[0].u == 0. && queries[1].u == .75 && queries[1].point.x == 5. &&
              chart.vertices[1].vertex.surfacePoint.z == 9. && boundary.nodes[4].point.z == 3.,
              "size uses classified physical XYZ and curve parameter, surface point stays separate");
        check(chart.hasBounds && chart.lower.x == 0. && chart.upper.y == 1. && chart.maximumLocalId == 4,
              "raw UV bounds and maximum BDS ID");
    }
    // 同一环重复 0D 不复用；后续环仅复用此前环的 0D，1D 总是创建新复制。
    auto shared = input;
    shared.edgeLoops.push_back(input.edgeLoops[0]); queries.clear();
    result = buildPeriodicBoundaryChart(boundary,shared,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 6 && chart.loopPositionCount == 10 &&
          chart.maximumLocalId == 6 && queries.size() == 6 && chart.loops[1] == std::vector<int>({0,6,2,3,4}),
          "cross-loop zero-dimensional reuse leaves native ID gaps while curve node gets a copy");
    if(chart.vertices.size() == 6) check(chart.vertices[1].periodicCounterpart == 6 &&
          chart.vertices[5].periodicCounterpart == 1 && chart.loops[0][1] == 1,
          "counterparts refer to local IDs, not compact vector offsets");
    shared.edgeLoops.push_back(input.edgeLoops[0]);
    result = buildPeriodicBoundaryChart(boundary,shared,options,sizes,chart);
    if(result.status.complete() && chart.vertices.size() == 7)
        check(chart.vertices[1].periodicCounterpart == 6 && chart.vertices[5].periodicCounterpart == 11 &&
              chart.vertices[6].periodicCounterpart == 6,
              "third copy overwrites previous counterpart exactly as native INV traversal");
    else check(false,"three-copy chart");
    auto repeated = input;
    repeated.edgeLoops = {{{1,false,{{1,{0.,0.}},{2,{1.,0.}},{1,{0.,0.}},{3,{0.,1.}},{1,{0.,0.}}},{}}}};
    result = buildPeriodicBoundaryChart(boundary,repeated,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 4 && chart.vertices[0].vertex.physicalId == 1 &&
          chart.vertices[2].vertex.physicalId == 1 && chart.vertices[0].periodicCounterpart == 2,
          "current loop does not merge even identical zero-dimensional UV copies");
    repeated.edgeLoops.push_back(repeated.edgeLoops[0]);
    result = buildPeriodicBoundaryChart(boundary,repeated,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 4 && chart.loops[1] == std::vector<int>({0,1,0,3}),
          "cross-loop reuse searches prior copies by BDS ID and keeps first match");
    auto relaxed = input;
    relaxed.edgeLoops[0][1].vertices[0].uv.x += .5e-6;
    result = buildPeriodicBoundaryChart(boundary,relaxed,options,sizes,chart);
    check(result.status.complete() && chart.selections[0].tolerance == 1.e-6*std::sqrt(2.) &&
          chart.vertices[2].vertex.uv.x == 1.+.5e-6,
          "native six-level tolerance retry accepts join without averaging its UV");
    auto alternate = input;
    alternate.edgeLoops[0][0].seam = true;
    alternate.edgeLoops[0][0].alternateCoordinates = {{0.,0.},{.5,0.},{1.,0.}};
    for(auto& vertex : alternate.edgeLoops[0][0].vertices) vertex.uv.x += 5.;
    result = buildPeriodicBoundaryChart(boundary,alternate,options,sizes,chart);
    check(result.status.complete() && chart.selections[0].seamFirst && chart.vertices[0].vertex.uv.x == 0.,
          "native outer retry tries alternate first seam after ordinary branch fails");
    auto small = input;
    small.edgeLoops = {{{1,false,{{1,{0.,0.}},{2,{1.,0.}},{3,{0.,1.}},{1,{0.,0.}}},{}}}};
    result = buildPeriodicBoundaryChart(boundary,small,options,sizes,chart);
    check(result.status.complete() && chart.route == FaceChartRoute::DirectTriangle && result.geometryEvaluations == 6,
          "periodic three-point early route still initializes BDS and sizes");
    small.edgeLoops[0][0].vertices.erase(small.edgeLoops[0][0].vertices.begin()+2);
    result = buildPeriodicBoundaryChart(boundary,small,options,sizes,chart);
    check(result.status.complete() && chart.route == FaceChartRoute::SkipInsufficientNodes,
          "periodic two-position early route");
    PeriodicBoundaryChart sentinel; sentinel.entity = 123;
    auto bad = input; bad.edgeLoops[0][0].vertices[0].physicalId = 99;
    result = buildPeriodicBoundaryChart(boundary,bad,options,sizes,sentinel);
    check(result.status.code == MeshingStatus::InvalidTopology && sentinel.entity == 123,
          "unknown physical identity fails atomically");
    bad = input; bad.surfacePoint = [](Point2D) -> Point3D { throw std::invalid_argument("CAD failure"); };
    result = buildPeriodicBoundaryChart(boundary,bad,options,sizes,sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel.entity == 123,
          "CAD exception does not masquerade as topology or publish partial nodes");
    bad.surfacePoint = [](Point2D) { return Point3D{std::numeric_limits<double>::quiet_NaN(),0.,0.}; };
    result = buildPeriodicBoundaryChart(boundary,bad,options,sizes,sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed, "nonfinite surface point fails explicitly");
    GeometrySizeSource failedSize;
    failedSize.sizeCallback = [](const MeshSizeQuery&, double) -> double {
        throw OperationFailure{{MeshingStatus::Cancelled,"cancel from size source"}};
    };
    result = buildPeriodicBoundaryChart(boundary,input,options,failedSize,sentinel);
    check(result.status.code == MeshingStatus::Cancelled && sentinel.entity == 123,
          "typed size-source cancellation survives the geometry query boundary");
    failedSize.sizeCallback = [](const MeshSizeQuery&, double) { return 0.; };
    result = buildPeriodicBoundaryChart(boundary,input,options,failedSize,chart);
    check(result.status.complete() && chart.invalidSizeFallbacks == 5 && chart.vertices[0].sizes.meshSize == 10.*1.5,
          "native invalid size fallback is recorded rather than silently discarded");
    RunControl control; control.maximumGeometryEvaluations = 3;
    result = buildPeriodicBoundaryChart(boundary,input,options,sizes,sentinel,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 3 && sentinel.entity == 123,
          "query budget counts new BDS surface and BGM queries atomically");
    control = {}; control.maximumNodes = 4;
    result = buildPeriodicBoundaryChart(boundary,input,options,sizes,sentinel,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel.entity == 123, "node position budget");
    control = {}; control.keepGoing = [] { return false; };
    result = buildPeriodicBoundaryChart(boundary,input,options,sizes,sentinel,control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel.entity == 123, "cancelled chart retains output");
    auto disconnected = input; disconnected.edgeLoops[0][1].vertices[0].uv.x = 5.;
    disconnected.edgeLoops[0][1].vertices[1].uv.x = 5.;
    result = buildPeriodicBoundaryChart(boundary,disconnected,options,sizes,sentinel);
    check(result.status.code == MeshingStatus::NonConverged && result.geometryEvaluations == 0 && sentinel.entity == 123,
          "all six tolerance attempts fail without allocating or querying BDS nodes");
    bad = input; bad.parameterUpper = bad.parameterLower;
    result = buildPeriodicBoundaryChart(boundary,bad,options,sizes,sentinel);
    check(result.status.code == MeshingStatus::InvalidInput, "zero UV scale is explicit invalid input");
    std::cout << "periodic boundary chart failures=" << failures << '\n';
    return failures ? 1 : 0;
}
