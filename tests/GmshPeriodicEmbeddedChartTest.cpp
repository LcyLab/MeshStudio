#include "mesh/surface/algorithm6/chart/PeriodicFaceChart.h"
#include <iostream>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok, const char* name) { if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    BoundaryMesh physical;
    for(std::uint64_t id = 1; id <= 7; ++id) {
        BoundaryNode node; node.id = id; node.point = {double(id),double(id)*.1,0.};
        node.dimension = id == 6 ? 1 : 0; node.entity = id == 6 ? 30 : int(id)+10;
        node.parameter = .4; physical.nodes.push_back(node);
    }
    PeriodicBoundaryChart boundary; boundary.entity = 2; boundary.maximumLocalId = 3;
    for(int id = 0; id < 4; ++id) boundary.vertices.push_back({{id,std::uint64_t(id+1),0,id+11,{double(id),0.},{}},{1.,1.},-1});
    PeriodicEmbeddedInput input; input.vertices = {5};
    PeriodicEmbeddedCurveInput curve; curve.entity = 30; curve.lines = {{5,6},{6,7}};
    input.curves = {curve}; input.modelCharacteristicLength = 10.;
    input.closestSurfaceCoordinates = [](Point3D p) { return Point2D{p.x,p.y}; };
    input.surfacePoint = [](Point2D uv) { return Point3D{uv.x,uv.y,2.}; };
    input.perturbInitialPoint = [](Point2D uv) { return Point2D{uv.x+.01,uv.y+.02}; };
    SurfaceMeshingOptions options; options.sizeFromPoints = false; options.sizeFactor = 1.25;
    GeometrySizeSource sizes; std::vector<MeshSizeQuery> queries;
    sizes.sizeCallback = [&](const MeshSizeQuery& q, double) { queries.push_back(q); return .2; };
    PeriodicEmbeddedChart chart;
    auto result = appendPeriodicEmbeddedChart(physical,boundary,input,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 4 && chart.expectedInitialPointCount == 4 &&
          !chart.initialPointCountMismatch && chart.maximumLocalId == 7,"embedded CAD point and its edge endpoint stay separate");
    if(chart.vertices.size() == 4 && chart.initialPoints.size() == 4 && queries.size() == 4) {
        check(chart.vertices[0].vertex.entity == -15 && chart.vertices[0].vertex.dimension == 0 &&
              chart.vertices[1].vertex.entity == -30 && chart.vertices[1].vertex.dimension == 1 &&
              chart.vertices[0].vertex.physicalId == chart.vertices[1].vertex.physicalId,"negative BDS classification preserves two roles");
        check(queries[1].dimension == 0 && queries[1].entity == 15 && queries[1].u == 0. &&
              queries[2].dimension == 1 && queries[2].entity == 30 && queries[2].u == .4 &&
              chart.vertices[2].sizes.meshSize == .2*1.25,"BGM uses original classification while BDS uses embedded edge");
        check(chart.initialPoints[0].coordinates.x == 5.+.01 && chart.vertices[0].vertex.uv.x == 5. &&
              chart.vertices[0].vertex.surfacePoint.z == 2. && chart.initialPoints[1].localId == 5,
              "DocRecord perturbation order stays separate from raw UV and surface point");
        check(chart.constraints.size() == 2 && chart.constraints[0].from == 5 && chart.constraints[0].to == 6 &&
              chart.constraints[1].from == 6 && chart.constraints[1].to == 7,"embedded lines use edge copy, not independent embedded vertex copy");
    } else check(false,"embedded metadata present");
    auto shared = input; shared.vertices.clear(); shared.curves[0].lines = {{1,6},{6,2}};
    result = appendPeriodicEmbeddedChart(physical,boundary,shared,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 1 && chart.constraints.size() == 2 &&
          chart.constraints[0].from == 0 && chart.constraints[1].to == 1,"single boundary copies reused without projection");
    auto second = shared.curves[0]; second.entity = 31; second.lines = {{6,3}}; shared.curves.push_back(second);
    result = appendPeriodicEmbeddedChart(physical,boundary,shared,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 1 && chart.vertices[0].vertex.entity == -30 &&
          chart.constraints.size() == 3,"first embedded edge owns classification when a later edge shares a node");
    boundary.vertices.push_back({{4,1,0,11,{9.,0.},{}},{1.,1.},-1}); boundary.maximumLocalId = 4;
    shared.curves.resize(1); shared.curves[0].first = 0.; shared.curves[0].last = 1.;
    shared.curves[0].point = [](double t) { return Point3D{1.+t,0.,0.}; };
    shared.curves[0].onFace = [](double) { return Point2D{9.,0.}; };
    result = appendPeriodicEmbeddedChart(physical,boundary,shared,options,sizes,chart);
    check(result.status.complete() && chart.constraints.size() == 2 && chart.constraints[0].from == 4,
          "multi-copy endpoint chooses exact pcurve UV before creating new point");
    auto saved = chart;
    auto bad = shared; bad.closestSurfaceCoordinates = [](Point3D) -> Point2D { throw std::runtime_error("projection failed"); };
    result = appendPeriodicEmbeddedChart(physical,boundary,bad,options,sizes,chart);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && chart.constraints.size() == saved.constraints.size(),
          "failed projection preserves output");
    RunControl control; control.maximumGeometryEvaluations = 1;
    result = appendPeriodicEmbeddedChart(physical,boundary,shared,options,sizes,chart,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && chart.maximumLocalId == saved.maximumLocalId,"query budget remains explicit and atomic");
    control = {}; control.keepGoing = [] { return false; };
    result = appendPeriodicEmbeddedChart(physical,boundary,shared,options,sizes,chart,control);
    check(result.status.code == MeshingStatus::Cancelled,"cancelled embedded stage");
    auto tie = shared;
    tie.curves[0].point = [](double t) { return Point3D{2.*t,.2*t,0.}; };
    double chosen = -1.;
    tie.curves[0].onFace = [&](double t) { chosen = t; return Point2D{9.,0.}; };
    result = appendPeriodicEmbeddedChart(physical,boundary,tie,options,sizes,chart);
    check(result.status.complete() && chosen == 1. && chart.constraints[0].from == 4,
          "equal physical endpoint distance selects upper parameter");
    auto missing = shared; missing.curves[0].lines = {{1,6},{6,1}};
    missing.curves[0].onFace = [](double) { return Point2D{10.,0.}; };
    result = appendPeriodicEmbeddedChart(physical,boundary,missing,options,sizes,chart);
    check(result.status.complete() && chart.vertices.size() == 2 && chart.expectedInitialPointCount == 1 &&
          chart.initialPointCountMismatch && chart.seamSelectionFailures.size() == 2 && chart.constraints.empty(),
          "failed seam selection records count mismatch and later null override instead of silently keeping a constraint");
    PeriodicFaceChartInput whole;
    whole.trueBoundaryCurves = {{10,0.,1.,false,[](double t,int) { return Point2D{t,0.}; }}};
    whole.boundary.entity = 2; whole.boundary.modelCharacteristicLength = 10.;
    whole.boundary.parameterLower = {0.,0.}; whole.boundary.parameterUpper = {1.,1.};
    whole.boundary.surfacePoint = input.surfacePoint;
    whole.boundary.edgeLoops = {{{1,false,{{1,{0.,0.}},{2,{1.,0.}}},{}},
                                {2,false,{{2,{1.,0.}},{3,{1.,1.}}},{}},
                                {3,false,{{3,{1.,1.}},{4,{0.,1.}}},{}},
                                {4,false,{{4,{0.,1.}},{1,{0.,0.}}},{}}}};
    whole.embedded = input;
    PeriodicFaceChart complete;
    auto built = buildPeriodicFaceChart(physical,whole,options,sizes,complete);
    check(built.status.complete() && built.geometryEvaluations == 320 && complete.trueBoundary.size() == 299 &&
          complete.initialPoints.size() == 8 && complete.initialPoints[0].localId == 4 &&
          complete.initialPoints[4].localId == 0 && complete.expectedInitialPointCount == 8,
          "full chart composes true boundary, embedded prefix and boundary DocRecord order");
    auto noPerturbation = whole; noPerturbation.embedded.perturbInitialPoint = {};
    const auto priorInitialCount = complete.initialPoints.size();
    const auto missingPerturbation = buildPeriodicFaceChart(physical,noPerturbation,options,sizes,complete);
    check(missingPerturbation.status.code == MeshingStatus::InvalidInput && complete.initialPoints.size() == priorInitialCount,
          "full native chart requires an explicit perturbation stream instead of silently dropping random calls");
    if(built.status.complete()) {
        const auto oldCount = complete.initialPoints.size(); RunControl limit; limit.maximumGeometryEvaluations = 310;
        built = buildPeriodicFaceChart(physical,whole,options,sizes,complete,limit);
        check(built.status.code == MeshingStatus::BudgetExceeded && built.geometryEvaluations == 310 &&
              complete.initialPoints.size() == oldCount,"geometry budget accumulates across all chart phases atomically");
    }
    auto malformed = whole;
    malformed.boundary.edgeLoops = {{{1,false,{{1,{0.,0.}},{2,{1.,0.}},{1,{1.,1.}},{3,{0.,1.}},{1,{0.,0.}}},{}}}};
    malformed.embedded.curves = missing.curves;
    const auto savedInitialCount = complete.initialPoints.size();
    built = buildPeriodicFaceChart(physical,malformed,options,sizes,complete);
    check(built.status.code == MeshingStatus::InvalidTopology && complete.initialPoints.size() == savedInitialCount &&
          built.status.message.find("point-count mismatch") != std::string::npos,
          "composition rejects the native unsafe DocRecord count mismatch without changing output");
    whole.boundary.edgeLoops = {{{1,false,{{1,{0.,0.}},{2,{1.,0.}},{3,{0.,1.}},{1,{0.,0.}}},{}}}};
    whole.embedded.closestSurfaceCoordinates = [](Point3D) -> Point2D { throw std::runtime_error("must not query embedded after direct triangle"); };
    built = buildPeriodicFaceChart(physical,whole,options,sizes,complete);
    check(built.status.complete() && built.route == FaceChartRoute::DirectTriangle && complete.initialPoints.empty() &&
          complete.embedded.vertices.empty(),"native three-boundary-point early route precedes embedding and DocRecord");
    std::cout << "periodic embedded failures=" << failures << '\n'; return failures ? 1 : 0;
}
