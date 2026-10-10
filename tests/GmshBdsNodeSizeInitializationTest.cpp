/** @file GmshBdsNodeSizeInitializationTest.cpp
 * @brief 原MVertex尺寸查询/ID序/高维无约束/跳过与失败原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsNodeSizeInitialization.h"
#include <iostream>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok){ ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh inputMesh() {
    BdsMesh mesh;
    for(int id : {9,2,-1,5}) { BdsPoint p; p.id = id; p.uv = {id*.1,id*.2}; p.point = {id*.3,0.,0.}; p.sizes = {.4,.8}; p.classification = {1,2,BdsClassificationOrigin::FaceInterior}; insertBdsPoint(mesh,p); }
    mesh.geometry.insert({12,1}); return mesh;
}
}
int main() {
    const auto mesh = inputMesh(); BdsMesh output;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    BdsRecoveredVertexQueries vertices{{2,{0,101,nan,nan,{10.,20.,30.}}},{5,{1,102,.125,nan,{11.,21.,31.}}},{9,{2,103,0.,0.,{12.,22.,32.}}}};
    SurfaceMeshingOptions options; options.sizeMax = 20.;
    GeometrySizeSource source; std::vector<MeshSizeQuery> actual;
    source.backgroundField = [&](const MeshSizeQuery& query) { actual.push_back(query); return query.dimension == 0 ? .75 : 1.25; };
    auto result = initializeOrdinaryBdsNodeSizes(mesh,vertices,30.,options,source,output);
    check(result.status.complete() && result.queries.size() == 2 && actual.size() == 2,"two BGM calls and successful initialization");
    if(actual.size() == 2) {
        check(actual[0].entity == 101 && actual[0].dimension == 0 && actual[0].u == 0. && actual[0].v == 0. && actual[0].point.x == 10.,"0D original entity/physical XYZ, zero parameters despite temporary BDS classification");
        check(actual[1].entity == 102 && actual[1].dimension == 1 && actual[1].u == .125 && actual[1].v == 0. && actual[1].point.x == 11.,"1D original curve parameter and ID order");
    }
    if(output.points.size() == 4) {
        check(output.points.at(2).sizes.meshSize == .75 && output.points.at(2).sizes.backgroundSize == .75 && output.points.at(5).sizes.meshSize == 1.25,"both sizes initialized from BGM result");
        check(output.points.at(9).sizes.meshSize == 1.e22 && output.points.at(9).sizes.backgroundSize == 1.e22 && result.unconstrainedPoints == std::vector<int>{9},"original face vertex assigned MAX_LC without BGM call or sizeMax clamp");
        check(output.points.at(-1).sizes.meshSize == .4 && output.points.at(-1).sizes.backgroundSize == .8 && output.maximumPointId == mesh.maximumPointId,"unmapped outer node and ID allocation retained");
        check(output.points.at(2).classification.origin == BdsClassificationOrigin::FaceInterior && output.geometry.size() == mesh.geometry.size() && output.geometry.count({12,1}) == 1,"sizes do not reclassify or register geometry");
    } else check(false,"output published");
    BdsMesh sentinel; sentinel.maximumPointId = 888;
    result = initializeOrdinaryBdsNodeSizes(mesh,vertices,30.,options,source,sentinel,true);
    check(result.status.complete() && result.skippedInitialOnly && result.queries.empty() && sentinel.points.at(2).sizes.meshSize == .4,"initial-only skips complete size stage");
    sentinel = {}; sentinel.maximumPointId = 888; RunControl budget; budget.maximumGeometryEvaluations = 1;
    result = initializeOrdinaryBdsNodeSizes(mesh,vertices,30.,options,source,sentinel,false,budget);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 888 && result.queries.size() == 1,"BGM query budget failure preserves output");
    RunControl cancel; cancel.keepGoing = [] { return false; };
    result = initializeOrdinaryBdsNodeSizes(mesh,vertices,30.,options,source,sentinel,false,cancel);
    check(result.status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 888,"cancel preserves output");
    auto invalid = vertices; invalid.at(5).u = nan;
    result = initializeOrdinaryBdsNodeSizes(mesh,invalid,30.,options,source,sentinel);
    check(result.status.code == MeshingStatus::InvalidInput && sentinel.maximumPointId == 888,"invalid used curve parameter fails atomically");
    source.backgroundField = [](const MeshSizeQuery&) -> double { throw std::runtime_error("CAD sizing unavailable"); };
    result = initializeOrdinaryBdsNodeSizes(mesh,vertices,30.,options,source,sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel.maximumPointId == 888,"CAD query failure retains output and status");
    auto missing = vertices; missing.emplace(100,vertices.at(2));
    result = initializeOrdinaryBdsNodeSizes(mesh,missing,30.,options,{},sentinel);
    check(result.status.code == MeshingStatus::InvalidInput && sentinel.maximumPointId == 888,"recoverMap key must refer to an existing BDS point");
    return failures ? 1 : 0;
}
