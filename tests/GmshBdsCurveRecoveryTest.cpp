/** @file GmshBdsCurveRecoveryTest.cpp
 * @brief ordinary登记/边界pass/缺映射/失败提前返回/原生端点分类。
 * @date 2026-10-08
 * @modified 2026-10-09
 */
#include "mesh/surface/algorithm6/bds/BdsCurveRecovery.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok){ ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh square() {
    BdsMesh mesh; int id = 0;
    for(Point2D uv : {Point2D{0.,0.},Point2D{1.,0.},Point2D{1.,1.},Point2D{0.,1.}}) {
        BdsPoint p; p.id = id++; p.physicalId = 100+p.id; p.uv = uv; p.point = {uv.x,uv.y,0.}; insertBdsPoint(mesh,p);
    }
    appendBdsTriangle(mesh,{0,1,2}); appendBdsTriangle(mesh,{0,2,3}); return mesh;
}
BdsRecoveryCurve curve(int tag,std::uint64_t a,std::uint64_t b) {
    BdsRecoveryCurve c; c.entity = tag; c.segments.push_back({a,b}); c.endpoints = std::array<std::uint64_t,2>{a,b}; return c;
}
}
int main() {
    const auto mesh = square(); auto boundary = curve(11,100,101), embedded = curve(12,101,103);
    auto duplicate = curve(13,101,100), degenerate = curve(14,100,103); degenerate.meshDegenerated = true;
    std::vector<BdsRecoveryConstraint> registry;
    auto status = buildOrdinaryBdsRecoveryRegistry(mesh,{boundary,degenerate},{embedded,duplicate},registry);
    check(status.complete() && registry.size() == 2 && registry[0].first == 0 && registry[0].second == 1 && registry[0].curveEntity == 11 && registry[1].curveEntity == 12,"registry boundary then embedded, duplicate first owner and degenerate skip");
    BdsMesh output = mesh; auto result = recoverOrdinaryBdsBoundary(mesh,{boundary,degenerate},registry,output);
    check(result.status.complete() && !result.nativeFatal && !result.needsRefinement && result.attempts.size() == 1 && findBdsEdge(output,1,3) == -1 && output.geometry.count({14,1}) == 0,"boundary pass does not recover embedded or classify degenerate curve");
    check(output.edges[findBdsEdge(output,0,1)].classification.tag == 11 && output.points.at(0).classification.dimension == 0 && output.points.at(1).classification.tag == 1 && output.points.at(1).classification.dimension == 0,"curve degree1 and previously null endpoints use local IDs degree0");
    BdsMesh leaf;const auto existingLeaf=recoverBdsConstraintEdge(mesh,0,1,leaf,registry);
    check(result.attempts[0].recovery.recoveredEdge==existingLeaf.recoveredEdge && result.attempts[0].recovery.nativeFatal==existingLeaf.nativeFatal && result.attempts[0].recovery.scans.empty() && result.attempts[0].recovery.notRecovered.empty(),"existing boundary edge diagnostics match independent leaf");
    auto alias=mesh;const auto aliasResult=recoverOrdinaryBdsBoundary(alias,{boundary,duplicate},{},alias);
    check(aliasResult.status.complete() && aliasResult.attempts.size()==2 && alias.edges[findBdsEdge(alias,0,1)].classification.tag==13 && alias.points.at(0).classification.dimension==0,"existing edge alias input/output keeps final curve owner and endpoint labels");
    auto broken=mesh;broken.edges[findBdsEdge(broken,2,3)].faces.clear();BdsMesh invalidSentinel;invalidSentinel.maximumPointId=777;
    check(recoverOrdinaryBdsBoundary(broken,{boundary},{},invalidSentinel).status.code==MeshingStatus::InvalidTopology && invalidSentinel.maximumPointId==777,"unrelated malformed topology cannot bypass existing-edge recovery validation");
    auto crossing = curve(15,101,103), collision = curve(16,100,102);
    buildOrdinaryBdsRecoveryRegistry(mesh,{crossing,boundary},{collision},registry);
    output = mesh; result = recoverOrdinaryBdsBoundary(mesh,{crossing,boundary},registry,output);
    check(result.status.complete() && !result.nativeFatal && result.needsRefinement && result.notRecovered.size() == 2 && result.attempts.size() == 2,"self intersection ends curve nonfatal and continues next boundary curve");
    check(output.points.at(3).classification.dimension == -1 && output.edges[findBdsEdge(output,0,1)].classification.tag == 11,"early return skips failed curve endpoint classification");
    auto protectedMesh = mesh; protectedMesh.geometry.insert({99,1}); protectedMesh.edges[2].classification = {99,1};
    buildOrdinaryBdsRecoveryRegistry(protectedMesh,{crossing,boundary},{},registry);
    output = protectedMesh; result = recoverOrdinaryBdsBoundary(protectedMesh,{crossing,boundary},registry,output);
    check(result.status.complete() && result.nativeFatal && result.failedCurve == 15 && result.attempts.size() == 1 && output.geometry.count({11,1}) == 0 && output.geometry.count({15,1}) == 1,"fatal stops boundary before next curve but retains failed curve registration");
    auto missing = curve(17,100,999); missing.segments.push_back({100,101});
    buildOrdinaryBdsRecoveryRegistry(mesh,{missing},{},registry); output = mesh;
    result = recoverOrdinaryBdsBoundary(mesh,{missing},registry,output);
    check(result.status.complete() && result.skippedSegments == std::vector<std::pair<int,std::size_t>>{{17,0}} && result.attempts.size() == 1 && output.points.at(0).classification.dimension == -1,"missing mapping skips segment and prevents partial endpoint labeling");
    auto empty = curve(18,100,101); empty.segments.clear(); output = mesh;
    result = recoverOrdinaryBdsBoundary(mesh,{empty},{},output,true);
    check(result.status.complete() && result.needsRefinement && result.attempts.empty() && output.geometry.count({18,1}) == 1 && output.points.at(0).classification.dimension == 0,"empty curve still registers geometry/labels endpoints and refineAll requests retry");
    auto classified = mesh; classified.geometry.insert({77,2}); classified.points.at(0).classification = {77,2}; output = classified;
    buildOrdinaryBdsRecoveryRegistry(classified,{boundary},{},registry);
    result = recoverOrdinaryBdsBoundary(classified,{boundary},registry,output);
    check(result.status.complete() && output.points.at(0).classification.tag == 77 && output.points.at(0).classification.dimension == 2,"endpoint classification fills null only, not lower-degree correction");
    BdsMesh sentinel; sentinel.maximumPointId = 888; RunControl cancel; cancel.keepGoing = [] {return false;};
    check(recoverOrdinaryBdsBoundary(mesh,{boundary},registry,sentinel,false,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 888,"cancel retains output");
    RunControl budget; budget.maximumNodes = 3;
    check(recoverOrdinaryBdsBoundary(mesh,{boundary},registry,sentinel,false,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 888,"budget retains output");
    auto domain = mesh;
    for(auto& face : domain.triangles) face.classification = {1,2,BdsClassificationOrigin::FaceInterior};
    buildOrdinaryBdsRecoveryRegistry(domain,{}, {crossing},registry); output = domain;
    auto embeddedResult = recoverOrdinaryBdsEmbedded(domain,{crossing},registry,output);
    check(embeddedResult.status.complete() && !embeddedResult.hadFatalRecovery && embeddedResult.attempts.size() == 1 && findBdsEdge(output,1,3) == 5 && output.triangles.back().classification.origin == BdsClassificationOrigin::FaceInterior,"post-domain embedded flips missing edge and inherits temporary face identity");
    domain.geometry.insert({99,1}); domain.edges[2].classification = {99,1};
    buildOrdinaryBdsRecoveryRegistry(domain,{}, {crossing,boundary},registry); output = domain;
    embeddedResult = recoverOrdinaryBdsEmbedded(domain,{crossing,boundary},registry,output);
    check(embeddedResult.status.complete() && embeddedResult.hadFatalRecovery && embeddedResult.attempts.size() == 2 && output.geometry.count({11,1}) == 1 && output.edges[0].classification.tag == 11,"ignored native fatal recorded but later embedded curve still processed");
    check(recoverOrdinaryBdsEmbedded(domain,{crossing},registry,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 888,"embedded cancel atomic output");
    check(recoverOrdinaryBdsEmbedded(domain,{crossing},registry,sentinel,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 888,"embedded budget atomic output");
    return failures ? 1 : 0;
}
