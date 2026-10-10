/** @file GmshPeriodicBdsRecoveryTest.cpp
 * @brief 周期embedded先行/临时分类/缺边曲线记录/失败控制和副本身份。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/PeriodicBdsRecovery.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsMesh square() {
    BdsMesh mesh;
    for(int i = 0; i < 4; ++i) { BdsPoint p; p.id = i; p.physicalId = i+1; p.uv = {double(i == 1 || i == 2),double(i >= 2)}; p.point = {p.uv.x,p.uv.y,0.}; insertBdsPoint(mesh,p); }
    appendBdsTriangle(mesh,{0,1,2}); appendBdsTriangle(mesh,{0,2,3}); return mesh;
}
}
int main() {
    auto mesh = square(); PeriodicBdsRecoveryInput input; input.boundaryLoops = {{0,1,2,3}};
    BdsMesh output; auto result = recoverPeriodicBdsConstraints(mesh,input,output);
    check(result.status.complete() && !result.embeddedFailed && !result.needsRetry && result.attempts.size() == 4,"existing loop completes without retry");
    if(result.status.complete()) {
        const auto g = output.edges[findBdsEdge(output,0,1)].classification;
        check(g.tag == 1 && g.dimension == 1 && g.origin == BdsClassificationOrigin::FaceBoundary && output.geometry.empty(),"CLASS_E temporary identity is not registered geometry");
        check(result.attempts[0].alreadyPresent && !result.attempts[0].recovery && !result.attempts[0].curveEntity,"existing boundary bypasses recovery and curve lookup");
        check(output.points.at(0).classification.dimension == -1,"periodic recovery does not assign endpoint classification");
    }
    input.embedded = {{1,3}}; input.boundaryLoops.clear();
    result = recoverPeriodicBdsConstraints(mesh,input,output);
    check(result.status.complete() && result.attempts.size() == 1 && result.attempts[0].embedded && result.attempts[0].recovery && result.attempts[0].recovery->recoveredEdge >= 0 && findBdsEdge(output,1,3) >= 0,"embedded uses local IDs and recovers before loops");
    input.embedded.clear(); input.boundaryLoops = {{1,3,0}};
    BdsRecoveryCurve curve; curve.entity = 42; curve.segments = {{2,4}}; input.boundaryCurves = {curve};
    result = recoverPeriodicBdsConstraints(mesh,input,output);
    check(result.status.complete() && !result.needsRetry && result.attemptedBoundaryCurves.size() == 1 && result.attemptedBoundaryCurves[0].curveEntity == 42 && result.attempts[0].curveEntity == 42,"missing boundary records original CAD curve even when recovery succeeds");
    input.refineAllEdges = true; result = recoverPeriodicBdsConstraints(mesh,input,output);
    check(result.status.complete() && result.needsRetry,"refineAll forces retry after boundary pass");
    auto protectedMesh = mesh; protectedMesh.geometry.insert({9,1}); protectedMesh.edges[findBdsEdge(protectedMesh,0,2)].classification = {9,1};
    input.refineAllEdges = false; input.embedded = {{1,3},{0,1}};
    result = recoverPeriodicBdsConstraints(protectedMesh,input,output);
    check(result.status.complete() && result.embeddedFailed && !result.needsRetry && result.attempts.size() == 1,"embedded fatal stops subsequent embedded and boundary; no retry flag initialized");
    input.embedded.clear(); input.boundaryLoops = {{1,3,0}};
    result = recoverPeriodicBdsConstraints(protectedMesh,input,output);
    check(result.status.complete() && !result.embeddedFailed && result.needsRetry && result.attempts.size() == 3 && result.attemptedBoundaryCurves.size() == 1,"boundary failure still continues all loop edges and requests retry");
    auto copies = mesh; copies.points.at(3).physicalId = copies.points.at(0).physicalId;
    input.boundaryLoops = {{0,1,2,3}}; input.boundaryCurves.clear();
    result = recoverPeriodicBdsConstraints(copies,input,output);
    check(result.status.complete() && result.attempts.size() == 4,"multiple physical copies allowed while boundary local IDs stay distinct");
    BdsMesh sentinel; sentinel.maximumPointId = 999; RunControl cancel; cancel.keepGoing = [] { return false; };
    check(recoverPeriodicBdsConstraints(mesh,input,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 999,"cancel preserves output");
    input.boundaryLoops = {{0,99}};
    check(recoverPeriodicBdsConstraints(mesh,input,sentinel).status.code == MeshingStatus::InvalidInput && sentinel.maximumPointId == 999,"invalid loop preserves output");
    return failures ? 1 : 0;
}
