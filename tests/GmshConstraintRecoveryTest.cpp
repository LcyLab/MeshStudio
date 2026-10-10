/** @file GmshConstraintRecoveryTest.cpp
 * @brief 初始约束穿越/已有边/失败诊断/取消原子性验证。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/initial/ConstraintRecovery.h"
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok,const char* name) { if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    check(intersectRecoverySegments({-1.,0.},{1.,0.},{0.,-1.},{0.,1.}),"proper strict crossing");
    check(!intersectRecoverySegments({-1.,0.},{1.,0.},{0.,0.},{0.,1.}) &&
          !intersectRecoverySegments({-1.,0.},{1.,0.},{-.5,0.},{.5,0.}),"endpoint and collinear contacts excluded");
    check(!intersectRecoverySegments({-1.e-100,0.},{1.e-100,0.},{0.,-1.e-100},{0.,1.e-100}),"native negative-zero product underflow excludes crossing");
    check(intersectRecoverySegments({-1.e100,0.},{1.e100,0.},{0.,-1.e100},{0.,1.e100}),"native negative-infinity product retains crossing");
    WalkTriangulation mesh;
    auto status = buildWalkTriangulation({-2.,-2.},{2.,2.},{{1,{-.5,-.5}},{2,{.5,-.5}},{3,{.5,.5}},{4,{-.5,.5}}},mesh);
    if(!status.complete()) return 2;
    FaceChart chart;
    for(int i : {2,0,3,1}) { FaceChartVertex v; v.localId = 10+i; v.physicalId = i+1; chart.vertices.push_back(v); }
    chart.constraints = {{10,10,11,false},{11,11,12,false},{12,12,13,false},{13,13,10,false}};
    std::vector<InitialConstraint> mapped;
    status = prepareInitialConstraints(chart,mesh,mapped);
    check(status.complete() && mapped.size() == 4 && mapped[0].fromVertex == 4 && mapped[0].toVertex == 5 &&
          mapped[3].fromVertex == 7 && mapped[3].toVertex == 4,"chart IDs map through physical identity preserving edge order");
    auto missing = chart; missing.constraints[0].from = 99;
    if(!mapped.empty()) mapped[0].curveEntity = 909;
    status = prepareInitialConstraints(missing,mesh,mapped);
    check(status.code == MeshingStatus::InvalidTopology && mapped.size() == 4 && mapped[0].curveEntity == 909,"missing chart endpoint fails atomically");
    WalkTriangulation output;
    auto result = recoverConstraintEdges(mesh,{{4,5,10},{5,6,11},{6,7,12},{7,4,13}},output);
    bool valid = result.status.complete() && result.decisions.size() == 4 && output.vertices.size() == mesh.vertices.size();
    if(valid) for(const auto& d : result.decisions) {
        valid &= d.nativeResult == 0 && d.recoveredHalfEdge >= 0;
        if(d.recoveredHalfEdge >= 0) {
            const auto& e = output.halfEdges[d.recoveredHalfEdge];
            valid &= e.origin == d.constraint.fromVertex && output.halfEdges[e.next].origin == d.constraint.toVertex &&
                     e.curveEntity == d.constraint.curveEntity && output.halfEdges[e.opposite].curveEntity == d.constraint.curveEntity;
        }
    }
    check(valid,"existing boundary chain records zero and tags both directed sides");
    result = recoverConstraintEdges(mesh,{{4,6,21},{5,7,22}},output);
    valid = result.status.complete() && result.decisions.size() == 2 && result.decisions[1].nativeResult == 1 &&
            result.decisions[1].crossedEdges.size() == 1 && result.decisions[1].recoveredHalfEdge >= 0;
    check(valid,"crossed diagonal uses original recovery queue without protected-tag rule");
    const auto preservedCount = output.halfEdges.size();
    result = recoverConstraintEdges(mesh,{{4,4,99}},output);
    check(result.status.complete() && result.decisions.size() == 1 && result.decisions[0].nativeResult == -1 &&
          result.decisions[0].recoveredHalfEdge == -1,"native no-first-intersection diagnostic remains visible");
    output.halfEdges[0].curveEntity = 888;
    result = recoverConstraintEdges(mesh,{{4,99,99}},output);
    check(result.status.code == MeshingStatus::InvalidInput && output.halfEdges.size() == preservedCount && output.halfEdges[0].curveEntity == 888,"invalid pool endpoint fails atomically");
    RunControl control; int calls = 0; control.keepGoing = [&] { return ++calls < 8; };
    result = recoverConstraintEdges(mesh,{{4,6,21},{5,7,22}},output,control);
    check(result.status.code == MeshingStatus::Cancelled && output.halfEdges.size() == preservedCount && output.halfEdges[0].curveEntity == 888,"cancelled recovery preserves output");
    auto broken = mesh; broken.halfEdges[0].opposite = -1;
    result = recoverConstraintEdges(broken,{{4,5,10}},output);
    check(result.status.code == MeshingStatus::InvalidTopology,"broken reciprocal half-edge is rejected before traversal");
    control = {}; control.maximumNodes = mesh.vertices.size()-1;
    result = recoverConstraintEdges(mesh,{},output,control);
    check(result.status.code == MeshingStatus::BudgetExceeded,"input nodes count against budget");
    std::cout << "constraint recovery failures=" << failures << '\n'; return failures ? 1 : 0;
}
