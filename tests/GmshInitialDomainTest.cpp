/** @file GmshInitialDomainTest.cpp
 * @brief 外域/内域/孔的原生着色及 UV 翻边原子性验证。
 * @date 2026-10-07
 * @modified 2026-10-08
 */
#include "mesh/surface/algorithm6/initial/DomainClassification.h"
#include "mesh/surface/algorithm6/initial/InitialUvLegalization.h"
#include "mesh/surface/algorithm6/initial/ConstraintRecovery.h"
#include "mesh/surface/algorithm6/data/HalfEdgeTopology.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok,const char* name) { if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    const std::vector<InitialChartVertex> points{{1,{-1.,-1.}},{2,{1.,-1.}},{3,{1.,1.}},{4,{-1.,1.}},
        {5,{-.3,-.3}},{6,{.3,-.3}},{7,{.3,.3}},{8,{-.3,.3}}};
    WalkTriangulation mesh;
    if(!buildWalkTriangulation({-2.,-2.},{2.,2.},points,mesh).complete()) return 2;
    const std::vector<InitialConstraint> constraints{{4,5,10},{5,6,11},{6,7,12},{7,4,13},{8,9,20},{9,10,21},{10,11,22},{11,8,23}};
    WalkTriangulation recovered;
    const auto recovery = recoverConstraintEdges(mesh,constraints,recovered);
    if(!recovery.status.complete()) return 2;
    WalkTriangulation colored = recovered;
    auto result = classifyFaceDomain(recovered,47,colored);
    bool valid = result.status.complete() && result.interiorSeedHalfEdge >= 0 && !result.exteriorVisits.empty() && !result.interiorVisits.empty();
    int exterior = 0, interior = 0, unknown = 0;
    if(valid) for(const auto& f : colored.faces) {
        exterior += f.domain == -2; interior += f.domain == 47; unknown += f.domain == -1;
    }
    check(valid && exterior > 0 && interior > 0 && unknown == 2,"outer rectangle flood and hole residual remain explicit");
    WalkTriangulation legalized = recovered;
    auto uv = legalizeInitialUvMesh(colored,47,legalized);
    check(uv.status.complete() && !uv.passLimitReached && !uv.flippedHalfEdges.empty() && uv.flippedHalfEdges.back().empty(),"native final empty UV scan records convergence");
    const std::vector<InitialChartVertex> pressurePoints{{1,{-1.,-1.}},{2,{1.,-1.}},{3,{1.,1.}},{4,{-1.,1.}},
        {5,{-.5,0.}},{6,{.5,0.}},{7,{0.,-.05}},{8,{0.,.05}}};
    WalkTriangulation pressure, pressureRecovered, pressureColored, pressureLegalized;
    auto pressureStatus = buildWalkTriangulation({-2.,-2.},{2.,2.},pressurePoints,pressure);
    const auto pressureRecovery = recoverConstraintEdges(pressure,{{4,5,10},{5,6,11},{6,7,12},{7,4,13},{8,9,77}},pressureRecovered);
    const auto pressureDomain = classifyFaceDomain(pressureRecovered,47,pressureColored);
    const auto pressureUv = legalizeInitialUvMesh(pressureColored,47,pressureLegalized);
    valid = pressureStatus.complete() && pressureRecovery.status.complete() && pressureDomain.status.complete() && pressureUv.status.complete() &&
            pressureRecovery.decisions.size() == 5 && pressureUv.flippedHalfEdges.size() == 2 && pressureUv.flippedHalfEdges[0].size() == 1;
    if(valid) {
        const int edgeId = pressureRecovery.decisions.back().recoveredHalfEdge;
        const auto& e = pressureLegalized.halfEdges[edgeId]; const int to = pressureLegalized.halfEdges[e.next].origin;
        valid &= e.curveEntity == 77 && ((e.origin == 10 && to == 11) || (e.origin == 11 && to == 10));
    }
    check(valid,"native UV phase flips embedded diagonal and keeps its pool label");
    colored.vertices[0].physicalId = 888; // 输出哨兵，输入不依赖该对象。
    result = classifyFaceDomain(mesh,47,colored);
    check(result.status.code == MeshingStatus::NonConverged && colored.vertices[0].physicalId == 888,"missing native interior seed fails atomically");
    RunControl control; control.keepGoing = [] { return false; };
    result = classifyFaceDomain(recovered,47,colored,control);
    check(result.status.code == MeshingStatus::Cancelled && colored.vertices[0].physicalId == 888,"classification cancellation preserves output");
    legalized.vertices[0].physicalId = 999;
    uv = legalizeInitialUvMesh(recovered,47,legalized,control);
    check(uv.status.code == MeshingStatus::Cancelled && legalized.vertices[0].physicalId == 999,"UV cancellation preserves output");
    control = {}; control.maximumNodes = recovered.vertices.size()-1;
    result = classifyFaceDomain(recovered,47,colored,control);
    check(result.status.code == MeshingStatus::BudgetExceeded,"classification node budget is not domain completion");
    auto bad = recovered; bad.faces[0].halfEdge = 999;
    result = classifyFaceDomain(bad,47,colored);
    check(result.status.code == MeshingStatus::InvalidTopology,"invalid face head rejected before Color");
    std::cout << "initial domain failures=" << failures << '\n'; return failures ? 1 : 0;
}
