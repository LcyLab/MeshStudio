/** @file GmshParametricDomainTest.cpp
 * @brief FAR高界为零、严格端点/重复段/孔、空域和控制状态。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/chart/ParametricDomain.h"
#include <iostream>
#include <limits>
using namespace mesher; using namespace mesher::gmshReference;
int main() {
    int failures = 0; auto check = [&](bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } };
    Point2D far{99.,99.}; auto status = computeParametricFarPoint({-2.,-4.},{0.,0.},far);
    check(status.complete() && far.x == 2. && far.y == 4.,"FAR preserves high+(high-low) when high is zero");
    std::vector<TrueBoundarySegment> boundary = {{1,0,{0.,0.},{2.,0.}},{1,0,{2.,0.},{2.,2.}},{1,0,{2.,2.},{0.,2.}},{1,0,{0.,2.},{0.,0.}}};
    auto result = testParametricDomain(boundary,{.5,.5},{3.,4.});
    check(result.status.complete() && result.inside && result.crossingCount == 1,"odd strict crossing inside");
    result = testParametricDomain(boundary,{-.5,.5},{3.,4.}); check(!result.inside && result.crossingCount == 2,"even crossings outside");
    result = testParametricDomain(boundary,{.5,.5},{3.,3.}); check(!result.inside && !result.crossingCount,"ray through polygon vertex is not repaired into inside");
    result = testParametricDomain(boundary,{0.,.5},{3.,4.}); check(result.inside && result.crossingCount == 1,"point on edge retains strict source crossing result, no boundary override");
    boundary.push_back(boundary[2]); result = testParametricDomain(boundary,{.5,.5},{3.,4.}); check(!result.inside && result.crossingCount == 2,"duplicate crossed sampling segment counted independently");
    result = testParametricDomain({}, {.5,.5},{3.,4.}); check(result.status.complete() && !result.inside && !result.crossingCount,"non-null empty boundary rejects");
    far = {99.,99.}; status = computeParametricFarPoint({2.,0.},{1.,1.},far); check(status.code == MeshingStatus::InvalidInput && far.x == 99.,"invalid range leaves output");
    RunControl control; control.keepGoing = [] { return false; }; result = testParametricDomain(boundary,{.5,.5},{3.,4.},control); check(result.status.code == MeshingStatus::Cancelled,"cancel is explicit");
    result = testParametricDomain(boundary,{std::numeric_limits<double>::infinity(),0.},{3.,4.}); check(result.status.code == MeshingStatus::InvalidInput,"nonfinite query is explicit");
    return failures ? 1 : 0;
}
