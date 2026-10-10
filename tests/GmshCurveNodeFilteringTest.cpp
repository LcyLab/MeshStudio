#include "mesh/surface/algorithm6/boundary/CurveDiscretization.h"
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool passed, const char* name) { if(!passed) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    const std::vector<CurveNode> original{{1,.05,{.05,0.,0.},100.}, {2,.1,{.1,0.,0.},100.},
        {3,1.,{1.,0.,0.},100.}, {4,1.05,{1.05,0.,0.},100.}, {5,2.,{2.,0.,0.},100.}};
    auto nodes = original;
    std::vector<double> queried;
    auto result = filterCurveNodes(nodes, {}, 0., 0, [&](double t) { queried.push_back(t); return 1.; });
    check(result.status.complete() && nodes.size() == 2 && nodes[0].id == 3 && nodes[1].id == 5,
          "native last-retained-node filtering");
    check(result.removedNodeIds == std::vector<std::uint64_t>{1,2,4} && result.sizeEvaluations == 5,
          "removed physical identities and query count");
    check(queried == std::vector<double>{.05,.05,.5,1.025,1.5}, "native first-point and midpoint query parameters");
    nodes = original;
    result = filterCurveNodes(nodes, {}, 0., 3, [](double) { return 1.; });
    check(nodes.size() == original.size() && result.minimumNodeCountPreventedRemoval && result.removedNodeIds.empty(),
          "minimum count prevents the whole batch instead of partial removal");
    nodes = {{1,.3,{.3,0.,0.},-100.}};
    result = filterCurveNodes(nodes, {}, 0., 0, [](double) { return 1.; });
    check(nodes.size() == 1 && result.removedNodeIds.empty(), "strict chord threshold and stored size is ignored");
    nodes = {{1,.1,{0.,0.,0.},1.}, {2,.5,{.5,0.,0.},1.}};
    result = filterCurveNodes(nodes, {}, 0., 0, [](double) { return 1.; });
    check(nodes.size() == 1 && nodes[0].id == 2, "coincident candidate is removed without becoming last retained");
    nodes = {{1,.2,{.2,0.,0.},1.}, {2,.5,{.5,0.,0.},1.}};
    result = filterCurveNodes(nodes, {}, 0., 0, [](double t) { return .5 * (1. + 2. * t); });
    check(nodes.size() == 1 && nodes[0].id == 2, "varying size exposes the native first-node query location");
    nodes = original;
    RunControl budget; budget.maximumGeometryEvaluations = 2;
    result = filterCurveNodes(nodes, {}, 0., 0, [](double) { return 1.; }, budget);
    check(result.status.code == MeshingStatus::BudgetExceeded && nodes.size() == original.size() && result.sizeEvaluations == 2,
          "budget exit preserves original interior nodes");
    RunControl cancelled; cancelled.keepGoing = [] { return false; };
    result = filterCurveNodes(nodes, {}, 0., 0, [](double) { return 1.; }, cancelled);
    check(result.status.code == MeshingStatus::Cancelled && nodes.size() == original.size(), "cancelled filtering is atomic");
    result = filterCurveNodes(nodes, {}, 0., 0, [](double) { return std::numeric_limits<double>::quiet_NaN(); });
    check(result.status.code == MeshingStatus::GeometryQueryFailed && nodes.size() == original.size(), "illegal size query is explicit");
    int inverseCalls = 0;
    nodes = {{1,.4,{.4,0.,0.},1.}, {2,.8,{.8,0.,0.},1.}};
    result = filterCurveNodes(nodes, {}, [&] { ++inverseCalls; return 0.; },0,[](double) { return 1.; });
    check(result.status.complete() && inverseCalls == 0, "no CAD inverse query when first node retained");
    nodes = {{1,.05,{.05,0.,0.},1.}, {2,.1,{.1,0.,0.},1.}, {3,.5,{.5,0.,0.},1.}};
    queried.clear();
    result = filterCurveNodes(nodes, {}, [&] { ++inverseCalls; return .02; },0,
        [&](double t) { queried.push_back(t); return 1.; });
    check(result.status.complete() && inverseCalls == 2 && queried == std::vector<double>({.05,.060000000000000005,.26}) &&
          nodes.size() == 1 && nodes[0].id == 3,
          "inverse queried at each later candidate while last retained point is 0D");
    std::cout << "curve node filtering failures=" << failures << '\n';
    return failures ? 1 : 0;
}
