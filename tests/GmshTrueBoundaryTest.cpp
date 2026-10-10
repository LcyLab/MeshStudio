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
    std::vector<std::pair<double,int>> calls;
    TrueBoundaryCurveInput curve; curve.entity = 9; curve.first = -2.; curve.last = 3.;
    curve.onFace = [&](double t, int branch) { calls.push_back({t,branch}); return Point2D{t,t*t}; };
    std::vector<TrueBoundarySegment> output;
    auto result = sampleTrueBoundary({curve},output);
    check(result.status.complete() && output.size() == 299 && calls.size() == 300 && result.geometryEvaluations == 300,
          "nonseam has fixed 300 exact queries and 299 segments");
    if(output.size() == 299 && calls.size() == 300) {
        bool ordered = true;
        for(int i = 0; i < 300; ++i) ordered &= calls[i].second == 0 && calls[i].first == -2.+5.*(double(i)/299.);
        check(ordered && output.front().curveEntity == 9 && output.back().to.x == 3.,
              "nonunit lower bound and native k/299 interpolation, nonseam branch is zero");
        check(output[0].from.y == 4. && output[1].from.x == output[0].to.x,
              "each output pair repeats shared sample without resampling");
    }
    calls.clear(); curve.seam = true;
    curve.onFace = [&](double t, int branch) { calls.push_back({t,branch}); return Point2D{t,double(branch)}; };
    result = sampleTrueBoundary({curve},output);
    check(result.status.complete() && output.size() == 598 && calls.size() == 600 &&
          output[298].branch == 0 && output[299].branch == 1 && output[299].from.x == -2.,
          "seam uses full branch zero then full branch one");
    auto other = curve; other.entity = 2; other.seam = false;
    result = sampleTrueBoundary({curve,other},output);
    check(result.status.complete() && output.size() == 897 && output.front().curveEntity == 9 && output.back().curveEntity == 2,
          "caller-supplied native unique edge order is not sorted by CAD tag");
    other.first = other.last = 7.;
    result = sampleTrueBoundary({other},output);
    check(result.status.complete() && output.size() == 299 && output.front().from.x == 7. && output.back().to.x == 7.,
          "zero parameter span is sampled without silently dropping degenerate CAD edges");
    std::vector<TrueBoundarySegment> sentinel(1); sentinel[0].curveEntity = 123;
    result = sampleTrueBoundary({curve,curve},sentinel);
    check(result.status.code == MeshingStatus::InvalidInput && sentinel[0].curveEntity == 123,
          "sampler requires topology adapter to provide unique CAD edge identities");
    auto bad = curve; bad.onFace = {};
    result = sampleTrueBoundary({bad},sentinel);
    check(result.status.code == MeshingStatus::InvalidInput && sentinel[0].curveEntity == 123, "missing exact pcurve capability");
    bad = curve; bad.onFace = [](double, int) -> Point2D { throw std::invalid_argument("pcurve failure"); };
    result = sampleTrueBoundary({bad},sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && sentinel[0].curveEntity == 123,
          "CAD exception preserves output and retains query-failure classification");
    bad.onFace = [](double, int) { return Point2D{0.,std::numeric_limits<double>::quiet_NaN()}; };
    result = sampleTrueBoundary({bad},sentinel);
    check(result.status.code == MeshingStatus::GeometryQueryFailed, "nonfinite UV has explicit failure");
    bad.onFace = [](double, int) -> Point2D { throw OperationFailure{{MeshingStatus::Cancelled,"query cancelled"}}; };
    result = sampleTrueBoundary({bad},sentinel);
    check(result.status.code == MeshingStatus::Cancelled, "typed callback cancellation is not swallowed");
    RunControl control; control.maximumGeometryEvaluations = 299;
    result = sampleTrueBoundary({curve},sentinel,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 299 && sentinel[0].curveEntity == 123,
          "query budget never publishes partial boundary segments");
    control = {}; control.maximumNodes = 599;
    result = sampleTrueBoundary({curve},sentinel,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && sentinel[0].curveEntity == 123,
          "sampled-node budget counts both seam branches");
    control = {}; control.keepGoing = [] { return false; };
    result = sampleTrueBoundary({curve},sentinel,control);
    check(result.status.code == MeshingStatus::Cancelled && sentinel[0].curveEntity == 123, "entry cancellation is atomic");
    result = sampleTrueBoundary({},output);
    check(result.status.complete() && output.empty() && result.geometryEvaluations == 0, "empty unique edge list");
    std::cout << "true boundary failures=" << failures << '\n';
    return failures ? 1 : 0;
}
