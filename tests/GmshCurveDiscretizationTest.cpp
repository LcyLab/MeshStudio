#include "mesh/surface/algorithm6/boundary/CurveDiscretization.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;

int main()
{
    int failures = 0;
    auto check = [&](bool condition, const char* name) {
        if(!condition) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    auto close = [](double a, double b) { return std::abs(a - b) < 1.e-12; };
    check(computeCurvePointCount(10., 1) == 11 && computeCurvePointCount(10.02, 1) == 12 &&
          computeCurvePointCount(.2, 7) == 8, "native count transition and minimum segments");
    SurfaceMeshingOptions options;
    check(computeMinimumCurveSegments(CurveKind::Line, 1., false, 1, options) == 1 &&
          computeMinimumCurveSegments(CurveKind::Other, 1., false, 1, options) == 2,
          "native line and general-curve minimum segments");
    check(computeMinimumCurveSegments(CurveKind::Circle, 2. * std::acos(-1.), true, 1, options) == 7 &&
          computeMinimumCurveSegments(CurveKind::Ellipse, std::acos(-1.), false, 1, options) == 3,
          "native full circle and partial ellipse count");
    check(computeMinimumCurveSegments(CurveKind::Other, 1., true, 1, options) == 4 &&
          computeMinimumCurveSegments(CurveKind::Line, 1., false, 9, options) == 9,
          "closed-curve and prescribed minimum segments");
    auto stored = computeStoredCurveSize({0.,1.,2.,0.}, {1.,3.,4.,0.}, 5.);
    check(stored.status.complete() && stored.finiteResult && stored.size == 1.25,
          "stored curve size preserves shadowed speed instead of cumulative target");
    stored = computeStoredCurveSize({0.,1.,2.,0.}, {1.,2.,3.,0.}, 1.);
    check(stored.status.complete() && !stored.finiteResult && std::isinf(stored.size),
          "nonfinite stored metadata is exposed instead of replaced by a mesh size");
    CurveDensityTable table;
    const auto constantSize = [](double) { return 1.; };
    auto status = integrateCurveDensity(0., 1., [](double) { return Point3D{10., 0., 0.}; }, constantSize, table);
    check(status.complete() && table.samples.size() == 129 && table.densityEvaluations == 129,
          "minimum recursion depth even for constant density");
    bool exactTable = status.complete() && table.samples.size() == 129;
    if(exactTable) for(std::size_t i = 0; i < table.samples.size(); ++i) {
        const auto& sample = table.samples[i];
        exactTable = exactTable && close(sample.parameter, double(i) / 128.) &&
            close(sample.density, 10.) && close(sample.cumulativeDensity, 10. * double(i) / 128.);
    }
    check(exactTable, "constant curve cumulative table");
    std::vector<double> parameters{-9.};
    status = distributeCurveParameters(table, 11, parameters);
    bool exactParameters = status.complete() && parameters.size() == 11;
    if(exactParameters) for(int i = 0; i < 11; ++i) exactParameters &= close(parameters[i], double(i) / 10.);
    check(exactParameters, "linear inverse includes original endpoints");
    CurveDensityTable linearInverse;
    linearInverse.samples = {{0., 1., 0., 1.}, {.5, 3., 1., 1.}, {1., 1., 2., 1.}};
    status = distributeCurveParameters(linearInverse, 5, parameters);
    check(status.complete() && parameters == std::vector<double>({0., .25, .5, .75, 1.}),
          "table inverse is piecewise linear, not density polynomial inverse");
    CurveDensityTable backward;
    backward.samples = {{0., .2, 0., 2.}, {1., 4., 2.1, 4.}};
    status = smoothCurveSizes(backward, 1.5);
    check(status.complete() && close(backward.samples.front().density, 2. / 3.) &&
          close(backward.samples.back().cumulativeDensity, 7. / 3.), "native reverse sweep uses right endpoint speed");
    CurveDensityTable zeroDensity;
    zeroDensity.samples = {{0., 0., 0., 1.}, {1., 1., .5, 1.}};
    status = smoothCurveSizes(zeroDensity, 1.5);
    check(status.complete() && close(zeroDensity.samples.front().density, 2. / 3.),
          "zero density with positive speed is native infinity, not zero derivative");
    CurveDensityTable varying;
    status = integrateCurveDensity(0., 1., [](double) { return Point3D{10., 0., 0.}; },
        [](double t) { return 1. + 10. * t; }, varying);
    check(status.complete() && std::abs(varying.samples.back().cumulativeDensity - std::log(11.)) < 2.e-6,
          "variable density integral");
    const auto beforeTable = table.samples;
    const auto unchangedTable = [&] {
        if(table.samples.size() != beforeTable.size()) return false;
        for(std::size_t i = 0; i < beforeTable.size(); ++i) {
            const auto& a = table.samples[i]; const auto& b = beforeTable[i];
            if(a.parameter != b.parameter || a.density != b.density ||
               a.cumulativeDensity != b.cumulativeDensity || a.speed != b.speed) return false;
        }
        return true;
    };
    RunControl limited; limited.maximumGeometryEvaluations = 10;
    status = integrateCurveDensity(0., 1., [](double) { return Point3D{10., 0., 0.}; }, constantSize, table, 1.e-9, limited);
    check(status.code == MeshingStatus::BudgetExceeded && unchangedTable(),
          "budget termination retains output and is not completion");
    RunControl cancelled; cancelled.keepGoing = [] { return false; };
    status = integrateCurveDensity(0., 1., [](double) { return Point3D{10., 0., 0.}; }, constantSize, table, 1.e-9, cancelled);
    check(status.code == MeshingStatus::Cancelled && unchangedTable(), "explicit cancellation");
    status = integrateCurveDensity(0., 1., [](double) { return Point3D{1., 0., 0.}; },
        [](double) { return std::numeric_limits<double>::quiet_NaN(); }, table);
    check(status.code == MeshingStatus::GeometryQueryFailed && unchangedTable(), "invalid query retains output");
    status = smoothCurveSizes(table, std::sqrt(1.8), cancelled);
    check(status.code == MeshingStatus::Cancelled && unchangedTable(), "smoothing cancellation retains complete table");
    const auto beforeParameters = parameters;
    status = distributeCurveParameters(linearInverse, 5, parameters, cancelled);
    check(status.code == MeshingStatus::Cancelled && parameters == beforeParameters, "inverse cancellation retains output");
    bool rejected = false;
    try { computeCurvePointCount(std::numeric_limits<double>::max(), 1); }
    catch(const std::overflow_error&) { rejected = true; }
    check(rejected, "count overflow cannot silently wrap");
    std::cout << "curve discretization failures=" << failures << '\n';
    return failures ? 1 : 0;
}
