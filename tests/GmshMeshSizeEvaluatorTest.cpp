#include "mesh/surface/algorithm6/boundary/MeshSizeEvaluator.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool condition, const char* name) {
        if(!condition) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    auto close = [](double a, double b) { return std::abs(a - b) < 1.e-12; };
    SurfaceMeshingOptions options;
    options.sizeMin = 1.; options.sizeMax = 4.; options.sizeFactor = 2.;
    options.elementsPerCurvatureCircle = 8.;
    MeshSizeQuery query{1, 12, .2, 0., {2., 0., 0.}};
    GeometrySizeSource source;
    source.pointSize = [](const auto&) { return 6.; };
    source.maximumCurvature = [](const auto&) { return .1; };
    source.backgroundField = [](const auto&) { return 3.; };
    source.entitySize = [](const auto&) { return 2.; };
    source.prescribedCurveSize = [](const auto&) { return .5; };
    source.entityFactor = [](const auto&) { return 3.; };
    bool callbackSawMinimum = false;
    source.sizeCallback = [&](const auto& q, double size) {
        callbackSawMinimum = q.entity == 12 && close(size, .5);
        return 9.;
    };
    auto evaluated = evaluateMeshSize(query, 7., options, source);
    check(evaluated.status.complete() && callbackSawMinimum && close(evaluated.size, 24.),
          "callback before model/clamp, entity and global factors after clamp");
    source.sizeCallback = {};
    source.entityFactor = {};
    evaluated = evaluateMeshSize(query, 7., options, source);
    check(evaluated.status.complete() && close(evaluated.size, 2.), "minimum then clamp");
    options.sizeMin = 0.; options.sizeMax = 1.e22; options.sizeFactor = 1.;
    options.sizeFromPoints = false; options.elementsPerCurvatureCircle = 0.;
    source.pointSize = [](const auto&) -> double { throw std::runtime_error("disabled point source"); };
    source.maximumCurvature = [](const auto&) -> double { throw std::runtime_error("disabled curvature source"); };
    evaluated = evaluateMeshSize(query, 7., options, source);
    check(evaluated.status.complete() && close(evaluated.size, .5), "disabled sources never called");
    query.dimension = 2;
    source.prescribedCurveSize = [](const auto&) -> double { throw std::runtime_error("curve-only source"); };
    options.sizeFromPoints = true;
    evaluated = evaluateMeshSize(query, 7., options, source);
    check(evaluated.status.complete() && close(evaluated.size, 2.), "face skips point and curve-only sources");
    source = {};
    evaluated = evaluateMeshSize(query, 7., options, source);
    check(evaluated.status.complete() && close(evaluated.size, 7.), "unconstrained model size");
    source.sizeCallback = [](const auto&, double) { return 0.; };
    evaluated = evaluateMeshSize(query, 7., options, source);
    check(evaluated.status.complete() && evaluated.usedInvalidSizeFallback && close(evaluated.size, 7.),
          "nonpositive native fallback is explicit");
    options.sizeMin = .3;
    evaluated = evaluateMeshSize(query, 7., options, source);
    check(evaluated.status.complete() && !evaluated.usedInvalidSizeFallback && close(evaluated.size, .3),
          "minimum can repair nonpositive callback before native fallback");
    source.sizeCallback = [](const auto&, double) { return std::numeric_limits<double>::quiet_NaN(); };
    check(evaluateMeshSize(query, 7., options, source).status.code == MeshingStatus::GeometryQueryFailed,
          "nonfinite source query visible");
    source = {}; options.elementsPerCurvatureCircle = 8.;
    check(evaluateMeshSize(query, 7., options, source).status.code == MeshingStatus::GeometryQueryFailed,
          "enabled curvature capability required");
    auto curvature = evaluateCurvatureSize(.5, 8.);
    check(curvature.status.complete() && close(curvature.size, std::acos(-1.) / 2.), "curvature physical length");
    curvature = evaluateCurvatureSize(0., 8.);
    check(curvature.status.complete() && curvature.size >= 1.e22, "zero curvature unconstrained");
    curvature = evaluateCurvatureSize(.5, .25);
    check(curvature.status.complete() && curvature.clampedCurvatureCircleCount &&
          close(curvature.size, 4. * std::acos(-1.)), "native circle count lower bound");
    check(!evaluateCurvatureSize(-.5, 8.).status.complete(), "negative maximum curvature invalid");
    std::cout << "mesh size evaluator failures=" << failures << '\n';
    return failures ? 1 : 0;
}
