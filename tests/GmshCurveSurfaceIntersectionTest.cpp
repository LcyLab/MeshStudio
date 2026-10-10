#include "mesh/surface/algorithm6/geometry/CurveSurfaceIntersection.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool condition, const char* name) { if(!condition) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    const CurveFunction parabola = [](double t) { return Point3D{t, 0., t * t}; };
    const SurfaceFunction plane = [](Point2D uv) { return Point3D{uv.x, uv.y, 1.}; };
    auto result = solveCurveSurfaceIntersection(parabola, plane, {.5, .1, .5});
    check(result.intersectionFound && result.status.complete() && result.stop == IntersectionStop::NewtonStep &&
          std::abs(result.parameters[0] - 1.) < 1.e-6 && std::abs(result.parameters[1]) < 1.e-6 &&
          std::abs(result.parameters[2] - 1.) < 1.e-6, "native finite difference Newton convergence");
    result = solveCurveSurfaceIntersection(parabola, plane, {1., 0., 1.});
    check(!result.intersectionFound && result.status.code == MeshingStatus::NonConverged &&
          result.stop == IntersectionStop::ExactZeroResidualExit && result.parameters == Vector3{1., 0., 1.},
          "generic native exact-zero exit returns false");
    const Circle3 circle{{0., 0., 0.}, {1., 0., 0.}, {0., 1., 0.}, 1.};
    const SurfaceFunction zeroPlane = [](Point2D uv) { return Point3D{uv.x, uv.y, 0.}; };
    result = intersectCircleWithSurface(circle, zeroPlane, {1., 0., 0.});
    check(result.intersectionFound && result.stop == IntersectionStop::InitialResidual && result.residualEvaluations == 1,
          "circle initial residual succeeds before generic exact-zero exit");
    auto signedCircle = circle; signedCircle.radius = -1.;
    result = intersectCircleWithSurface(signedCircle,zeroPlane,{-1.,0.,0.});
    check(!result.intersectionFound && result.status.code == MeshingStatus::NonConverged &&
          result.stop == IntersectionStop::ExactZeroResidualExit && result.residualEvaluations == 2,
          "native signed radius keeps negative epsilon and exact-zero false exit");
    result = intersectCircleWithSurface(circle, plane, {1., 0., 0.});
    check(result.intersectionFound && result.stop == IntersectionStop::NewtonStep && result.lastResidualNorm == 1.,
          "rank-deficient native step stop does not add final residual gate");
    result = solveCurveSurfaceIntersection(parabola, plane, {1.e6 + 1., 0., 1.});
    check(!result.intersectionFound && result.stop == IntersectionStop::ParameterNormLimit && result.residualEvaluations == 0,
          "native parameter norm cap");
    RunControl limited; limited.maximumGeometryEvaluations = 2;
    result = solveCurveSurfaceIntersection(parabola, plane, {.5, .1, .5}, limited);
    check(!result.intersectionFound && result.status.code == MeshingStatus::BudgetExceeded &&
          result.parameters == Vector3{.5, .1, .5}, "finite difference budget retains initial parameters");
    RunControl cancelled; cancelled.keepGoing = [] { return false; };
    result = intersectCircleWithSurface(circle, zeroPlane, {1., 0., 0.}, cancelled);
    check(result.status.code == MeshingStatus::Cancelled && !result.intersectionFound, "cancellation precedes fast path");
    result = solveCurveSurfaceIntersection(parabola, [](Point2D) { return Point3D{std::numeric_limits<double>::quiet_NaN(), 0., 0.}; }, {.5, .1, .5});
    check(result.status.code == MeshingStatus::GeometryQueryFailed && result.stop == IntersectionStop::InvalidQuery,
          "illegal CAD query differs from ordinary nonconvergence");
    std::cout << "curve surface intersection failures=" << failures << '\n';
    return failures ? 1 : 0;
}
