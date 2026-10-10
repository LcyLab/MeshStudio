#include "mesh/surface/algorithm6/geometry/SurfaceMetric.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool condition, const char* name) { if(!condition) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    auto close = [](double a, double b) { return std::abs(a - b) < 1.e-12; };
    auto evaluated = evaluateSurfaceMetric({2., 0., 0.}, {1., 1., 0.});
    check(evaluated.status.complete() && evaluated.metric.gUU == 4. && evaluated.metric.gUV == 2. &&
          evaluated.metric.gVV == 2. && !evaluated.singularDerivatives, "Gram matrix differs from Cholesky factors");
    auto circle = computeMetricCircumcircle({Point2D{0., 0.}, {1., 0.}, {0., 1.}}, evaluated.metric);
    check(circle.status.complete() && close(circle.center.x, .5) && close(circle.center.y, 0.) &&
          close(circle.radiusSquared, 1.) && !circle.singularLinearSystem, "UV shear retains physical circumcircle");
    circle = computeMetricCircumcircle({Point2D{0., 0.}, {1., 0.}, {.5, std::sqrt(3.) * .5}}, {1., 0., 1.});
    check(circle.status.complete() && close(circle.center.x, .5) && close(circle.center.y, std::sqrt(3.) / 6.) &&
          close(circle.radiusSquared, 1. / 3.), "equilateral metric circle");
    circle = computeMetricCircumcircle({Point2D{5., 5.}, {6., 5.}, {7., 5.}}, {1., 0., 1.});
    check(circle.status.complete() && circle.singularLinearSystem && circle.center.x == 0. && circle.center.y == 0. &&
          circle.radiusSquared == 50., "native singular solve output is flagged");
    evaluated = evaluateSurfaceMetric({1., 0., 0.}, {0., 0., 0.});
    check(evaluated.status.complete() && evaluated.singularDerivatives, "periodic pole derivative state explicit");
    check(!evaluateSurfaceMetric({std::numeric_limits<double>::infinity(), 0., 0.}, {0., 1., 0.}).status.complete(),
          "nonfinite derivative query rejected");
    auto physical = computePhysicalCircumradius({Point3D{2., 3., 4.}, {4., 3., 4.}, {3., 4., 4.}});
    check(physical.status.complete() && close(physical.center.x, 3.) && close(physical.center.y, 3.) &&
          close(physical.center.z, 4.) && close(physical.radius, 1.) && !physical.degenerateTriangle,
          "physical radius independent of UV and target size");
    physical = computePhysicalCircumradius({Point3D{0., 0., 0.}, {1., 0., 0.}, {2., 0., 0.}});
    check(physical.status.complete() && physical.degenerateTriangle && physical.radius == 99999.,
          "native degenerate physical sentinel explicit");
    std::cout << "surface metric failures=" << failures << '\n';
    return failures ? 1 : 0;
}
