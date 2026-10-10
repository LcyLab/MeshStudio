#include "geometry/predicates/GeomPredicates.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
int main()
{
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if(!condition) { ++failures; std::cerr << "FAIL " << message << '\n'; }
    };
    const double epsilon = std::numeric_limits<double>::epsilon();
    const Point2D a{1. + epsilon, 1.}, b{1., 1. - epsilon}, origin{};
    check(orient2dDet(a,b,origin) == 0., "fixture loses the sign in ordinary products");
    check(orient2dValue(a,b,origin) == -epsilon * epsilon, "adaptive numerical area preserves product residual");
    check(orient2dValue(b,a,origin) == epsilon * epsilon, "numerical area is antisymmetric");
    check(orient2dValue(a,b,{1.e16,1.e16}) == -epsilon * epsilon,
          "full coordinate-tail expansion survives a large translated origin");
    check(orient2d(a,b,origin) == -1, "existing sign contract remains available");
    check(orient2dValue({0.,0.},{4.,0.},{0.,3.}) == 12., "numerical interface keeps area magnitude");
    check(orient2dValue({0.,0.},{4.,0.},{0.,3.}) > orient2dValue({0.,0.},{2.,0.},{0.,3.}),
          "Walk can compare two non-unit orientation values");
    check(orient2dValue({1.,1.},{2.,2.},{3.,3.}) == 0., "exact collinearity remains zero");
    const Point2D x{1.,0.}, y{0.,1.}, inner{.1,.1};
    check(incircleOriented(origin,x,y,inner) == 1 && incircleOriented(origin,y,x,inner) == -1,
          "oriented circle sign retains triangle orientation");
    check(incircle(origin,y,x,inner) == 1, "existing normalized circle sign is unchanged");
    check(incircleOriented(origin,x,{2.,0.},{1.,1.}) == 1 && incircle(origin,x,{2.,0.},{1.,1.}) == 0,
          "oriented collinear determinant differs from normalized undefined circle");
    check(incircleOriented(origin,x,y,{1.,1.}) == 0, "oriented exact cocircular tie remains zero");
    std::cout << "geometry predicate failures=" << failures << '\n';
    return failures ? 1 : 0;
}
