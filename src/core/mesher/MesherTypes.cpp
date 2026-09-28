#include "MesherTypes.h"

namespace mesher {

bool circumcircle(const Point2D& a, const Point2D& b, const Point2D& c,
                  Point2D& center, double& radius)
{
    const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if (std::fabs(d) < 1e-300)   // 三点共线（调用方应保证非退化，此处仅防除零）
        return false;
    const double a2 = a.x * a.x + a.y * a.y;
    const double b2 = b.x * b.x + b.y * b.y;
    const double c2 = c.x * c.x + c.y * c.y;
    center.x = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    center.y = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    radius = dist2D(center, a);
    return true;
}

} // namespace mesher
