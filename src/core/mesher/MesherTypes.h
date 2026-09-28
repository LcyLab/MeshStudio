#pragma once
// ============================================================
// mesher 基础类型与小工具（纯 C++17，无任何第三方依赖）
// 职责：只定义几何数据类型和数值小函数，不包含任何网格算法。
// 2D 原型（Delaunay2D/Constrained2D/Ruppert2D）与 3D 阶段共用本模块。
// ============================================================

#include <cmath>
#include <cstdint>

namespace mesher {

struct Point2D { double x = 0.0, y = 0.0; };
struct Point3D { double x = 0.0, y = 0.0, z = 0.0; };
struct Vec2D  { double x = 0.0, y = 0.0; };

inline Vec2D  sub(const Point2D& a, const Point2D& b) { return { a.x - b.x, a.y - b.y }; }
inline double dot(const Vec2D& a, const Vec2D& b)     { return a.x * b.x + a.y * b.y; }
inline double cross(const Vec2D& a, const Vec2D& b)   { return a.x * b.y - a.y * b.x; }

inline double dist2D(const Point2D& a, const Point2D& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline Point2D midPoint(const Point2D& a, const Point2D& b)
{
    return { 0.5 * (a.x + b.x), 0.5 * (a.y + b.y) };
}

/// 三角形有向面积 ×2（与 orient2d 同号）：> 0 表示 a,b,c 逆时针
inline double signedArea2(const Point2D& a, const Point2D& b, const Point2D& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline double triangleArea(const Point2D& a, const Point2D& b, const Point2D& c)
{
    return 0.5 * std::fabs(signedArea2(a, b, c));
}

/// 点 p 到线段 ab 的最短距离
inline double distanceToSegment(const Point2D& p, const Point2D& a, const Point2D& b)
{
    const Vec2D ab = sub(b, a);
    const double len2 = dot(ab, ab);
    if (len2 <= 0.0)
        return dist2D(p, a);
    double t = dot(sub(p, a), ab) / len2;
    t = (t < 0.0) ? 0.0 : (t > 1.0 ? 1.0 : t);
    return dist2D(p, { a.x + t * ab.x, a.y + t * ab.y });
}

/// 三角形外接圆。共线（退化）时返回 false，否则输出圆心与半径。
bool circumcircle(const Point2D& a, const Point2D& b, const Point2D& c,
                  Point2D& center, double& radius);

} // namespace mesher
