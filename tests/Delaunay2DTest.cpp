// ============================================================
// 2D Delaunay 原型自测（控制台程序）
// 验证四件事，分别对应 3D 阶段的四大核心机制：
//   1. 谓词符号正确性（GeomPredicates）
//   2. Bowyer-Watson 增量三角化：欧拉公式 / 局部 Delaunay 性质 / 面积守恒
//   3. Lawson 翻转约束恢复：约束边全部成为网格边
//   4. Ruppert 细化主循环：边长上界 / 最小角保证 / 域面积不变 / 质心均在域内
// 另输出 ascii STL（z=0 薄片）到 test_output/ 供可视化检查。
// ============================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "mesh/planar/Constrained2D.h"
#include "mesh/planar/Delaunay2D.h"
#include "geometry/predicates/GeomPredicates.h"
#include "mesh/data/Mesh2D.h"
#include "mesh/planar/Ruppert2D.h"

using namespace mesher;

namespace {

int g_failures = 0;

void check(bool condition, const char* what)
{
    if (condition) {
        std::cout << "[PASS] " << what << std::endl;
    } else {
        std::cerr << "[FAIL] " << what << std::endl;
        ++g_failures;
    }
}

// ---- 通用小工具（仅测试用） ----

// 被至少一个三角形引用的顶点数（mesh.points 可能残留孤立点，如 super 顶点）
int activeVertexCount(const Mesh2D& mesh)
{
    std::vector<char> used(mesh.points.size(), 0);
    for (const Tri2D& t : mesh.tris)
        for (int e = 0; e < 3; ++e)
            used[t.v[e]] = 1;
    return static_cast<int>(std::count(used.begin(), used.end(), 1));
}

// 欧拉公式：V - E + F = 1（单连通平面剖分）
bool checkEuler(const Mesh2D& mesh)
{
    const int F = static_cast<int>(mesh.tris.size());
    const int V = activeVertexCount(mesh);
    int boundaryEdges = 0;
    for (const Tri2D& t : mesh.tris)
        for (int e = 0; e < 3; ++e)
            if (t.n[e] < 0)
                ++boundaryEdges;
    const int E = (3 * F + boundaryEdges) / 2;
    return V - E + F == 1;
}

// 局部 Delaunay：每条内部边两侧，任一侧顶点不得落在另一侧三角形外接圆内
bool checkLocalDelaunay(const Mesh2D& mesh, double tol)
{
    for (int t = 0; t < static_cast<int>(mesh.tris.size()); ++t) {
        const Tri2D& tri = mesh.tris[t];
        for (int e = 0; e < 3; ++e) {
            const int nb = tri.n[e];
            if (nb < 0)
                continue;
            const int u = tri.v[e], v = tri.v[(e + 1) % 3];
            const int w = tri.v[0] + tri.v[1] + tri.v[2] - u - v;          // t 侧顶点
            const Tri2D& ntri = mesh.tris[nb];
            const int q = ntri.v[0] + ntri.v[1] + ntri.v[2] - u - v;        // nb 侧顶点
            if (incircleDet(mesh.points[u], mesh.points[v], mesh.points[q],
                            mesh.points[w]) > tol)
                return false;
        }
    }
    return true;
}

// Andrew monotone chain 凸包（返回逆时针顶点索引）
std::vector<int> convexHull(const std::vector<Point2D>& pts)
{
    std::vector<int> idx(pts.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {
        return (pts[a].x != pts[b].x) ? (pts[a].x < pts[b].x) : (pts[a].y < pts[b].y);
    });
    std::vector<int> hull;
    for (const int i : idx) {   // 下链
        while (hull.size() >= 2
               && orient2dDet(pts[hull[hull.size() - 2]], pts[hull.back()], pts[i]) <= 0.0)
            hull.pop_back();
        hull.push_back(i);
    }
    const int lower = static_cast<int>(hull.size());
    for (int k = static_cast<int>(idx.size()) - 2; k >= 0; --k) {   // 上链
        const int i = idx[k];
        while (hull.size() > lower
               && orient2dDet(pts[hull[hull.size() - 2]], pts[hull.back()], pts[i]) <= 0.0)
            hull.pop_back();
        hull.push_back(i);
    }
    hull.pop_back();
    return hull;
}

double polygonArea(const std::vector<Point2D>& poly)
{
    double a2 = 0.0;
    for (int i = 0; i < static_cast<int>(poly.size()); ++i) {
        const Point2D& p = poly[i];
        const Point2D& q = poly[(i + 1) % poly.size()];
        a2 += p.x * q.y - q.x * p.y;
    }
    return 0.5 * std::fabs(a2);
}

// 射线法：点在多边形内
bool pointInPolygon(const Point2D& p, const std::vector<Point2D>& poly)
{
    bool inside = false;
    for (int i = 0, j = static_cast<int>(poly.size()) - 1; i < static_cast<int>(poly.size()); j = i++) {
        const Point2D& a = poly[i];
        const Point2D& b = poly[j];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double xint = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (p.x < xint)
                inside = !inside;
        }
    }
    return inside;
}

// 输出 ascii STL（z=0 薄片，法向 +z），供视口/Paraview 可视化
void writeStl(const char* path, const Mesh2D& mesh)
{
    std::error_code ec;
    std::filesystem::create_directories("test_output", ec);
    std::ofstream out(path);
    if (!out)
        return;
    out << "solid mesh\n";
    for (const Tri2D& t : mesh.tris) {
        out << "  facet normal 0 0 1\n    outer loop\n";
        for (int e = 0; e < 3; ++e) {
            const Point2D& p = mesh.points[t.v[e]];
            out << "      vertex " << p.x << " " << p.y << " 0\n";
        }
        out << "    endloop\n  endfacet\n";
    }
    out << "endsolid mesh\n";
}

// ---- 测试 1：谓词符号 ----
void testPredicates()
{
    std::cout << "\n== 1) 谓词符号 ==\n";
    check(orient2d({ 0, 0 }, { 1, 0 }, { 0, 1 }) > 0, "orient2d CCW > 0");
    check(orient2d({ 0, 0 }, { 0, 1 }, { 1, 0 }) < 0, "orient2d CW < 0");
    check(orient2d({ 0, 0 }, { 1, 1 }, { 2, 2 }) == 0, "orient2d collinear == 0");
    // 单位三角形外接圆心 (0.5, 1/6)；p 在圆内/圆外
    check(incircle({ 0, 0 }, { 1, 0 }, { 0, 1 }, { 0.5, 0.5 }) > 0, "incircle inside > 0");
    check(incircle({ 0, 0 }, { 1, 0 }, { 0, 1 }, { 5, 5 }) < 0, "incircle outside < 0");
    // 朝向无关性：顺时针三角形输入，结果一致
    const double d1 = incircleDet({ 0, 0 }, { 1, 0 }, { 0, 1 }, { 0.5, 0.5 });
    const double d2 = incircleDet({ 0, 0 }, { 0, 1 }, { 1, 0 }, { 0.5, 0.5 });
    check(d1 > 0.0 && d2 > 0.0, "incircleDet orientation-independent (both > 0)");
}

// ---- 测试 2：随机点 Delaunay ----
void testRandomDelaunay(int n)
{
    std::cout << "\n== 2) 随机 " << n << " 点 Delaunay ==\n";
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::vector<Point2D> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i)
        pts.push_back({ uni(rng), uni(rng) });

    Mesh2D mesh;
    Delaunay2DResult res = delaunay2D(mesh, pts);
    check(res.ok, "delaunay2D ok");
    if (!res.ok) {
        std::cerr << "  error: " << res.error << std::endl;
        return;
    }
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    if (!verr.empty())
        std::cerr << "  validate: " << verr << std::endl;
    check(checkEuler(mesh), "Euler V-E+F == 1");
    check(checkLocalDelaunay(mesh, 1e-9), "local Delaunay property");

    // 面积守恒：三角形面积和 == 凸包面积（凸包只用被引用的活跃顶点，排除 super 残留）
    std::vector<char> used(mesh.points.size(), 0);
    for (const Tri2D& t : mesh.tris)
        for (int e = 0; e < 3; ++e)
            used[t.v[e]] = 1;
    std::vector<Point2D> active;
    for (int i = 0; i < static_cast<int>(mesh.points.size()); ++i)
        if (used[i])
            active.push_back(mesh.points[i]);
    const std::vector<int> hull = convexHull(active);
    std::vector<Point2D> hullPts;
    for (const int i : hull)
        hullPts.push_back(active[i]);
    const double triSum = mesh.totalArea();
    const double hullArea = polygonArea(hullPts);
    check(std::fabs(triSum - hullArea) < 1e-9 * std::max(1.0, hullArea),
          "area conserved (triSum == hullArea)");

    std::cout << "  V=" << activeVertexCount(mesh) << " F=" << mesh.tris.size()
              << " steps=" << res.insertSteps
              << " hullArea=" << hullArea << " triArea=" << triSum << std::endl;
    writeStl("test_output/delaunay_random.stl", mesh);
}

// ---- 测试 3：约束恢复（正方形 + 强制另一条对角线） ----
void testConstrained()
{
    std::cout << "\n== 3) 约束恢复（正方形+对角线） ==\n";
    const std::vector<Point2D> pts = {
        { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 }
    };
    // 正方形四条边 + 一条对角线 (0,2)；无约束时对角线是否已是网格边不定，
    // 强制恢复另一条对角线 (1,3) 作为考验。
    // 注意：delaunay2D 的 super 顶点占索引 0,1,2，输入点从 3 开始，约束边要 +3。
    const std::vector<std::pair<int, int>> constraints = {
        { 3, 4 }, { 4, 5 }, { 5, 6 }, { 6, 3 }, { 4, 6 }
    };
    Mesh2D mesh;
    Delaunay2DResult res = delaunay2D(mesh, pts);
    check(res.ok, "delaunay2D ok");
    if (!res.ok)
        return;
    std::string err;
    check(enforceConstraints(mesh, constraints, &err), "enforceConstraints ok");
    if (!err.empty())
        std::cerr << "  error: " << err << std::endl;
    check(mesh.validate(&err) && checkEuler(mesh), "mesh valid after constraints");
    bool allPresent = true;
    for (const auto& e : constraints) {
        const bool ok = mesh.hasEdgeAny(e.first, e.second);
        if (!ok) {
            allPresent = false;
            std::cerr << "  missing edge: (" << e.first << "," << e.second << ")"
                      << " = (" << mesh.points[e.first].x << "," << mesh.points[e.first].y
                      << ")-(" << mesh.points[e.second].x << "," << mesh.points[e.second].y
                      << ")" << std::endl;
            for (int k = 0; k < static_cast<int>(mesh.tris.size()); ++k) {
                const Tri2D& t = mesh.tris[k];
                std::cerr << "    tri " << k << " v=(" << t.v[0] << "," << t.v[1] << ","
                          << t.v[2] << ") n=(" << t.n[0] << "," << t.n[1] << "," << t.n[2]
                          << ")" << std::endl;
            }
        }
    }
    check(allPresent, "all constraint edges present in mesh");
    // 对角线 (1,3) 把正方形分为两个三角形
    check(mesh.tris.size() == 2, "square split into 2 triangles by diagonal");
}

// ---- 测试 4：L 形细化 ----
void testLShapeRefine()
{
    std::cout << "\n== 4) L 形 Ruppert 细化 ==\n";
    // L 形（逆时针），面积 = 3；输入点索引从 3 开始（super 占 0,1,2）
    const std::vector<Point2D> pts = {
        { 0, 0 }, { 2, 0 }, { 2, 1 }, { 1, 1 }, { 1, 2 }, { 0, 2 }
    };
    std::vector<std::pair<int, int>> constraints = {
        { 3, 4 }, { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 8 }, { 8, 3 }
    };

    Mesh2D mesh;
    Delaunay2DResult res = delaunay2D(mesh, pts);
    check(res.ok, "delaunay2D ok");
    if (!res.ok)
        return;
    std::string err;
    check(enforceConstraints(mesh, constraints, &err), "enforceConstraints ok");
    if (!err.empty())
        std::cerr << "  error: " << err << std::endl;

    RuppertParams params;
    params.maxEdgeLength = 0.15;
    RuppertResult rr = ruppertRefine(mesh, constraints, params);
    check(rr.converged, "ruppertRefine converged");
    if (!rr.converged)
        std::cerr << "  error: " << rr.error << std::endl;
    check(mesh.validate(&err), "mesh valid after refine");
    check(checkEuler(mesh), "Euler after refine");

    // 边界约束段全部在网格中（细化可能已把长段分裂，constraints 返回最终段）
    bool allPresent = true;
    for (const auto& e : constraints)
        allPresent = allPresent && mesh.hasEdgeAny(e.first, e.second);
    check(allPresent, "all (split) constraint edges present");

    // 最小角保证（理论 ~20.7°，留数值余量）
    check(rr.minAngleDeg > 18.0, "min angle > 18 deg");
    // 边长上界
    check(rr.maxEdgeLen < 0.15 * 1.5, "max edge < 1.5 * target");
    // 域面积守恒（L 形面积 3；容差放宽——delaunay2D 的符号扰动带来 ~1e-8 坐标误差）
    const double finalArea = mesh.totalArea();
    check(std::fabs(finalArea - 3.0) < 1e-6, "domain area conserved (== 3)");
    if (std::fabs(finalArea - 3.0) >= 1e-6)
        std::cerr << "  area=" << std::setprecision(12) << finalArea
                  << " diff=" << (finalArea - 3.0) << std::endl;

    // 全部三角形质心落在多边形内（域保持验证）
    bool allInside = true;
    for (const Tri2D& t : mesh.tris) {
        const Point2D c = { (mesh.points[t.v[0]].x + mesh.points[t.v[1]].x + mesh.points[t.v[2]].x) / 3.0,
                            (mesh.points[t.v[0]].y + mesh.points[t.v[1]].y + mesh.points[t.v[2]].y) / 3.0 };
        if (!pointInPolygon(c, pts))
            allInside = false;
    }
    check(allInside, "all triangle centroids inside polygon");

    std::cout << "  tris=" << rr.triCount << " inserted=" << rr.insertedPoints
              << " iter=" << rr.iterations
              << " minAngle=" << rr.minAngleDeg << " deg"
              << " maxEdge=" << rr.maxEdgeLen << std::endl;
    writeStl("test_output/lshape_refined.stl", mesh);
}

} // namespace

int main()
{
    std::cout << "===== 2D Delaunay 原型自测 =====" << std::endl;

    testPredicates();
    testRandomDelaunay(500);
    testConstrained();
    testLShapeRefine();

    std::cout << "\n===== 结果: " << (g_failures == 0 ? "全部通过" : "存在失败") << " (failures="
              << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
