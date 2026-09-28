// ============================================================
// 表面网格输入准备自测（M5，控制台程序）
// 验证：STL 式"三角形汤"（顶点重复、无拓扑）经焊接后得到闭合流形表面，
//       水密检查与闭合体积正确；缺失三角形能被水密检查发现。
// ============================================================

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "core/mesher/SurfaceMesh3D.h"

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

// 立方体 12 个外向三角形（顶点索引 0-7）
const int kCubeTris[12][3] = {
    { 0, 4, 7 }, { 0, 7, 3 },   // x=0 (-x)
    { 1, 2, 6 }, { 1, 6, 5 },   // x=1 (+x)
    { 0, 1, 5 }, { 0, 5, 4 },   // y=0 (-y)
    { 3, 7, 6 }, { 3, 6, 2 },   // y=1 (+y)
    { 0, 3, 2 }, { 0, 2, 1 },   // z=0 (-z)
    { 4, 5, 6 }, { 4, 6, 7 }    // z=1 (+z)
};

const Point3D kCubeVerts[8] = {
    { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 },
    { 0, 0, 1 }, { 1, 0, 1 }, { 1, 1, 1 }, { 0, 1, 1 }
};

// 生成 STL 式三角形汤：每个三角形独立顶点（36 个顶点）
void makeSoup(std::vector<Point3D>& verts, std::vector<std::array<int, 3>>& tris)
{
    for (int t = 0; t < 12; ++t) {
        const int base = static_cast<int>(verts.size());
        for (int k = 0; k < 3; ++k)
            verts.push_back(kCubeVerts[kCubeTris[t][k]]);
        tris.push_back({ base, base + 1, base + 2 });
    }
}

void testWeldAndWatertight()
{
    std::cout << "\n== 1) cube triangle soup: weld -> watertight ==" << std::endl;
    std::vector<Point3D> verts;
    std::vector<std::array<int, 3>> tris;
    makeSoup(verts, tris);
    check(verts.size() == 36, "soup has 36 duplicated vertices");

    const WeldResult w = weldSurfaceMesh(verts, tris, 1e-9);
    check(w.surface.vertices.size() == 8, "welded to 8 unique vertices");
    check(w.mergedVertices == 28, "28 duplicate vertices merged");
    check(w.surface.triangles.size() == 12, "12 triangles kept");
    check(w.droppedTriangles == 0, "no degenerate triangles dropped");

    std::string err;
    check(isWatertight(w.surface, &err), "watertight");
    if (!err.empty())
        std::cerr << "  error: " << err << std::endl;
    const double vol = enclosedVolume(w.surface);
    check(std::fabs(vol - 1.0) < 1e-9, "enclosed volume == 1 (outward normals)");
    std::cout << "  volume=" << vol << std::endl;
}

void testNonWatertight()
{
    std::cout << "\n== 2) missing triangle detected ==" << std::endl;
    std::vector<Point3D> verts;
    std::vector<std::array<int, 3>> tris;
    makeSoup(verts, tris);
    tris.pop_back();   // 去掉一个三角形
    const WeldResult w = weldSurfaceMesh(verts, tris, 1e-9);
    std::string err;
    check(!isWatertight(w.surface, &err), "missing triangle -> not watertight");
    std::cout << "  error: " << err << std::endl;
}

void testToleranceWeld()
{
    std::cout << "\n== 3) tolerance weld (near-duplicate vertices) ==" << std::endl;
    std::vector<Point3D> verts;
    std::vector<std::array<int, 3>> tris;
    makeSoup(verts, tris);
    // 给部分顶点加 1e-7 抖动（模拟不同 CAD 导出的坐标误差）
    for (int i = 0; i < static_cast<int>(verts.size()); i += 3)
        verts[i].x += 1e-7;
    const WeldResult w = weldSurfaceMesh(verts, tris, 1e-6);
    check(w.surface.vertices.size() == 8, "tolerance weld -> 8 vertices");
    std::string err;
    check(isWatertight(w.surface, &err), "watertight after tolerance weld");
}

// 用例 4：近似重复点跨「量化格子」边界
// 背景：焊接用 floor(v/tol) 建格子。距离 < tol 的两点完全可能被格边界分开
//       （如 x = 0.5·tol 与 x = 1.2·tol 分属 cell 0 / cell 1）。
//       只查自身格子的旧实现会漏合并 → 一条棱被 4 个面共享 → 不水密。
//       STEP/IGES/BRep 逐面离散（BRepMesh）在共享棱上产生的就是这种顶点。
void testCrossCellWeld()
{
    std::cout << "\n== 4) near-duplicates straddling a cell boundary ==" << std::endl;
    std::vector<Point3D> verts;
    std::vector<std::array<int, 3>> tris;
    makeSoup(verts, tris);

    const double tol = 1e-6;
    // 按三角形序号的奇偶把整片顶点推到 0.5·tol / 1.2·tol 两个格子。
    // 同一角点的多个副本因此分布在相邻格子，必须靠邻域探测才能合并。
    for (int t = 0; t < 12; ++t) {
        const double off = (t % 2 == 0) ? 0.5 * tol : 1.2 * tol;
        for (int k = 0; k < 3; ++k)
            verts[static_cast<std::size_t>(3 * t + k)].x += off;
    }

    const WeldResult w = weldSurfaceMesh(verts, tris, tol);
    check(w.surface.vertices.size() == 8,
          "cross-cell: welded to 8 unique vertices (neighborhood probe)");
    check(w.mergedVertices == 28, "cross-cell: 28 duplicate vertices merged");
    check(w.droppedTriangles == 0, "cross-cell: no degenerate triangles dropped");

    std::string err;
    check(isWatertight(w.surface, &err), "cross-cell: watertight");
    if (!err.empty())
        std::cerr << "  error: " << err << std::endl;
    const double vol = enclosedVolume(w.surface);
    check(std::fabs(vol - 1.0) < 1e-5, "cross-cell: enclosed volume == 1");
    std::cout << "  volume=" << vol << std::endl;
}

// 用例 5：大坐标偏移 + 负坐标
// 背景：旧 quantKey 把坐标量化成 21 位整数（偏移 1048576），有效范围只有
//       ±1048575·tol。引擎传 tol = diag·1e-7 ≈ 1.73e-7（单位立方体），
//       范围仅 ±0.18 —— 把立方体平移到 ±500 后所有坐标被钳到同一个格子，
//       焊接塌成 1 个顶点、12 个三角形全部退化被丢弃，网格彻底丢失。
void testLargeOffsetAndNegative()
{
    std::cout << "\n== 5) large offset + negative coordinates ==" << std::endl;
    std::vector<Point3D> verts;
    std::vector<std::array<int, 3>> tris;
    makeSoup(verts, tris);
    const double ox = -500.0, oy = -500.0, oz = 500.0;   // 三轴都远离原点
    for (Point3D& p : verts) {
        p.x += ox;
        p.y += oy;
        p.z += oz;
    }

    const double diag = std::sqrt(3.0);
    const WeldResult w = weldSurfaceMesh(verts, tris, diag * 1e-7);
    check(w.surface.vertices.size() == 8,
          "far-offset: welded to 8 vertices (no 21-bit clamp collapse)");
    check(w.mergedVertices == 28, "far-offset: 28 duplicate vertices merged");
    check(w.droppedTriangles == 0, "far-offset: no degenerate triangles dropped");

    std::string err;
    check(isWatertight(w.surface, &err), "far-offset: watertight");
    const double vol = enclosedVolume(w.surface);
    check(std::fabs(vol - 1.0) < 1e-4, "far-offset: enclosed volume == 1");
    std::cout << "  volume=" << vol << std::endl;
}

} // namespace

int main()
{
    std::cout << "===== surface mesh prep self-test (M5) =====" << std::endl;

    testWeldAndWatertight();
    testNonWatertight();
    testToleranceWeld();
    testCrossCellWeld();
    testLargeOffsetAndNegative();

    std::cout << "\n===== result: " << (g_failures == 0 ? "ALL PASS" : "FAILURES")
              << " (failures=" << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
