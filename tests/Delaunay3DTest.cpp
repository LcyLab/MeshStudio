// ============================================================
// 3D Delaunay 原型自测（M1 验证，控制台程序）
// 验证：
//   1. 3D 谓词符号正确性（orient3d / insphere）
//   2. 正四面体 4 点 → 1 个四面体 + 体积正确
//   3. 立方体 + 内部随机点 → 体积守恒（≈1）
//   4. 随机 300 点 → 欧拉公式 V-E+F-T=1 / 局部 Delaunay / 拓扑自检
// ============================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "core/mesher/Delaunay3D.h"
#include "core/mesher/GeomPredicates.h"
#include "core/mesher/TetMesh3D.h"

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

// 被至少一个四面体引用的顶点数（points 可能残留孤立点，如 super 顶点）
int activeVertexCount(const TetMesh3D& mesh)
{
    std::vector<char> used(mesh.points.size(), 0);
    for (const Tet4& t : mesh.tets)
        for (int e = 0; e < 4; ++e)
            used[t.v[e]] = 1;
    return static_cast<int>(std::count(used.begin(), used.end(), 1));
}

// 3D 欧拉公式：V - E + F - T = 1（单连通实心体），F 为总无向面数
bool checkEuler(const TetMesh3D& mesh)
{
    const int T = static_cast<int>(mesh.tets.size());
    const int V = activeVertexCount(mesh);
    std::set<std::pair<int, int>> edges;         // 无向边
    std::set<std::tuple<int, int, int>> faces;   // 无向面（顶点排序）
    for (const Tet4& t : mesh.tets) {
        for (int e = 0; e < 4; ++e) {
            const int a = t.v[(e + 1) % 4];
            const int b = t.v[(e + 2) % 4];
            const int c = t.v[(e + 3) % 4];
            edges.insert({ std::min(a, b), std::max(a, b) });
            edges.insert({ std::min(b, c), std::max(b, c) });
            edges.insert({ std::min(c, a), std::max(c, a) });
            int x = a, y = b, z = c;
            if (x > y) std::swap(x, y);
            if (y > z) std::swap(y, z);
            if (x > y) std::swap(x, y);
            faces.insert({ x, y, z });
        }
    }
    const int E = static_cast<int>(edges.size());
    const int F = static_cast<int>(faces.size());
    const bool ok = V - E + F - T == 1;
    if (!ok)
        std::cerr << "  Euler: V=" << V << " E=" << E << " F=" << F << " T=" << T
                  << " -> " << (V - E + F - T) << std::endl;
    return ok;
}

// 局部 Delaunay：每个内部面两侧，对侧顶点不得落在邻居四面体外接球内
bool checkLocalDelaunay(const TetMesh3D& mesh, double tol)
{
    for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
        const Tet4& tet = mesh.tets[t];
        for (int e = 0; e < 4; ++e) {
            const int nb = tet.n[e];
            if (nb < 0)
                continue;
            const int opp = tet.v[e];   // t 中面 e 的对侧顶点
            const Tet4& nt = mesh.tets[nb];
            if (insphereDet(mesh.points[nt.v[0]], mesh.points[nt.v[1]],
                            mesh.points[nt.v[2]], mesh.points[nt.v[3]],
                            mesh.points[opp]) > tol)
                return false;
        }
    }
    return true;
}

// ---- 测试 1：3D 谓词 ----
void testPredicates3D()
{
    std::cout << "\n== 1) 3D 谓词符号 ==" << std::endl;
    // 坐标轴四面体 (0,0,0)(1,0,0)(0,1,0)(0,0,1)：右手系
    check(orient3d({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }) > 0, "orient3d right-handed > 0");
    check(orient3d({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 }) < 0, "orient3d swapped < 0");
    check(orient3d({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }) == 0, "orient3d coplanar == 0");
    // 坐标轴四面体外接球圆心 (0.25,0.25,0.25)，半径 ≈0.433；
    // 用严格球内非球心点（球心会与 4 顶点共球，insphereDet==0）
    check(insphere({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.25, 0.3, 0.25 }) > 0,
          "insphere inside > 0");
    check(insphere({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 10, 10, 10 }) < 0,
          "insphere outside < 0");
    const double d1 = insphereDet({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.25, 0.3, 0.25 });
    const double d2 = insphereDet({ 0, 0, 0 }, { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, { 0.25, 0.3, 0.25 });
    check(d1 > 0.0 && d2 > 0.0, "insphereDet orientation-independent (both > 0)");
}

// ---- 测试 2：正四面体 ----
void testSingleTet()
{
    std::cout << "\n== 2) 正四面体 4 点 ==" << std::endl;
    const std::vector<Point3D> pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }
    };
    TetMesh3D mesh;
    Delaunay3DResult res = delaunay3D(mesh, pts);
    check(res.ok, "delaunay3D ok");
    if (!res.ok) {
        std::cerr << "  error: " << res.error << std::endl;
        return;
    }
    check(mesh.tets.size() == 1, "single tet produced");
    if (mesh.tets.size() != 1) {
        std::cerr << "  tets=" << mesh.tets.size() << " volume=" << mesh.totalVolume() << std::endl;
        for (int k = 0; k < static_cast<int>(mesh.tets.size()); ++k)
            std::cerr << "    tet " << k << " v=(" << mesh.tets[k].v[0] << ","
                      << mesh.tets[k].v[1] << "," << mesh.tets[k].v[2] << ","
                      << mesh.tets[k].v[3] << ")" << std::endl;
    }
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    // 体积 = 1/6
    check(std::fabs(mesh.totalVolume() - 1.0 / 6.0) < 1e-9, "volume == 1/6");
    std::cout << "  volume=" << mesh.totalVolume() << std::endl;
}

// ---- 测试 3：立方体 + 内部点，体积守恒 ----
void testCubeVolume()
{
    std::cout << "\n== 3) 立方体 + 内部随机点（体积守恒） ==" << std::endl;
    std::vector<Point3D> pts;
    for (int i = 0; i < 8; ++i)
        pts.push_back({ (i & 1) ? 1.0 : 0.0, (i & 2) ? 1.0 : 0.0, (i & 4) ? 1.0 : 0.0 });
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> uni(0.1, 0.9);
    for (int i = 0; i < 50; ++i)
        pts.push_back({ uni(rng), uni(rng), uni(rng) });

    TetMesh3D mesh;
    Delaunay3DResult res = delaunay3D(mesh, pts);
    check(res.ok, "delaunay3D ok");
    if (!res.ok) {
        std::cerr << "  error: " << res.error << std::endl;
        return;
    }
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    check(checkEuler(mesh), "Euler V-E+F-T == 1");
    check(checkLocalDelaunay(mesh, 1e-9), "local Delaunay property");
    // 凸包 = 扰动后的立方体，体积 ≈ 1
    check(std::fabs(mesh.totalVolume() - 1.0) < 1e-4, "volume conserved (cube == 1)");
    std::cout << "  V=" << activeVertexCount(mesh) << " T=" << mesh.tets.size()
              << " volume=" << mesh.totalVolume() << std::endl;
}

// ---- 测试 4：随机 300 点 ----
void testRandom3D(int n)
{
    std::cout << "\n== 4) 随机 " << n << " 点 Delaunay ==" << std::endl;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::vector<Point3D> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i)
        pts.push_back({ uni(rng), uni(rng), uni(rng) });

    TetMesh3D mesh;
    Delaunay3DResult res = delaunay3D(mesh, pts);
    check(res.ok, "delaunay3D ok");
    if (!res.ok) {
        std::cerr << "  error: " << res.error << std::endl;
        return;
    }
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    if (!verr.empty())
        std::cerr << "  validate: " << verr << std::endl;
    check(checkEuler(mesh), "Euler V-E+F-T == 1");
    check(checkLocalDelaunay(mesh, 1e-9), "local Delaunay property");
    std::cout << "  V=" << activeVertexCount(mesh) << " T=" << mesh.tets.size()
              << " steps=" << res.insertSteps << std::endl;
}

} // namespace

int main()
{
    std::cout << "===== 3D Delaunay 四面体化自测 =====" << std::endl;

    testPredicates3D();
    testSingleTet();
    testCubeVolume();
    testRandom3D(300);

    std::cout << "\n===== 结果: " << (g_failures == 0 ? "全部通过" : "存在失败") << " (failures="
              << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
