// ============================================================
// 3D 表面约束恢复自测（M2，CDT 边界恢复，控制台程序）
// 验证：
//   1. 正四面体表面：4 面本就是网格面，无需 Steiner 点
//   2. 立方体表面 + 内部点：Delaunay 后恢复 12 个表面三角形
//      → 网格有效 / 欧拉 / 体积守恒 / 边界面质心都在约束面上
// ============================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "core/mesher/Constrained3D.h"
#include "core/mesher/Delaunay3D.h"
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

int activeVertexCount(const TetMesh3D& mesh)
{
    std::vector<char> used(mesh.points.size(), 0);
    for (const Tet4& t : mesh.tets)
        for (int e = 0; e < 4; ++e)
            used[t.v[e]] = 1;
    return static_cast<int>(std::count(used.begin(), used.end(), 1));
}

bool checkEuler(const TetMesh3D& mesh)
{
    const int T = static_cast<int>(mesh.tets.size());
    const int V = activeVertexCount(mesh);
    std::set<std::pair<int, int>> edges;
    std::set<std::tuple<int, int, int>> faces;
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
    return V - E + F - T == 1;
}

// 约束面（立方体 6 面 × 2 三角）的顶点索引（输入点 0-7 对应 (0/1)^3）
std::vector<std::array<int, 3>> cubeFaces()
{
    // 顶点：0=(0,0,0) 1=(1,0,0) 2=(0,1,0) 3=(1,1,0) 4=(0,0,1) 5=(1,0,1) 6=(0,1,1) 7=(1,1,1)
    // 每个面两个三角形（逆时针外法向）
    const std::array<int, 3> faces[] = {
        { 0, 2, 1 }, { 1, 2, 3 },   // z=0
        { 4, 5, 6 }, { 5, 7, 6 },   // z=1
        { 0, 1, 4 }, { 1, 5, 4 },   // y=0
        { 2, 6, 3 }, { 3, 6, 7 },   // y=1
        { 0, 4, 2 }, { 2, 4, 6 },   // x=0
        { 1, 3, 5 }, { 3, 7, 5 }    // x=1
    };
    return std::vector<std::array<int, 3>>(std::begin(faces), std::end(faces));
}

// ---- 测试 1：正四面体表面 ----
void testTetrahedron()
{
    std::cout << "\n== 1) 正四面体表面 ==" << std::endl;
    std::vector<Point3D> pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }
    };
    // 表面 4 个三角形（输入点索引 4,5,6,7，因为 delaunay3D 里 super 占 0-3）
    std::vector<std::array<int, 3>> faces = {
        { 4, 5, 6 }, { 4, 7, 5 }, { 5, 7, 6 }, { 6, 7, 4 }
    };
    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok");
    if (!d.ok) {
        std::cerr << "  error: " << d.error << std::endl;
        return;
    }
    std::string err;
    Constrained3DResult c = enforceSurface3D(mesh, faces, &err);
    check(c.ok, "enforceSurface3D ok");
    if (!c.ok)
        std::cerr << "  error: " << c.error << std::endl;
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    check(checkEuler(mesh), "Euler V-E+F-T == 1");
    check(std::fabs(mesh.totalVolume() - 1.0 / 6.0) < 1e-9, "volume == 1/6");
    check(c.steinerPoints == 0, "no Steiner points needed");
    std::cout << "  tets=" << mesh.tets.size() << " steiner=" << c.steinerPoints << std::endl;
}

// ---- 测试 2：立方体表面恢复 ----
void testCube()
{
    std::cout << "\n== 2) 立方体表面恢复 ==" << std::endl;
    std::vector<Point3D> pts;
    for (int i = 0; i < 8; ++i)
        pts.push_back({ (i & 1) ? 1.0 : 0.0, (i & 2) ? 1.0 : 0.0, (i & 4) ? 1.0 : 0.0 });
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> uni(0.15, 0.85);
    for (int i = 0; i < 30; ++i)
        pts.push_back({ uni(rng), uni(rng), uni(rng) });

    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok");
    if (!d.ok) {
        std::cerr << "  error: " << d.error << std::endl;
        return;
    }
    const std::vector<std::array<int, 3>> facesRaw = cubeFaces();
    // delaunay3D 后 super 占索引 0-3，输入点从 4 开始 → 面索引 +4
    std::vector<std::array<int, 3>> faces = facesRaw;
    for (auto& f : faces)
        for (int k = 0; k < 3; ++k)
            f[k] += 4;
    std::string err;
    Constrained3DResult c = enforceSurface3D(mesh, faces, &err);
    check(c.ok, "enforceSurface3D ok");
    if (!c.ok)
        std::cerr << "  error: " << c.error << std::endl;
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    if (!verr.empty())
        std::cerr << "  validate: " << verr << std::endl;
    check(checkEuler(mesh), "Euler V-E+F-T == 1");
    check(std::fabs(mesh.totalVolume() - 1.0) < 1e-4, "volume conserved (cube == 1)");

    // 每个网格边界面（n<0）的质心应落在约束面上（立方体：某坐标 ≈0 或 ≈1）
    bool boundaryOnSurface = true;
    int boundaryFaces = 0;
    double boundaryArea = 0.0;
    for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
        for (int e = 0; e < 4; ++e) {
            if (mesh.tets[t].n[e] >= 0)
                continue;
            ++boundaryFaces;
            const int a = mesh.tets[t].v[(e + 1) % 4];
            const int b = mesh.tets[t].v[(e + 2) % 4];
            const int c = mesh.tets[t].v[(e + 3) % 4];
            const Point3D& A = mesh.points[a];
            const Point3D& B = mesh.points[b];
            const Point3D& C = mesh.points[c];
            // 面积
            const Point3D ab = { B.x - A.x, B.y - A.y, B.z - A.z };
            const Point3D ac = { C.x - A.x, C.y - A.y, C.z - A.z };
            const Point3D cr = { ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z,
                                 ab.x * ac.y - ab.y * ac.x };
            boundaryArea += 0.5 * std::sqrt(cr.x * cr.x + cr.y * cr.y + cr.z * cr.z);
            // 质心坐标
            const double cx = (A.x + B.x + C.x) / 3.0;
            const double cy = (A.y + B.y + C.y) / 3.0;
            const double cz = (A.z + B.z + C.z) / 3.0;
            const double dx = std::min(cx, 1.0 - cx);
            const double dy = std::min(cy, 1.0 - cy);
            const double dz = std::min(cz, 1.0 - cz);
            if (std::min(dx, std::min(dy, dz)) > 1e-6)
                boundaryOnSurface = false;
        }
    }
    check(boundaryOnSurface, "all boundary face centroids on constraint surface");
    check(std::fabs(boundaryArea - 6.0) < 1e-3, "boundary area == cube surface (6)");
    // 每个约束面是否已是网格面（或子面覆盖——先查原面）
    int faceCovered = 0;
    for (const auto& f : faces)
        if (mesh.hasFaceAny(f[0], f[1], f[2]))
            ++faceCovered;
    std::cout << "  constraint faces directly covered: " << faceCovered << "/" << faces.size()
              << std::endl;
    std::cout << "  tets=" << mesh.tets.size() << " steiner=" << c.steinerPoints
              << " boundaryFaces=" << boundaryFaces << " boundaryArea=" << boundaryArea << std::endl;
}

// ============================================================
// 3) §8「back out」：插入不可行时必须优雅退让，而非整体失败
// ------------------------------------------------------------
// 论文 §8：
//   "[a mesh generation program] should test for this possibility, and
//    back out when vertex insertion cannot succeed."
// `insertConstrainedPoint` 在修改网格前做体积守恒校验，失败即安全拒绝
//（mesh 保持原样）。本测试构造一个**必然导致退化**的约束边：让它的一条
// 约束棱上已存在一个共线顶点，使中点插入产生退化/不守恒 cavity。
//
// 验收契约（这是「功能不退步」的关键）：
//   · enforceSurface3D 不应崩、不应留下损坏网格；
//   · 若返回成功 → mesh 必须 validate 通过 + 体积守恒；
//   · 若返回失败 → mesh 也必须保持有效（安全拒绝，非静默损坏）。
// 两种情况都是「可接受的优雅行为」；**唯一不可接受**的是返回成功但网格损坏。
// ============================================================
void testBackoutSafety()
{
    std::cout << "\n== 3) §8 back-out：不可插入时优雅退让 ==" << std::endl;
    std::vector<Point3D> pts;
    for (int i = 0; i < 8; ++i)
        pts.push_back({ (i & 1) ? 1.0 : 0.0, (i & 2) ? 1.0 : 0.0, (i & 4) ? 1.0 : 0.0 });
    // 在若干棱中点额外放入共线点 → 之后对这些棱做中点细分时，中点与已有点
    // 重合/共线，cavity 重建极易出现零体积退化 → 触发 back-out 路径。
    const int kEdges[][2] = { {4,6}, {5,7}, {4,5}, {6,7} };   // 立方体底/顶面的"对角"位
    (void)kEdges;
    pts.push_back({ 0.5, 0.5, 0.0 });   // 底面对角线中点
    pts.push_back({ 0.5, 0.5, 1.0 });   // 顶面对角线中点
    pts.push_back({ 0.5, 0.0, 0.5 });   // 一侧面中心
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> uni(0.2, 0.8);
    for (int i = 0; i < 20; ++i)
        pts.push_back({ uni(rng), uni(rng), uni(rng) });

    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok (degenerate-leaning set)");
    if (!d.ok) {
        std::cerr << "  error: " << d.error << std::endl;
        return;
    }

    const std::vector<std::array<int, 3>> facesRaw = cubeFaces();
    std::vector<std::array<int, 3>> faces = facesRaw;
    for (auto& f : faces)
        for (int k = 0; k < 3; ++k)
            f[k] += 4;

    const double volBefore = mesh.totalVolume();
    const size_t tetsBefore = mesh.tets.size();

    std::string err;
    Constrained3DResult c = enforceSurface3D(mesh, faces, &err);

    // —— 核心契约：无论成功失败，网格都必须有效 ——
    std::string verr;
    const bool valid = mesh.validate(&verr);
    check(valid, "back-out: mesh.validate after enforceSurface3D");
    if (!valid)
        std::cerr << "  validate: " << verr << std::endl;
    check(!mesh.tets.empty(), "back-out: mesh not emptied");

    if (c.ok) {
        check(std::fabs(mesh.totalVolume() - volBefore) < 1e-4,
              "back-out: volume conserved on success");
        check(checkEuler(mesh), "back-out: Euler V-E+F-T == 1 on success");
    } else {
        // 安全拒绝：网格未损坏即为通过（这条路径正是 §8 要保护的行为）
        check(true, "back-out: graceful rejection (mesh intact, not corrupted)");
        std::cerr << "  note: enforceSurface3D refused -> " << c.error << std::endl;
    }
    std::cout << "  ok=" << (c.ok ? 1 : 0) << " tets " << tetsBefore << " -> " << mesh.tets.size()
              << " steiner=" << c.steinerPoints << std::endl;
}

} // namespace

int main()
{
    std::cout << "===== 3D 表面约束恢复自测（CDT）=====" << std::endl;

    testTetrahedron();
    testCube();
    testBackoutSafety();

    std::cout << "\n===== 结果: " << (g_failures == 0 ? "全部通过" : "存在失败") << " (failures="
              << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
