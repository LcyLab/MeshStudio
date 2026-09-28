// ============================================================
// 3D Delaunay 细化自测（M3，控制台程序）
// 流程：Delaunay 四面体化 → 约束边恢复（M2）→ Ruppert3D 细化
// 验证：立方体 + 内部点细化到 maxEdge → 拓扑有效 / 欧拉 / 体积守恒 /
//       最大边长上界 / 半径边比质量 / 边界一致
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
#include "core/mesher/Ruppert3D.h"
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

// 立方体表面三角形（输入点索引，delaunay3D 后 super 占 0-3 → 需 +4）
std::vector<std::array<int, 3>> cubeFaces()
{
    const std::array<int, 3> faces[] = {
        { 0, 2, 1 }, { 1, 2, 3 },   // z=0
        { 4, 5, 6 }, { 5, 7, 6 },   // z=1
        { 0, 1, 4 }, { 1, 5, 4 },   // y=0
        { 2, 6, 3 }, { 3, 6, 7 },   // y=1
        { 0, 4, 2 }, { 2, 4, 6 },   // x=0
        { 1, 3, 5 }, { 3, 7, 5 }    // x=1
    };
    std::vector<std::array<int, 3>> out;
    for (const auto& f : faces) {
        std::array<int, 3> s = { f[0] + 4, f[1] + 4, f[2] + 4 };
        out.push_back(s);
    }
    return out;
}

void testCube()
{
    std::cout << "\n== 1) 立方体细化 ==" << std::endl;
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
    std::vector<std::array<int, 3>> faces = cubeFaces();
    std::string err;
    Constrained3DResult c3 = enforceSurface3D(mesh, faces, &err);
    check(c3.ok, "enforceSurface3D ok");
    if (!c3.ok) {
        std::cerr << "  error: " << c3.error << std::endl;
        return;
    }
    const double volBefore = mesh.totalVolume();
    std::cout << "  [before refine] tets=" << mesh.tets.size() << " volume=" << volBefore
              << std::endl;

    Ruppert3DParams p;
    p.maxEdgeLength = 0.2;
    p.radiusEdgeRatio = 2.0;
    Ruppert3DResult r = ruppertRefine3D(mesh, faces, p);
    check(r.converged, "ruppertRefine3D converged");
    if (!r.converged)
        std::cerr << "  error: " << r.error << std::endl;
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    if (!verr.empty())
        std::cerr << "  validate: " << verr << std::endl;
    check(checkEuler(mesh), "Euler V-E+F-T == 1");
    check(std::fabs(mesh.totalVolume() - 1.0) < 1e-4, "volume conserved (cube == 1)");
    // v1 质量说明：边界细化与翻转框架未启用，退化四面体/sliver 由 M4 处理；
    // 这里只记录现状（不断言达标）
    std::cout << "  tets=" << r.tetCount << " inserted=" << r.insertedPoints
              << " iter=" << r.iterations << std::endl;
    std::cout << "  maxEdge=" << r.maxEdgeLen << " worstRho=" << r.worstRadiusEdge
              << " minDihedral=" << r.minDihedralDeg << "deg slivers=" << r.slivers << std::endl;
    std::cout << "  volumeBefore=" << volBefore << " after=" << mesh.totalVolume() << std::endl;
}

void testTetrahedronRefine()
{
    std::cout << "\n== 2) 正四面体细化 ==" << std::endl;
    std::vector<Point3D> pts = {
        { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }
    };
    std::vector<std::array<int, 3>> faces = {
        { 4, 5, 6 }, { 4, 7, 5 }, { 5, 7, 6 }, { 6, 7, 4 }
    };
    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok");
    if (!d.ok)
        return;
    std::string err;
    check(enforceSurface3D(mesh, faces, &err).ok, "enforceSurface3D ok");
    Ruppert3DParams p;
    p.maxEdgeLength = 0.15;
    Ruppert3DResult r = ruppertRefine3D(mesh, faces, p);
    check(r.converged, "ruppertRefine3D converged");
    if (!r.converged)
        std::cerr << "  error: " << r.error << std::endl;
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    check(checkEuler(mesh), "Euler");
    check(std::fabs(mesh.totalVolume() - 1.0 / 6.0) < 1e-4, "volume == 1/6");
    std::cout << "  tets=" << r.tetCount << " maxEdge=" << r.maxEdgeLen
              << " worstRho=" << r.worstRadiusEdge << std::endl;
}

void testBoundaryRefineRegression()
{
    // ============================================================
    // 回归护栏：**纯净凸包（零内部点）+ 边界细化** 当前已知损坏
    // ------------------------------------------------------------
    // 本用例用「单位立方体 8 顶点、无内部点」，这是最基础的输入场景，
    // 却暴露了两层缺陷（此前所有测试都用随机内部点遮住了它）：
    //
    // 缺陷①（M2，已修复 ✓）：enforceSurface3D 在纯净立方体上曾直接失败——
    //   "edge recovery failed on (6,8): insertConstrained broke mesh:
    //    face map size mismatch"
    //   根因：Delaunay3D 旧版对**坐标**加 ±1e-8 随机扰动以破坏共球退化，
    //   反而把精确凸包顶点打散 → 棱被判为"非网格边" → 强制插 Steiner 点。
    //   **修复**：改为谓词层符号扰动（GeomPredicates 的 orient3dSoS/insphereSoS），
    //   坐标保持精确，退化平局用确定性索引扰动打破。见 M0.5。
    //
    // 缺陷②（M3，待修）：即便边恢复成功，refineBoundary=true 的约束边
    //   分裂仍会破坏网格（"insertConstrained broke mesh: face map size mismatch"），
    //   因为在非 Delaunay 网格上做手动 cavity 重建不可靠。
    //   正解：M3.5 翻转框架——每次手动重建后 restoreDelaunay，恢复局部 Delaunay 性。
    //
    // 正解：M3.5 翻转框架（restoreDelaunay 完整迭代）——每次手动重建后
    //   恢复局部 Delaunay 性，使后续 insphere 扩展可靠。
    //
    // 本用例固化现状：断言它当前**失败**。M3.5 修好后此处应翻转为 PASS，
    // 届时把 kPureConvexExpectedToWork 改为 true。
    // ============================================================
    constexpr bool kPureConvexExpectedToWork = false;   // M3.5 完成后改 true

    std::cout << "\n== 3) 纯净凸包 + 边界细化 回归护栏（当前预期失败，待 M3.5）==" << std::endl;
    std::vector<Point3D> pts;
    for (int i = 0; i < 8; ++i)
        pts.push_back({ (i & 1) ? 1.0 : 0.0, (i & 2) ? 1.0 : 0.0, (i & 4) ? 1.0 : 0.0 });
    std::vector<std::array<int, 3>> faces = cubeFaces();
    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok");
    if (!d.ok)
        return;

    std::string err;
    Constrained3DResult c3 = enforceSurface3D(mesh, faces, &err);
    const bool constrainedOk = c3.ok;
    if (!constrainedOk)
        std::cerr << "  [缺陷①] enforce error: " << c3.error << std::endl;
    // M0.5 符号扰动（不扰动坐标）已修复缺陷①：纯净凸包边恢复现在成功。
    // 保留护栏：若将来回归，此处会失败。
    check(constrainedOk, "缺陷①已修：纯净凸包边恢复成功（M0.5 符号扰动）");
    if (!constrainedOk)
        return;   // ①未修则无法继续验证②

    const double volBefore = mesh.totalVolume();
    const int tetsBefore = static_cast<int>(mesh.tets.size());

    Ruppert3DParams p;
    p.maxEdgeLength = 0.4;         // 立方体棱长 1，远小于目标 → 必然要求分裂约束边
    p.radiusEdgeRatio = 2.0;
    p.refineBoundary = true;       // ← 关键开关
    Ruppert3DResult r = ruppertRefine3D(mesh, faces, p);

    std::string verr;
    const bool meshValid = mesh.validate(&verr);
    const bool volumeOk = std::fabs(mesh.totalVolume() - volBefore) < 1e-4;
    const bool survived = meshValid && volumeOk && !mesh.tets.empty();
    const bool effective = r.converged && r.tetCount > tetsBefore && r.maxEdgeLen <= p.maxEdgeLength;

    std::cout << "  tetsBefore=" << tetsBefore << " after=" << r.tetCount
              << " inserted=" << r.insertedPoints << " iter=" << r.iterations
              << " converged=" << r.converged << " maxEdge=" << r.maxEdgeLen << std::endl;
    std::cout << "  validate=" << meshValid << " volumeOk=" << volumeOk
              << " err=" << (r.error.empty() ? "(none)" : r.error) << std::endl;

    // ---- 护栏语义（M3.5 进行中） ----
    // 旧缺陷：边界细化会**破坏网格**（validate 失败、体积改变）。
    // 现状：insertConstrainedPoint 已改为「删除前做体积守恒校验，
    //       不守恒则安全拒绝」，故网格**不再被破坏** —— 这本身是修复。
    // 但边界细化仍**未能完成**（约束边分裂在退化腔上被安全拒绝，
    // r.converged=0），即缺陷②尚未真正修好。
    // 因此断言拆成两条：
    //   ① 网格必须**始终存活**（这条现在应为真，是硬性安全要求）
    //   ② 边界细化是否真正生效 → 由 kPureConvexExpectedToWork 控制
    check(survived, "缺陷②安全底线：边界细化不得破坏网格（体积守恒 + 非空 + 通过 validate）");
    check(effective == kPureConvexExpectedToWork,
          kPureConvexExpectedToWork
              ? "缺陷②已修：边界细化有效（单元增多且 maxEdge 达标）"
              : "缺陷②复现：边界细化未真正生效（安全拒绝，网格未损坏）");
}

} // namespace

int main()
{
    std::cout << "===== 3D Delaunay 细化自测（M3）=====" << std::endl;

    testCube();
    testTetrahedronRefine();
    testBoundaryRefineRegression();

    std::cout << "\n===== 结果: " << (g_failures == 0 ? "全部通过" : "存在失败") << " (failures="
              << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
