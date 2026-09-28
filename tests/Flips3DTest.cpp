// ============================================================
// 3D 翻转框架自测（M3.5，控制台程序）
// 验证：
//   1. 已是 Delaunay 的网格 → restoreDelaunay 应 0 翻转（幂等）
//   2. 干净 Delaunay 网格上 flip23：2 个四面体 → 3 个，拓扑/体积保持
//   3. 细化网格上 restoreDelaunay：局部 Delaunay 违规下降、体积保持
//      （受限：细化产物含近退化单元，翻转数值敏感，v1 只保证不恶化）
// ============================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "core/mesher/Constrained3D.h"
#include "core/mesher/Delaunay3D.h"
#include "core/mesher/Flips3D.h"
#include "core/mesher/GeomPredicates.h"
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

// 局部 Delaunay 违规数：每个内部面，对侧顶点落在邻居外接球内。
// ★ 必须与 restoreDelaunay / flip23 内部使用**同一个谓词**（insphereSoS）★
// ------------------------------------------------------------
// 旧实现用 `insphereDet(...) > 1e-9`（带容差的朴素浮点谓词），而实现内部用
// `insphereSoS(...) > 0`（符号扰动、绝不返回 0）。两者在共球退化处判定不同：
// 实测同一退化输入，朴素谓词只看到 2 个违规，SoS 谓词看到 61 个 ——
// 也就是说**旧测试在测一个与实现不同的判据**，自然无法验证实现的护栏。
// 修复：统一到 insphereSoS，并传入顶点索引（SoS 依赖索引打破平局）。
int localDelaunayViolations(const TetMesh3D& mesh)
{
    int bad = 0;
    for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
        const Tet4 tet = mesh.tets[t];
        for (int e = 0; e < 4; ++e) {
            const int nb = tet.n[e];
            if (nb < 0 || nb < t)
                continue;
            const Tet4 nt = mesh.tets[nb];
            int e2 = -1;
            for (int k = 0; k < 4; ++k) {
                bool shared = false;
                for (int m = 1; m <= 3; ++m)
                    if (nt.v[k] == tet.v[(e + m) % 4]) shared = true;
                if (!shared) e2 = nt.v[k];
            }
            if (e2 < 0 || e2 == tet.v[e])
                continue;
            if (insphereSoS(mesh.points[tet.v[0]], mesh.points[tet.v[1]],
                            mesh.points[tet.v[2]], mesh.points[tet.v[3]],
                            mesh.points[e2],
                            tet.v[0], tet.v[1], tet.v[2], tet.v[3], e2) > 0)
                ++bad;
        }
    }
    return bad;
}

void testIdempotent()
{
    std::cout << "\n== 1) Delaunay mesh: restoreDelaunay should do 0 flips ==" << std::endl;
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::vector<Point3D> pts;
    for (int i = 0; i < 120; ++i)
        pts.push_back({ uni(rng), uni(rng), uni(rng) });
    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok");
    if (!d.ok)
        return;
    const int before = localDelaunayViolations(mesh);
    const int flips = restoreDelaunay(mesh);
    const int after = localDelaunayViolations(mesh);
    check(before == 0, "input already local-Delaunay");
    check(flips == 0, "no flips needed on Delaunay mesh");
    check(after == 0, "still local-Delaunay");
    std::cout << "  before=" << before << " flips=" << flips << " after=" << after << std::endl;
}

void testFlip23OnCleanMesh()
{
    std::cout << "\n== 2) clean mesh: flip23 (2 tets -> 3 tets) ==" << std::endl;
    std::mt19937 rng(23);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::vector<Point3D> pts;
    for (int i = 0; i < 60; ++i)
        pts.push_back({ uni(rng), uni(rng), uni(rng) });
    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok");
    if (!d.ok)
        return;
    const double vol0 = mesh.totalVolume();
    const int tets0 = static_cast<int>(mesh.tets.size());

    int done = 0;
    for (int t = 0; t < static_cast<int>(mesh.tets.size()) && done == 0; ++t) {
        const Tet4 tet = mesh.tets[t];
        for (int e = 0; e < 4 && done == 0; ++e) {
            const int nb = tet.n[e];
            if (nb < 0)
                continue;
            std::string err;
            if (flip23(mesh, t, nb, &err)) {
                ++done;
                std::string verr;
                check(mesh.validate(&verr), "mesh valid after flip23");
                if (!verr.empty())
                    std::cerr << "  validate: " << verr << std::endl;
                check(static_cast<int>(mesh.tets.size()) == tets0 + 1, "tet count 2 -> 3");
                check(std::fabs(mesh.totalVolume() - vol0) < 1e-9, "volume preserved by flip23");
                std::cout << "  flipped tets " << t << "," << nb << " -> "
                          << mesh.tets.size() << " tets" << std::endl;
            }
        }
    }
    check(done == 1, "flip23 executed");
}

void testRestoreAfterRefine()
{
    std::cout << "\n== 3) restoreDelaunay after refine (limited) ==" << std::endl;
    std::vector<Point3D> pts;
    for (int i = 0; i < 8; ++i)
        pts.push_back({ (i & 1) ? 1.0 : 0.0, (i & 2) ? 1.0 : 0.0, (i & 4) ? 1.0 : 0.0 });
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> uni(0.15, 0.85);
    for (int i = 0; i < 25; ++i)
        pts.push_back({ uni(rng), uni(rng), uni(rng) });

    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    check(d.ok, "delaunay3D ok");
    if (!d.ok)
        return;
    const std::array<int, 3> facesRaw[] = {
        { 0, 2, 1 }, { 1, 2, 3 }, { 4, 5, 6 }, { 5, 7, 6 },
        { 0, 1, 4 }, { 1, 5, 4 }, { 2, 6, 3 }, { 3, 6, 7 },
        { 0, 4, 2 }, { 2, 4, 6 }, { 1, 3, 5 }, { 3, 7, 5 }
    };
    std::vector<std::array<int, 3>> faces;
    for (const auto& f : facesRaw)
        faces.push_back({ f[0] + 4, f[1] + 4, f[2] + 4 });

    std::string err;
    check(enforceSurface3D(mesh, faces, &err).ok, "enforceSurface3D ok");
    Ruppert3DParams p;
    p.maxEdgeLength = 0.5;
    Ruppert3DResult r = ruppertRefine3D(mesh, faces, p);
    check(r.converged, "ruppertRefine3D converged");

    const double vol0 = mesh.totalVolume();
    const int before = localDelaunayViolations(mesh);
    const int flips = restoreDelaunay(mesh);
    const int after = localDelaunayViolations(mesh);
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate");
    if (!verr.empty())
        std::cerr << "  validate: " << verr << std::endl;
    check(std::fabs(mesh.totalVolume() - vol0) < 1e-6, "volume preserved by flips");
    check(after <= before, "local Delaunay violations not increased");
    std::cout << "  violations before=" << before << " flips=" << flips
              << " after=" << after << std::endl;
    std::cout << "  tets=" << mesh.tets.size() << " volume=" << mesh.totalVolume() << std::endl;
}

// ============================================================
// 4) 退化输入（精确共球）下的「不恶化」契约
// ------------------------------------------------------------
// 背景（实测发现，2026-09-24）：本测试先前只覆盖**随机非退化**输入，
// 因此从未碰到退化路径。补一个精确共球的输入（立方体面上规则格点，
// 必然产生 5+ 顶点共球）把契约钉死。
//
// 理论依据：2D 的 Lawson 翻转保证每次 incircle 翻转都严格改善 Delaunay 性；
// **3D 没有这个性质** —— 3D 翻转图存在环，单次 flip23 可能让违规数增加。
// 实测（未加护栏时）：37 次几何合法的 flip23 使违规面 61 → 90，单调恶化。
// 因此 `restoreDelaunay` 必须带「不恶化」护栏（试翻 + 只接受严格改善），
// 否则它在退化网格上是有害的。
//
// 本测试断言（护栏生效后的**硬契约**）：
//   · 违规数**不增加**（before >= after）—— 这是本次修复的核心保证；
//   · 不崩、不产生非法网格（validate 通过）；
//   · 体积守恒。
// ============================================================
void testRestoreOnDegenerateInput()
{
    std::cout << "\n== 4) restoreDelaunay on degenerate (cospherical) input ==" << std::endl;
    std::vector<Point3D> pts;
    for (int i = 0; i < 8; ++i)
        pts.push_back({ (i & 1) ? 1.0 : 0.0, (i & 2) ? 1.0 : 0.0, (i & 4) ? 1.0 : 0.0 });
    // 立方体六个面上都加 3x3 规则格点 -> 大量精确共球退化。
    // 这个输入专门用来**压测「不恶化」护栏**：未加护栏时它会让
    // restoreDelaunay 做 37 次几何合法的 flip23、违规面 61 → 90（单调恶化）；
    // 加护栏后只保留真正改善的那 1 次，违规面 61 → 60。
    for (int a = 1; a <= 3; ++a)
        for (int b = 1; b <= 3; ++b) {
            pts.push_back({ a / 4.0, b / 4.0, 0.0 });
            pts.push_back({ a / 4.0, b / 4.0, 1.0 });
            pts.push_back({ a / 4.0, 0.0, b / 4.0 });
            pts.push_back({ a / 4.0, 1.0, b / 4.0 });
            pts.push_back({ 0.0, a / 4.0, b / 4.0 });
            pts.push_back({ 1.0, a / 4.0, b / 4.0 });
        }

    TetMesh3D mesh;
    Delaunay3DResult d = delaunay3D(mesh, pts);
    if (!d.ok) {
        // 合法结局：输入本身超出当前能力（论文 §9 所述）。必须**明确报错**。
        check(!d.error.empty(), "degenerate input rejected with a diagnostic error");
        std::cout << "  delaunay3D refused: " << d.error << std::endl;
        return;
    }
    check(true, "delaunay3D accepted degenerate input");

    const double vol0 = mesh.totalVolume();
    const int before = localDelaunayViolations(mesh);
    const int flips = restoreDelaunay(mesh);
    const int after = localDelaunayViolations(mesh);

    // —— 硬契约 ——
    std::string verr;
    check(mesh.validate(&verr), "degenerate: mesh.validate after restoreDelaunay");
    if (!verr.empty())
        std::cerr << "  validate: " << verr << std::endl;
    check(std::fabs(mesh.totalVolume() - vol0) < 1e-6,
          "degenerate: volume conserved by flips");
    check(!mesh.tets.empty(), "degenerate: mesh not emptied");
    check(after <= before,
          "degenerate: local Delaunay violations NOT increased (no-degradation guard)");
    std::cout << "  violations before=" << before << " flips=" << flips
              << " after=" << after << std::endl;
}

} // namespace

int main()
{
    std::cout << "===== 3D flips self-test (M3.5) =====" << std::endl;

    testIdempotent();
    testFlip23OnCleanMesh();
    testRestoreAfterRefine();
    testRestoreOnDegenerateInput();

    std::cout << "\n===== result: " << (g_failures == 0 ? "ALL PASS" : "FAILURES")
              << " (failures=" << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
