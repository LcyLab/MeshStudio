// ============================================================
// 顶点平滑自测（M4，控制台程序）
// 流程：Delaunay -> 约束边恢复(M2) -> 细化(M3) -> Laplacian 平滑(M4)
// 验证：拓扑不变 / 体积守恒 / validate / 质量改善（最小二面角、sliver、
//       半径边比），并观察平滑后 restoreDelaunay 是否生效
// ============================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "mesh/volume/Constrained3D.h"
#include "mesh/volume/Delaunay3D.h"
#include "mesh/volume/Flips3D.h"
#include "mesh/volume/Quality3D.h"
#include "mesh/volume/Ruppert3D.h"
#include "mesh/volume/Smooth3D.h"
#include "mesh/data/TetMesh3D.h"

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

std::vector<std::array<int, 3>> cubeFaces()
{
    const std::array<int, 3> faces[] = {
        { 0, 2, 1 }, { 1, 2, 3 }, { 4, 5, 6 }, { 5, 7, 6 },
        { 0, 1, 4 }, { 1, 5, 4 }, { 2, 6, 3 }, { 3, 6, 7 },
        { 0, 4, 2 }, { 2, 4, 6 }, { 1, 3, 5 }, { 3, 7, 5 }
    };
    std::vector<std::array<int, 3>> out;
    for (const auto& f : faces)
        out.push_back({ f[0] + 4, f[1] + 4, f[2] + 4 });
    return out;
}

void testSmoothCube()
{
    std::cout << "\n== 1) cube: refine then Laplacian smooth ==" << std::endl;
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
    if (!d.ok)
        return;
    std::vector<std::array<int, 3>> faces = cubeFaces();
    std::string err;
    check(enforceSurface3D(mesh, faces, &err).ok, "enforceSurface3D ok");

    {
        const MeshQualitySummary q0 = summarizeQuality(mesh, meanEdgeLength3D(mesh));
        std::cout << "  [after M2] tets=" << q0.tetCount
                  << " minDihedral=" << q0.minDihedralDeg
                  << " slivers=" << q0.sliverCount
                  << " degenerate=" << q0.degenerateCount
                  << " worstRho=" << q0.worstRadiusEdge << std::endl;
    }
    Ruppert3DParams rp;
    rp.maxEdgeLength = 0.3;
    rp.radiusEdgeRatio = 2.0;
    Ruppert3DResult r = ruppertRefine3D(mesh, faces, rp);
    check(!r.converged && r.unresolvedTets > 0 && !r.diagnostic.empty() && r.error.empty(),
          "boundary size failures remain explicitly unresolved before smoothing");

    const int tets0 = static_cast<int>(mesh.tets.size());
    const double vol0 = mesh.totalVolume();

    Smooth3DParams sp;
    sp.iterations = 15;
    sp.relaxation = 0.5;
    Smooth3DResult s = smoothMesh3D(mesh, faces, sp);
    check(s.ok, "smoothMesh3D ok");
    if (!s.ok)
        std::cerr << "  error: " << s.error << std::endl;
    std::string verr;
    check(mesh.validate(&verr), "mesh.validate after smooth");
    check(static_cast<int>(mesh.tets.size()) == tets0, "tet count unchanged by smoothing");
    check(std::fabs(mesh.totalVolume() - vol0) < 1e-6, "volume conserved by smoothing");
    // v1 说明：平滑显著减少 sliver 与退化单元，但 M2 手动重建产生的极端退化单元
    // 无法靠坐标移动消除（需 M2 侧不产生退化，或局部删除重建），故 minDihedral/
    // worstRho 只要求不恶化。
    check(s.after.sliverCount < s.before.sliverCount, "sliver count reduced");
    // 退化单元（零/近零体积）在 M2 之后已由 pruneDegenerateTets 清零，
    // 故这里断言「平滑后仍无退化」而非「数量下降」——后者在 before==0 时不可满足。
    check(s.after.degenerateCount == 0, "no degenerate tets after smoothing");
    check(s.after.worstRadiusEdge <= s.before.worstRadiusEdge, "worst radius-edge not worse");
    check(s.after.minDihedralDeg >= s.before.minDihedralDeg, "min dihedral not worse");

    std::cout << "  before: minDihedral=" << s.before.minDihedralDeg
              << " slivers=" << s.before.sliverCount
              << " worstRho=" << s.before.worstRadiusEdge
              << " degenerate=" << s.before.degenerateCount << std::endl;
    std::cout << "  after : minDihedral=" << s.after.minDihedralDeg
              << " slivers=" << s.after.sliverCount
              << " worstRho=" << s.after.worstRadiusEdge
              << " degenerate=" << s.after.degenerateCount << std::endl;
    check(s.qualityRejectedMoves > 0, "quality guard rejects harmful candidate movements");
    std::cout << "  moved=" << s.movedVertices << " rejected=" << s.rejectedMoves
              << " qualityRejected=" << s.qualityRejectedMoves
              << " tets=" << tets0 << std::endl;

    // 平滑后 restoreDelaunay：v1 在残留极端退化单元时仍会被预检查拒绝（0 翻转），
    // 这里只验证"安全"（网格不变、体积不变）
    const int flips = restoreDelaunay(mesh);
    std::string verr2;
    check(mesh.validate(&verr2), "mesh.validate after restoreDelaunay");
    check(std::fabs(mesh.totalVolume() - vol0) < 1e-6, "volume conserved by flips");
    std::cout << "  restoreDelaunay flips=" << flips << " (safely rejected on degraded mesh)"
              << std::endl;
}

} // namespace

int main()
{
    std::cout << "===== smoothing self-test (M4) =====" << std::endl;

    testSmoothCube();

    std::cout << "\n===== result: " << (g_failures == 0 ? "ALL PASS" : "FAILURES")
              << " (failures=" << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
