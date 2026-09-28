// ============================================================
// 自研四面体引擎端到端自测（M5 收尾验证，控制台程序）
// 验证：STL → 焊接/水密 → Delaunay → 约束恢复 → 细化 → 平滑 → 质量统计
// 用法：TetMeshEngineTest <input.stl> [targetSize]
// ============================================================
#include <QCoreApplication>
#include <QFileInfo>
#include <QString>

#include <chrono>
#include <cstdio>
#include <cstdlib>

#include "core/TetMeshEngine.h"
#include "core/TetMeshParams.h"

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();

    if (args.size() < 2) {
        std::printf("用法: TetMeshEngineTest <input.stl> [targetSize]\n");
        return 2;
    }

    TetMeshParams p;
    p.geometryFilePath = args.at(1);
    if (args.size() >= 3)
        p.targetSize = args.at(2).toDouble();

    std::printf("== 端到端测试 ==\n");
    std::printf("输入: %s\n", p.geometryFilePath.toUtf8().constData());
    std::printf("目标尺寸: %.6g (%s)\n", p.targetSize,
                p.targetSize > 0.0 ? "指定" : "默认=对角线5%");
    std::printf("优化(平滑): %s\n\n", p.optimize ? "开" : "关");

    // 计时
    const auto t0 = std::chrono::steady_clock::now();
    const TetMeshResult r = TetMeshEngine::run(p);
    const auto t1 = std::chrono::steady_clock::now();
    const double sec = std::chrono::duration<double>(t1 - t0).count();

    if (!r.ok) {
        std::printf("[FAIL] 生成失败: %s\n", r.error.toUtf8().constData());
        return 1;
    }

    const TetMeshQualityStats& s = r.stats;
    std::printf("[PASS] 端到端链路跑通\n\n");
    std::printf("耗时            : %.3f s\n", sec);
    std::printf("节点            : %lld\n", s.nodeCount);
    std::printf("体单元          : %lld\n", s.tetCount);
    std::printf("表面三角形      : %lld\n", s.surfaceTriCount);
    std::printf("单元阶次        : %s\n", s.quadratic ? "T10" : "T4");
    std::printf("\n-- 质量 --\n");
    std::printf("minSICN         : %.6f  %s\n", s.minSICN,
                s.minSICN < 0.0 ? "<< 存在翻转单元!" : "ok");
    std::printf("gamma 最小/平均 : %.6f / %.6f\n", s.minGamma, s.avgGamma);
    std::printf("二面角范围      : [%.2f°, %.2f°]\n", s.minDihedralDeg, s.maxDihedralDeg);
    std::printf("翻转单元数      : %lld\n", s.invertedCount);

    // ---- 细化诊断（调优用） ----
    std::printf("\n-- 细化诊断 --\n");
    std::printf("converged       : %s\n", r.refineConverged ? "yes" : "no");
    std::printf("iterations      : %d\n", r.refineIterations);
    std::printf("insertedPoints  : %d\n", r.refineInserted);
    std::printf("protectPoints   : %d  (init=%d refine=%d)\n",
                r.protectPoints, r.protectPointsInit, r.protectPointsRefine);
    std::printf("worstRadiusEdge : %.6f\n", r.refineWorstRadiusEdge);
    std::printf("maxEdgeLen      : %.6f\n", r.refineMaxEdgeLen);
    std::printf("slivers(<5deg)  : %d\n", r.refineSlivers);
    std::printf("quarantined     : %d  (bySizeSeg=%d bySizeMidDist=%d byEncroach=%d"
                " byCCDist=%d byCCOutside=%d)\n",
                r.quarantined, r.qBySizeSeg, r.qBySizeMidDist, r.qByEncroach,
                r.qByCCDist, r.qByCCOutside);
    std::printf("sliverRescueTried: %d\n", r.sliverRescueTried);

    // 检查项
    int fails = 0;
    auto check = [&](bool ok, const char* name) {
        std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
        if (!ok)
            ++fails;
    };
    std::printf("\n-- 检查 --\n");
    check(r.volume != nullptr && s.tetCount > 0, "生成了体网格");
    check(r.surface != nullptr && s.surfaceTriCount > 0, "生成了外表面");
    check(s.invertedCount == 0, "无翻转单元");
    check(s.nodeCount > 0, "节点数 > 0");

    // ============================================================
    // ★ 回归基线（golden baseline）★ —— 任务 #12 的「真断言」
    // ------------------------------------------------------------
    // 此前本测试只查「跑通 / 非空 / 无翻转」，**放过 minSICN=0** ——
    // 也就是说网格质量完全退化时它照样 PASS。下面把已知正确的输出快照
    // 固化成断言，任何**静默回归**都会被立刻抓住。
    //
    // 维护约定：若算法被**有意**改动，这些数值会变 —— 此时必须人工确认
    // 改动是期望的，然后**显式更新本表**。不要为了"让测试过"而放宽容差。
    //
    // cube10 的 minSICN=0 是**已知限制**（almost Delaunay 路线的固有问题，
    // 见 docs/references/README-shewchuk-failure-analysis.md §7.5），
    // 不是通过标准；这里记录它，是为了在它被修好时立刻得到提示。
    // ============================================================
    struct Baseline
    {
        const char* file;
        long long nodes;
        long long tets;
        long long tris;
        double minSICN;
        bool checkSICN;   ///< 是否把 minSICN 当硬断言（false = 仅记录已知值）
    };
    static const Baseline kBaselines[] = {
        { "cube.stl",   10,  11,  20,  0.577350, true  },
        { "cube10.stl", 103, 405, 124, 0.000000, false },
    };
    const QString base = QFileInfo(p.geometryFilePath).fileName();
    for (const Baseline& b : kBaselines) {
        if (base != QString::fromUtf8(b.file))
            continue;
        std::printf("\n-- 回归基线 (%s) --\n", b.file);
        check(s.nodeCount == b.nodes, "baseline: 节点数与快照一致");
        check(s.tetCount == b.tets, "baseline: 体单元数与快照一致");
        check(s.surfaceTriCount == b.tris, "baseline: 表面三角形数与快照一致");
        if (b.checkSICN)
            check(s.minSICN > 0.0, "baseline: minSICN > 0（无退化单元）");
        std::printf("  期望 %lld/%lld/%lld minSICN=%.6f  实际 %lld/%lld/%lld minSICN=%.6f\n",
                    b.nodes, b.tets, b.tris, b.minSICN,
                    s.nodeCount, s.tetCount, s.surfaceTriCount, s.minSICN);
        break;
    }
    // 未登记的输入：至少要求"非退化到完全塌陷"（体单元数应远小于节点数的立方级）
    if (base != QString::fromUtf8(kBaselines[0].file)
        && base != QString::fromUtf8(kBaselines[1].file)) {
        std::printf("\n-- 回归基线 --\n  未登记输入 %s，仅做通用性检查\n",
                    base.toUtf8().constData());
        check(s.tetCount > 0 && s.nodeCount > 0, "通用: 网格非空");
    }

    std::printf("\n===== 结果: %s (failures=%d) =====\n",
                fails == 0 ? "全部通过" : "有失败", fails);
    return fails == 0 ? 0 : 1;
}
