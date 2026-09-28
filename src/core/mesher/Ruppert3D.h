#pragma once
// ============================================================
// Ruppert3D —— Delaunay 四面体细化主循环（M3）
// 职责：在 CDT（约束 Delaunay 四面体化）上迭代插入 Steiner 点，直到
//   · 所有约束边长度 ≤ maxEdgeLength（大小函数）
//   · 所有四面体半径边比 R/Lmin ≤ radiusEdgeRatio（质量）
// 策略（Shewchuk 2002 Delaunay refinement in 3D）：
//   1. 超长约束边 → 分裂（优先于一切）
//   2. 外接球心 encroach 某约束子段（在子段直径球内）→ 分裂该子段
//   3. 否则插入外接球心（带约束面阻挡，不破坏边界）
// 3D 注：Ruppert 3D 不保证消除 sliver（radius-edge 达标但极薄），
// 由 M4 的 sliver 后处理处理。v1 先保证 radius-edge 与边长上界。
// ============================================================

#include <array>
#include <string>
#include <vector>

#include "TetMesh3D.h"

namespace mesher {

struct Ruppert3DParams
{
    double maxEdgeLength = 1.0;         ///< 目标最大边长
    double radiusEdgeRatio = 2.0;       ///< 3D 常用 ρ=2.0（Ruppert 3D 保证弱于 2D）
    int maxIterations = 300000;         ///< 保护上限
    bool refineBoundary = false;        ///< v1 关闭：边界细化（约束边分裂）留待 M3.5
    /// 约束边恢复的阶段 1（保护球，Shewchuk p.8）。默认关闭 = 沿用原行为。
    /// 打开后每次调用 recoverConstraintEdges 都会先插保护球点。
    bool edgeProtection = false;
};

struct Ruppert3DResult
{
    bool converged = false;
    std::string error;
    int insertedPoints = 0;
    int iterations = 0;
    int tetCount = 0;
    double maxEdgeLen = 0.0;          ///< 全网格（含约束边）最大边长
    double worstRadiusEdge = 0.0;     ///< 最坏 R/Lmin（>ρ 即未收敛）
    double minDihedralDeg = 180.0;    ///< 全网格最小二面角（sliver 指标）
    int slivers = 0;                  ///< 二面角 < 5° 的四面体数
    int sliverRescueTried = 0;        ///< 薄片抢救（翻转）尝试次数（诊断）
    int sliverRescueOk = 0;           ///< 薄片抢救成功次数（诊断）
    int quarantined = 0;              ///< 被隔离跳过的四面体数（诊断）
    int qBySizeSeg = 0;               ///< 隔离点：超长但边界不细化（诊断）
    int qBySizeMidDist = 0;           ///< 隔离点：中点过近（诊断）
    int qBySizeMidInsert = 0;         ///< 隔离点：中点插入失败（诊断）
    int qByEncroach = 0;              ///< 隔离点：encroach 且边界不细化（诊断）
    int qByCCDist = 0;                ///< 隔离点：球心过近（诊断）
    int qByCCOutside = 0;             ///< 隔离点：球心在域外（诊断）
    int protectPoints = 0;            ///< 阶段 1 保护球插入点数（诊断，累计）
};

/// 细化既有 CDT。surfaceFaces 为输入输出参数：
/// 超长约束边会被分裂，返回时反映最终边界子段。
Ruppert3DResult ruppertRefine3D(TetMesh3D& mesh,
                                std::vector<std::array<int, 3>>& surfaceFaces,
                                const Ruppert3DParams& params);

} // namespace mesher
