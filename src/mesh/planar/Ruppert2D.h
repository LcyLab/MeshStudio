#pragma once
// ============================================================
// Ruppert2D —— Delaunay 细化主循环（M3 的 2D 简化版）
// 职责：在约束 Delaunay 三角化（CDT）上迭代插入 Steiner 点，直到
//   · 所有约束边长度 ≤ maxEdgeLength（大小函数）
//   · 所有三角形半径边比 R/Lmin ≤ radiusEdgeRatio（质量）
// 策略（Ruppert 1995 / Shewchuk 1998）：
//   1. 超长约束边 → 分裂为两段（优先于一切）
//   2. 外接圆心落入某约束边"直径圆"（圆心距边 < 半长）→ 分裂该约束边，
//      而不是插入圆心（保证边界不被破坏、且细化收敛）
//   3. 否则插入外接圆心；若圆心距现有点过近 → 插入最短边中点
// 该主循环（坏单元 → 选择插入点 → 重插）将原样迁移到 3D 四面体细化。
// ============================================================

#include <string>
#include <utility>
#include <vector>

#include "mesh/data/Mesh2D.h"

namespace mesher {

struct RuppertParams
{
    double maxEdgeLength = 1.0;                       ///< 目标最大边长
    double radiusEdgeRatio = 1.4142135623730951;      ///< √2：理论保证最小角 ≥ ~20.7°
    int maxIterations = 200000;                       ///< 保护上限（理论保证收敛）
};

struct RuppertResult
{
    bool converged = false;
    std::string error;
    int insertedPoints = 0;      ///< 插入的 Steiner 点数
    int iterations = 0;          ///< 主循环实际迭代数
    int triCount = 0;
    double minAngleDeg = 0.0;    ///< 全网格最小内角
    double maxEdgeLen = 0.0;     ///< 全网格（含约束边）最大边长
};

/// 细化既有 CDT。constraints 为输入输出参数：
/// 超长约束边会被分裂，返回时反映最终边界段（每段都是网格边）。
RuppertResult ruppertRefine(Mesh2D& mesh,
                            std::vector<std::pair<int, int>>& constraints,
                            const RuppertParams& params);

} // namespace mesher
