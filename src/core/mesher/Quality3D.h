#pragma once
// ============================================================
// Quality3D —— 四面体质量指标（M4）
// 职责：统一的质量度量与网格统计，供细化（Ruppert3D）、平滑（Smooth3D）
// 与最终验收共用：
//   · 二面角（min/max）—— sliver 的核心指标（radius-edge 无法识别 sliver）
//   · 半径边比 R/Lmin —— 质量主指标（与 Ruppert 判据一致）
//   · sliver / 退化单元计数
// 度量依据见 docs/tet-mesh-design.md §6。
// ============================================================

#include "TetMesh3D.h"

namespace mesher {

struct TetQuality
{
    double minDihedralDeg = 180.0;   ///< 最小二面角（内角）
    double radiusEdge = 0.0;         ///< 外接球半径 / 最短边
    double volume = 0.0;             ///< 体积（退化时接近 0）
    bool degenerate = false;         ///< 近共面（体积相对阈值过小）
};

/// 单个四面体质量（共面/退化时 degenerate = true）
TetQuality tetQuality(const TetMesh3D& mesh, const Tet4& t, double edgeScale);

struct MeshQualitySummary
{
    int tetCount = 0;
    double minDihedralDeg = 180.0;   ///< 全网格最小二面角
    double maxDihedralDeg = 0.0;
    double worstRadiusEdge = 0.0;    ///< 最坏 R/Lmin
    double minVolume = 0.0;
    int sliverCount = 0;             ///< 二面角 < sliverAngleDeg 的四面体
    int degenerateCount = 0;         ///< 近共面四面体
    double sliverAngleDeg = 5.0;
};

/// 全网格质量统计。edgeScale 用于退化判定（建议取网格平均边长或 maxEdge）。
MeshQualitySummary summarizeQuality(const TetMesh3D& mesh, double edgeScale,
                                    double sliverAngleDeg = 5.0);

} // namespace mesher
