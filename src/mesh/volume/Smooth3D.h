#pragma once
// ============================================================
// Smooth3D —— 顶点平滑（M4 质量后处理）
// 职责：Laplacian 平滑内部顶点，改善网格质量（提升最小二面角、减少 sliver、
// 降低半径边比）。边界顶点（约束面上的点）固定不动，保证几何不被破坏。
// 每次移动检查正体积、无新退化，局部最小内二面角/最坏半径边比不得恶化；否则回退。
// 平滑不改变拓扑（只动坐标），因此 tets 数、体积（边界固定）保持不变。
// 平滑消除近退化单元后，restoreDelaunay（Flips3D）才能充分生效。
// ============================================================

#include <array>
#include <string>
#include <vector>

#include "Quality3D.h"
#include "mesh/data/TetMesh3D.h"

namespace mesher {

struct Smooth3DParams
{
    int iterations = 10;         ///< 平滑迭代次数
    double relaxation = 0.5;     ///< 松弛因子（0.5 常用，1.0 = 直接移到质心）
    bool preserveBoundary = true;///< 固定约束面上的顶点（默认开）
};

struct Smooth3DResult
{
    bool ok = false;
    std::string error;
    int movedVertices = 0;           ///< 累计成功移动次数
    int rejectedMoves = 0;           ///< 因防翻转或质量护栏而回退的次数
    int qualityRejectedMoves = 0;    ///< 其中因质量护栏回退的次数
    MeshQualitySummary before;
    MeshQualitySummary after;
};

/// 对网格做 Laplacian 平滑。surfaceFaces 提供边界顶点集合（其顶点不动）。
Smooth3DResult smoothMesh3D(TetMesh3D& mesh,
                            const std::vector<std::array<int, 3>>& surfaceFaces,
                            const Smooth3DParams& params);

/// 网格平均边长（质量统计的退化判定尺度）
double meanEdgeLength3D(const TetMesh3D& mesh);

} // namespace mesher
