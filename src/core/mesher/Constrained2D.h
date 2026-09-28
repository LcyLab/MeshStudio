#pragma once
// ============================================================
// Constrained2D —— Lawson 翻转恢复约束边（M2 的 2D 简化版）
// 职责：把一组约束边（如多边形边界、内部特征线）恢复进既有 Delaunay 三角化，
// 得到约束 Delaunay 三角化（CDT）。
// 思路：对每条尚未成为网格边的约束边 (a,b)，反复翻转"穿越 ab 的网格边"所在
// 四边形，直到 (a,b) 成为网格边。这就是 3D 边界恢复（Si Hang 方法）的 2D 雏形。
// 局限（v1）：被翻转四边形必须是凸的（w、q 分居被翻边两侧），否则报错。
// ============================================================

#include <string>
#include <utility>
#include <vector>

#include "Mesh2D.h"

namespace mesher {

/// 恢复全部约束边。约束边为顶点索引对（方向任意，可重复）。
/// 成功后网格中每条约束边都存在（含反向）。
bool enforceConstraints(Mesh2D& mesh,
                        const std::vector<std::pair<int, int>>& constraintEdges,
                        std::string* error);

} // namespace mesher
