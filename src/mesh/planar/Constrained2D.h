#pragma once
// ============================================================
// Constrained2D —— Lawson 翻转恢复约束边（M2 的 2D 简化版）
// 职责：把一组约束边（如多边形边界、内部特征线）恢复进既有 Delaunay 三角化，
// 得到保留约束的三角化；调用方随后对非约束边做 Delaunay 合法化。
// 思路：对每条尚未成为网格边的约束边 (a,b)，反复翻转"穿越 ab 的网格边"所在
// 四边形，直到 (a,b) 成为网格边。这就是 3D 边界恢复（Si Hang 方法）的 2D 雏形。
// 非凸四边形推迟处理，已恢复约束锁定；交叉约束及无法恢复的退化输入明确报错。
// ============================================================

#include <string>
#include <utility>
#include <vector>

#include "mesh/data/Mesh2D.h"

namespace mesher {
struct MeshEditDelta;

/// 恢复全部约束边。约束边为顶点索引对（方向任意，可重复）。
/// 成功后每条约束由已有节点间的连续子边表示；失败时原网格保持不变。
bool enforceConstraints(Mesh2D& mesh,
                        const std::vector<std::pair<int, int>>& constraintEdges,
                        std::string* error,MeshObservation observation = {},MeshEditDelta* delta=nullptr);

/// 按段上已有共线节点展开约束；不新增点、不移动坐标。
bool expandConstraintEdges(const Mesh2D& mesh,
                           const std::vector<std::pair<int,int>>& edges,
                           std::vector<std::pair<int,int>>& segments,
                           std::string* error,MeshObservation observation = {});

/// 从凸包外部传播奇偶域标记；仅 boundaryEdges 翻转域内/域外。
/// 内部特征约束不传入 boundaryEdges。非闭合/矛盾边界失败，输出保持不变。
bool classifyConstrainedDomain(const Mesh2D& mesh,
                               const std::vector<std::pair<int,int>>& boundaryEdges,
                               std::vector<bool>& inDomain,std::string* error,
                               MeshObservation observation = {});

} // namespace mesher
