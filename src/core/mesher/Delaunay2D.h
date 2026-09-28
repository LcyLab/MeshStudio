#pragma once
// ============================================================
// Delaunay2D —— Bowyer-Watson 增量 Delaunay 三角化（M1 的 2D 简化版）
// 职责：
//   1. delaunay2D()：对点集生成 Delaunay 三角化（super 三角形包裹 + 逐个插入 + 清理）
//   2. insertPoint()：单点插入既有网格，供细化阶段（Ruppert2D）复用
// 数值处理：内部先把坐标平移缩放归一化（super 尺度可控、数值稳定），
// 完成后还原为原始坐标。三个 super 顶点会残留在 points 中但无三角形引用。
// ============================================================

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "Mesh2D.h"

namespace mesher {

struct Delaunay2DResult
{
    bool ok = false;
    std::string error;
    std::vector<int> triIds;     ///< 有效三角形 id（super 相关已剔除）
    long long insertSteps = 0;   ///< 插入循环总步数（性能观测）
};

/// 对 point 集合生成 Delaunay 三角化。点被追加进 mesh.points。
Delaunay2DResult delaunay2D(Mesh2D& mesh, std::vector<Point2D> points);

/// 单点插入既有网格（细化阶段复用，坐标用原始坐标系）。
/// blockedEdges：约束边（顶点索引对，方向任意），cavity 收集时不得跨越。
/// startTri 为 walk 定位起点；outNewVertex 输出新顶点索引（失败时为 -1）。
bool insertPoint(Mesh2D& mesh, const Point2D& p,
                 const std::vector<std::pair<int, int>>& blockedEdges,
                 int startTri, int* outNewVertex, int* outTri,
                 std::string* error);

} // namespace mesher
