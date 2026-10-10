#pragma once
/** @file HalfEdgeTopology.h
 * @brief 初始半边池的连接更新；不计算翻边资格或几何公式。
 * @date 2026-10-07
 * @modified 2026-10-08
 */
#include "mesh/surface/algorithm6/initial/WalkTriangulation.h"
#include <array>
namespace mesher::gmshReference {
/** 验证稳定三角池的索引/首边/入射边/循环/对边和有限UV，不改变任何标签。
 * 取消与节点预算显式返回；几何形状/域/约束正确性由各阶段另验。
 */
OperationStatus validateHalfEdgeTopology(const WalkTriangulation& mesh, const RunControl& control = {});
/** 对应 PolyMesh::createFace；索引须有效且来自同一池。
 * 更新三角循环、面首边和点入射边；保留 opposite/曲线/域标签。
 */
void rewriteHalfEdgeTriangle(WalkTriangulation& mesh, int face,
    const std::array<int,3>& vertices, const std::array<int,3>& edges);
/** 对应 PolyMesh::swap_edge，保留池序和原生两面的写入顺序。
 * 索引须有效，几何资格由调用模块判断；外围无对边时返回 false。
 */
bool swapHalfEdgeDiagonal(WalkTriangulation& mesh, int edge);
}
