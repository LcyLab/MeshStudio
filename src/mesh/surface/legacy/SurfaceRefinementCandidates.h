#pragma once
/** @file SurfaceRefinementCandidates.h
 * @brief 有界的局部物理度量候选；只构造位置，不改变拓扑。
 * @date 2026-10-02
 * @modified 2026-10-07
 */
#include "SurfaceChart.h"
namespace mesher {
// 返回靠近最短边的偏心点及度量外心；minimumNewEdge 约束偏心点到基边端点的度量长度。
// 既有短边不受此下限；候选可能在源片外，调用方仍检查裁剪域/约束与真实三维新增边。
std::vector<Point2D> surfaceRefinementCandidates(const std::array<Point2D,3>& uv,
                                                const SurfaceMetric& metric,double target,double minimumNewEdge=0);
struct SurfaceEdgeSplitPoint {
    Point2D uv{};
    Point3D point{};
    double fraction=0;
    int iterations=0;
};
// 沿原UV线段求到两个CAD端点等距的分点，保持拓扑边分支；不是全局投影或弧长中点。
// 最多48次二分。失败不发布结果，CAD异常原样传播，调用方仍验收子片方向和几何限制。
bool surfaceBalancedEdgePoint(const GeometrySource& geometry,int face,Point2D a,Point2D b,
                              SurfaceEdgeSplitPoint& output,std::string* error);
}
