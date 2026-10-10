#pragma once
/** @file WalkTriangulation.h
 * @brief 普通初始入口的纯 UV 半边剖分；保留原版池编号及可变面首边。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/chart/FaceChart.h"
namespace mesher::gmshReference {
struct WalkMeshVertex {
    Point2D uv;
    std::uint64_t physicalId = 0; // 外围点为 0；转换 BDS 时才赋 -1..-4。
    int incidentHalfEdge = -1;
};
struct WalkHalfEdge {
    int origin = -1, face = -1, previous = -1, next = -1, opposite = -1;
    int curveEntity = -1;
};
struct WalkFace {
    int halfEdge = -1, domain = -1;
};
struct WalkTriangulation {
    std::vector<WalkMeshVertex> vertices;
    std::vector<WalkHalfEdge> halfEdges;
    std::vector<WalkFace> faces;
};
/** 对应 initialize_rectangle -> Walk -> split_triangle -> 局部 Delaunay 翻边。
 * lower/upper 是入口已扩大的盒；点按给定次序插入，不排序、不扰动。
 * 保留池顺序、原始有向 incircle、共圆不翻、LIFO/已处理边规则。
 * 不恢复约束/域或转换 BDS；失败原子保留 output。
 */
OperationStatus buildWalkTriangulation(Point2D lower, Point2D upper,
    const std::vector<InitialChartVertex>& points, WalkTriangulation& output, const RunControl& control = {});
}
