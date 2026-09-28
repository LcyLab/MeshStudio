#pragma once
// ============================================================
// SurfaceMesh3D —— 表面三角网格输入准备（M5）
// 职责：把"三角形汤"（STL 等来源，顶点重复、无拓扑）整理成网格生成器
// 可用的表面网格：顶点焊接（容差内合并）+ 水密性检查 + 闭合体积校验。
// 这是自研四面体引擎的输入前置：边界必须闭合且拓扑一致，否则
// 边界恢复无法建立约束。
// 纯 C++17，不依赖 VTK/OCC（文件读取由调用方负责）。
// ============================================================

#include <array>
#include <string>
#include <vector>

#include "MesherTypes.h"

namespace mesher {

struct SurfaceMesh3D
{
    std::vector<Point3D> vertices;
    std::vector<std::array<int, 3>> triangles;
};

struct WeldResult
{
    SurfaceMesh3D surface;
    int mergedVertices = 0;      ///< 被合并掉的重复顶点数
    int droppedTriangles = 0;    ///< 因退化（顶点重合）被剔除的三角形数
};

/// 顶点焊接：容差内的重复顶点合并为一个，重建三角形索引并剔除退化三角形。
///
/// 容差语义：**真实欧氏距离** —— 只有当候选点与代表点的距离 <= tolerance
/// 时才合并。实现用「floor(v/tol) 的 64 位格子坐标」建哈希，并扫描 3×3×3
/// 邻域，因此跨格边界的近似重复点（B-Rep 逐面离散的共享棱顶点就是这种）
/// 也能正确合并；负坐标与任意坐标量级均正确。
WeldResult weldSurfaceMesh(const std::vector<Point3D>& vertices,
                           const std::vector<std::array<int, 3>>& triangles,
                           double tolerance);

/// 水密性检查：每条无向边恰好被 2 个三角形共享（闭合流形）。
/// 失败时把原因写入 error（含问题边数量）。
bool isWatertight(const SurfaceMesh3D& surface, std::string* error);

/// 闭合表面围成的体积（散度定理）。符号反映整体法向朝向；
/// 与 isWatertight 一起用于校验输入几何。
double enclosedVolume(const SurfaceMesh3D& surface);

} // namespace mesher
