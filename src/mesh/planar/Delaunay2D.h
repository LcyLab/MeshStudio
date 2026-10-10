#pragma once
// ============================================================
// Delaunay2D —— Bowyer-Watson 增量 Delaunay 三角化（M1 的 2D 简化版）
// 职责：
//   1. delaunay2D()：增量三角化，检查真实凸包覆盖；有限 super 点影响窄域时从真实凸包重建
//   2. insertPoint()：单点插入既有网格，供细化阶段（Ruppert2D）复用
// 默认归一化并保留原型扰动；preserveCoordinates 路径保持输入 UV，
// 可用正定线性度量做圆判定，方向判定仍使用 UV。三个 super 点无三角形引用。
// ============================================================

#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include <functional>

#include "mesh/data/Mesh2D.h"

namespace mesher {
struct MeshEditDelta;
class SurfaceDomainState;

// L=[xx xy;0 yy]，xx、yy 须为有限正数；仅影响圆判定，保留拓扑坐标。
struct LinearMetric2D { double xx=1,xy=0,yy=1; };

struct Delaunay2DResult
{
    bool ok = false;
    std::string error;
    std::vector<int> triIds;     ///< 有效三角形 id（super 相关已剔除）
    long long insertSteps = 0;   ///< 插入循环总步数（性能观测）
};

/// 对点集生成 Delaunay 三角化；preserveCoordinates=true 时不归一化或移动输入点。
/// metric 非空时须使用 preserveCoordinates；乘积不允许上下溢。
Delaunay2DResult delaunay2D(Mesh2D& mesh, std::vector<Point2D> points,
                           bool preserveCoordinates = false,const LinearMetric2D* metric=nullptr,
                           MeshObservation observation = {});

/// 单点插入既有网格（细化阶段复用，坐标用原始坐标系）。
/// blockedEdges：约束边（顶点索引对，方向任意），cavity 收集时不得跨越。
/// startTri 为 walk 定位起点；outNewVertex 输出新顶点索引（失败时为 -1）。
/// 重复点和约束段上的点明确拒绝；失败保持原网格及坐标不变，输出索引为 -1。
/// delta 仅成功时公布；domain 非空时要求空腔源片同域，调用方提交后同步标签。
/// accept 可在提交前验证候选空腔；返回 false 或抛异常完整回滚，不发布新点/编辑报告。
/// circleTest 可覆盖局部空腔的圆判定，返回 -1/0/1（圆外/共圆/圆内）。
/// 判定须在同一局部坐标中一致；非法返回值拒绝，异常传播，均完整回滚。
/// 原 UV 方向、空腔可见性、裁剪域和受保护约束仍由插入过程验证。
bool insertPoint(Mesh2D& mesh, const Point2D& p,
                 const std::vector<std::pair<int, int>>& blockedEdges,
                 int startTri, int* outNewVertex, int* outTri,
                 std::string* error,const LinearMetric2D* metric=nullptr,
                 MeshObservation observation = {},MeshEditDelta* delta=nullptr,
                 const SurfaceDomainState* domain=nullptr,
                 const std::function<bool(const Mesh2D&,int,const MeshEditDelta&)>& accept={},
                 const std::function<int(const Mesh2D&,const Tri2D&,Point2D)>& circleTest={});

} // namespace mesher
