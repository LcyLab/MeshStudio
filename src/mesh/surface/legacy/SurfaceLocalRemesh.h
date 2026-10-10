#pragma once
/** @file SurfaceLocalRemesh.h
 * @brief 表面约束分边、内部一环重剖分与已验收图表恢复；原节点/UV和失败状态保留。
 * @date 2026-10-03
 * @modified 2026-10-05
 */
#include "SurfaceDomainState.h"
#include <array>
#include <functional>
namespace mesher {
// 从已验收的逐片 UV/全局节点重建图表；不再剖分或改变原片，保留周期分支。
// 失败保持全部输出；所有拓扑边界成为约束，由调用方提供正向 UV 三角形。
bool rebuildAcceptedSurfaceChart(const std::vector<std::array<int,3>>& triangles,
    const std::vector<std::array<Point2D,3>>& uv,Mesh2D& mesh,std::vector<int>& global,
    std::vector<std::pair<int,int>>& constraints,std::string* error,MeshObservation observation={});
/** 局部拆分一个显式边界段，保留所有原点及其索引。
 * CAD 曲线的中间 UV 可偏离旧直段，但所有入射子片必须保持正定向。
 * 成功更新约束与域；拒绝/异常恢复拓扑、点索引、约束及域，异常继续传播。
 * 不执行 Delaunay 合法化或三维几何验收；调用方通过 accept 和后续判据负责。
 */
bool trySplitSurfaceBoundaryEdge(Mesh2D& mesh,int a,int b,Point2D point,
    std::vector<std::pair<int,int>>& constraints,SurfaceDomainState& domain,int* inserted,
    MeshEditDelta* published,std::string* error,MeshObservation observation={},
    const std::function<bool(const Mesh2D&,int,const MeshEditDelta&)>& accept={});
// 三角形成本为有限正质量值；不允许的三维/尺寸/几何候选返回 infinity。
using SurfaceTriangleCost=std::function<double(std::array<int,3>)>;
/** 移除未受约束的内部点连接，最多 16 个邻点，在原 UV 多边形内求最小最差成本剖分。
 * 不删除/移动点槽，原参数索引保持；输出压缩再丢弃无引用三维节点。
 * 输入网格/域标签须有效；调用方通过成本限制质量/几何，并保护 CAD/周期节点。
 * 无完整内部圆盘、受约束、无有效剖分或验收拒绝时不发布修改。
 * 几何成本由调用方提供；事务覆盖三角形/版本/半边和域标签，异常原样传播。
 * 可提供当前版本的完整入射槽索引；拓扑提交后必须重建，不允许复用旧槽编号。
 */
bool tryRemeshSurfaceVertex(Mesh2D& mesh,int vertex,const std::vector<std::pair<int,int>>& constraints,
    SurfaceDomainState& domain,const SurfaceTriangleCost& cost,MeshEditDelta* published,std::string* error,
    MeshObservation observation={},const std::function<bool(const Mesh2D&,const MeshEditDelta&)>& accept={},
    const std::vector<int>* incidentSlots=nullptr);
}
