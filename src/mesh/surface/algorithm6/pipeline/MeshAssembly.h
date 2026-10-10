#pragma once
/** @file MeshAssembly.h
 * @brief 按物理身份装配独立面结果、CAD标签和逐角UV；不生成或优化网格。
 * @date 2026-10-08
 * @modified 2026-10-09
 */
#include "mesh/surface/algorithm6/data/FaceMeshTypes.h"
#include "mesh/surface/algorithm6/boundary/BoundaryRegistry.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"
namespace mesher::gmshReference {
struct AssemblyFace {
    const FaceMesh* mesh = nullptr;
    int bodyId = -1;
    std::function<Point3D(Point2D)> orientedNormal;
};
/** 普通/周期面装配；共享0D/1D按(body,physicalId)，2D按(body,face,id)识别。
 * 周期每角UV来自折叠前分支及平滑后内部参数；普通面使用节点参数视图。
 * 检查重复片、退化、面内边关联及身份合并后的整模型边关联/方向，
 * 不删坏片、不按坐标焊接。结果CAD实体标签为零基；输入实体身份为一基。
 * 仅成功原子替换output。可选failedFace返回一基CAD面身份，未知/成功为-1。
 * 回调持有者须长于本次串行调用。
 */
OperationStatus assembleSurfaceMesh(const BoundaryMesh&,const std::vector<AssemblyFace>&,
    SurfaceMeshResult& output,const RunControl& = {},int* failedFace = nullptr);
}
