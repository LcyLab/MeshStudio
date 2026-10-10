#pragma once
/** @file OccMeshVertexParameters.h
 * @brief 原始OCC节点参数源；数据/选择算法由MeshVertexParameters负责。
 * @date 2026-10-08
 */
#include "OccCadGeometry.h"
#include "mesh/surface/algorithm6/geometry/MeshVertexParameters.h"
#include "mesh/surface/algorithm6/postprocess/SurfaceSmoothing.h"
/** 固定默认OCC直接pcurve/投影路径，face为零基，返回源实体ID为一基。
 * geometry须长于源；每源串行独占投影器。通用反求失败分支尚未实现时明确失败。
 */
mesher::gmshReference::MeshVertexParameterSource makeOccMeshVertexParameterSource(const OccCadGeometry& geometry,int face);
/** 为已转换的一阶FaceMesh节点提供原平滑几何源；1D/2D节点必须持有原存储参数。
 * 不以NodeSizeState的UV代替原参数。正常seam单点false仍返回UV，匹配原面积调用。
 * geometry须长于返回源；只创建串行几何回调，不修改网格或释放周期临时节点。
 */
mesher::gmshReference::SurfaceSmoothingSource makeOccSurfaceSmoothingSource(const OccCadGeometry& geometry,int face);
