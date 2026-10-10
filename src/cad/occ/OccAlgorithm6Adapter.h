#pragma once
/** @file OccAlgorithm6Adapter.h
 * @brief BREP到独立算法6输入；OCC拓扑/求值，不生成三角形。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "OccMeshVertexParameters.h"
#include "mesh/surface/algorithm6/pipeline/SurfaceGenerator.h"
class OccAlgorithm6Adapter {
public:
    /// 持有本次运行的 OCC 几何对象，供算法回调使用。
    explicit OccAlgorithm6Adapter(const TopoDS_Shape& shape);
    /** 回调引用适配器拥有的geometry；适配器须长于输入及所有串行生成调用。
     * BREP中的INTERNAL边作为嵌入约束保留；不推断BREP没有保存的外部嵌入关系。
     */
    // 点尺寸以本次BREP的一基CAD顶点身份给定；未指定沿用原生无约束/模型尺度默认。
    mesher::gmshReference::Algorithm6ModelInput buildModelInput(const std::map<int,double>& vertexSizes = {}) const;
    /// 将零基 OCC 面号转换为单面输入；边界、实体身份和所有回调仍来自本次 BRep。
    /// 单独使用该输入时，调用方须提供同一模型的共享 BoundaryMesh。
    mesher::gmshReference::Algorithm6FaceInput buildFaceInput(int face) const;
    /// 返回适配器持有的 CAD 几何查询对象。
    const OccCadGeometry& geometry() const { return m_geometry; }
private:
    OccCadGeometry m_geometry;
};
