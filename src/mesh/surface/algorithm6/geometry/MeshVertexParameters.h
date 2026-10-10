#pragma once
/** @file MeshVertexParameters.h
 * @brief 原始节点单点/多候选/邻边参数；与前沿槽、扰动UV和周期副本身份分开。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/FaceMeshTypes.h"
namespace mesher::gmshReference {
enum class ParameterEntityKind { Cad, DiscreteCurve, BoundaryLayerCurve, GmshVertex };
struct MeshVertexParameterInput {
    FaceVertex vertex;
    bool classified = true;
    std::array<bool,2> hasParameters{true,true};
    ParameterEntityKind kind = ParameterEntityKind::Cad;
};
struct ParameterCurveGeometry {
    int beginVertex = -1, endVertex = -1;
    double first = 0., last = 0.;
    bool parametrized = true, seam = false, onFace = false;
};
struct MeshVertexParameterSource {
    int face = -1;
    bool discreteSurface = false, plane = false;
    std::map<int,ParameterCurveGeometry> curves;
    std::map<int,std::vector<int>> vertexIncidentCurves; // 原创建序，闭边只登记一次。
    std::function<Point2D(int,int)> vertexCoordinates;
    std::function<Point2D(int,double,int)> edgeCoordinates;
    std::function<double(int,Point3D)> edgeParameterFromPoint;
    std::function<Point2D(Point3D,bool)> surfaceCoordinates;
};
struct MeshVertexParametersResult {
    OperationStatus status;
    std::vector<Point2D> parameters;
    bool seamPresent = false;
    std::size_t geometryEvaluations = 0;
};
/** 原getAllParameters，保留incident曲线序、seam端点-1/+1和两端重复。
 * 曲线存储参数恰为0才反求；无参数化曲线返回空候选。源元数据不计几何查询。
 */
MeshVertexParametersResult collectMeshVertexParameters(const MeshVertexParameterInput& vertex,
    const MeshVertexParameterSource& source,const RunControl& control = {});
struct MeshVertexParameterOptions { bool onSurface = true, failOnSeam = true; int direction = 1; };
struct MeshVertexParameterResult {
    OperationStatus status;
    Point2D parameter;
    bool succeeded = false;
    std::size_t geometryEvaluations = 0;
};
/** 原reparamMeshVertexOnFace；单点1D不执行参数0反求。failOnSeam返回false但保留已求参数。
 * 原缺1D参数分支读取未初始化t，无法确定性复刻；明确InvalidInput，不伪造t=0。
 */
MeshVertexParameterResult reparameterizeMeshVertexOnFace(const MeshVertexParameterInput& vertex,
    const MeshVertexParameterSource& source,const MeshVertexParameterOptions& options = {},const RunControl& control = {});
struct MeshEdgeParameterResult {
    OperationStatus status;
    std::array<Point2D,2> parameters{};
    std::array<std::size_t,2> selectedCandidates{};
    bool usedSurfaceFallback = false;
    std::size_t geometryEvaluations = 0;
};
/** 原reparamMeshEdgeOnFace：两端分别对另一端候选0选严格最近，非全候选对搜索。
 * 任一端无候选则两端都用face.parFromPoint；两个收集及回退共用查询预算。
 */
MeshEdgeParameterResult reparameterizeMeshEdgeOnFace(const MeshVertexParameterInput& first,
    const MeshVertexParameterInput& second,const MeshVertexParameterSource& source,const RunControl& control = {});
}
