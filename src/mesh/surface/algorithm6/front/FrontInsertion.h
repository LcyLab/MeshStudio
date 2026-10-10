#pragma once
/** @file FrontInsertion.h
 * @brief 原insertVertexB验收/局部提交与insertAPoint节点生命周期；几何查询由显式源提供。
 * @date 2026-10-08
 */
#include "FrontCavity.h"
#include <functional>
namespace mesher::gmshReference {
struct FrontInsertionOptions {
    bool extendBoundarySize = true,verifyStarShapeness = true,force = false;
};
struct FrontInsertionTriangle {
    FrontTriangle triangle;
    double firstDistance = 0.,secondDistance = 0.,edgeLength = 0.,boundaryHeight = 1.e22,angleCosine = 0.,parameterArea = 0.;
    bool tooClose = false;
};
struct FrontInsertionValidation {
    OperationStatus status;
    int nativeCode = 0; // 1接受；-1单片/-2Euler/-3面积/-4距离或角度/-5严格面积等号。
    double oldArea = 0.,newArea = 0.,relativeAreaTolerance = 0.;
    bool tooClose = false;
    std::vector<FrontInsertionTriangle> triangles; // shell原序；拒绝时保留计算诊断，不公开到运行池。
};
struct FrontInsertionCommit {
    OperationStatus status;
    std::vector<int> createdTriangles,neighborhood; // neighborhood保留[new,otherSide,...]的重复片。
    FrontQueueUpdate registration,activity;
};
struct FrontInsertionResult {
    OperationStatus status;
    int nativeCode = 0;
    FrontInsertionValidation validation;
    FrontInsertionCommit commit;
};
struct FrontSurfacePoint { Point3D point; bool succeeded = true; };
struct FrontInsertionSource {
    std::function<FrontSurfacePoint(Point2D)> surfacePoint;
    std::function<MeshSizeEvaluation(const MeshSizeQuery&)> backgroundSize;
};
struct FrontPointInsertionResult {
    OperationStatus status;
    bool inserted = false,pointQueried = false,backgroundQueried = false,nodeAppended = false;
    int nativeCode = 0; // 0无包含片；-9正常failed GPoint；其他为insertVertexB的返回码。
    std::uint64_t allocatedVertex = 0;
    std::size_t parameterIndex = std::numeric_limits<std::size_t>::max(),geometryEvaluations = 0;
    FrontCavityResult cavity;
    FrontSurfacePoint surface;
    InitializedNodeSize node;
    MeshSizeQuery sizeQuery;
    MeshSizeEvaluation sizeEvaluation;
    FrontInsertionResult triangles;
    FrontFailureUpdate failure;
};
/** 原insertVertexB的早退、新片尺寸/物理R/h、距离/角度及UV面积验收；不修改输入。
 * 新节点及双尺寸显式传入，不查询CAD/BGM，不替换为初始MTri3的近似1/3常数。
 * force只绕过距离/角度拒绝，verifyStarShapeness只控制1e-12/1e12面积容差。
 * coincident端点产生的cosine NaN保持原比较语义；输入/面积/半径非有限明确失败。
 */
FrontInsertionValidation validateFrontInsertion(const FrontMesh& front,const FaceMesh& mesh,const NodeSizeState& nodes,
    const FrontCavityResult& cavity,const FaceVertex& vertex,const InitializedNodeSize& node,
    const FrontInsertionOptions& options = {},const RunControl& control = {});
/** 仅提交nativeCode=1的验收结果：新片按shell序追加，局部交错序connectTris，再登记/更新队列。
 * 原空腔deleted及AllTris陈旧成员保持，不修改FaceMesh/节点参数槽或初始排序快照。
 * 取消/异常恢复本次新增片、局部邻接和队列增量，不复制全网格。
 */
FrontInsertionCommit commitFrontInsertion(FrontMesh& front,FrontQueueState& queue,const FrontCavityResult& cavity,
    const FrontInsertionValidation& validation,const RunControl& control = {});
/** 对应原insertVertexB正常控制：验收成功提交，-3/-4/-5清空腔标记，早退-1/-2不清。
 * 外层insertAPoint还需要恢复全部正常拒绝、设置worst半径及发布/销毁节点，尚未由此函数负责。
 */
FrontInsertionResult insertFrontTriangles(FrontMesh& front,FrontQueueState& queue,const FaceMesh& mesh,const NodeSizeState& nodes,
    const FrontCavityResult& cavity,const FaceVertex& vertex,const InitializedNodeSize& node,
    const FrontInsertionOptions& options = {},const RunControl& control = {});
/** 原算法6 insertAPoint：空腔/包含片→CAD点→创建身份→mesh插值→独立BGM→append参数/尺寸。
 * 然后验收/局部提交；成功才公开FaceVertex/interiorVertices和live索引。正常拒绝保留参数槽
 * 与已消费身份，销毁live节点并恢复空腔/置worst半径-1；无包含片不查询/分配，半径0。
 * worst必须已从活动集pop，算法6入口不接受force。源回调只读；failed GPoint仍查询BGM和append，再走-9。
 * 取消/预算/查询异常恢复入口状态，不冒充正常拒绝；不全图复制、不执行参数域门控。
 */
FrontPointInsertionResult insertFrontPoint(FrontMesh& front,FrontQueueState& queue,FaceMesh& mesh,NodeSizeState& nodes,
    int worst,Point2D point,SurfaceMetric metric,const FrontInsertionSource& source,
    const FrontInsertionOptions& options = {},const RunControl& control = {});
}
