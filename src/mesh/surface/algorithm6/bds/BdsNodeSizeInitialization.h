#pragma once
/** @file BdsNodeSizeInitialization.h
 * @brief 普通BDS原MVertex查询元数据的BGM双尺寸初始化；不清理或插值前沿尺寸。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
#include "mesh/surface/algorithm6/boundary/MeshSizeEvaluator.h"
namespace mesher::gmshReference {
// 对应recoverMap中原MVertex::onWhat()/XYZ/曲线参数，以BDS localID索引。
// 与BDS自身分类/UV/面求值XYZ分开；缺键对应原版未映射节点，尺寸保持。
using BdsRecoveredVertexQueries = std::map<int,MeshSizeQuery>;
struct BdsNodeSizeQuery {
    int pointId = 0;
    MeshSizeQuery query;
    MeshSizeEvaluation evaluation;
};
struct BdsNodeSizeInitializationResult {
    OperationStatus status;
    bool skippedInitialOnly = false;
    std::vector<BdsNodeSizeQuery> queries;
    std::vector<int> unconstrainedPoints;
};
/** 普通Algo6 BDS尺寸阶段：按BDS节点ID序，原实体0D以(0,0)，1D以(u,0)
 * 求BGM，并同时赋meshSize/backgroundSize；其它实体维数直接赋MAX_LC。
 * 不使用BDS分类、UV、坐标替代原MVertex数据，不计算边长或前沿插值。
 * onlyInitialMesh对应原入口初始网格模式，跳过整尺寸段；Algo6实际为false。
 * maximumGeometryEvaluations在此限定BGM调用数；取消/预算/非法查询/查询失败
 * 均保留output，诊断可记录已尝试查询。成功原子替换output。
 */
BdsNodeSizeInitializationResult initializeOrdinaryBdsNodeSizes(const BdsMesh& input,
    const BdsRecoveredVertexQueries& recoveredVertices,double modelCharacteristicLength,
    const SurfaceMeshingOptions& options,const GeometrySizeSource& source,BdsMesh& output,
    bool onlyInitialMesh = false,const RunControl& control = {});
}
