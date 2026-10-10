#pragma once
/** @file FrontMeshTransfer.h
 * @brief 前沿运行集合到面连接的转移、朝向和单次等价映射；不平滑或释放运行池。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontQueue.h"
#include "mesh/surface/algorithm6/front/NodeSizeInitialization.h"
namespace mesher::gmshReference {
struct FrontMeshTransferOptions {
    bool usePhysicalNormals = false; // 原版有boundary-layer列时使用XYZ，否则使用运行UV。
    const std::map<std::uint64_t,std::uint64_t>* equivalence = nullptr;
};
struct FrontMeshTransferResult {
    OperationStatus status;
    std::vector<int> consumedTriangles, skippedDeletedTriangles, reversedTriangles, collapsedTriangles;
    std::vector<std::array<int,2>> equivalentTriangles; // 首代表与后续重复的运行池索引。
    bool equivalenceDiagnosticsEvaluated = false;
};
/** 按运行AllTris顺序替换output.triangles，保留input的节点/编号/ghost关联。
 * 以首活片的原double法线统一朝向，reverse只交换顶点1/2；首片退化不另选基准。
 * 非空equivalence先诊断重复，再逐顶点单次映射并丢弃身份退化片，不去重或递归映射。
 * 读取input/front/queue/nodes，不释放运行池或改队列；释放由调用编排层负责。
 * 取消/非法数据不发布半转换结果。output可与input为同一对象。
 */
FrontMeshTransferResult transferFrontMesh(const FrontMesh& front,const FrontQueueState& queue,
    const FaceMesh& input,const NodeSizeState& nodes,const FrontMeshTransferOptions& options,
    FaceMesh& output,const RunControl& control = {});
}
