#pragma once
/** @file NodeSizeInitialization.h
 * @brief 前沿前物理邻边尺寸、边界半段下限和embedded双尺寸；不生成MTri3或插点。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/FaceMeshTypes.h"
#include "mesh/surface/algorithm6/boundary/MeshSizeEvaluator.h"
#include <set>
namespace mesher::gmshReference {
struct NodeSizeCurve {
    bool meshDegenerated = false;
    std::vector<std::array<std::uint64_t,2>> segments; // 原CAD线段节点身份，不替换为缝副本。
};
struct NodeSizeInitializationInput {
    // 原vSizesMap的身份遍历序，必须恰好覆盖实际加入的节点；不按UV或XYZ推断。
    std::vector<std::uint64_t> vertexOrder;
    // 每个embedded GVertex的首网格节点及prescribedMeshSizeAtVertex，按CAD遍历序。
    std::vector<std::pair<std::uint64_t,double>> embeddedVertices;
    std::vector<NodeSizeCurve> boundaryCurves, embeddedCurves;
};
struct InitializedNodeSize {
    std::uint64_t vertex = 0;
    Point2D parameters;
    NodeSize sizes;
};
struct NodeSizeState {
    std::vector<InitializedNodeSize> nodes;
    std::map<std::uint64_t,std::size_t> indices;
    // 无向身份对只用于约束成员查询；不表示原生MEdge指针遍历顺序。
    std::set<std::array<std::uint64_t,2>> internalEdges;
};
struct NodeReparameterization {
    std::uint64_t vertex = 0;
    Point2D parameters; // 实际CAD反求结果，在周期parametricCoordinates覆盖之前。
};
struct EmbeddedNodeSizeQuery {
    std::uint64_t vertex = 0;
    MeshSizeQuery query;
    MeshSizeEvaluation evaluation;
};
struct NodeSizeInitializationResult {
    OperationStatus status;
    std::vector<NodeReparameterization> reparameterizations;
    std::vector<EmbeddedNodeSizeQuery> embeddedQueries;
};
using NodeReparameterizationSource = std::function<Point2D(const FaceVertex&)>;
/** 按buildMeshGenerationDataStructures的节点阶段：三角邻边最短物理长度，
 * 只排除fake与其original之间的边，随后embedded点取小、boundary半段下限。
 * 每个节点都先查询CAD参数，仅embedded原节点以面实体/原XYZ/反求参数查询BGM；
 * 普通节点background=mesh。周期UV覆盖在BGM查询之后，不跳过CAD反求。
 * 保留原版未引用embedded/边界节点map[]的0初值及无合格邻边的-1尺寸。
 * 不使用BDS lc/lcBGM或MEdgeVertex storedSize，不构造MTri3/队列、修改连接或折叠缝。
 * 输出按显式vertexOrder索引；取消/预算/非法数据/几何失败保持output。
 */
NodeSizeInitializationResult initializeNodeSizes(const FaceMesh& mesh,
    const NodeSizeInitializationInput& input,const NodeReparameterizationSource& reparameterize,
    double modelCharacteristicLength,const SurfaceMeshingOptions& options,
    const GeometrySizeSource& source,NodeSizeState& output,const RunControl& control = {});
}
