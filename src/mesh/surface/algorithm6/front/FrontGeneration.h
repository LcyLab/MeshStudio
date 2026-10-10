#pragma once
/** @file FrontGeneration.h
 * @brief 算法6动态前沿编排；不重建全队列、不转移最终网格或平滑。
 * @date 2026-10-08
 */
#include "FrontInsertion.h"
#include "FrontPointPlacement.h"
#include "mesh/surface/algorithm6/chart/ParametricDomain.h"
namespace mesher::gmshReference {
struct FrontGenerationSource {
    std::function<std::array<Point2D,2>()> parameterBounds; // 原CAD范围，返回[lower,upper]，每运行一次查询。
    FrontMetricSource metric;
    SurfaceFunction surface;
    std::function<FrontSurfacePoint(Point2D)> surfacePoint; // 新节点CAD查询保留GPoint成功标志；圆求交只消费坐标。
    std::function<MeshSizeEvaluation(const MeshSizeQuery&)> backgroundSize;
};
struct FrontGenerationOptions {
    bool extendBoundarySize = true;
    const std::vector<TrueBoundarySegment>* trueBoundary = nullptr; // nullptr不门控，非空指针即使向量空也门控。
    std::size_t maximumPops = std::numeric_limits<std::size_t>::max(); // 运行保护，不是原ITER收敛阈值。
};
struct FrontGenerationEvent {
    FrontSelection selection;
    std::optional<FrontPointCandidate> candidate;
    bool domainTested = false,insertionAttempted = false;
    ParametricDomainResult domain;
    FrontPointInsertionResult insertion;
};
struct FrontGenerationResult {
    OperationStatus status;
    Point2D farPoint;
    std::size_t pops = 0,candidates = 0,insertionAttempts = 0,inserted = 0,domainRejected = 0,geometryEvaluations = 0;
};
using FrontGenerationObserver = std::function<void(const FrontGenerationEvent&)>;
/** 接受已初始化FrontMesh/FrontQueue；每轮pop→候选/普通CAD修正→可选域门控→insertFrontPoint。
 * 源回调只读；新片立即反馈同一队列，队列为空才正常结束。固定force=false/verifyStar=true。
 * 几何总预算累计bounds一次、metric一次、每次surface/BGM一次。保护失败保留此前完成轮次，
 * 恢复本次未完成轮次的活动头；不全图复制。观察回调在完成轮次后调用，异常明确返回失败，
 * 已完成轮次保留；不通过日志控制算法。输出仍是运行池，最终转移/副本折叠/平滑另有职责。
 */
FrontGenerationResult generateFrontMesh(FrontMesh& front,FrontQueueState& queue,FaceMesh& mesh,NodeSizeState& nodes,
    const FrontGenerationSource& source,const FrontGenerationOptions& options = {},
    const RunControl& control = {},const FrontGenerationObserver& observer = {});
}
