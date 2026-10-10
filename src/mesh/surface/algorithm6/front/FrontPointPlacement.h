#pragma once
/** @file FrontPointPlacement.h
 * @brief 算法6度量候选与普通CAD圆修正；不判参数域/空腔、不插点。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/FrontMeshTypes.h"
#include "NodeSizeInitialization.h"
#include "mesh/surface/algorithm6/geometry/SurfaceMetric.h"
#include "mesh/surface/algorithm6/geometry/CurveSurfaceIntersection.h"
#include <optional>
namespace mesher::gmshReference {
using FrontMetricSource = std::function<SurfaceMetric(Point2D)>;
struct FrontPointCandidate {
    Point2D metricQuery,planarCenter,metricCenter,edgeMidpoint,fastPoint,point;
    SurfaceMetric metric;
    NodeSize edgeMeanSizes;
    double targetSize = 0.,distanceToCenter = 0.,height = 0.,directionNorm = 0.,metricRatio = 0.;
    Circle3 circle;
    double normalNorm = 0.;
    bool skippedDegenerateNormal = false,correctionAttempted = false,intersectionFound = false,retainedFastPoint = false;
    std::size_t geometryEvaluations = 0;
};
struct FrontPointPlacementResult {
    OperationStatus status;
    std::size_t geometryEvaluations = 0;
    std::optional<CurveSurfaceIntersectionResult> intersection;
};
/** optimalPointFrontal：先普通UV外心，再在UV重心查度量并构造度量外心；
 * 原边号端点双尺寸均值，L=min(h*sqrt(3)*0.5,度量外心距离)，保留原算术序。
 * 不使用最短边形状指标、不查询新点BGM或判参数域。一次metric回调计一几何查询。
 * 非有限候选明确NonConverged；取消/预算/非法输入/查询失败不发布output。
 */
FrontPointPlacementResult computeFrontPoint(const FrontTriangle& triangle,int activeEdge,const NodeSizeState& nodes,
    bool extendBoundarySize,const FrontMetricSource& source,FrontPointCandidate& output,const RunControl& control = {});
/** optimalPointFrontalB普通CAD分支：以原物理边构造圆，norm<1e-12保留快候选；
 * 圆求交成功覆盖UV，正常失败/原生捕获的CAD求值失败保留快候选并返回显式诊断。
 * 取消/预算不当作求交失败回退；查询预算累计输入候选已用次数。
 * 保留尺寸导致的有符号圆半径和原epsilon，不以正尺寸假设拒绝有限候选。
 * 无HXT离散面专用圆求交；不生成节点或检查包含片。
 */
FrontPointPlacementResult correctFrontPointOnSurface(const FrontTriangle& triangle,int activeEdge,const FaceMesh& mesh,
    const FrontPointCandidate& input,const SurfaceFunction& surface,FrontPointCandidate& output,const RunControl& control = {});
}
