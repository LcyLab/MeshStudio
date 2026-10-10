#pragma once
/** @file InitialMeshLegalization.h
 * @brief BDS初始度量合法化的动态池扫描/配置去重/圆内决策；不裁域或插点。
 * @date 2026-10-08
 */
#include "BdsSwapQualification.h"
#include "mesh/surface/algorithm6/geometry/SurfaceMetric.h"
namespace mesher::gmshReference {
struct InitialMetricCircleDecision {
    OperationStatus status;
    MetricCircumcircle circle;
    double queryDistanceSquared = 0., tolerance = 0.;
    bool inside = false;
};
/** 数组重载inCircumCircleAniso：前三点构圆、第四点度量距离；半径平方
 * <=1e3/<=1e5/更大时容差1e-12/1e-11/1e-9。奇异原点外心仍按原版判断，
 * 标记诊断，不用robust incircle或随尺寸缩放的自有容差替代。
 */
InitialMetricCircleDecision shouldFlipInitialEdge(const std::array<Point2D,4>& points,SurfaceMetric metric);
enum class InitialEdgeDecision { EndpointsUnmodified, NotTwoFaces, ConfigurationVisited, OutsideCircle, SwapRejected, Swapped };
struct InitialEdgeAttempt {
    int edge = 0;
    InitialEdgeDecision decision = InitialEdgeDecision::NotTwoFaces;
    std::optional<std::array<int,4>> configuration;
    Point2D midpoint;
    SurfaceMetric metric;
    InitialMetricCircleDecision circle;
};
struct InitialLegalizationScan {
    std::size_t startingEdges = 0, endingEdges = 0, swaps = 0;
    std::vector<InitialEdgeAttempt> attempts; // 只记录活边，池追加在同轮继续遍历。
};
struct InitialMeshLegalizationResult {
    OperationStatus status;
    std::size_t swaps = 0, metricQueries = 0;
    std::set<std::array<int,4>> configurations; // 四节点ID排序，跨所有轮保存。
    std::vector<InitialLegalizationScan> scans;
};
using InitialSurfaceMetricSource = std::function<MetricEvaluation(Point2D)>;
/** 对应delaunayizeBDS：端点modified/双面→登记未见配置→边中点度量→圆内
 * →原Quality(false)资格→BdsTopology交换；动态扫描新增边，零交换末轮停止。
 * 配置在查询前登记，包括圆外/资格拒绝；不清理deleted池记录或重置modified。
 * source只求CAD度量，预算限定调用数；取消/预算/几何失败保留output。
 * 成功原子发布，纯算法不链接CAD/Gmsh，不混入旧产品优化或尺寸条件。
 */
InitialMeshLegalizationResult legalizeInitialMesh(const BdsMesh& input,
    const InitialSurfaceMetricSource& source,BdsMesh& output,const RunControl& control = {});
}
