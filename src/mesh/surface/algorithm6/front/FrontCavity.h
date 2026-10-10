#pragma once
/** @file FrontCavity.h
 * @brief 固定候选度量的前沿空腔、原DFS壳边及包含片；不分配新节点或提交拓扑。
 * @date 2026-10-08
 */
#include "FrontPointLocation.h"
#include "mesh/surface/algorithm6/geometry/SurfaceMetric.h"
namespace mesher::gmshReference {
struct FrontShellEdge { int triangle = -1,edge = -1; };
struct FrontCavityCircle {
    int triangle = -1;
    MetricCircumcircle circle;
    double distanceSquared = 0.;
    bool inside = false;
};
struct FrontCavityResult {
    OperationStatus status;
    bool worstInsideCircle = false;
    std::vector<int> triangles;
    std::vector<bool> entryDeleted; // 与triangles同序，仅供组合操作取消/异常恢复；正常失败仍清标记。
    std::vector<FrontShellEdge> shell;
    std::vector<FrontCavityCircle> circleTests;
    std::vector<std::pair<int,FrontPointCoordinates>> coordinateTests;
    FrontTriangleSearch search;
    int containingTriangle = -1;
    FrontPointCoordinates coordinates;
};
/** insertAPoint的初始圆判断/recurFindCavityAniso/包含片阶段；MTriangle重载无减容差。
 * 内部边按精确节点身份保护，不折叠缝；DFS按边0..2，deleted邻片不加入shell。
 * 原正常结果保留新空腔deleted标记，即使找不到包含片；调用方再rollbackFrontCavity。
 * 搜索可返回原已deleted起点，仍按原递归加入。取消/异常恢复调用前标记；不全图复制或查询CAD。
 */
FrontCavityResult buildFrontCavity(FrontMesh& mesh,const NodeSizeState& nodes,const FrontTriangleSet& allTriangles,
    int worst,Point2D point,SurfaceMetric metric,bool forceSearch = false,const RunControl& control = {});
/** 原正常失败恢复：先校验索引，再将cavity成员deleted=false，含原已deleted搜索起点。
 * 没有加入cavity的旧deleted邻片保持；不改变队列半径。区别于取消/异常的入口状态恢复。
 */
OperationStatus rollbackFrontCavity(FrontMesh& mesh,const FrontCavityResult& cavity);
}
