#pragma once
/** @file FrontPointLocation.h
 * @brief 前沿invMapUV/search4Triangle及实际包含片mesh尺寸插值，无CAD/BGM或空腔修改。
 * @date 2026-10-08
 */
#include "FrontQueue.h"
#include "NodeSizeInitialization.h"
namespace mesher::gmshReference {
struct FrontPointCoordinates {
    OperationStatus status;
    Point2D local;
    bool inside = false,singularSystem = false;
};
struct FrontTriangleSearch {
    OperationStatus status;
    int triangle = -1;
    FrontPointCoordinates coordinates;
    std::vector<int> visitedTriangles,crossedEdges,forceScannedTriangles;
    std::vector<std::pair<int,FrontPointCoordinates>> coordinateTests;
    bool noCrossedEdge = false,iterationLimit = false;
};
/** 原sys2x2和invMapUV；奇异系统local=(0,0)，仍按原规则判inside并明确标记。
 * 不用面积orient替代；最后1-u-v> -tol严格比较。输入/算术非有限明确失败。
 */
FrontPointCoordinates mapFrontPointToTriangle(const FrontTriangle& triangle,const NodeSizeState& nodes,
    Point2D point,double tolerance = 1.e-8);
/** 原intersection_segments_2：两次有向面积乘积>0拒绝；含相触/共线/下溢。
 * 共线且分离仍返回true，不增加包围盒规则；非有限输入/谓词值抛OperationFailure。
 */
bool intersectFrontSearchSegments(Point2D a,Point2D b,Point2D c,Point2D d);
/** 原search4Triangle：走到UV重心的首相交边，保留原ITER++上限；force才扫AllTris。
 * 初始/邻片不检查deleted，force扫描才跳过deleted；不自动开force或全图定位。
 */
FrontTriangleSearch findContainingFrontTriangle(const FrontMesh& mesh,const NodeSizeState& nodes,
    const FrontTriangleSet& allTriangles,int start,Point2D point,bool force = false,const RunControl& control = {});
/** insertAPoint的lc1=(1-u-v)*h0+u*h1+v*h2；允许原容差带/有符号尺寸，不插值BGM。
 * 不检查inside或夹紧权重；失败不发布output。
 */
OperationStatus interpolateNodeMeshSize(const FrontTriangle& triangle,const NodeSizeState& nodes,
    Point2D local,double& output);
}
