#pragma once
/** @file BdsConstraintRecovery.h
 * @brief 原生BDS数值求交、恢复翻边资格、边池扫描与失败诊断。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
namespace mesher::gmshReference {
struct BdsSegmentIntersection {
    bool systemSolved = false, intersects = false;
    Point2D parameters; // 第一边t、第二边s；奇异时两个值均0。
};
/** BDS::Intersect_Edges_2d的原始double sys2x2及闭区间[0,1]。
 * norm按原pow/相加序，abs(det)/norm <1e-16视为奇异；不加几何容差。
 * 不同于PolyMesh严格面积乘积相交。身份共享端点由恢复循环先排除。
 */
BdsSegmentIntersection intersectBdsRecoverySegments(Point2D first,Point2D second,Point2D from,Point2D to);
/** 原生swap_edge公共保护和BDS_SwapEdgeTestRecover的面积数值乘积>0。
 * force只绕过config_modified，不绕过已分类曲线/双面/凸性；不改拓扑。
 */
bool canSwapBdsRecoveryEdge(const BdsMesh& mesh,int edge,const BdsEdgeNeighborhood& neighborhood,bool force = false);
struct BdsRecoveryConstraint {
    int first = 0, second = 0, curveEntity = 0;
};
struct BdsRecoveryScan {
    std::vector<int> intersectedEdges;
    std::vector<std::pair<int,bool>> attempts;
};
struct BdsRecoveryResult {
    OperationStatus status;
    int recoveredEdge = -1;
    bool nativeFatal = false;
    std::vector<BdsRecoveryConstraint> notRecovered; // 唯一端点对、按ID递增；CAD自交时两条都记录。
    std::vector<BdsRecoveryScan> scans;
};
/** 单次BDS::recover_edge；不分配CAD分类、不处理曲线外层pass或重试。
 * 先查已有边；每轮按原池序求交，遇已登记约束自交记录双方并返回非fatal失败。
 * 依次尝试相交边，第一个能翻就重扫；无相交边仍缺目标/无可翻边/ix>300为fatal。
 * Complete仅说明原生调用结束，须另查recoveredEdge/nativeFatal/notRecovered。
 * 原生正常失败可发布本次尝试后的拓扑；取消/预算/非法输入失败保留output。
 * registered空则对应原版e2r=null；非空时CAD自交的当前目标也须存在登记中。
 * 输入必须为纯三角BDS（算法6基线）；不支持重组四边形。
 */
BdsRecoveryResult recoverBdsConstraintEdge(const BdsMesh& input,int first,int second,
    BdsMesh& output,const std::vector<BdsRecoveryConstraint>& registered = {},const RunControl& control = {});
}
