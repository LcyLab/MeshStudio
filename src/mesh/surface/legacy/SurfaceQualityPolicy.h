#pragma once
/** @file SurfaceQualityPolicy.h
 * @brief 几何尖角/新增边可行性保护与合格片形状目标；不豁免普通边界。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include "geometry/types/MesherTypes.h"
#include <array>
#include <map>
namespace mesher {
// 等边为 1；退化或非有限输入为 0，长度尺度不改变指标。
double surfaceShapeQuality(Point3D a,Point3D b,Point3D c);
struct SurfaceShapeScore {
    double worstRadiusEdge=0,minimumShape=1,sumShape=0;
    int count=0;
    /// 将浮点展开按指定符号相加。
    void add(double radiusEdge,double shape);
    // 相同片数，最差 radius/edge 与最低 q 均不恶化，平均 q 严格改善。
    bool improves(const SurfaceShapeScore& before) const;
};
enum class SurfaceQualityReason {Eligible,ProtectedCorner,MinimumSize,NoCandidate,BudgetLimit};
class SurfaceQualityPolicy {
    double floor,angleLimit,radiusEdgeLimit;std::map<int,double> corners;
public:
    /// 校验尺寸和半径边长比，建立表面质量判断规则。
    SurfaceQualityPolicy(double minSize,double radiusEdgeRatio);
    // 仅传入 CAD 角点的有效内角；凹角不保护，近似曲线采样角不当作 CAD 尖角。
    void protectCorner(int vertex,Point3D position,double interiorAngleDeg);
    // 保留短边 s 时，新边 >= floor 导致 R/s >= floor/(2s)。仅排除已违反此必要条件的插点。
    // 不是 CAD 不可改善证明；重剖分移除内部短边等操作仍可改善。
    // localFloor=0 使用常规下限；正值为有几何来源的局部下限，不取消尖角保护。
    SurfaceQualityReason decision(std::array<int,3> vertices,double shortestEdge,double localFloor=0) const;
};
/// 将表面质量判定原因转换为报告名称。
const char* surfaceQualityReasonName(SurfaceQualityReason reason);
// 检查普通裁剪段被坏片对顶点侵入，候选分点不恶化最差子片、改善总和并保持新增边下限。
// 可为CAD参数中点或偏移分点，调用方确保分点位于原CAD段及其相邻参数区间。
// 关联面重建可能改变邻域，最终最差值/超标数仍须验收，不可仅依赖两子片预估。
bool qualityBoundarySplitAllowed(Point3D a,Point3D b,Point3D opposite,Point3D midpoint,
    double minSize,double radiusEdgeRatio);
// 域内面片连接到另一条平行裁剪段时，允许协调窄带分点试验。
// 单个子片可暂时恶化；调用方必须整批重建并验收最终超标数/最差值、几何和相交。
bool narrowBoundarySplitTrialAllowed(Point3D a,Point3D b,Point3D opposite,Point3D oppositeNeighbor,
    Point3D midpoint,double minSize,double radiusEdgeRatio);
// 细侧坏片对面的粗 CAD 段分点，协调错开的两侧节点；midpoint 位于 p-q。
// 与上面的分点试验一样，需要整批最终验收，不作局部改善保证。
bool narrowBoundaryAlignmentTrialAllowed(Point3D a,Point3D b,Point3D p,Point3D q,
    Point3D midpoint,double minSize,double radiusEdgeRatio);
}
