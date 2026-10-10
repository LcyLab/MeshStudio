#pragma once
/** @file ConstraintRecovery.h
 * @brief 普通 PolyMesh 阶段的有序约束恢复；不做域分类或 BDS 恢复。
 * @date 2026-10-07
 */
#include "WalkTriangulation.h"
namespace mesher::gmshReference {
struct InitialConstraint {
    int fromVertex = -1, toVertex = -1, curveEntity = -1; // 顶点池索引，不是 BDS localId。
};
struct InitialConstraintDecision {
    InitialConstraint constraint;
    int nativeResult = -1; // 0 已存在；正值是初始穿越数；-1/-2/-3 为原生恢复失败。
    int recoveredHalfEdge = -1;
    std::vector<int> crossedEdges; // 恢复前原生追踪序，便于阶段对照。
};
struct ConstraintRecoveryResult {
    OperationStatus status;
    std::vector<InitialConstraintDecision> decisions;
};
/** 原版 intersect 的严格双方向面积乘积判据（包含 double 上下溢语义）。
 * 坐标和 orient2dValue 须有限；共线、端点相触及乘积下溢均不算穿越。
 */
bool intersectRecoverySegments(Point2D a, Point2D b, Point2D c, Point2D d);
/** 普通单 UV 图表的 BDS localId -> 物理身份 -> 初始顶点池索引转换。
 * 保留图表约束序，不按池顺序重排；多副本路线尚需独立原生 closest 规则。
 * 失败原子保留 output，不调用 CAD。
 */
OperationStatus prepareInitialConstraints(const FaceChart& chart, const WalkTriangulation& mesh,
    std::vector<InitialConstraint>& output, const RunControl& control = {});
/** 对应 GFaceInitialMesh(recover=1) 的逐线段 recover_edge/getEdge/标签阶段。
 * 输入已按 CAD 边/线段/副本次序转换为顶点池索引，保留该次序。
 * -1/-2/-3 记录诊断并按原版继续；不将其称为已恢复。
 * 原生半边标签不是此阶段翻边锁，不能额外跳过已标记边。
 * 取消/预算/非法拓扑/不动队列返回失败并原子保留 output。
 * 不动队列是原版无限循环的显式诊断，不用额外翻边/插点修补。
 */
ConstraintRecoveryResult recoverConstraintEdges(const WalkTriangulation& input,
    const std::vector<InitialConstraint>& constraints, WalkTriangulation& output, const RunControl& control = {});
}
