#pragma once
/** @file PeriodicBdsRecovery.h
 * @brief 周期BDS的embedded先行/边界环恢复及原生重试诊断；不标域或重建1D。
 * @date 2026-10-08
 */
#include "BdsCurveRecovery.h"
namespace mesher::gmshReference {
struct PeriodicBdsRecoveryInput {
    std::vector<std::array<int,2>> embedded; // 图表选定的局部副本，不按物理身份重新选点。
    std::vector<std::vector<int>> boundaryLoops;
    std::vector<BdsRecoveryCurve> boundaryCurves; // getGEdge的原曲线/有向物理线段顺序。
    bool refineAllEdges = false;
};
struct PeriodicBdsRecoveryAttempt {
    bool embedded = false;
    std::size_t loop = 0, segment = 0;
    int first = 0, second = 0, edge = -1;
    bool alreadyPresent = false;
    std::optional<BdsRecoveryResult> recovery; // boundary原已存在时不调用leaf。
    std::optional<int> curveEntity; // 仅boundary缺边且leaf执行后调用getGEdge。
};
struct PeriodicBdsRecoveryResult {
    OperationStatus status;
    bool embeddedFailed = false, needsRetry = false;
    std::vector<BdsRecoveryConstraint> attemptedBoundaryCurves; // 原edgesNotRecovered；缺边成功也登记。
    std::vector<PeriodicBdsRecoveryAttempt> attempts;
};
/** meshGeneratorPeriodic：逐embedded recover_edge，无registry，任意失败停止全部；
 * 随后逐环闭合边，原已有边不调用leaf，缺边恢复后按原物理链查CAD曲线，
 * 即使恢复成功仍记edgesNotRecovered。所有成功边赋临时CLASS_E=(1,1)，
 * 不登记CAD分类或补0D端点。boundary失败继续后续段，needsRetry或refineAll
 * 交上层串行/重试编排；不执行重建、域、尺寸或清理。
 * Complete表示调用序完成；embeddedFailed/needsRetry须另查，原生正常失败
 * 发布此前尝试池，取消/非法/预算失败保留output。节点身份用独立BDS局部ID。
 */
PeriodicBdsRecoveryResult recoverPeriodicBdsConstraints(const BdsMesh& input,
    const PeriodicBdsRecoveryInput& constraints,BdsMesh& output,const RunControl& control = {});
}
