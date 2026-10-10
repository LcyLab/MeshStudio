#pragma once
/** @file BdsCurveRecovery.h
 * @brief 普通面BDS约束登记、域前边界/域后嵌入曲线恢复和端点分类；不着域或清理池。
 * @date 2026-10-08
 * @modified 2026-10-09
 */
#include "BdsConstraintRecovery.h"
namespace mesher::gmshReference {
struct BdsRecoveryCurve {
    int entity = -1;
    bool meshDegenerated = false;
    std::vector<std::array<std::uint64_t,2>> segments; // 原GEdge::lines序，使用物理身份。
    std::optional<std::array<std::uint64_t,2>> endpoints; // begin/end的首个0D网格点；无begin时空。
};
struct BdsCurveRecoveryAttempt {
    int curveEntity = -1;
    std::size_t segmentIndex = 0;
    int first = 0, second = 0;
    BdsRecoveryResult recovery;
};
struct BdsBoundaryRecoveryResult {
    OperationStatus status;
    bool nativeFatal = false, needsRefinement = false;
    int failedCurve = -1;
    std::vector<BdsRecoveryConstraint> notRecovered;
    std::vector<BdsCurveRecoveryAttempt> attempts;
    std::vector<std::pair<int,std::size_t>> skippedSegments; // 一端无inverse映射时按原版跳过。
};
/** 普通pass1：依boundary、embedded原曲线/线段序登记；退化边跳过。
 * 端点对按局部ID排序/去重，第一次登记的CAD曲线身份保留。
 * 普通单副本物理inverse映射从BDS点构建；缺映射跳段，多副本显式拒绝。
 * 成功原子替换output；无CAD调用，不改网格分类或连接。
 */
OperationStatus buildOrdinaryBdsRecoveryRegistry(const BdsMesh& mesh,
    const std::vector<BdsRecoveryCurve>& boundary,const std::vector<BdsRecoveryCurve>& embedded,
    std::vector<BdsRecoveryConstraint>& output,const RunControl& control = {});
/** 普通域标记前的boundary pass2，只遍历boundary；embedded在域标记后恢复。
 * 曲线分类在处理线段前登记。缺inverse映射跳段；成功边标CAD curve。
 * 某段恢复失败结束该曲线；fatal结束全部boundary，非fatal继续下条曲线。
 * 正常处理完曲线才给空分类的begin/end赋(localID,0)，不是CAD点tag。
 * nativeFatal或notRecovered/refineAll由上层失败/重试编排处理，本函数不细分1D。
 * pass入口/出口校验完整工作池；已有边保持单边调用的空扫描诊断，避免重复全池复制。
 * Complete表示pass执行结束；正常失败发布原尝试拓扑，控制/非法失败保留output。
 */
BdsBoundaryRecoveryResult recoverOrdinaryBdsBoundary(const BdsMesh& input,
    const std::vector<BdsRecoveryCurve>& boundary,const std::vector<BdsRecoveryConstraint>& registered,
    BdsMesh& output,bool refineAllEdges = false,const RunControl& control = {});
struct BdsEmbeddedRecoveryResult {
    OperationStatus status;
    bool hadFatalRecovery = false; // 原版外层忽略curve bool；保留诊断，不据此中止其它embedded。
    std::vector<BdsRecoveryConstraint> notRecovered;
    std::vector<BdsCurveRecoveryAttempt> attempts;
    std::vector<std::pair<int,std::size_t>> skippedSegments;
};
/** 普通域标记后的embedded pass2；登记表仍使用域前pass1产生的全表。
 * 与boundary相同curve/空端点分类和单段提前返回，但外层按原版继续其它curve。
 * Complete只表示完成调用序列，hadFatalRecovery/notRecovered须另查；不自动重试。
 * 不执行尺寸初始化/裁域/cleanup；成功替换工作池，控制/非法失败保留output。
 */
BdsEmbeddedRecoveryResult recoverOrdinaryBdsEmbedded(const BdsMesh& input,
    const std::vector<BdsRecoveryCurve>& embedded,const std::vector<BdsRecoveryConstraint>& registered,
    BdsMesh& output,const RunControl& control = {});
}
