/** @file BdsCurveRecovery.cpp
 * @brief 普通面边界与域后embedded恢复pass；外层失败控制分开。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsCurveRecovery.h"
#include <algorithm>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 检查调用方是否要求取消当前操作。
void checkRunning(const RunControl& control) {
    if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"BDS curve recovery cancelled"}};
}
/// 检查操作状态或前置条件，失败时抛出本模块的明确错误。
void require(OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; }
/// 将节点对按编号排序，生成无向边查找键。
auto pairKey(int first,int second) { return std::make_pair(std::min(first,second),std::max(first,second)); }
/// 建立物理节点身份到 BDS 节点编号的映射并拒绝重复身份。
std::map<std::uint64_t,int> physicalInverse(const BdsMesh& mesh,const RunControl& control) {
    std::map<std::uint64_t,int> inverse;
    for(const auto& entry : mesh.points) {
        checkRunning(control); const auto& point = entry.second;
        if(point.physicalId && !inverse.emplace(point.physicalId,point.id).second)
            throw OperationFailure{{MeshingStatus::InvalidInput,"ordinary BDS physical identity has multiple local copies"}};
    }
    return inverse;
}
/// 将边界物理节点对转换为 BDS 节点对。
std::optional<std::array<int,2>> mapSegment(const std::map<std::uint64_t,int>& inverse,std::array<std::uint64_t,2> segment) {
    const auto first = inverse.find(segment[0]), second = inverse.find(segment[1]);
    if(first == inverse.end() || second == inverse.end()) return std::nullopt;
    return std::array<int,2>{first->second,second->second};
}
/// 检查恢复曲线的 CAD 实体身份是否合法。
void requireCurve(const BdsRecoveryCurve& curve) {
    if(curve.entity < 0) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid BDS CAD curve identity"}};
}
}
/// 普通pass1：依boundary、embedded原曲线/线段序登记；退化边跳过。
OperationStatus buildOrdinaryBdsRecoveryRegistry(const BdsMesh& mesh,const std::vector<BdsRecoveryCurve>& boundary,
    const std::vector<BdsRecoveryCurve>& embedded,std::vector<BdsRecoveryConstraint>& output,const RunControl& control)
{
    try {
        checkRunning(control); require(validateBdsTopology(mesh,control)); const auto inverse = physicalInverse(mesh,control);
        std::map<std::pair<int,int>,BdsRecoveryConstraint> registry;
        for(const auto* curves : {&boundary,&embedded}) for(const auto& curve : *curves) {
            checkRunning(control); if(curve.meshDegenerated) continue; requireCurve(curve);
            for(auto segment : curve.segments) {
                checkRunning(control); const auto mapped = mapSegment(inverse,segment); if(!mapped) continue;
                const auto key = pairKey((*mapped)[0],(*mapped)[1]); registry.emplace(key,BdsRecoveryConstraint{key.first,key.second,curve.entity});
            }
        }
        std::vector<BdsRecoveryConstraint> result; for(const auto& entry : registry) result.push_back(entry.second);
        checkRunning(control); output = std::move(result); return {};
    } catch(const OperationFailure& error) { return error.status; }
      catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
}
namespace {
/// 逐条恢复 BDS 曲线约束并收集无法恢复的边界段。
BdsBoundaryRecoveryResult recoverCurves(const BdsMesh& input,const std::vector<BdsRecoveryCurve>& curves,
    const std::vector<BdsRecoveryConstraint>& registered,BdsMesh& output,bool refineAllEdges,bool stopOnFatal,const RunControl& control)
{
    BdsBoundaryRecoveryResult result;
    try {
        checkRunning(control); require(validateBdsTopology(input,control)); const auto inverse = physicalInverse(input,control);
        BdsMesh mesh = input; std::map<std::pair<int,int>,BdsRecoveryConstraint> notRecovered;
        for(const auto& curve : curves) {
            checkRunning(control); if(curve.meshDegenerated) continue; requireCurve(curve);
            const BdsClassification classification{curve.entity,1}; mesh.geometry.insert(classification);
            bool curveCompleted = true;
            for(std::size_t i = 0; i < curve.segments.size(); ++i) {
                checkRunning(control); const auto mapped = mapSegment(inverse,curve.segments[i]);
                if(!mapped) { result.skippedSegments.emplace_back(curve.entity,i); continue; }
                BdsRecoveryResult recovery;
                recovery.recoveredEdge = findBdsEdge(mesh,(*mapped)[0],(*mapped)[1]);
                // 原recover_edge对已有边立即返回。当前工作池在pass入口/出口校验，
                // 不为这种无连接改写的情况逐段复制并再次校验整个网格。
                if(recovery.recoveredEdge < 0) {
                    BdsMesh recovered;
                    recovery = recoverBdsConstraintEdge(mesh,(*mapped)[0],(*mapped)[1],recovered,registered,control);
                    require(recovery.status); mesh = std::move(recovered);
                }
                for(auto failed : recovery.notRecovered) notRecovered.emplace(pairKey(failed.first,failed.second),failed);
                const int edge = recovery.recoveredEdge; const bool fatal = recovery.nativeFatal;
                result.attempts.push_back({curve.entity,i,(*mapped)[0],(*mapped)[1],std::move(recovery)});
                if(edge >= 0) mesh.edges[edge].classification = classification;
                else {
                    curveCompleted = false;
                    if(fatal) { result.nativeFatal = true; result.failedCurve = curve.entity; }
                    break;
                }
            }
            if(stopOnFatal && result.nativeFatal) break;
            if(curveCompleted && curve.endpoints) {
                const auto mapped = mapSegment(inverse,*curve.endpoints);
                if(mapped) for(int id : *mapped) {
                    auto& point = mesh.points.at(id);
                    if(point.classification.dimension < 0) { point.classification = {id,0}; mesh.geometry.insert(point.classification); }
                }
            }
        }
        for(const auto& entry : notRecovered) result.notRecovered.push_back(entry.second);
        result.needsRefinement = !result.nativeFatal && (!result.notRecovered.empty() || refineAllEdges);
        checkRunning(control); require(validateBdsTopology(mesh,control)); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
/// 恢复普通面全部 CAD 边界并收集细分请求。
BdsBoundaryRecoveryResult recoverOrdinaryBdsBoundary(const BdsMesh& input,const std::vector<BdsRecoveryCurve>& boundary,
    const std::vector<BdsRecoveryConstraint>& registered,BdsMesh& output,bool refineAllEdges,const RunControl& control)
{ return recoverCurves(input,boundary,registered,output,refineAllEdges,true,control); }
/// 恢复普通面内嵌曲线约束。
BdsEmbeddedRecoveryResult recoverOrdinaryBdsEmbedded(const BdsMesh& input,const std::vector<BdsRecoveryCurve>& embedded,
    const std::vector<BdsRecoveryConstraint>& registered,BdsMesh& output,const RunControl& control)
{
    auto pass = recoverCurves(input,embedded,registered,output,false,false,control);
    return {std::move(pass.status),pass.nativeFatal,std::move(pass.notRecovered),std::move(pass.attempts),std::move(pass.skippedSegments)};
}
}
