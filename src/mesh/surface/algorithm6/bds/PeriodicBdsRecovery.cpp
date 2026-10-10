/** @file PeriodicBdsRecovery.cpp
 * @brief 周期BDS恢复驱动。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "PeriodicBdsRecovery.h"
#include <algorithm>
#include <exception>
namespace mesher::gmshReference {
/// 恢复周期图表边界及内嵌约束并记录未恢复项。
PeriodicBdsRecoveryResult recoverPeriodicBdsConstraints(const BdsMesh& input,
    const PeriodicBdsRecoveryInput& constraints,BdsMesh& output,const RunControl& control) {
    PeriodicBdsRecoveryResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"periodic BDS recovery cancelled"}}; };
        auto require = [](OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; };
        check(); require(validateBdsTopology(input,control)); BdsMesh mesh = input;
        const BdsClassification boundaryMarker{1,1,BdsClassificationOrigin::FaceBoundary};
        auto endpoints = [&](int first,int second) {
            if(first == second || !mesh.points.count(first) || !mesh.points.count(second)) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid periodic recovery local endpoint identities"}};
        };
        auto recover = [&](PeriodicBdsRecoveryAttempt& attempt) {
            BdsMesh recovered; attempt.recovery = recoverBdsConstraintEdge(mesh,attempt.first,attempt.second,recovered,{},control);
            require(attempt.recovery->status); mesh = std::move(recovered); attempt.edge = attempt.recovery->recoveredEdge;
        };
        for(std::size_t i = 0; i < constraints.embedded.size(); ++i) {
            check(); const auto pair = constraints.embedded[i]; endpoints(pair[0],pair[1]);
            PeriodicBdsRecoveryAttempt attempt; attempt.embedded = true; attempt.segment = i; attempt.first = pair[0]; attempt.second = pair[1];
            attempt.alreadyPresent = findBdsEdge(mesh,pair[0],pair[1]) >= 0; recover(attempt);
            if(attempt.edge >= 0) mesh.edges[attempt.edge].classification = boundaryMarker;
            else result.embeddedFailed = true;
            result.attempts.push_back(std::move(attempt)); if(result.embeddedFailed) break;
        }
        // 原版在embedded全部成功之后才初始化doItAgain，也不为该pass登记CAD分类。
        if(!result.embeddedFailed) {
            result.needsRetry = constraints.refineAllEdges;
            std::map<std::pair<int,int>,BdsRecoveryConstraint> attempted;
            for(std::size_t i = 0; i < constraints.boundaryLoops.size(); ++i) {
                const auto& loop = constraints.boundaryLoops[i];
                for(std::size_t j = 0; j < loop.size(); ++j) {
                    check(); const int first = loop[j], second = loop[(j+1)%loop.size()]; endpoints(first,second);
                    PeriodicBdsRecoveryAttempt attempt; attempt.loop = i; attempt.segment = j; attempt.first = first; attempt.second = second;
                    attempt.edge = findBdsEdge(mesh,first,second); attempt.alreadyPresent = attempt.edge >= 0;
                    if(!attempt.alreadyPresent) {
                        recover(attempt);
                        const auto p1 = mesh.points.at(first).physicalId, p2 = mesh.points.at(second).physicalId;
                        // getGEdge按原CAD曲线/物理线段顺序查首个owner，恢复成功也登记。
                        bool found = false;
                        if(p1 && p2) for(const auto& curve : constraints.boundaryCurves) {
                            check(); if(curve.entity < 0) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid periodic boundary CAD curve"}};
                            for(auto segment : curve.segments) if((segment[0] == p1 && segment[1] == p2) || (segment[1] == p1 && segment[0] == p2)) {
                                attempt.curveEntity = curve.entity; const auto key = std::minmax(first,second);
                                attempted.emplace(std::make_pair(key.first,key.second),BdsRecoveryConstraint{key.first,key.second,curve.entity}); found = true; break;
                            }
                            if(found) break;
                        }
                    }
                    if(attempt.edge >= 0) mesh.edges[attempt.edge].classification = boundaryMarker;
                    else result.needsRetry = true;
                    result.attempts.push_back(std::move(attempt));
                }
            }
            for(const auto& entry : attempted) result.attemptedBoundaryCurves.push_back(entry.second);
        }
        check(); require(validateBdsTopology(mesh,control)); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
