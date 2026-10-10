#pragma once
/** @file GmshPeriodicBdsRecoveryJson.h
 * @brief 周期恢复只读阶段输出；不做恢复计算。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/PeriodicBdsRecovery.h"
#include "GmshBdsPoolJson.h"
inline void writePeriodicBdsRecoveryJson(std::ostream& out,const mesher::gmshReference::BdsMesh& mesh,
    const mesher::gmshReference::PeriodicBdsRecoveryResult& result) {
    out << "{\"bds\":"; writeBdsPoolJson(out,mesh,true);
    out << ",\"embedded_failed\":" << (result.embeddedFailed ? "true" : "false") << ",\"needs_retry\":" << (result.needsRetry ? "true" : "false") << ",\"attempted_boundary_curves\":[";
    for(std::size_t i = 0; i < result.attemptedBoundaryCurves.size(); ++i) { if(i) out << ','; const auto& c = result.attemptedBoundaryCurves[i]; out << '[' << c.first << ',' << c.second << ',' << c.curveEntity << ']'; }
    out << "],\"attempts\":[";
    for(std::size_t i = 0; i < result.attempts.size(); ++i) {
        if(i) out << ','; const auto& a = result.attempts[i];
        out << "{\"embedded\":" << (a.embedded ? "true" : "false") << ",\"loop\":" << a.loop << ",\"segment\":" << a.segment << ",\"first\":" << a.first << ",\"second\":" << a.second << ",\"edge\":" << a.edge << ",\"already_present\":" << (a.alreadyPresent ? "true" : "false") << ",\"curve\":";
        if(a.curveEntity) out << *a.curveEntity; else out << "null";
        out << ",\"recovery\":";
        if(!a.recovery) out << "null";
        else {
            const auto& r = *a.recovery; out << "{\"edge\":" << r.recoveredEdge << ",\"fatal\":" << (r.nativeFatal ? "true" : "false") << ",\"scans\":[";
            for(std::size_t j = 0; j < r.scans.size(); ++j) {
                if(j) out << ','; const auto& s = r.scans[j]; out << "{\"intersected\":[";
                for(std::size_t k = 0; k < s.intersectedEdges.size(); ++k) { if(k) out << ','; out << s.intersectedEdges[k]; }
                out << "],\"attempts\":[";
                for(std::size_t k = 0; k < s.attempts.size(); ++k) { if(k) out << ','; out << '[' << s.attempts[k].first << ',' << (s.attempts[k].second ? "true" : "false") << ']'; } out << "]}";
            }
            out << "]}";
        }
        out << '}';
    }
    out << "]}";
}
