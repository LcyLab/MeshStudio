#pragma once
/** @file GmshBdsMetricJson.h
 * @brief 普通/周期度量合法化的测试快照；不查询CAD或决定翻边。
 * @date 2026-10-08
 */
#include "GmshBdsPoolJson.h"
#include "mesh/surface/algorithm6/bds/InitialMeshLegalization.h"
inline void writeBdsMetricJson(std::ostream& out,const mesher::gmshReference::BdsMesh& mesh,
    const mesher::gmshReference::InitialMeshLegalizationResult& result)
{
    using namespace mesher::gmshReference;
    out << "{\"bds\":"; writeBdsPoolJson(out,mesh,true);
    out << ",\"swaps\":" << result.swaps << ",\"metric_queries\":" << result.metricQueries << ",\"configurations\":[";
    auto configuration = [&](const std::array<int,4>& q) { out << '[' << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << ']'; };
    bool first = true; for(const auto& q : result.configurations) { if(!first) out << ','; first = false; configuration(q); }
    out << "],\"scans\":[";
    for(std::size_t i = 0; i < result.scans.size(); ++i) {
        if(i) out << ','; const auto& scan = result.scans[i];
        out << "{\"starting_edges\":" << scan.startingEdges << ",\"ending_edges\":" << scan.endingEdges << ",\"swaps\":" << scan.swaps << ",\"attempts\":[";
        for(std::size_t j = 0; j < scan.attempts.size(); ++j) {
            if(j) out << ','; const auto& a = scan.attempts[j];
            out << "{\"edge\":" << a.edge << ",\"decision\":" << int(a.decision) << ",\"configuration\":";
            if(a.configuration) configuration(*a.configuration); else out << "null";
            out << ",\"metric_query\":";
            if(a.decision >= InitialEdgeDecision::OutsideCircle) {
                out << "{\"uv\":[" << a.midpoint.x << ',' << a.midpoint.y << "],\"metric\":[" << a.metric.gUU << ',' << a.metric.gUV << ',' << a.metric.gVV
                    << "],\"circle\":[" << a.circle.circle.center.x << ',' << a.circle.circle.center.y << ',' << a.circle.circle.radiusSquared << ',' << a.circle.queryDistanceSquared << ',' << a.circle.tolerance
                    << "],\"inside\":" << (a.circle.inside ? "true" : "false") << '}';
            } else out << "null";
            out << '}';
        }
        out << "]}";
    }
    out << "]}";
}
