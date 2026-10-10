#pragma once
/** @file GmshFrontGenerationJson.h
 * @brief 动态生成观察共用输出；只记录完成轮次，不控制生成决策。
 * @date 2026-10-08
 */
#include "GmshFrontPointInsertionJson.h"
#include "mesh/surface/algorithm6/front/FrontGeneration.h"
#include <iomanip>
#include <locale>
#include <sstream>
inline void writeFrontGenerationCandidateJson(std::ostream& out,const mesher::gmshReference::FrontPointCandidate& c,
    const std::vector<std::array<double,5>>& queries)
{
    const auto count = c.geometryEvaluations-1;
    if(count > queries.size()) throw std::runtime_error("candidate query count exceeds captured queries");
    const double values[] = {c.metricQuery.x,c.metricQuery.y,c.planarCenter.x,c.planarCenter.y,c.metricCenter.x,c.metricCenter.y,
        c.edgeMidpoint.x,c.edgeMidpoint.y,c.fastPoint.x,c.fastPoint.y,c.metric.gUU,c.metric.gUV,c.metric.gVV,
        c.edgeMeanSizes.meshSize,c.edgeMeanSizes.backgroundSize,c.targetSize,c.distanceToCenter,c.height,c.directionNorm,c.metricRatio,
        c.circle.center.x,c.circle.center.y,c.circle.center.z,c.circle.cosineAxis.x,c.circle.cosineAxis.y,c.circle.cosineAxis.z,
        c.circle.sineAxis.x,c.circle.sineAxis.y,c.circle.sineAxis.z,c.circle.radius,c.normalNorm,double(c.skippedDegenerateNormal),
        double(c.correctionAttempted),double(c.intersectionFound),c.point.x,c.point.y,1.,double(count)};
    out << "{\"status\":0,\"values\":["; for(int j = 0; j < 38; ++j) { if(j) out << ','; out << values[j]; }
    out << "],\"queries\":["; for(std::size_t i = 0; i < count; ++i) { if(i) out << ','; out << '['; for(int j = 0; j < 5; ++j) { if(j) out << ','; out << queries[i][j]; } out << ']'; } out << "]}";
}
inline mesher::gmshReference::FrontGenerationResult writeFrontGenerationJson(std::ostream& out,
    mesher::gmshReference::FrontMesh& front,mesher::gmshReference::FrontQueueState& queue,
    mesher::gmshReference::FaceMesh& mesh,mesher::gmshReference::NodeSizeState& nodes,
    const mesher::gmshReference::FrontGenerationSource& source,const mesher::gmshReference::FrontGenerationOptions& options,
    double modelLength,const mesher::gmshReference::RunControl& control = {})
{
    using namespace mesher; using namespace mesher::gmshReference;
    const auto oldInteriorCount = mesh.interiorVertices.size();
    const auto initialAll = queue.allTriangles,initialActive = queue.activeTriangles;
    auto ids = [](const FrontTriangleSet& values) { std::set<int> result; for(const auto& key : values) result.insert(key.triangle); return result; };
    auto previousAll = ids(initialAll),previousActive = ids(initialActive);
    std::vector<std::array<double,5>> queries; auto observed = source;
    observed.metric = [&](Point2D uv) { queries.clear(); return source.metric(uv); };
    observed.surface = [&](Point2D uv) { const auto p = source.surface(uv); queries.push_back({uv.x,uv.y,p.x,p.y,p.z}); return p; };
    std::ostringstream events; events.imbue(std::locale::classic()); events << std::setprecision(17); std::size_t rows = 0;
    auto writeDelta = [&](const FrontTriangleSet& set,std::set<int>& previous) {
        const auto current = ids(set); events << "[["; std::size_t n = 0;
        for(int i : previous) if(!current.count(i)) { if(n++) events << ','; events << i; }
        events << "],["; n = 0; for(int i : current) if(!previous.count(i)) { if(n++) events << ','; events << i; }
        events << "]]"; previous = current;
    };
    const auto result = generateFrontMesh(front,queue,mesh,nodes,observed,options,control,[&](const FrontGenerationEvent& event) {
        if(rows++) events << ','; const auto& s = event.selection;
        events << "{\"selection\":[" << s.triangle << ',' << int(s.deleted) << ',' << int(s.activityEvaluated) << ',' << int(s.active) << ',' << s.edge << ',' << int(s.requiresInsertion) << "],\"candidate\":";
        if(event.candidate) writeFrontGenerationCandidateJson(events,*event.candidate,queries); else events << "null";
        events << ",\"domain_tested\":" << (event.domainTested ? "true" : "false") << ",\"domain\":[" << (event.domain.inside ? "true" : "false") << ',' << event.domain.crossingCount << "],\"insertion\":";
        if(event.insertionAttempted) writeFrontPointInsertionJson(events,event.insertion,front,queue,mesh,nodes,modelLength,false); else events << "null";
        events << ",\"all_delta\":"; writeDelta(queue.allTriangles,previousAll); events << ",\"active_delta\":"; writeDelta(queue.activeTriangles,previousActive); events << '}';
    });
    if(!result.status.complete()) throw OperationFailure{result.status};
    auto writeSet = [&](const FrontTriangleSet& set) { out << '['; std::size_t i = 0; for(const auto& key : set) { if(i++) out << ','; out << key.triangle; } out << ']'; };
    out << "{\"far\":[" << result.farPoint.x << ',' << result.farPoint.y << "],\"initial_all\":"; writeSet(initialAll); out << ",\"initial_active\":"; writeSet(initialActive);
    out << ",\"events\":[" << events.str() << "],\"stats\":[" << result.pops << ',' << result.candidates << ',' << result.insertionAttempts << ',' << result.inserted << ',' << result.domainRejected << ',' << result.geometryEvaluations << "],\"final\":";
    FrontPointInsertionResult snapshot; writeFrontPointInsertionJson(out,snapshot,front,queue,mesh,nodes,modelLength);
    out << ",\"interiors\":["; for(std::size_t i = oldInteriorCount; i < mesh.interiorVertices.size(); ++i) { if(i > oldInteriorCount) out << ','; out << mesh.interiorVertices[i]; }
    out << "],\"vertices\":["; std::size_t row = 0; for(const auto& entry : nodes.indices) { if(row++) out << ','; const auto& v = mesh.vertices.at(entry.first); out << '[' << v.id << ',' << v.point.x << ',' << v.point.y << ',' << v.point.z << ',' << v.dimension << ']'; } out << "]}";
    return result;
}
