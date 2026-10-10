#pragma once
/** @file GmshFrontPointInsertionJson.h
 * @brief 单次实际insertAPoint查询/槽/身份/拓扑输出；不执行外层选点或参数域门控。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontInsertion.h"
#include <cmath>
#include <ostream>
#include <stdexcept>
inline void writeFrontPointInsertionJson(std::ostream& out,const mesher::gmshReference::FrontPointInsertionResult& result,
    const mesher::gmshReference::FrontMesh& front,const mesher::gmshReference::FrontQueueState& queue,
    const mesher::gmshReference::FaceMesh& mesh,const mesher::gmshReference::NodeSizeState& nodes,double modelLength,bool includeRuntime = true)
{
    using namespace mesher::gmshReference;
    if(!result.status.complete()) throw std::runtime_error(result.status.message);
    auto number = [&](double value) { if(std::isfinite(value)) out << value; else if(std::isnan(value)) out << "\"nan\""; else out << (value < 0. ? "\"-inf\"" : "\"inf\""); };
    out << "{\"inserted\":" << (result.inserted ? "true" : "false") << ",\"code\":" << result.nativeCode << ",\"model_length\":" << modelLength;
    out << ",\"cavity\":["; for(std::size_t i = 0; i < result.cavity.triangles.size(); ++i) { if(i) out << ','; out << result.cavity.triangles[i]; }
    out << "],\"shell\":["; for(std::size_t i = 0; i < result.cavity.shell.size(); ++i) { if(i) out << ','; const auto& e = result.cavity.shell[i]; out << '[' << e.triangle << ',' << e.edge << ']'; }
    out << "],\"containing\":" << result.cavity.containingTriangle << ",\"coordinates\":[";
    if(result.cavity.containingTriangle >= 0) out << result.cavity.coordinates.local.x << ',' << result.cavity.coordinates.local.y; else out << "0,0";
    out << "],\"point_queried\":" << (result.pointQueried ? "true" : "false") << ",\"background_queried\":" << (result.backgroundQueried ? "true" : "false");
    out << ",\"new_vertex\":";
    if(result.allocatedVertex) out << '[' << result.allocatedVertex << ',' << result.surface.point.x << ',' << result.surface.point.y << ',' << result.surface.point.z << ",2," << mesh.entity << ']'; else out << "null";
    out << ",\"new_slot\":";
    if(result.nodeAppended) out << '[' << result.node.vertex << ',' << result.node.parameters.x << ',' << result.node.parameters.y << ',' << result.node.sizes.meshSize << ',' << result.node.sizes.backgroundSize << ',' << result.parameterIndex << ']'; else out << "null";
    if(includeRuntime) {
    out << ",\"slots\":["; for(std::size_t i = 0; i < nodes.nodes.size(); ++i) { if(i) out << ','; const auto& n = nodes.nodes[i]; out << '[' << n.vertex << ',' << n.parameters.x << ',' << n.parameters.y << ',' << n.sizes.meshSize << ',' << n.sizes.backgroundSize << ']'; }
    out << "],\"live_indices\":["; std::size_t row = 0; for(const auto& entry : nodes.indices) { if(row++) out << ','; out << '[' << entry.first << ',' << entry.second << ']'; }
    out << "],\"maximum_vertex_id\":" << mesh.maximumVertexId << ",\"interior_appended\":" << int(result.inserted);
    }
    const auto& v = result.triangles.validation;
    out << ",\"validation\":[" << v.oldArea << ',' << v.newArea << ',' << v.relativeAreaTolerance << ',' << (v.tooClose ? "true" : "false") << "],\"created\":[";
    for(std::size_t i = 0; i < v.triangles.size(); ++i) {
        if(i) out << ','; const auto& c = v.triangles[i]; const auto& t = c.triangle;
        const double values[] = {double(t.vertices[0]),double(t.vertices[1]),double(t.vertices[2]),t.meanSizes.meshSize,t.meanSizes.backgroundSize,t.targetSize,t.radius,
            c.firstDistance,c.secondDistance,c.edgeLength,c.boundaryHeight,c.angleCosine,c.parameterArea,double(c.tooClose)};
        out << '['; for(int j = 0; j < 14; ++j) { if(j) out << ','; number(values[j]); } out << ']';
    }
    out << ']';
    if(includeRuntime) {
    out << ",\"pool\":[";
    for(std::size_t i = 0; i < front.triangles.size(); ++i) {
        if(i) out << ','; const auto& t = front.triangles[i]; out << '[';
        for(auto id : t.vertices) out << id << ','; number(t.radius); out << ',' << int(t.deleted);
        for(int neighbor : t.neighbors) out << ',' << neighbor; out << ']';
    }
    auto set = [&](const FrontTriangleSet& items) { out << '['; std::size_t i = 0; for(const auto& key : items) { if(i++) out << ','; out << key.triangle; } out << ']'; };
    out << "],\"all\":"; set(queue.allTriangles); out << ",\"active\":"; set(queue.activeTriangles);
    }
    out << ",\"neighborhood\":[";
    for(std::size_t i = 0; i < result.triangles.commit.neighborhood.size(); ++i) { if(i) out << ','; out << result.triangles.commit.neighborhood[i]; }
    out << "]}";
}
