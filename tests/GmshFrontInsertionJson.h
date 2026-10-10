#pragma once
/** @file GmshFrontInsertionJson.h
 * @brief 冻结已分配节点的insertVertexB叶测试输出；无CAD/BGM或insertAPoint生命周期。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontInsertion.h"
#include <cmath>
#include <ostream>
#include <stdexcept>
inline void writeFrontInsertionJson(std::ostream& out,mesher::gmshReference::FrontMesh& front,
    mesher::gmshReference::FrontQueueState& queue,const mesher::gmshReference::FaceMesh& mesh,
    const mesher::gmshReference::NodeSizeState& nodes,const mesher::gmshReference::FrontCavityResult& cavity,
    const mesher::gmshReference::FaceVertex& vertex,const mesher::gmshReference::InitializedNodeSize& node,
    const mesher::gmshReference::FrontInsertionOptions& options)
{
    using namespace mesher::gmshReference;
    const auto result = insertFrontTriangles(front,queue,mesh,nodes,cavity,vertex,node,options);
    if(!result.status.complete()) throw std::runtime_error(result.status.message);
    const auto& v = result.validation;
    auto number = [&](double value) { if(std::isfinite(value)) out << value; else if(std::isnan(value)) out << "\"nan\""; else out << (value < 0. ? "\"-inf\"" : "\"inf\""); };
    out << "{\"code\":" << result.nativeCode << ",\"validation\":[" << v.oldArea << ',' << v.newArea << ',' << v.relativeAreaTolerance << ',' << (v.tooClose ? "true" : "false") << "],\"created\":[";
    for(std::size_t i = 0; i < v.triangles.size(); ++i) {
        if(i) out << ','; const auto& c = v.triangles[i]; const auto& t = c.triangle;
        const double row[] = {double(t.vertices[0]),double(t.vertices[1]),double(t.vertices[2]),t.meanSizes.meshSize,t.meanSizes.backgroundSize,t.targetSize,t.radius,
            c.firstDistance,c.secondDistance,c.edgeLength,c.boundaryHeight,c.angleCosine,c.parameterArea,double(c.tooClose)};
        out << '['; for(int j = 0; j < 14; ++j) { if(j) out << ','; number(row[j]); } out << ']';
    }
    out << "],\"pool\":[";
    for(std::size_t i = 0; i < front.triangles.size(); ++i) {
        if(i) out << ','; const auto& t = front.triangles[i]; out << '[';
        for(auto id : t.vertices) out << id << ','; number(t.radius); out << ',' << int(t.deleted);
        for(int neighbor : t.neighbors) out << ',' << neighbor; out << ']';
    }
    auto set = [&](const FrontTriangleSet& items) { out << '['; std::size_t i = 0; for(const auto& key : items) { if(i++) out << ','; out << key.triangle; } out << ']'; };
    out << "],\"all\":"; set(queue.allTriangles); out << ",\"active\":"; set(queue.activeTriangles); out << ",\"neighborhood\":[";
    for(std::size_t i = 0; i < result.commit.neighborhood.size(); ++i) { if(i) out << ','; out << result.commit.neighborhood[i]; }
    out << "]}";
}
