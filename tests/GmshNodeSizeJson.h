#pragma once
/** @file GmshNodeSizeJson.h
 * @brief 前沿节点尺寸、索引、约束与实际查询的只读测试序列化。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/NodeSizeInitialization.h"
#include <ostream>
inline void writeNodeSizeJson(std::ostream& out,const mesher::gmshReference::NodeSizeState& state,
    const mesher::gmshReference::NodeSizeInitializationResult& result)
{
    out << "{\"nodes\":[";
    for(std::size_t i = 0; i < state.nodes.size(); ++i) {
        if(i) out << ','; const auto& n = state.nodes[i];
        out << '[' << n.vertex << ',' << n.parameters.x << ',' << n.parameters.y << ',' << n.sizes.meshSize << ',' << n.sizes.backgroundSize << ']';
    }
    auto mapping = [&](const char* name,const auto& values) {
        out << ",\"" << name << "\":["; std::size_t row = 0;
        for(const auto& e : values) { if(row++) out << ','; out << '[' << e.first << ',' << e.second << ']'; } out << ']';
    };
    out << ']'; mapping("indices",state.indices); out << ",\"internal_edges\":["; std::size_t row = 0;
    for(const auto& e : state.internalEdges) { if(row++) out << ','; out << '[' << e[0] << ',' << e[1] << ']'; }
    out << "],\"reparameterizations\":[";
    for(std::size_t i = 0; i < result.reparameterizations.size(); ++i) {
        if(i) out << ','; const auto& q = result.reparameterizations[i]; out << '[' << q.vertex << ',' << q.parameters.x << ',' << q.parameters.y << ']';
    }
    out << "],\"embedded_queries\":[";
    for(std::size_t i = 0; i < result.embeddedQueries.size(); ++i) {
        if(i) out << ','; const auto& e = result.embeddedQueries[i]; const auto& q = e.query;
        out << '[' << e.vertex << ',' << q.dimension << ',' << q.entity << ',' << q.u << ',' << q.v << ',' << q.point.x << ',' << q.point.y << ',' << q.point.z << ',' << e.evaluation.size << ']';
    }
    out << "]}";
}
