#pragma once
/** @file GmshBdsPoolJson.h
 * @brief 只读 BDS 池和原邻接序导出；测试用，不排序活边/面。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
#include <ostream>
inline void writeBdsPoolJson(std::ostream& out,const mesher::gmshReference::BdsMesh& mesh,bool includeOrigins = false)
{
    using namespace mesher::gmshReference;
    out << "{\"maximum_id\":" << mesh.maximumPointId << ",\"points\":[";
    std::size_t row = 0;
    for(const auto& entry : mesh.points) {
        if(row++) out << ','; const auto& p = entry.second;
        out << '[' << p.id << ',' << p.physicalId << ',' << p.uv.x << ',' << p.uv.y << ',' << p.point.x << ',' << p.point.y << ',' << p.point.z
            << ',' << p.classification.tag << ',' << p.classification.dimension << ',' << p.sizes.meshSize << ',' << p.sizes.backgroundSize
            << ',' << int(p.configurationModified) << ',' << p.degenerated << ',';
        if(p.periodicCounterpart) out << *p.periodicCounterpart; else out << "null";
        out << ']';
    }
    auto list = [&](const std::vector<int>& values) { out << '['; for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; out << values[i]; } out << ']'; };
    out << "],\"point_edges\":["; row = 0;
    for(const auto& entry : mesh.points) { if(row++) out << ','; list(entry.second.edges); }
    out << "],\"edges\":[";
    for(std::size_t i = 0; i < mesh.edges.size(); ++i) {
        if(i) out << ','; const auto& e = mesh.edges[i];
        out << '[' << e.first << ',' << e.second << ',' << int(e.deleted) << ',' << e.classification.tag << ',' << e.classification.dimension << ']';
    }
    out << "],\"edge_faces\":[";
    for(std::size_t i = 0; i < mesh.edges.size(); ++i) { if(i) out << ','; list(mesh.edges[i].faces); }
    out << "],\"triangles\":[";
    for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        if(i) out << ','; const auto& t = mesh.triangles[i];
        out << '[' << t.edges[0] << ',' << t.edges[1] << ',' << t.edges[2] << ',' << int(t.deleted) << ',' << t.classification.tag << ',' << t.classification.dimension << ']';
    }
    out << "],\"triangle_nodes\":[";
    for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        if(i) out << ','; const auto nodes = getBdsTriangleNodes(mesh,int(i));
        out << '[' << nodes[0] << ',' << nodes[1] << ',' << nodes[2] << ']';
    }
    out << "],\"geometry\":["; row = 0;
    for(auto g : mesh.geometry) { if(row++) out << ','; out << '[' << g.tag << ',' << g.dimension << ']'; }
    out << ']';
    if(includeOrigins) {
        out << ",\"point_geometry_origin\":["; row = 0;
        for(const auto& entry : mesh.points) { if(row++) out << ','; out << int(entry.second.classification.origin); }
        out << "],\"edge_geometry_origin\":[";
        for(std::size_t i = 0; i < mesh.edges.size(); ++i) { if(i) out << ','; out << int(mesh.edges[i].classification.origin); }
        out << "],\"triangle_geometry_origin\":[";
        for(std::size_t i = 0; i < mesh.triangles.size(); ++i) { if(i) out << ','; out << int(mesh.triangles[i].classification.origin); } out << ']';
    }
    out << '}';
}
