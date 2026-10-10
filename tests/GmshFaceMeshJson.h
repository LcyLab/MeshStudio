#pragma once
/** @file GmshFaceMeshJson.h
 * @brief 前沿前节点身份/UV/三角形的只读测试序列化。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/postprocess/FaceMeshConversion.h"
#include <ostream>
inline void writeFaceMeshJson(std::ostream& out,const mesher::gmshReference::FaceMesh& mesh,
    const mesher::gmshReference::FaceMeshConversionResult& result)
{
    out << "{\"entity\":" << mesh.entity << ",\"maximum_id\":" << mesh.maximumVertexId << ",\"vertices\":[";
    std::size_t row = 0;
    for(const auto& entry : mesh.vertices) {
        if(row++) out << ','; const auto& p = entry.second;
        out << '[' << p.id << ',' << p.physicalId << ',' << int(p.origin) << ',' << p.dimension << ',' << p.entity << ','
            << p.point.x << ',' << p.point.y << ',' << p.point.z << ',' << p.parameters.x << ',' << p.parameters.y << ',' << p.storedSize << ']';
    }
    auto list = [&](const auto& values) { out << '['; for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; out << values[i]; } out << ']'; };
    auto mapping = [&](const char* name,const auto& values) {
        out << ",\"" << name << "\":["; std::size_t n = 0;
        for(const auto& p : values) { if(n++) out << ','; out << '[' << p.first << ',' << p.second << ']'; } out << ']';
    };
    out << ']'; mapping("bds_to_vertex",mesh.bdsToVertex); mapping("equivalence",mesh.equivalence);
    out << ",\"parametric_coordinates\":["; row = 0;
    for(const auto& p : mesh.parametricCoordinates) { if(row++) out << ','; out << '[' << p.first << ',' << p.second.x << ',' << p.second.y << ']'; }
    out << "],\"unreconstructed_seams\":"; list(mesh.unreconstructedSeams);
    out << ",\"interior_vertices\":"; list(mesh.interiorVertices); out << ",\"triangles\":[";
    for(std::size_t i = 0; i < mesh.triangles.size(); ++i) { if(i) out << ','; list(mesh.triangles[i]); }
    out << "],\"conversion\":{\"skipped_deleted\":"; list(result.skippedDeletedTriangles);
    out << ",\"collapsed\":"; list(result.collapsedTriangles); out << ",\"created_local_nodes\":"; list(result.createdLocalNodes); out << "}}";
}
