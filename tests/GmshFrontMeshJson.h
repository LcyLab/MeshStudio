#pragma once
/** @file GmshFrontMeshJson.h
 * @brief 初始MTri3池、半径排序及原边邻接的只读测试序列化。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontTriangleInitialization.h"
#include <ostream>
inline void writeFrontMeshJson(std::ostream& out,const mesher::gmshReference::FrontMesh& mesh,
    const mesher::gmshReference::FrontTriangleInitializationResult& result)
{
    out << "{\"triangles\":[";
    for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        if(i) out << ','; const auto& t = mesh.triangles[i];
        out << '[' << t.vertices[0] << ',' << t.vertices[1] << ',' << t.vertices[2]
            << ',' << t.meanSizes.meshSize << ',' << t.meanSizes.backgroundSize << ',' << t.targetSize << ',' << t.radius;
        for(int neighbor : t.neighbors) out << ',' << neighbor;
        out << ',' << (t.deleted ? "true" : "false") << ']';
    }
    auto list = [&](const char* name,const auto& values) {
        out << "],\"" << name << "\":[";
        for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; out << values[i]; }
    };
    list("ordered",mesh.orderedTriangles); list("rejected_duplicates",result.rejectedDuplicateTriangles);
    out << "]}";
}
