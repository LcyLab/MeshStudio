#pragma once
/** @file GmshFrontMeshTransferJson.h
 * @brief 转移后的面连接及决策只读输出，不执行算法。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/postprocess/FrontMeshTransfer.h"
#include <ostream>
#include <stdexcept>
inline void writeFrontMeshTransferJson(std::ostream& out,const mesher::gmshReference::FrontMeshTransferResult& result,
    const mesher::gmshReference::FaceMesh& mesh) {
    if(!result.status.complete()) throw std::runtime_error(result.status.message);
    auto ints = [&](const std::vector<int>& values) { out << '['; for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; out << values[i]; } out << ']'; };
    out << "{\"consumed\":"; ints(result.consumedTriangles); out << ",\"deleted\":"; ints(result.skippedDeletedTriangles);
    out << ",\"reversed\":"; ints(result.reversedTriangles); out << ",\"collapsed\":"; ints(result.collapsedTriangles);
    out << ",\"equivalence_evaluated\":" << (result.equivalenceDiagnosticsEvaluated ? "true" : "false") << ",\"equivalent\":[";
    for(std::size_t i = 0; i < result.equivalentTriangles.size(); ++i) { if(i) out << ','; out << '[' << result.equivalentTriangles[i][0] << ',' << result.equivalentTriangles[i][1] << ']'; }
    out << "],\"triangles\":["; for(std::size_t i = 0; i < mesh.triangles.size(); ++i) { if(i) out << ','; const auto& ids = mesh.triangles[i]; out << '[' << ids[0] << ',' << ids[1] << ',' << ids[2] << ']'; } out << "]}";
}
