#pragma once
/** @file GmshInitialPoolJson.h
 * @brief 对照端稳定池序列化；不参与生产算法。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/initial/WalkTriangulation.h"
#include <ostream>
inline void writeInitialPoolFields(std::ostream& out, const mesher::gmshReference::WalkTriangulation& mesh)
{
    out << "\"vertices\":[";
    for(std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        if(i) out << ','; const auto& v = mesh.vertices[i];
        out << '[' << v.physicalId << ',' << v.uv.x << ',' << v.uv.y << ',' << v.incidentHalfEdge << ']';
    }
    out << "],\"half_edges\":[";
    for(std::size_t i = 0; i < mesh.halfEdges.size(); ++i) {
        if(i) out << ','; const auto& e = mesh.halfEdges[i];
        out << '[' << e.origin << ',' << e.face << ',' << e.previous << ',' << e.next << ',' << e.opposite << ',' << e.curveEntity << ']';
    }
    out << "],\"faces\":[";
    for(std::size_t i = 0; i < mesh.faces.size(); ++i) { if(i) out << ','; out << '[' << mesh.faces[i].halfEdge << ',' << mesh.faces[i].domain << ']'; }
    out << ']';
}
