#pragma once
/** @file GmshSurfaceSmoothingJson.h
 * @brief 平滑事件和最终面网格的只读输出，不参与几何或验收。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/postprocess/SurfaceSmoothing.h"
#include <ostream>
#include <stdexcept>
inline void writeSurfaceSmoothingJson(std::ostream& out,const mesher::gmshReference::SurfaceSmoothingResult& result,
    const std::vector<mesher::gmshReference::SmoothingVertexEvent>& events,const mesher::gmshReference::FaceMesh& mesh) {
    if(!result.status.complete()) throw std::runtime_error(result.status.message);
    auto uv = [&](mesher::Point2D p) { out << '[' << p.x << ',' << p.y << ']'; };
    auto xyz = [&](mesher::Point3D p) { out << '[' << p.x << ',' << p.y << ',' << p.z << ']'; };
    out << "{\"visits\":" << result.vertexVisits << ",\"eligible\":" << result.eligibleVertices
        << ",\"writes\":" << result.acceptedWrites << ",\"queries\":" << result.geometryEvaluations << ",\"events\":[";
    for(std::size_t i = 0; i < events.size(); ++i) {
        if(i) out << ','; const auto& e = events[i];
        out << "{\"iteration\":" << e.iteration << ",\"vertex\":" << e.vertex << ",\"skip\":" << int(e.skip) << ",\"before\":"; uv(e.before);
        out << ",\"position\":[" << e.position.position.x << ',' << e.position.position.y << ',' << e.position.totalWeight << "],\"neighbors\":[";
        for(std::size_t j = 0; j < e.position.neighbors.size(); ++j) {
            if(j) out << ','; const auto& n = e.position.neighbors[j];
            out << '[' << n.vertex << ',' << n.parameters.x << ',' << n.parameters.y << ',' << n.direction.x << ',' << n.direction.y << ',' << n.metric.gUU << ',' << n.metric.gUV << ',' << n.metric.gVV << ',' << n.weight << ']';
        }
        out << "],\"trials\":[";
        for(std::size_t j = 0; j < e.trials.size(); ++j) {
            if(j) out << ','; const auto& t = e.trials[j]; const auto& a = t.acceptance;
            out << '[' << t.factor << ','; uv(t.trial); out << ',' << (a.accepted ? "true" : "false") << ',' << (a.surface.succeeded ? "true" : "false") << ','; xyz(a.surface.point);
            out << ',' << a.oldArea << ',' << a.newArea << ',' << a.oldMinimumEta << ',' << a.newMinimumEta << ',' << (t.published ? "true" : "false") << ',' << (t.publicationPoint.succeeded ? "true" : "false") << ','; xyz(t.publicationPoint.point);
            out << ','; uv(t.actual); out << ',' << a.geometryEvaluations << ']';
        }
        out << "]}";
    }
    out << "],\"vertices\":["; std::size_t i = 0;
    for(const auto& entry : mesh.vertices) {
        if(i++) out << ','; const auto& v = entry.second;
        out << '[' << v.id << ',' << v.parameters.x << ',' << v.parameters.y << ',' << v.point.x << ',' << v.point.y << ',' << v.point.z << ']';
    }
    out << "],\"triangles\":[";
    for(std::size_t j = 0; j < mesh.triangles.size(); ++j) { if(j) out << ','; const auto& t = mesh.triangles[j]; out << '[' << t[0] << ',' << t[1] << ',' << t[2] << ']'; } out << "]}";
}
