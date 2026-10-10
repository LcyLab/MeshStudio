#pragma once
/** @file GmshFrontCavityJson.h
 * @brief 空腔/包含片/重心尺寸与显式标记恢复的测试序列化；不提交新片。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontCavity.h"
#include <ostream>
#include <stdexcept>
inline void writeFrontCavityJson(std::ostream& out,mesher::gmshReference::FrontMesh& mesh,
    const mesher::gmshReference::NodeSizeState& nodes,const mesher::gmshReference::FrontTriangleSet& all,
    int worst,mesher::Point2D point,mesher::gmshReference::SurfaceMetric metric,bool force = false)
{
    using namespace mesher::gmshReference;
    const auto cavity = buildFrontCavity(mesh,nodes,all,worst,point,metric,force);
    if(!cavity.status.complete()) throw std::runtime_error(cavity.status.message);
    auto boolean = [&](bool value) { out << (value ? "true" : "false"); };
    auto list = [&](const std::vector<int>& values) { out << '['; for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; out << values[i]; } out << ']'; };
    out << "{\"worst_inside\":"; boolean(cavity.worstInsideCircle); out << ",\"containing\":" << cavity.containingTriangle;
    const auto& coordinates = cavity.coordinates;
    out << ",\"coordinates\":[" << coordinates.local.x << ',' << coordinates.local.y << ','; boolean(coordinates.inside); out << ','; boolean(coordinates.singularSystem); out << ']';
    out << ",\"cavity\":"; list(cavity.triangles); out << ",\"shell\":[";
    for(std::size_t i = 0; i < cavity.shell.size(); ++i) { if(i) out << ','; out << '[' << cavity.shell[i].triangle << ',' << cavity.shell[i].edge << ']'; }
    out << "],\"circles\":[";
    for(std::size_t i = 0; i < cavity.circleTests.size(); ++i) {
        if(i) out << ','; const auto& c = cavity.circleTests[i];
        out << '[' << c.triangle << ',' << c.circle.center.x << ',' << c.circle.center.y << ',' << c.circle.radiusSquared << ',' << c.distanceSquared << ',' << double(c.inside) << ']';
    }
    out << "],\"maps\":[";
    for(std::size_t i = 0; i < cavity.coordinateTests.size(); ++i) {
        if(i) out << ','; const auto& entry = cavity.coordinateTests[i]; const auto& c = entry.second;
        out << '[' << entry.first << ',' << c.local.x << ',' << c.local.y << ',' << double(c.inside) << ',' << double(c.singularSystem) << ']';
    }
    out << "],\"search\":{\"visits\":"; list(cavity.search.visitedTriangles); out << ",\"crossed\":"; list(cavity.search.crossedEdges);
    out << ",\"force_scanned\":"; list(cavity.search.forceScannedTriangles); out << ",\"no_edge\":"; boolean(cavity.search.noCrossedEdge); out << ",\"limit\":"; boolean(cavity.search.iterationLimit); out << '}';
    out << ",\"mesh_size\":";
    if(cavity.containingTriangle >= 0) {
        double size; const auto status = interpolateNodeMeshSize(mesh.triangles.at(cavity.containingTriangle),nodes,coordinates.local,size);
        if(!status.complete()) throw std::runtime_error(status.message); out << size;
    } else out << "null";
    out << ",\"deleted\":["; for(std::size_t i = 0; i < mesh.triangles.size(); ++i) { if(i) out << ','; boolean(mesh.triangles[i].deleted); }
    const auto restored = rollbackFrontCavity(mesh,cavity); if(!restored.complete()) throw std::runtime_error(restored.message);
    out << "],\"rolled_back\":["; for(std::size_t i = 0; i < mesh.triangles.size(); ++i) { if(i) out << ','; boolean(mesh.triangles[i].deleted); } out << "]}";
}
