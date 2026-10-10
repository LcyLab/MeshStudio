#pragma once
/** @file GmshFrontQueueJson.h
 * @brief 初始活动集/逐次pop与显式状态操作的测试序列化；不执行候选/插点。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontQueue.h"
#include <ostream>
#include <stdexcept>
inline void writeFrontQueueJson(std::ostream& out,mesher::gmshReference::FrontMesh mesh,
    mesher::gmshReference::FrontQueueState queue,const std::vector<std::array<double,3>>& operations = {})
{
    using namespace mesher::gmshReference;
    auto require = [](const OperationStatus& status) { if(!status.complete()) throw std::runtime_error(status.message); };
    auto list = [&](const FrontTriangleSet& values) { out << '['; std::size_t row = 0; for(const auto& key : values) { if(row++) out << ','; out << key.triangle; } out << ']'; };
    auto boolean = [&](bool value) { out << (value ? "true" : "false"); };
    auto pop = [&] {
        const auto selected = popActiveTriangle(mesh,queue); require(selected.status);
        out << '[' << selected.triangle << ','; boolean(selected.deleted); out << ','; boolean(selected.activityEvaluated); out << ','; boolean(selected.active);
        out << ',' << selected.edge << ','; boolean(selected.requiresInsertion); out << ']';
    };
    out << "{\"limit\":" << queue.limit << ",\"initial_all\":"; list(queue.allTriangles); out << ",\"initial_active\":"; list(queue.activeTriangles);
    out << ",\"operations\":[";
    for(std::size_t i = 0; i < operations.size(); ++i) {
        if(i) out << ','; const int kind = static_cast<int>(operations[i][0]),index = static_cast<int>(operations[i][1]); const double parameter = operations[i][2];
        out << "{\"result\":";
        if(kind == 1) pop();
        else {
            out << '[';
            if(kind == 0) { const auto activity = findActiveEdge(mesh,index,queue.limit,static_cast<int>(parameter)); require(activity.status); boolean(activity.active); out << ',' << activity.edge; }
            else if(kind == 2) { const auto result = markInsertionFailure(mesh,queue,index,parameter == 0. ? FrontInsertionFailure::ContainingTriangleNotFound : FrontInsertionFailure::PointInsertionRejected); require(result.status); boolean(result.reinserted); }
            else if(kind == 3) { mesh.triangles.at(index).deleted = parameter != 0.; boolean(mesh.triangles.at(index).deleted); }
            else if(kind == 4) { const auto result = updateActiveTriangles(mesh,{index},queue); require(result.status); boolean(!result.inserted.empty()); }
            else if(kind == 5) { const auto result = registerFrontTriangles(mesh,{index},queue); require(result.status); boolean(!result.inserted.empty()); }
            else if(kind == 6) { mesh.triangles.at(index).neighbors.fill(static_cast<int>(parameter)); boolean(true); }
            else throw std::runtime_error("unknown queue operation");
            out << ']';
        }
        out << ",\"all\":"; list(queue.allTriangles); out << ",\"active\":"; list(queue.activeTriangles);
        out << ",\"radii\":["; for(std::size_t k = 0; k < mesh.triangles.size(); ++k) { if(k) out << ','; out << mesh.triangles[k].radius; }
        out << "],\"deleted\":["; for(std::size_t k = 0; k < mesh.triangles.size(); ++k) { if(k) out << ','; boolean(mesh.triangles[k].deleted); } out << "]}";
    }
    out << "],\"pops\":["; std::size_t row = 0; while(!queue.activeTriangles.empty()) { if(row++) out << ','; pop(); }
    out << "],\"all\":"; list(queue.allTriangles); out << ",\"active\":"; list(queue.activeTriangles); out << '}';
}
