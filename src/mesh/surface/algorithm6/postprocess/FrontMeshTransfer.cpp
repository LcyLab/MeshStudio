/** @file FrontMeshTransfer.cpp
 * @brief 前沿结束后的面连接转移。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontMeshTransfer.h"
#include <algorithm>
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 检查前沿转移阶段是否被取消。
void checkTransferControl(const RunControl& control) {
    if(control.keepGoing && !control.keepGoing())
        throw OperationFailure{{MeshingStatus::Cancelled,"front mesh transfer cancelled"}};
}
/// 按节点身份读取转移所需的物理或参数坐标。
Point3D transferPoint(std::uint64_t id,const FaceMesh& mesh,const NodeSizeState& nodes,bool physical) {
    const auto vertex = mesh.vertices.find(id);
    if(vertex == mesh.vertices.end() || vertex->second.id != id)
        throw OperationFailure{{MeshingStatus::InvalidTopology,"transfer face vertex is missing"}};
    Point3D point;
    if(physical) point = vertex->second.point;
    else {
        const auto index = nodes.indices.find(id);
        if(index == nodes.indices.end() || index->second >= nodes.nodes.size() || nodes.nodes[index->second].vertex != id)
            throw OperationFailure{{MeshingStatus::InvalidTopology,"transfer runtime UV index is invalid"}};
        const auto uv = nodes.nodes[index->second].parameters; point = {uv.x,uv.y,0.};
    }
    if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        throw OperationFailure{{MeshingStatus::InvalidInput,"transfer coordinate is nonfinite"}};
    return point;
}
/// 计算转移三角形的法向，供方向一致性判定使用。
Point3D computeTransferNormal(const std::array<std::uint64_t,3>& triangle,
    const FaceMesh& mesh,const NodeSizeState& nodes,bool physical) {
    const auto p0 = transferPoint(triangle[0],mesh,nodes,physical);
    const auto p1 = transferPoint(triangle[1],mesh,nodes,physical);
    const auto p2 = transferPoint(triangle[2],mesh,nodes,physical);
    const double a[] = {p1.x-p0.x,p1.y-p0.y,p1.z-p0.z};
    const double b[] = {p2.x-p0.x,p2.y-p0.y,p2.z-p0.z};
    // 保留Numeric::prodve/norme的运算顺序及零模长，不使用robust符号或hypot。
    Point3D normal{a[1]*b[2]-a[2]*b[1],-a[0]*b[2]+a[2]*b[0],a[0]*b[1]-a[1]*b[0]};
    const double length = std::sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
    if(length != 0.) { const double reciprocal = 1./length; normal.x *= reciprocal; normal.y *= reciprocal; normal.z *= reciprocal; }
    if(!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z))
        throw OperationFailure{{MeshingStatus::NonConverged,"transfer normal arithmetic is nonfinite"}};
    return normal;
}
/// 应用一次节点等价映射并检查映射后的面节点。
std::uint64_t mapEquivalentVertexOnce(std::uint64_t id,const FaceMesh& mesh,
    const std::map<std::uint64_t,std::uint64_t>& equivalence) {
    const auto mapped = equivalence.find(id); const auto result = mapped == equivalence.end() ? id : mapped->second;
    const auto vertex = mesh.vertices.find(result);
    if(vertex == mesh.vertices.end() || vertex->second.id != result)
        throw OperationFailure{{MeshingStatus::InvalidTopology,"transfer equivalence destination is missing"}};
    return result;
}
}
/// 将前沿三角形转移到面网格，处理等价节点及方向。
FrontMeshTransferResult transferFrontMesh(const FrontMesh& front,const FrontQueueState& queue,
    const FaceMesh& input,const NodeSizeState& nodes,const FrontMeshTransferOptions& options,FaceMesh& output,const RunControl& control) {
    FrontMeshTransferResult result;
    try {
        checkTransferControl(control);
        if(input.vertices.size() > control.maximumNodes)
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"transfer face vertex budget exceeded"}};
        FaceMesh mesh = input; mesh.triangles.clear(); std::vector<int> retained;
        for(const auto& key : queue.allTriangles) {
            checkTransferControl(control);
            if(key.triangle < 0 || std::size_t(key.triangle) >= front.triangles.size())
                throw OperationFailure{{MeshingStatus::InvalidTopology,"transfer triangle pool index is invalid"}};
            const auto& triangle = front.triangles[key.triangle]; const auto current = makeFrontTriangleKey(triangle,key.triangle);
            if(key.radius != current.radius || key.vertices != current.vertices)
                throw OperationFailure{{MeshingStatus::InvalidTopology,"transfer AllTris key is stale"}};
            result.consumedTriangles.push_back(key.triangle);
            if(triangle.deleted) { result.skippedDeletedTriangles.push_back(key.triangle); continue; }
            for(auto id : triangle.vertices) transferPoint(id,input,nodes,options.usePhysicalNormals);
            mesh.triangles.push_back(triangle.vertices); retained.push_back(key.triangle);
        }
        if(mesh.triangles.size() > 1) {
            const auto first = computeTransferNormal(mesh.triangles.front(),input,nodes,options.usePhysicalNormals);
            for(std::size_t i = 1; i < mesh.triangles.size(); ++i) {
                checkTransferControl(control);
                const auto normal = computeTransferNormal(mesh.triangles[i],input,nodes,options.usePhysicalNormals);
                if(first.x*normal.x+first.y*normal.y+first.z*normal.z < 0.) {
                    std::swap(mesh.triangles[i][1],mesh.triangles[i][2]); result.reversedTriangles.push_back(retained[i]);
                }
            }
        }
        if(options.equivalence) {
            result.equivalenceDiagnosticsEvaluated = true;
            std::map<std::array<std::uint64_t,3>,int> firstEquivalent;
            std::vector<std::array<std::uint64_t,3>> mappedTriangles; mappedTriangles.reserve(mesh.triangles.size());
            for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
                checkTransferControl(control); auto mapped = mesh.triangles[i];
                for(auto& id : mapped) id = mapEquivalentVertexOnce(id,input,*options.equivalence);
                auto key = mapped; std::sort(key.begin(),key.end());
                const auto inserted = firstEquivalent.emplace(key,retained[i]);
                if(!inserted.second) result.equivalentTriangles.push_back({inserted.first->second,retained[i]});
                // 原splitEquivalentTriangles只诊断，computeEquivalences随后只丢身份退化片。
                if(mapped[0] == mapped[1] || mapped[0] == mapped[2] || mapped[1] == mapped[2]) result.collapsedTriangles.push_back(retained[i]);
                else mappedTriangles.push_back(mapped);
            }
            mesh.triangles = std::move(mappedTriangles);
        }
        checkTransferControl(control); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
