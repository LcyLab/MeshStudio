/** @file FaceMeshConversion.cpp
 * @brief recoverMap视图和BDS三角转换。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FaceMeshConversion.h"
#include <cmath>
#include <exception>
#include <limits>
namespace mesher::gmshReference {
namespace {
/// 检查运行取消状态，取消时停止当前阶段。
void check(const RunControl& control) {
    if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"face mesh conversion cancelled"}};
}
/// 检查操作状态或前置条件，失败时抛出本模块的明确错误。
void require(OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; }
/// 检查面节点身份、实体分类、位置和参数。
void validateVertex(const FaceVertex& v) {
    if(!v.id || v.dimension < 0 || v.dimension > 2 || v.entity <= 0 ||
       !std::isfinite(v.point.x) || !std::isfinite(v.point.y) || !std::isfinite(v.point.z) ||
       !std::isfinite(v.parameters.x) || !std::isfinite(v.parameters.y) || !std::isfinite(v.storedSize))
        throw OperationFailure{{MeshingStatus::InvalidInput,"invalid original or face vertex metadata"}};
    if(v.origin == FaceVertexOrigin::Original && v.id != v.physicalId)
        throw OperationFailure{{MeshingStatus::InvalidInput,"original vertex must retain physical identity"}};
    if(v.origin == FaceVertexOrigin::FaceCreated && (v.physicalId || v.dimension != 2))
        throw OperationFailure{{MeshingStatus::InvalidInput,"invalid newly created face vertex identity"}};
}
}
/// 核对 BDS 节点与转换后的面网格节点是否一致。
OperationStatus validateFaceMeshVertices(const BdsMesh& bds,const FaceMesh& mesh,const RunControl& control) {
    try {
        check(control); require(validateBdsTopology(bds,control));
        if(mesh.entity <= 0) return {MeshingStatus::InvalidInput,"invalid face mesh entity"};
        if(mesh.vertices.size() > control.maximumNodes) return {MeshingStatus::BudgetExceeded,"face vertex budget exceeded"};
        for(const auto& entry : mesh.vertices) {
            check(control); const auto& v = entry.second; validateVertex(v);
            if(entry.first != v.id || v.id > mesh.maximumVertexId) return {MeshingStatus::InvalidTopology,"invalid face vertex pool identity"};
            if(v.origin == FaceVertexOrigin::PeriodicCopy) {
                const auto eq = mesh.equivalence.find(v.id);
                if(eq == mesh.equivalence.end()) return {MeshingStatus::InvalidTopology,"periodic copy has no original identity"};
                const auto original = mesh.vertices.find(eq->second);
                if(original == mesh.vertices.end() || original->second.origin != FaceVertexOrigin::Original ||
                   original->second.physicalId != v.physicalId || v.dimension > 1)
                    return {MeshingStatus::InvalidTopology,"invalid periodic copy equivalence"};
            }
        }
        for(const auto& entry : mesh.bdsToVertex) {
            check(control); const auto p = bds.points.find(entry.first); const auto v = mesh.vertices.find(entry.second);
            if(p == bds.points.end() || v == mesh.vertices.end() || p->second.physicalId != v->second.physicalId)
                return {MeshingStatus::InvalidTopology,"invalid BDS to face vertex reference"};
        }
        for(const auto& entry : mesh.parametricCoordinates) {
            check(control);
            if(!mesh.vertices.count(entry.first) || !std::isfinite(entry.second.x) || !std::isfinite(entry.second.y))
                return {MeshingStatus::InvalidTopology,"invalid face parametric-coordinate reference"};
        }
        return {};
    } catch(const OperationFailure& error) { return error.status; }
      catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
}
/// 由 BDS 节点建立面网格节点并保留物理身份。
OperationStatus initializeFaceMeshVertices(const BdsMesh& bds,int faceEntity,const std::vector<FaceVertex>& originals,FaceMesh& output,const RunControl& control) {
    try {
        check(control); require(validateBdsTopology(bds,control)); FaceMesh mesh; mesh.entity = faceEntity;
        std::map<std::uint64_t,FaceVertex> originalById;
        for(const auto& v : originals) {
            check(control); validateVertex(v);
            if(v.origin != FaceVertexOrigin::Original || !originalById.emplace(v.id,v).second)
                throw OperationFailure{{MeshingStatus::InvalidInput,"duplicate or nonoriginal vertex input"}};
        }
        for(const auto& entry : bds.points) {
            check(control); const auto physical = entry.second.physicalId; if(!physical) continue;
            const auto original = originalById.find(physical);
            if(original == originalById.end()) throw OperationFailure{{MeshingStatus::InvalidInput,"BDS original physical vertex metadata missing"}};
            if(!mesh.vertices.count(physical)) {
                if(mesh.vertices.size() >= control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"original face vertex budget exceeded"}};
                mesh.vertices.emplace(physical,original->second);
                if(mesh.maximumVertexId < physical) mesh.maximumVertexId = physical;
            }
            mesh.bdsToVertex.emplace(entry.first,physical);
        }
        require(validateFaceMeshVertices(bds,mesh,control)); check(control); output = std::move(mesh); return {};
    } catch(const OperationFailure& error) { return error.status; }
      catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
}
/// 将清理后的 BDS 节点和三角形转换为面网格。
FaceMeshConversionResult convertBdsToFaceMesh(const BdsMesh& bds,const FaceMesh& input,FaceMesh& output,const RunControl& control) {
    FaceMeshConversionResult result;
    try {
        check(control); require(validateFaceMeshVertices(bds,input,control)); FaceMesh mesh = input;
        for(std::size_t i = 0; i < bds.triangles.size(); ++i) {
            check(control); if(bds.triangles[i].deleted) { result.skippedDeletedTriangles.push_back(int(i)); continue; }
            const auto nodes = getBdsTriangleNodes(bds,int(i)); std::array<std::uint64_t,3> vertices{};
            for(int k = 0; k < 3; ++k) {
                check(control); auto found = mesh.bdsToVertex.find(nodes[k]);
                if(found == mesh.bdsToVertex.end()) {
                    if(mesh.vertices.size() >= control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"new face vertex budget exceeded"}};
                    if(mesh.maximumVertexId == std::numeric_limits<std::uint64_t>::max()) throw OperationFailure{{MeshingStatus::InvalidInput,"face vertex identity overflow"}};
                    const auto& p = bds.points.at(nodes[k]); FaceVertex v;
                    v.id = ++mesh.maximumVertexId; v.origin = FaceVertexOrigin::FaceCreated;
                    v.point = p.point; v.parameters = p.uv; v.dimension = 2; v.entity = mesh.entity;
                    mesh.vertices.emplace(v.id,v); found = mesh.bdsToVertex.emplace(nodes[k],v.id).first;
                    mesh.interiorVertices.push_back(v.id); result.createdLocalNodes.push_back(nodes[k]);
                }
                vertices[k] = found->second;
            }
            // 原版先分配全部缺失节点，再仅按MVertex身份判断是否退化。
            if(vertices[0] == vertices[1] || vertices[0] == vertices[2] || vertices[1] == vertices[2]) result.collapsedTriangles.push_back(int(i));
            else mesh.triangles.push_back(vertices);
        }
        check(control); require(validateFaceMeshVertices(bds,mesh,control)); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
