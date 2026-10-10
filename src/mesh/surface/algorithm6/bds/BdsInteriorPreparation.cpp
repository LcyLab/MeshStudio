/** @file BdsInteriorPreparation.cpp
 * @brief 普通/周期BDS内域池准备。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsInteriorPreparation.h"
#include <algorithm>
#include <exception>
#include <numeric>
namespace mesher::gmshReference {
namespace {
/// 检查运行取消状态，取消时停止当前阶段。
void check(const RunControl& control) {
    if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"BDS interior preparation cancelled"}};
}
/// 读取池压缩后的编号，并拒绝引用已删除条目。
int remap(const std::vector<int>& indices,int index) {
    if(index < 0 || std::size_t(index) >= indices.size() || indices[index] < 0)
        throw OperationFailure{{MeshingStatus::InvalidTopology,"BDS cleanup references a deleted or invalid pool entry"}};
    return indices[index];
}
/// 移除已删除的 BDS 边和三角形，并重建各池编号映射。
BdsPoolRemapping compactPools(BdsMesh& mesh,const RunControl& control) {
    auto survivors = [&](const auto& pool) {
        std::vector<int> order(pool.size()); std::iota(order.begin(),order.end(),0);
        const auto last = std::partition(order.begin(),order.end(),[&](int index) { check(control); return !pool[index].deleted; });
        order.erase(last,order.end()); return order;
    };
    const auto triangles = survivors(mesh.triangles), edges = survivors(mesh.edges);
    BdsPoolRemapping mapping; mapping.triangles.assign(mesh.triangles.size(),-1); mapping.edges.assign(mesh.edges.size(),-1);
    for(std::size_t i = 0; i < triangles.size(); ++i) mapping.triangles[triangles[i]] = int(i);
    for(std::size_t i = 0; i < edges.size(); ++i) mapping.edges[edges[i]] = int(i);
    std::vector<BdsTriangle> compactTriangles; compactTriangles.reserve(triangles.size());
    for(int index : triangles) {
        check(control); auto triangle = mesh.triangles[index];
        for(int& edge : triangle.edges) edge = remap(mapping.edges,edge);
        compactTriangles.push_back(std::move(triangle));
    }
    std::vector<BdsEdge> compactEdges; compactEdges.reserve(edges.size());
    for(int index : edges) {
        check(control); auto edge = mesh.edges[index];
        for(int& face : edge.faces) face = remap(mapping.triangles,face);
        compactEdges.push_back(std::move(edge));
    }
    for(auto& entry : mesh.points) { check(control); for(int& edge : entry.second.edges) edge = remap(mapping.edges,edge); }
    mesh.triangles = std::move(compactTriangles); mesh.edges = std::move(compactEdges); return mapping;
}
}
/// 整理 BDS 内部网格，保留有效约束和节点分类。
BdsInteriorPreparationResult prepareBdsInterior(const BdsMesh& input,BdsMesh& output,const RunControl& control)
{
    BdsInteriorPreparationResult result;
    try {
        auto require = [](OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; };
        check(control); require(validateBdsTopology(input,control)); BdsMesh mesh = input;
        for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
            check(control); auto& face = mesh.triangles[i]; if(face.classification.dimension >= 0) continue;
            result.unclassifiedTriangles.push_back(int(i));
            for(int index : face.edges) { auto& faces = mesh.edges.at(index).faces; faces.erase(std::remove(faces.begin(),faces.end(),int(i)),faces.end()); }
            face.deleted = true;
        }
        result.firstCleanup = compactPools(mesh,control);
        const BdsClassification interior{1,2,BdsClassificationOrigin::FaceInterior};
        for(std::size_t i = 0; i < mesh.edges.size(); ++i) {
            check(control); auto& edge = mesh.edges[i];
            if(edge.faces.empty()) {
                result.orphanEdges.push_back(int(i));
                for(int id : {edge.first,edge.second}) { auto& edges = mesh.points.at(id).edges; edges.erase(std::remove(edges.begin(),edges.end(),int(i)),edges.end()); }
                edge.deleted = true;
            } else {
                if(edge.classification.dimension < 0) edge.classification = interior;
                for(int id : {edge.first,edge.second}) {
                    auto& g = mesh.points.at(id).classification;
                    if(g.dimension < 0 || g.dimension > edge.classification.dimension) g = edge.classification;
                }
            }
        }
        result.secondCleanup = compactPools(mesh,control);
        for(int id : {-1,-2,-3,-4}) {
            check(control); const auto point = mesh.points.find(id); if(point == mesh.points.end()) continue;
            if(!point->second.edges.empty()) throw OperationFailure{{MeshingStatus::InvalidTopology,"BDS outer point still has live edges before deletion"}};
            mesh.points.erase(point); result.removedOuterPoints.push_back(id);
        }
        check(control); require(validateBdsTopology(mesh,control)); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
/// 完成普通面内部网格清理并初始化可转换状态。
BdsInteriorPreparationResult prepareOrdinaryBdsInterior(const BdsMesh& input,BdsMesh& output,const RunControl& control)
{
    return prepareBdsInterior(input,output,control);
}
}
