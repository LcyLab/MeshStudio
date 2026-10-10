/** @file BdsTopology.cpp
 * @brief BDS 节点/边/面池及双向邻接的原位操作。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsTopology.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
namespace mesher::gmshReference {
namespace {
/// 将无效输入或拓扑条件报告为本模块的操作失败。
void invalid(const char* message) { throw OperationFailure{{MeshingStatus::InvalidTopology,message}}; }
/// 检查节点的坐标和相关数值是否有限。
bool finitePoint(const BdsPoint& p) {
    return std::isfinite(p.uv.x) && std::isfinite(p.uv.y) && std::isfinite(p.point.x) &&
        std::isfinite(p.point.y) && std::isfinite(p.point.z) && std::isfinite(p.sizes.meshSize) && std::isfinite(p.sizes.backgroundSize);
}
/// 检查指定 BDS 节点是否存在。
void requirePoint(const BdsMesh& mesh,int id) { if(mesh.points.find(id) == mesh.points.end()) invalid("BDS point not found"); }
/// 按编号查询 BDS 边，越界时报告拓扑错误。
const BdsEdge& edgeAt(const BdsMesh& mesh,int index) {
    if(index < 0 || std::size_t(index) >= mesh.edges.size()) invalid("invalid BDS edge index");
    return mesh.edges[index];
}
/// 查找两条 BDS 边的共同节点。
int commonNode(const BdsEdge& a,const BdsEdge& b) {
    if(a.first == b.first || a.first == b.second) return a.first;
    if(a.second == b.first || a.second == b.second) return a.second;
    invalid("BDS triangle edges have no common node"); return 0;
}
/// 复用已有 BDS 边，或新建边并登记节点关联。
int appendEdge(BdsMesh& mesh,int first,int second) {
    const int found = findBdsEdge(mesh,first,second);
    if(found >= 0) return found;
    if(mesh.edges.size() >= std::size_t(std::numeric_limits<int>::max())) invalid("BDS edge pool exceeds index range");
    BdsEdge edge; edge.first = std::min(first,second); edge.second = std::max(first,second);
    const int index = int(mesh.edges.size()); mesh.edges.push_back(edge);
    mesh.points.at(edge.first).edges.push_back(index); mesh.points.at(edge.second).edges.push_back(index);
    return index;
}
}
/// 登记 BDS 节点并检查重复身份和数值有效性。
void insertBdsPoint(BdsMesh& mesh,BdsPoint point)
{
    if(!finitePoint(point)) throw OperationFailure{{MeshingStatus::InvalidInput,"non-finite BDS point"}};
    if(!point.edges.empty()) throw OperationFailure{{MeshingStatus::InvalidInput,"new BDS point already has edges"}};
    const int id = point.id;
    if(!mesh.points.emplace(id,std::move(point)).second) throw OperationFailure{{MeshingStatus::InvalidInput,"duplicate BDS point ID"}};
    mesh.maximumPointId = std::max(mesh.maximumPointId,id);
    const auto classification = mesh.points.at(id).classification;
    if(classification.dimension >= 0 && classification.origin == BdsClassificationOrigin::Registered) mesh.geometry.insert(classification);
}
/// 按无向节点对查找未删除的 BDS 边。
int findBdsEdge(const BdsMesh& mesh,int first,int second)
{
    const auto point = mesh.points.find(first);
    if(point == mesh.points.end()) return -1;
    for(int index : point->second.edges) {
        const auto& edge = edgeAt(mesh,index);
        if((edge.first == first && edge.second == second) || (edge.second == first && edge.first == second)) return index;
    }
    return -1;
}
/// 追加 BDS 三角形并建立边、节点和三角形的关联。
int appendBdsTriangle(BdsMesh& mesh,const std::array<int,3>& nodes)
{
    for(int id : nodes) requirePoint(mesh,id);
    if(nodes[0] == nodes[1] || nodes[1] == nodes[2] || nodes[2] == nodes[0]) invalid("BDS triangle has repeated point ID");
    if(mesh.triangles.size() >= std::size_t(std::numeric_limits<int>::max())) invalid("BDS triangle pool exceeds index range");
    BdsTriangle triangle;
    for(int i = 0; i < 3; ++i) triangle.edges[i] = appendEdge(mesh,nodes[i],nodes[(i+1)%3]);
    const int index = int(mesh.triangles.size()); mesh.triangles.push_back(triangle);
    for(int edge : triangle.edges) mesh.edges[edge].faces.push_back(index);
    return index;
}
/// 由 BDS 三角形的三条边还原节点编号。
std::array<int,3> getBdsTriangleNodes(const BdsMesh& mesh,int triangle)
{
    if(triangle < 0 || std::size_t(triangle) >= mesh.triangles.size()) invalid("invalid BDS triangle index");
    const auto& t = mesh.triangles[triangle];
    const auto& a = edgeAt(mesh,t.edges[0]); const auto& b = edgeAt(mesh,t.edges[1]); const auto& c = edgeAt(mesh,t.edges[2]);
    return {commonNode(a,c),commonNode(a,b),commonNode(b,c)};
}
/// 核对 BDS 节点、边、三角形及其相互关联。
OperationStatus validateBdsTopology(const BdsMesh& mesh,const RunControl& control)
{
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"BDS topology validation cancelled"}}; };
        check();
        if(mesh.points.size() > control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"BDS node budget exceeded"}};
        auto classification = [&](BdsClassification g) {
            if(g.dimension < -1 || g.dimension > 3) invalid("invalid BDS classification dimension");
            if(g.origin == BdsClassificationOrigin::Registered) {
                if(g.dimension >= 0 && mesh.geometry.find(g) == mesh.geometry.end()) invalid("unregistered BDS classification");
            }
            else if(g.origin == BdsClassificationOrigin::FaceInterior) {
                if(g.tag != 1 || g.dimension != 2) invalid("invalid BDS interior marker identity");
            }
            else if(g.origin == BdsClassificationOrigin::FaceExterior) {
                if(!((g.tag == 1 && g.dimension == 3) || (g.tag == 3 && g.dimension == 2))) invalid("invalid BDS exterior marker identity");
            }
            else if(g.origin == BdsClassificationOrigin::FaceBoundary) {
                if(g.tag != 1 || g.dimension != 1) invalid("invalid BDS boundary marker identity");
            }
            else invalid("unknown BDS classification origin");
        };
        std::vector<int> endpointCounts(mesh.edges.size(),0), faceCounts(mesh.triangles.size(),0);
        for(auto g : mesh.geometry) { check(); if(g.dimension < 0 || g.dimension > 3 || g.origin != BdsClassificationOrigin::Registered) invalid("invalid registered BDS geometry"); }
        int maximumId = 0;
        for(const auto& entry : mesh.points) {
            check(); const auto& p = entry.second;
            if(entry.first != p.id || !finitePoint(p)) invalid("invalid BDS point identity or value");
            maximumId = std::max(maximumId,p.id); classification(p.classification);
            if(p.periodicCounterpart) requirePoint(mesh,*p.periodicCounterpart);
            std::set<int> unique;
            for(int index : p.edges) {
                const auto& e = edgeAt(mesh,index);
                if(e.deleted || (e.first != p.id && e.second != p.id) || !unique.insert(index).second) invalid("invalid BDS point edge adjacency");
                ++endpointCounts[index];
            }
        }
        if(mesh.maximumPointId < maximumId) invalid("BDS maximum point ID below live IDs");
        for(std::size_t i = 0; i < mesh.edges.size(); ++i) {
            check(); const auto& e = mesh.edges[i]; classification(e.classification);
            if(e.deleted) {
                if(endpointCounts[i] != 0 || !e.faces.empty()) invalid("deleted BDS edge still attached");
                continue;
            }
            requirePoint(mesh,e.first); requirePoint(mesh,e.second);
            if(e.first >= e.second || endpointCounts[i] != 2) invalid("invalid BDS edge endpoints");
            std::set<int> unique;
            for(int index : e.faces) {
                if(index < 0 || std::size_t(index) >= mesh.triangles.size() || !unique.insert(index).second) invalid("invalid BDS edge face index");
                const auto& t = mesh.triangles[index];
                if(t.deleted || std::count(t.edges.begin(),t.edges.end(),int(i)) != 1) invalid("invalid BDS edge face adjacency");
                ++faceCounts[index];
            }
        }
        for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
            check(); const auto& t = mesh.triangles[i]; classification(t.classification);
            if(t.deleted) { if(faceCounts[i] != 0) invalid("deleted BDS face still attached"); continue; }
            if(faceCounts[i] != 3) invalid("BDS triangle adjacency incomplete");
            const auto nodes = getBdsTriangleNodes(mesh,int(i));
            if(nodes[0] == nodes[1] || nodes[1] == nodes[2] || nodes[2] == nodes[0]) invalid("invalid BDS triangle node cycle");
            for(int edge : t.edges) if(edgeAt(mesh,edge).deleted) invalid("BDS triangle references deleted edge");
        }
        return {};
    } catch(const OperationFailure& error) { return error.status; }
      catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
}
/// 读取 BDS 边两侧的三角形和局部节点关系。
std::optional<BdsEdgeNeighborhood> getBdsEdgeNeighborhood(const BdsMesh& mesh,int edge)
{
    const auto& e = edgeAt(mesh,edge);
    if(e.deleted || e.faces.size() != 2) return std::nullopt;
    BdsEdgeNeighborhood n; n.first = e.first; n.second = e.second; n.faces = {e.faces[0],e.faces[1]};
    const auto firstNodes = getBdsTriangleNodes(mesh,n.faces[0]);
    const auto secondNodes = getBdsTriangleNodes(mesh,n.faces[1]);
    for(int k = 0; k < 2; ++k) {
        const auto& nodes = k == 0 ? firstNodes : secondNodes;
        auto opposite = std::find_if(nodes.begin(),nodes.end(),[&](int id) { return id != e.first && id != e.second; });
        if(opposite == nodes.end()) invalid("BDS edge has no opposite triangle node");
        n.oppositeNodes[k] = *opposite;
    }
    for(int i = 0; i < 3; ++i) if(firstNodes[i] == e.first) {
        n.orientation = firstNodes[(i+1)%3] == e.second ? 1 : -1; break;
    }
    if(!n.orientation) invalid("BDS first face does not contain edge endpoint");
    auto faceEdge = [&](int face,int first,int second) {
        for(int index : mesh.triangles[face].edges) {
            const auto& side = edgeAt(mesh,index);
            if(side.first == std::min(first,second) && side.second == std::max(first,second)) return index;
        }
        invalid("BDS neighborhood side edge not found"); return -1;
    };
    n.sideEdges = {faceEdge(n.faces[0],e.first,n.oppositeNodes[0]),faceEdge(n.faces[0],n.oppositeNodes[0],e.second),
        faceEdge(n.faces[1],e.first,n.oppositeNodes[1]),faceEdge(n.faces[1],n.oppositeNodes[1],e.second)};
    return n;
}
/// 替换 BDS 共享对角线并同步局部关联。
void swapBdsEdgeTopology(BdsMesh& mesh,int edge,const BdsEdgeNeighborhood& n)
{
    const auto current = getBdsEdgeNeighborhood(mesh,edge);
    if(!current || current->first != n.first || current->second != n.second || current->faces != n.faces ||
       current->oppositeNodes != n.oppositeNodes || current->sideEdges != n.sideEdges || current->orientation != n.orientation)
        invalid("stale BDS edge swap neighborhood");
    if(n.sideEdges[0] == n.sideEdges[2] || n.sideEdges[1] == n.sideEdges[3] || n.oppositeNodes[0] == n.oppositeNodes[1])
        invalid("degenerate BDS edge swap topology");
    if(mesh.edges.size() >= std::size_t(std::numeric_limits<int>::max()) || mesh.triangles.size() > std::size_t(std::numeric_limits<int>::max())-2)
        invalid("BDS swap exceeds pool index range");
    const auto edgeClass = mesh.edges[edge].classification;
    const auto firstClass = mesh.triangles[n.faces[0]].classification, secondClass = mesh.triangles[n.faces[1]].classification;
    for(int face : n.faces) {
        for(int index : mesh.triangles[face].edges) {
            auto& faces = mesh.edges[index].faces; faces.erase(std::remove(faces.begin(),faces.end(),face),faces.end());
        }
        mesh.triangles[face].deleted = true;
    }
    for(int id : {n.first,n.second}) {
        auto& edges = mesh.points.at(id).edges; edges.erase(std::remove(edges.begin(),edges.end(),edge),edges.end());
    }
    mesh.edges[edge].deleted = true;
    // 原版直接new另一对角线，不通过add_edge查重；旧稳定池记录保持。
    BdsEdge diagonal; diagonal.first = std::min(n.oppositeNodes[0],n.oppositeNodes[1]); diagonal.second = std::max(n.oppositeNodes[0],n.oppositeNodes[1]);
    diagonal.classification = edgeClass; const int added = int(mesh.edges.size()); mesh.edges.push_back(diagonal);
    for(int id : n.oppositeNodes) mesh.points.at(id).edges.push_back(added);
    auto append = [&](std::array<int,3> edges,BdsClassification classification) {
        const int face = int(mesh.triangles.size()); mesh.triangles.push_back({edges,false,classification});
        for(int index : edges) mesh.edges[index].faces.push_back(face);
    };
    if(n.orientation == 1) {
        append({n.sideEdges[0],n.sideEdges[2],added},firstClass);
        append({added,n.sideEdges[3],n.sideEdges[1]},secondClass);
    } else {
        append({n.sideEdges[2],n.sideEdges[0],added},firstClass);
        append({n.sideEdges[3],added,n.sideEdges[1]},secondClass);
    }
    for(int id : {n.first,n.second,n.oppositeNodes[0],n.oppositeNodes[1]}) mesh.points.at(id).configurationModified = true;
}
}
