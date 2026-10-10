/** @file HalfEdgeTopology.cpp
 * @brief 原生 createFace/swap_edge 的稳定池连接操作。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "HalfEdgeTopology.h"
#include <cmath>
namespace mesher::gmshReference {
/// 校验初始网格的半边环、反向连接和节点关联。
OperationStatus validateHalfEdgeTopology(const WalkTriangulation& input, const RunControl& control)
{
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"initial half-edge operation cancelled"}}; };
        check();
        if(input.vertices.size() > control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"initial half-edge node budget exceeded"}};
        auto edgeValid = [&](int i) { return i >= 0 && std::size_t(i) < input.halfEdges.size(); };
        auto vertexValid = [&](int i) { return i >= 0 && std::size_t(i) < input.vertices.size(); };
        auto faceValid = [&](int i) { return i >= 0 && std::size_t(i) < input.faces.size(); };
        auto fail = [] { throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid initial half-edge topology"}}; };
        if(input.faces.empty() || input.halfEdges.size() != 3*input.faces.size()) fail();
        for(std::size_t i = 0; i < input.vertices.size(); ++i) {
            const auto& v = input.vertices[i];
            if(!std::isfinite(v.uv.x) || !std::isfinite(v.uv.y)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite initial half-edge vertex"}};
            if(!edgeValid(v.incidentHalfEdge) || input.halfEdges[v.incidentHalfEdge].origin != int(i)) fail();
        }
        for(std::size_t i = 0; i < input.halfEdges.size(); ++i) {
            check(); const auto& e = input.halfEdges[i];
            if(!vertexValid(e.origin) || !faceValid(e.face) || !edgeValid(e.previous) || !edgeValid(e.next)) fail();
            if(input.halfEdges[e.next].previous != int(i) || input.halfEdges[e.previous].next != int(i)) fail();
            if(e.opposite != -1 && (!edgeValid(e.opposite) || input.halfEdges[e.opposite].opposite != int(i) ||
                input.halfEdges[e.opposite].origin != input.halfEdges[e.next].origin)) fail();
        }
        for(std::size_t i = 0; i < input.faces.size(); ++i) {
            const int h0 = input.faces[i].halfEdge;
            if(!edgeValid(h0)) fail();
            const int h1 = input.halfEdges[h0].next, h2 = input.halfEdges[h1].next;
            if(input.halfEdges[h2].next != h0 || input.halfEdges[h0].face != int(i) ||
                input.halfEdges[h1].face != int(i) || input.halfEdges[h2].face != int(i)) fail();
        }
        check(); return {};
    } catch(const OperationFailure& error) { return error.status; }
}
/// 重写一个初始三角形的顶点及半边环。
void rewriteHalfEdgeTriangle(WalkTriangulation& mesh, int face,
    const std::array<int,3>& vertices, const std::array<int,3>& edges)
{
    for(int i = 0; i < 3; ++i) {
        auto& e = mesh.halfEdges[edges[i]];
        e.origin = vertices[i]; e.face = face; e.next = edges[(i+1)%3]; e.previous = edges[(i+2)%3];
        mesh.vertices[vertices[i]].incidentHalfEdge = edges[i];
    }
    mesh.faces[face].halfEdge = edges[0];
}
/// 交换两个相邻初始三角形的对角线。
bool swapHalfEdgeDiagonal(WalkTriangulation& mesh, int edge)
{
    const int opposite = mesh.halfEdges[edge].opposite;
    if(opposite < 0) return false;
    const int e1 = mesh.halfEdges[edge].next, e2 = mesh.halfEdges[e1].next;
    const int o1 = mesh.halfEdges[opposite].next, o2 = mesh.halfEdges[o1].next;
    const int v0 = mesh.halfEdges[o1].origin, v1 = mesh.halfEdges[o2].origin;
    const int v2 = mesh.halfEdges[opposite].origin, v3 = mesh.halfEdges[e2].origin;
    const int f0 = mesh.halfEdges[edge].face, f1 = mesh.halfEdges[o2].face;
    rewriteHalfEdgeTriangle(mesh,f0,{v0,v1,v3},{o1,opposite,e2});
    rewriteHalfEdgeTriangle(mesh,f1,{v1,v2,v3},{o2,e1,edge});
    return true;
}
}
