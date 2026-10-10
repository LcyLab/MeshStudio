/** @file WalkTriangulation.cpp
 * @brief 原版矩形/Walk/分裂/局部翻边的稳定编号半边实现。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "WalkTriangulation.h"
#include "mesh/surface/algorithm6/data/HalfEdgeTopology.h"
#include "geometry/predicates/GeomPredicates.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
namespace mesher::gmshReference {
namespace {
class WalkBuilder {
public:
    WalkTriangulation mesh;
    /// 保存运行控制，建立步行插点剖分的工作状态。
    explicit WalkBuilder(const RunControl& control) : control_(control) {}
    /// 检查取消回调，取消时终止当前算法阶段。
    void checkCancellation() const {
        if(control_.keepGoing && !control_.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"ordinary initial triangulation cancelled"}};
    }
    /// 写入一个三角形的顶点及半边环。
    void createTriangleCycle(int face, const std::array<int,3>& vertices, const std::array<int,3>& edges) {
        rewriteHalfEdgeTriangle(mesh,face,vertices,edges);
    }
    /// 建立包围输入的矩形及两个初始三角形。
    void initializeOuterRectangle(Point2D lower, Point2D upper) {
        mesh.vertices = {{{lower.x,lower.y}},{{lower.x,upper.y}},{{upper.x,upper.y}},{{upper.x,lower.y}}};
        mesh.halfEdges.resize(6); mesh.faces.resize(2);
        createTriangleCycle(0,{0,2,3},{0,1,2}); createTriangleCycle(1,{2,0,1},{3,4,5});
        mesh.halfEdges[0].opposite = 3; mesh.halfEdges[3].opposite = 0;
    }
    /// 沿半边邻接定位包含指定 UV 点的三角形。
    int locateInitialTriangle(int face, Point2D p) const {
        int edge = mesh.faces[face].halfEdge;
        std::vector<int> visited;
        while(edge >= 0) {
            checkCancellation();
            if(std::find(visited.begin(),visited.end(),edge) != visited.end())
                throw OperationFailure{{MeshingStatus::NonConverged,"ordinary initial Walk repeated a half-edge"}};
            visited.push_back(edge);
            const auto& e0 = mesh.halfEdges[edge]; const auto& e1 = mesh.halfEdges[e0.next]; const auto& e2 = mesh.halfEdges[e1.next];
            const auto a = mesh.vertices[e0.origin].uv, b = mesh.vertices[e1.origin].uv, c = mesh.vertices[e2.origin].uv;
            const double s0 = -orient2dValue(a,b,p), s1 = -orient2dValue(b,c,p), s2 = -orient2dValue(c,a,p);
            if(!std::isfinite(s0) || !std::isfinite(s1) || !std::isfinite(s2))
                throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite ordinary Walk determinant"}};
            // 原版按数值比较两个负行列式；不能只用符号或更换 tie 规则。
            if(s0 >= 0. && s1 >= 0. && s2 >= 0.) return e0.face;
            if(s0 <= 0. && s1 >= 0. && s2 >= 0.) edge = e0.opposite;
            else if(s1 <= 0. && s0 >= 0. && s2 >= 0.) edge = e1.opposite;
            else if(s2 <= 0. && s0 >= 0. && s1 >= 0.) edge = e2.opposite;
            else if(s0 <= 0. && s1 <= 0.) edge = s0 > s1 ? e0.opposite : e1.opposite;
            else if(s0 <= 0. && s2 <= 0.) edge = s0 > s2 ? e0.opposite : e2.opposite;
            else if(s1 <= 0. && s2 <= 0.) edge = s1 > s2 ? e1.opposite : e2.opposite;
            else throw OperationFailure{{MeshingStatus::InvalidTopology,"ordinary initial Walk has no crossing decision"}};
        }
        throw OperationFailure{{MeshingStatus::InvalidTopology,"ordinary initial Walk left the outer rectangle"}};
    }
    /// 判断初始网格中的共享边是否违反 Delaunay 条件。
    bool shouldFlipInitialEdge(int edge) const {
        const auto& e = mesh.halfEdges[edge];
        if(e.opposite < 0) return false;
        const auto& e1 = mesh.halfEdges[e.next]; const auto& e2 = mesh.halfEdges[e1.next];
        const auto& opposite = mesh.halfEdges[e.opposite]; const auto& opposite1 = mesh.halfEdges[opposite.next];
        const auto& opposite2 = mesh.halfEdges[opposite1.next];
        return incircleOriented(mesh.vertices[e.origin].uv,mesh.vertices[e1.origin].uv,
            mesh.vertices[e2.origin].uv,mesh.vertices[opposite2.origin].uv) < 0;
    }
    /// 替换共享对角线并更新半边拓扑。
    void swapInitialEdge(int edge) {
        swapHalfEdgeDiagonal(mesh,edge);
    }
    /// 插入图表节点，将命中三角形拆分并局部合法化。
    void splitInitialTriangle(int face, const InitialChartVertex& p) {
        const int vertex = int(mesh.vertices.size()); mesh.vertices.push_back({p.uv,p.physicalId});
        const int e0 = mesh.faces[face].halfEdge, e1 = mesh.halfEdges[e0].next, e2 = mesh.halfEdges[e1].next;
        const int v0 = mesh.halfEdges[e0].origin, v1 = mesh.halfEdges[e1].origin, v2 = mesh.halfEdges[e2].origin;
        const int base = int(mesh.halfEdges.size()); mesh.halfEdges.resize(base+6);
        for(int i = 0; i < 3; ++i) {
            mesh.halfEdges[base+i].opposite = base+i+3; mesh.halfEdges[base+i+3].opposite = base+i;
        }
        const int f1 = int(mesh.faces.size()), domain = mesh.faces[face].domain;
        mesh.faces.push_back({-1,domain}); mesh.faces.push_back({-1,domain});
        createTriangleCycle(face,{v0,v1,vertex},{e0,base+4,base});
        createTriangleCycle(f1,{v1,v2,vertex},{e1,base+5,base+1});
        createTriangleCycle(f1+1,{v2,v0,vertex},{e2,base+3,base+2});
        std::vector<int> stack{e0,e1,e2}, touched;
        while(!stack.empty()) {
            checkCancellation();
            const int edge = stack.back(); touched.push_back(edge); stack.pop_back();
            if(!shouldFlipInitialEdge(edge)) continue;
            swapInitialEdge(edge);
            for(int side : {edge,mesh.halfEdges[edge].opposite}) {
                const int next = mesh.halfEdges[side].next, last = mesh.halfEdges[next].next;
                for(int candidate : {next,last}) {
                    const int opposite = mesh.halfEdges[candidate].opposite;
                    if(std::find(touched.begin(),touched.end(),candidate) == touched.end() &&
                       std::find(touched.begin(),touched.end(),opposite) == touched.end()) stack.push_back(candidate);
                }
            }
        }
    }
private:
    const RunControl& control_;
};
}
/// 按输入顺序在外围矩形中步行定位、插点和换边。
OperationStatus buildWalkTriangulation(Point2D lower, Point2D upper,
    const std::vector<InitialChartVertex>& points, WalkTriangulation& output, const RunControl& control)
{
    try {
        WalkBuilder builder(control); builder.checkCancellation();
        auto finite = [](Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); };
        if(!finite(lower) || !finite(upper) || !(lower.x < upper.x) || !(lower.y < upper.y))
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid ordinary initial rectangle"}};
        if(control.maximumNodes < 4 || points.size() > control.maximumNodes-4 ||
           points.size() > std::size_t((std::numeric_limits<int>::max()-6)/6))
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"ordinary initial pool budget exceeded"}};
        for(const auto& p : points) if(!finite(p.uv) || p.uv.x < lower.x || p.uv.x > upper.x || p.uv.y < lower.y || p.uv.y > upper.y)
            throw OperationFailure{{MeshingStatus::InvalidInput,"ordinary initial point outside finite rectangle"}};
        builder.initializeOuterRectangle(lower,upper);
        int face = 0;
        for(const auto& p : points) {
            builder.checkCancellation(); face = builder.locateInitialTriangle(face,p.uv); builder.splitInitialTriangle(face,p);
        }
        builder.checkCancellation(); output = std::move(builder.mesh); return {};
    } catch(const OperationFailure& error) { return error.status; }
      catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
}
}
