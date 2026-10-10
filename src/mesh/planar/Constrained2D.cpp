// 文件职责：二维约束恢复：在已有剖分中恢复指定线段，并提供受约束的插点和域提取操作。
#include "Constrained2D.h"

#include "geometry/predicates/GeomPredicates.h"
#include "mesh/data/Mesh2DTransaction.h"

#include <algorithm>
#include <deque>
#include <cmath>
#include <unordered_set>

namespace mesher {

namespace {

/// 将有序节点对编码为边查找键。
uint64_t edgeKey(int a,int b) {
    if (a>b) std::swap(a,b);
    return (uint64_t(uint32_t(a))<<32)|uint32_t(b);
}

// 非凸四边形不能翻转。推迟这些相交边，待相邻翻转使它们变为凸形；
// 已恢复的约束不允许被后续恢复删除。
bool recoverEdge(Mesh2D& mesh, int a, int b,
                 const std::unordered_set<uint64_t>& locked,std::string* error,MeshObservation observation)
{
    MeshCountScope visits{observation,MeshCounter::TriangleVisits};
    if (a == b)
        return true;
    const int guard = static_cast<int>(mesh.tris.size()) * 8 + 1024;

    const auto crosses=[&](int p,int q) {
        return orient2d(mesh.points[a],mesh.points[b],mesh.points[p])*
               orient2d(mesh.points[a],mesh.points[b],mesh.points[q])<0 &&
               orient2d(mesh.points[p],mesh.points[q],mesh.points[a])*
               orient2d(mesh.points[p],mesh.points[q],mesh.points[b])<0;
    };
    std::deque<std::pair<int,int>> pending;
    for (const Tri2D& tri:mesh.tris) {
      ++visits.value;
      for (int k=0;k<3;++k) {
        const int p=tri.v[k],q=tri.v[(k+1)%3];
        if (p<q && crosses(p,q)) pending.push_back({p,q});
      }
    }
    size_t deferred=0;
    for (int iter = 0; iter < guard; ++iter) {
        if (mesh.hasEdgeAny(a, b))
            return true;
        if (pending.empty()) {
            if (error) *error="recoverEdge: no crossing edge (collinear or invalid PSLG)";
            return false;
        }
        const auto [x1,x2]=pending.front(); pending.pop_front();
        if (!mesh.hasEdgeAny(x1,x2) || !crosses(x1,x2)) continue;
        if (locked.count(edgeKey(x1,x2))) {
            if (error) *error="recoverEdge: intersecting constraints";
            return false;
        }
        int t=-1,qTri=-1,pv=-1,qv=-1;
        const int he=mesh.findHalfedge(x1,x2);
        if (he>=0) {
            ++visits.value;
            t=he/3; const int k=he%3;
            const Tri2D& tri=mesh.tris[t];
                pv=tri.v[(k+2)%3]; qTri=tri.n[k];
                if (qTri>=0) for (int v:mesh.tris[qTri].v)
                    if (v!=x1 && v!=x2) qv=v;
        }
        if (t<0 || qTri<0 || qv<0) {
            if (error) *error="recoverEdge: crossing edge at mesh boundary";
            return false;
        }
        if (orient2d(mesh.points[pv],mesh.points[qv],mesh.points[x1])*
            orient2d(mesh.points[pv],mesh.points[qv],mesh.points[x2])>=0) {
            pending.push_back({x1,x2});
            if (++deferred>=pending.size()) {
                if (error) *error="recoverEdge: no convex crossing quadrilateral";
                return false;
            }
            continue;
        }
        deferred=0;
        const int hi = std::max(t, qTri);
        const int lo = std::min(t, qTri);
        mesh.removeTri(hi);
        mesh.removeTri(lo);
        // 新三角形先按 CCW 定向再创建。
        int u1 = x1, v1 = pv, w1 = qv;
        if (orient2d(mesh.points[u1], mesh.points[v1], mesh.points[w1]) < 0)
            std::swap(v1, w1);
        mesh.addTri(u1, v1, w1);
        int u2 = pv, v2 = x2, w2 = qv;
        if (orient2d(mesh.points[u2], mesh.points[v2], mesh.points[w2]) < 0)
            std::swap(v2, w2);
        mesh.addTri(u2, v2, w2);
        if (crosses(pv,qv)) pending.push_back({pv,qv});
    }

    if (error) *error = "recoverEdge: iteration limit exceeded";
    return false;
}

} // namespace

/// 将约束边展开为经过现有共线节点的实际子段。
bool expandConstraintEdges(const Mesh2D& mesh,
                           const std::vector<std::pair<int,int>>& edges,
                           std::vector<std::pair<int,int>>& segments,
                           std::string* error,MeshObservation observation)
{
    MeshCountScope visits{observation,MeshCounter::PointVisits};
    std::vector<std::pair<int,int>> result;
    for (const auto& edge:edges) {
        const int a=edge.first,b=edge.second;
        if (a<0 || b<0 || a>=int(mesh.points.size()) || b>=int(mesh.points.size())) {
            if (error) *error="constraint vertex index out of range";
            return false;
        }
        if (a==b) continue;
        const auto p=mesh.points[a],q=mesh.points[b];
        if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(q.x)||!std::isfinite(q.y) ||
            (p.x==q.x && p.y==q.y)) {
            if (error) *error="constraint has non-finite or coincident endpoints";
            return false;
        }
        if (mesh.hasEdgeAny(a,b)) { result.push_back(edge); continue; }
        std::vector<int> chain{a,b};
        for (int v=0;v<int(mesh.points.size());++v) {
            ++visits.value;
            if (v==a || v==b) continue;
            const auto r=mesh.points[v];
            if (r.x<std::min(p.x,q.x) || r.x>std::max(p.x,q.x) ||
                r.y<std::min(p.y,q.y) || r.y>std::max(p.y,q.y)) continue;
            if (orient2d(p,q,r)==0) chain.push_back(v);
        }
        const bool xAxis=std::abs(q.x-p.x)>=std::abs(q.y-p.y);
        const bool increasing=xAxis?q.x>p.x:q.y>p.y;
        std::sort(chain.begin(),chain.end(),[&](int u,int v) {
            const double cu=xAxis?mesh.points[u].x:mesh.points[u].y;
            const double cv=xAxis?mesh.points[v].x:mesh.points[v].y;
            return cu==cv?u<v:(increasing?cu<cv:cu>cv);
        });
        for (size_t i=1;i<chain.size();++i) {
            const auto u=mesh.points[chain[i-1]],v=mesh.points[chain[i]];
            if (u.x==v.x && u.y==v.y) {
                if (error) *error="coincident mesh vertices on constraint";
                return false;
            }
            result.push_back({chain[i-1],chain[i]});
        }
    }
    segments=std::move(result);
    return true;
}

/// 逐条恢复约束线段。
static bool enforceSegments(Mesh2D& candidate,const std::vector<std::pair<int,int>>& segments,
                            std::string* error,MeshObservation observation) {
    std::unordered_set<uint64_t> locked;
    for (const auto& e : segments) {
        if (e.first == e.second)
            continue;
        if (candidate.hasEdgeAny(e.first, e.second)) {
            locked.insert(edgeKey(e.first,e.second));
            continue;
        }
        if (!recoverEdge(candidate, e.first, e.second, locked,error,observation)) {
            if (error) *error += " (constraint " + std::to_string(e.first) +
                                 "->" + std::to_string(e.second) + ")";
            return false;
        }
        locked.insert(edgeKey(e.first,e.second));
    }
    return true;
}

/// 恢复二维约束边，并以事务保护局部网格修改。
bool enforceConstraints(Mesh2D& mesh,const std::vector<std::pair<int,int>>& edges,
                        std::string* error,MeshObservation observation,MeshEditDelta* delta) {
    MeshStageScope scope(observation,MeshStage::ConstraintRecovery);
    std::vector<std::pair<int,int>> segments;
    if (!expandConstraintEdges(mesh,edges,segments,error,observation)) return false;
    Mesh2DTransaction transaction(mesh,observation);
    return enforceSegments(mesh,segments,error,observation) && transaction.commit(error,delta,segments);
}

namespace testing {
/// 通过完整网格备份恢复约束边，供等价性对照使用。
bool enforceConstraintsReference(Mesh2D& mesh,const std::vector<std::pair<int,int>>& edges,
                                 std::string* error,MeshObservation observation) {
    std::vector<std::pair<int,int>> segments;
    if (!expandConstraintEdges(mesh,edges,segments,error,observation)) return false;
    observation.count(MeshCounter::WholeMeshCopies,1);
    observation.count(MeshCounter::CopiedPoints,mesh.points.size());
    observation.count(MeshCounter::CopiedTriangles,mesh.tris.size());
    Mesh2D candidate=mesh;
    if (!enforceSegments(candidate,segments,error,observation) || !candidate.validate(error,observation)) return false;
    mesh=std::move(candidate); return true;
}
} // namespace testing

/// 根据约束线环和孔洞标记三角形所属的计算域。
bool classifyConstrainedDomain(const Mesh2D& mesh,
                               const std::vector<std::pair<int,int>>& boundaryEdges,
                               std::vector<bool>& inDomain,std::string* error,MeshObservation observation)
{
    MeshStageScope scope(observation,MeshStage::DomainClassification);
    observation.count(MeshCounter::WholeDomainClassifications,1);
    MeshCountScope visits{observation,MeshCounter::TriangleVisits};
    if (!mesh.validate(error,observation)) return false;
    std::vector<std::pair<int,int>> segments;
    if (!expandConstraintEdges(mesh,boundaryEdges,segments,error,observation)) return false;
    std::unordered_set<uint64_t> boundary;
    std::vector<int> degree(mesh.points.size(),0);
    for (const auto& e:segments) {
        if (!mesh.hasEdgeAny(e.first,e.second)) {
            if (error) *error="domain boundary segment is absent from triangulation";
            return false;
        }
        if (boundary.insert(edgeKey(e.first,e.second)).second) { ++degree[e.first]; ++degree[e.second]; }
    }
    for (int d:degree) if (d%2) {
        if (error) *error="domain boundary has an open chain";
        return false;
    }
    std::vector<int> parity(mesh.tris.size(),-1);
    std::deque<int> pending;
    const auto mark=[&](int t,int value) {
        if (parity[t]>=0) return parity[t]==value;
        parity[t]=value; pending.push_back(t); return true;
    };
    // 凸包外是域外；跨外环或孔的边界时翻转奇偶值。
    for (int t=0;t<int(mesh.tris.size());++t) {
      ++visits.value;
      for (int k=0;k<3;++k) {
        const auto tri=mesh.tris[t];
        if (tri.n[k]<0 && !mark(t,int(boundary.count(edgeKey(tri.v[k],tri.v[(k+1)%3]))))) {
            if (error) *error="inconsistent domain boundary at mesh hull";
            return false;
        }
      }
    }
    while (!pending.empty()) {
        ++visits.value;
        const int t=pending.front(); pending.pop_front();
        const auto tri=mesh.tris[t];
        for (int k=0;k<3;++k) if (tri.n[k]>=0) {
            const int value=parity[t]^int(boundary.count(edgeKey(tri.v[k],tri.v[(k+1)%3])));
            if (!mark(tri.n[k],value)) {
                if (error) *error="inconsistent domain boundary parity";
                return false;
            }
        }
    }
    std::vector<bool> result;
    result.reserve(parity.size());
    for (int p:parity) {
        if (p<0) { if (error) *error="domain component has no exterior seed"; return false; }
        result.push_back(p!=0);
    }
    inDomain=std::move(result);
    return true;
}

} // namespace mesher
