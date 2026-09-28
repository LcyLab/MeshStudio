#include "Constrained2D.h"

#include "GeomPredicates.h"

#include <algorithm>

namespace mesher {

namespace {

// 找顶点 a 所在的任意三角形（O(T) 线性查找，原型阶段可接受）
int findTriContainingVertex(const Mesh2D& mesh, int vid)
{
    for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
        const Tri2D& tri = mesh.tris[i];
        if (tri.v[0] == vid || tri.v[1] == vid || tri.v[2] == vid)
            return i;
    }
    return -1;
}

// 找到"线段 ab 从顶点 a 出发穿出"的三角形：含 a，且 a 的对边两端分居 ab 两侧。
// 返回三角形 id 并输出对边两端 (x1, x2)；找不到返回 -1。
int findExitingTriangle(const Mesh2D& mesh, int a, int b, int& x1, int& x2)
{
    for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
        const Tri2D& tri = mesh.tris[i];
        int k = -1;
        if (tri.v[0] == a) k = 0;
        else if (tri.v[1] == a) k = 1;
        else if (tri.v[2] == a) k = 2;
        if (k < 0)
            continue;
        const int p = tri.v[(k + 1) % 3];
        const int q = tri.v[(k + 2) % 3];
        const double o1 = orient2dDet(mesh.points[a], mesh.points[b], mesh.points[p]);
        const double o2 = orient2dDet(mesh.points[a], mesh.points[b], mesh.points[q]);
        // a 的对边 (p,q) 被 ab 从 a 出发穿过 ⟺ p、q 分居 ab 两侧
        if ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) {
            x1 = p;
            x2 = q;
            return i;
        }
        // 端点在 ab 上（o==0）属于退化/共线输入，不在此恢复（交由调用方/细化处理）
    }
    return -1;
}

// 恢复单条约束边 (a,b)：反复翻转"ab 穿出"的边所在四边形，直到 (a,b) 成为网格边。
// 每次 flip 都会减少 ab 与网格边的交叉数（Lawson 性质），故必然终止（guard 保护）。
bool recoverEdge(Mesh2D& mesh, int a, int b, std::string* error)
{
    if (a == b)
        return true;
    const int guard = static_cast<int>(mesh.tris.size()) * 8 + 1024;

    for (int iter = 0; iter < guard; ++iter) {
        if (mesh.hasEdgeAny(a, b))
            return true;

        // 找 ab 穿出的含 a 三角形
        int x1 = -1, x2 = -1;
        const int t = findExitingTriangle(mesh, a, b, x1, x2);
        if (t < 0) {
            if (error) *error = "recoverEdge: no crossing edge found (degenerate input?)";
            return false;
        }
        const Tri2D& tri = mesh.tris[t];
        // a 在 t 中的位置 k；a 的对边 (x1,x2) = (v[k+1], v[k+2]) 的邻居是 n[k]
        const int k = (tri.v[0] == a) ? 0 : (tri.v[1] == a) ? 1 : 2;
        const int qTri = tri.n[k];
        if (qTri < 0) {
            if (error) *error = "recoverEdge: crossing edge at mesh boundary";
            return false;
        }
        // qv = 对侧三角形 qTri 的第三顶点（非 x1、x2）
        const Tri2D& q = mesh.tris[qTri];
        int qv = -1;
        for (int e = 0; e < 3; ++e)
            if (q.v[e] != x1 && q.v[e] != x2) { qv = q.v[e]; break; }

        // flip：删除 t 与 qTri（从大到小），对角线 (x1,x2) 换成 (a, qv)
        const int hi = std::max(t, qTri);
        const int lo = std::min(t, qTri);
        mesh.removeTri(hi);
        mesh.removeTri(lo);
        // 新三角形 (x1, a, qv) 与 (a, x2, qv)，先按 CCW 定向再创建
        int u1 = x1, v1 = a, w1 = qv;
        if (orient2dDet(mesh.points[u1], mesh.points[v1], mesh.points[w1]) < 0.0)
            std::swap(v1, w1);
        mesh.addTri(u1, v1, w1);
        int u2 = a, v2 = x2, w2 = qv;
        if (orient2dDet(mesh.points[u2], mesh.points[v2], mesh.points[w2]) < 0.0)
            std::swap(v2, w2);
        mesh.addTri(u2, v2, w2);
    }

    if (error) *error = "recoverEdge: iteration limit exceeded";
    return false;
}

} // namespace

bool enforceConstraints(Mesh2D& mesh,
                        const std::vector<std::pair<int, int>>& constraintEdges,
                        std::string* error)
{
    for (const auto& e : constraintEdges) {
        if (e.first == e.second)
            continue;
        if (mesh.hasEdgeAny(e.first, e.second))
            continue;
        if (!recoverEdge(mesh, e.first, e.second, error))
            return false;
    }
    return true;
}

} // namespace mesher
