#include "Mesh2D.h"

#include "GeomPredicates.h"

#include <algorithm>

namespace mesher {

namespace {
// 顶点对 (a,b) 折叠成半边表 key；顶点索引视为无符号 32 位
inline uint64_t halfedgeKey(int a, int b)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}
} // namespace

int Mesh2D::addTri(int a, int b, int c)
{
    Tri2D t;
    t.v[0] = a;
    t.v[1] = b;
    t.v[2] = c;
    const int id = static_cast<int>(tris.size());
    tris.push_back(t);

    addTriEdges(id);
    for (int e = 0; e < 3; ++e) {
        const int oppHe = oppositeHalfedge(id * 3 + e);
        if (oppHe >= 0)
            setNeighbor(id * 3 + e, oppHe);
    }
    return id;
}

void Mesh2D::removeTri(int triId)
{
    const Tri2D& t = tris[triId];

    // 1) 擦除被删三角形贡献的有向半边
    removeTriEdges(triId);

    // 2) 被删三角形的邻居中，指回 triId 的引用置 -1
    for (int e = 0; e < 3; ++e) {
        const int nb = t.n[e];
        if (nb < 0) continue;
        for (int e2 = 0; e2 < 3; ++e2) {
            if (tris[nb].n[e2] == triId) { tris[nb].n[e2] = -1; break; }
        }
    }

    // 3) swap-erase 紧凑删除：最后一个三角形搬到 triId 位置
    const int lastId = static_cast<int>(tris.size()) - 1;
    if (lastId != triId) {
        tris[triId] = tris[lastId];
        const Tri2D& mv = tris[triId];
        // 被移动三角形的邻居引用与半边表 key 一并迁移（lastId → triId）
        for (int e = 0; e < 3; ++e) {
            const int oldHe = lastId * 3 + e;
            const int newHe = triId * 3 + e;
            const int nb = mv.n[e];
            if (nb >= 0) {
                for (int e2 = 0; e2 < 3; ++e2) {
                    if (tris[nb].n[e2] == lastId) { tris[nb].n[e2] = triId; break; }
                }
            }
            const uint64_t key = halfedgeKey(mv.v[e], mv.v[(e + 1) % 3]);
            auto it = m_halfedge.find(key);
            if (it != m_halfedge.end() && it->second == oldHe)
                it->second = newHe;
        }
    }
    tris.pop_back();
}

int Mesh2D::oppositeHalfedge(int he) const
{
    const int src = srcOf(he);
    const int dst = dstOf(he);
    auto it = m_halfedge.find(halfedgeKey(dst, src));  // 反向有向边
    return (it != m_halfedge.end()) ? it->second : -1;
}

int Mesh2D::findOpposite(int triId, int e) const
{
    const int oppHe = oppositeHalfedge(triId * 3 + e);
    return (oppHe >= 0) ? oppHe / 3 : -1;
}

bool Mesh2D::hasEdgeAny(int a, int b) const
{
    return m_halfedge.count(halfedgeKey(a, b)) != 0
        || m_halfedge.count(halfedgeKey(b, a)) != 0;
}

void Mesh2D::addTriEdges(int triId)
{
    for (int e = 0; e < 3; ++e) {
        const int he = triId * 3 + e;
        const uint64_t key = halfedgeKey(srcOf(he), dstOf(he));
        m_halfedge[key] = he;   // 有向边唯一，重复出现即拓扑非法（由 validate 报出）
    }
}

void Mesh2D::removeTriEdges(int triId)
{
    for (int e = 0; e < 3; ++e) {
        const int he = triId * 3 + e;
        const uint64_t key = halfedgeKey(srcOf(he), dstOf(he));
        auto it = m_halfedge.find(key);
        if (it != m_halfedge.end() && it->second == he)
            m_halfedge.erase(it);
    }
}

void Mesh2D::setNeighbor(int heA, int heB)
{
    // 调用方保证 heA 与 heB 是同一无向边的两个方向
    tris[heA / 3].n[heA % 3] = heB / 3;
    tris[heB / 3].n[heB % 3] = heA / 3;
}

bool Mesh2D::validate(std::string* error) const
{
    const auto fail = [error](const char* msg) {
        if (error) *error = msg;
        return false;
    };
    const int V = static_cast<int>(points.size());

    for (const Tri2D& t : tris) {
        for (int e = 0; e < 3; ++e) {
            if (t.v[e] < 0 || t.v[e] >= V)
                return fail("vertex index out of range");
            if (t.v[e] == t.v[(e + 1) % 3])
                return fail("degenerate triangle (duplicate vertex)");
            if (t.n[e] < -1 || t.n[e] >= static_cast<int>(tris.size()))
                return fail("neighbor index out of range");
        }
    }

    for (const Tri2D& t : tris) {
        if (orient2dDet(points[t.v[0]], points[t.v[1]], points[t.v[2]]) < 0.0)
            return fail("triangle not CCW");
    }

    for (int id = 0; id < static_cast<int>(tris.size()); ++id) {
        const Tri2D& t = tris[id];
        for (int e = 0; e < 3; ++e) {
            const int nb = t.n[e];
            if (nb >= 0) {
                const Tri2D& tn = tris[nb];
                bool found = false;
                for (int e2 = 0; e2 < 3; ++e2) {
                    if (tn.n[e2] == id
                        && tn.v[e2] == t.v[(e + 1) % 3]
                        && tn.v[(e2 + 1) % 3] == t.v[e]) {
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return fail("neighbor not symmetric");
            } else {
                // 声明无邻居的边不允许存在反向半边
                const uint64_t key = halfedgeKey(t.v[(e + 1) % 3], t.v[e]);
                if (m_halfedge.count(key) != 0)
                    return fail("boundary edge has hidden opposite halfedge");
            }
        }
    }

    if (static_cast<int>(m_halfedge.size()) != static_cast<int>(tris.size()) * 3) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "halfedge map size mismatch (%zu vs %zu)",
                      m_halfedge.size(), tris.size() * 3);
        return fail(buf);
    }
    return true;
}

double Mesh2D::totalArea() const
{
    double sum = 0.0;
    for (const Tri2D& t : tris)
        sum += triangleArea(points[t.v[0]], points[t.v[1]], points[t.v[2]]);
    return sum;
}

} // namespace mesher
