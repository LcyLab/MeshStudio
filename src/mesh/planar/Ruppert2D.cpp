// 文件职责：二维细化：优先处理被侵入的约束段，再为不合格三角形插点；预算和拒绝原因显式返回。
#include "Ruppert2D.h"

#include "Delaunay2D.h"
#include "geometry/predicates/GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace mesher {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 顶点对折叠键（与 Mesh2D 半边表同一约定）
inline uint64_t edgeKey(int a, int b)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}

// 分裂约束边 constraints[idx]：(a,b) → (a,m), (m,b)。
// 分裂点 m 为线段中点。不能直接用 insertPoint 插入：m 恰在约束边 (a,b) 上，
// 若 (a,b) 作为 cavity 边界保留，会重建出 (m,a,b) 共线退化三角形。
// 正确做法：cavity 强制包含 (a,b) 两侧三角形（从"含 (a,b) 的三角形"出发 BFS），
// 重建时剔除 (a,b)，让 m 与洞边界开链相连 → (a,m)、(m,b) 自然成为网格边。
bool splitConstraint(int idx, Mesh2D& mesh,
                     std::vector<std::pair<int, int>>& constraints,
                     int startTri, int* outTri, std::string* error)
{
    (void)startTri;
    const auto e = constraints[idx];

    // 1) 含约束边 {a,b} 的三角形（一侧或两侧，1~2 个）作为 cavity 起点
    std::vector<int> seed;
    for (int t = 0; t < static_cast<int>(mesh.tris.size()); ++t) {
        const Tri2D& tri = mesh.tris[t];
        bool hasA = false, hasB = false;
        for (int k = 0; k < 3; ++k) {
            if (tri.v[k] == e.first) hasA = true;
            if (tri.v[k] == e.second) hasB = true;
        }
        if (hasA && hasB)
            seed.push_back(t);
    }
    if (seed.empty()) {
        if (error) *error = "splitConstraint: constraint edge not present in mesh";
        return false;
    }

    const Point2D m = midPoint(mesh.points[e.first], mesh.points[e.second]);

    // 2) cavity 收集：从 seed 出发 BFS，圆含 m（含共圆）即并入；其余约束边阻挡
    std::unordered_set<uint64_t> blocked;
    blocked.reserve(constraints.size() * 2);
    for (int i = 0; i < static_cast<int>(constraints.size()); ++i) {
        if (i == idx) continue;
        blocked.insert(edgeKey(constraints[i].first, constraints[i].second));
        blocked.insert(edgeKey(constraints[i].second, constraints[i].first));
    }
    std::vector<char> inCav(mesh.tris.size(), 0);
    std::vector<int> cavity;
    std::vector<int> queue = seed;
    for (int s : seed)
        inCav[s] = 1;
    while (!queue.empty()) {
        const int ct = queue.back();
        queue.pop_back();
        cavity.push_back(ct);
        const Tri2D& tri = mesh.tris[ct];
        for (int k = 0; k < 3; ++k) {
            const int nb = tri.n[k];
            if (nb < 0 || inCav[nb])
                continue;
            if (blocked.count(edgeKey(tri.v[k], tri.v[(k + 1) % 3])) != 0)
                continue;   // 不跨越约束边
            const Tri2D& ntri = mesh.tris[nb];
            // 邻居含其他约束边 → 不进 cavity：其约束边无法与 m 正确重建
            //（共线退化），且通常与 m 不相邻，应保持原样
            bool hasCons = false;
            for (int kk = 0; kk < 3; ++kk) {
                if (blocked.count(edgeKey(ntri.v[kk], ntri.v[(kk + 1) % 3])) != 0) {
                    hasCons = true;
                    break;
                }
            }
            if (hasCons)
                continue;
            if (incircle(mesh.points[ntri.v[0]], mesh.points[ntri.v[1]],
                         mesh.points[ntri.v[2]], m) >= 0) {
                inCav[nb] = 1;
                queue.push_back(nb);
            }
        }
    }

    // 3) 洞边界：cavity 内三角形中，对侧不在 cavity 的边。
    //    被分裂的 (a,b) 不重建（m 与 a、b 共线必然退化）。
    //    其他约束边：若与 m 共线则跳过（保持原样——共线的相邻子段不会进 cavity，
    //    其持有三角形未删）；不共线则重建（(m,p,q) 合法，且自动与对侧三角形共享）。
    struct Edge { int a, b; };
    std::vector<Edge> boundary;
    for (const int ct : cavity) {
        const Tri2D& tri = mesh.tris[ct];
        for (int k = 0; k < 3; ++k) {
            const int nb = tri.n[k];
            if (nb >= 0 && inCav[nb])
                continue;
            const int p = tri.v[k], q = tri.v[(k + 1) % 3];
            if ((p == e.first && q == e.second) || (p == e.second && q == e.first))
                continue;   // 被分裂的约束边
            if (blocked.count(edgeKey(p, q)) != 0
                && std::fabs(orient2dDet(m, mesh.points[p], mesh.points[q])) < 1e-12)
                continue;   // 与 m 共线的约束边：保持原样
            if (orient2d(m, mesh.points[p], mesh.points[q]) > 0)
                boundary.push_back({ p, q });
            else
                boundary.push_back({ q, p });
        }
    }

    // 4) 删除 cavity（从大到小，swap-erase 安全）
    std::sort(cavity.begin(), cavity.end(), std::greater<int>());
    for (const int ct : cavity)
        mesh.removeTri(ct);

    // 5) 插入 m 并重建：m 与每条洞边界边组成三角形（addTri 自动建邻居）
    const int newV = mesh.addPoint(m);
    for (const Edge& be : boundary)
        mesh.addTri(newV, be.a, be.b);

    // 6) 更新约束集合：原 (a,b) 分裂为 (a,m) 与 (m,b)
    constraints[idx] = { e.first, newV };
    constraints.push_back({ newV, e.second });
    if (outTri) *outTri = newV;
    return true;
}

/// 计算三角形的最小内角。
double minAngleOfTriangle(const Mesh2D& mesh, const Tri2D& tri)
{
    const double a = dist2D(mesh.points[tri.v[0]], mesh.points[tri.v[1]]);
    const double b = dist2D(mesh.points[tri.v[1]], mesh.points[tri.v[2]]);
    const double c = dist2D(mesh.points[tri.v[2]], mesh.points[tri.v[0]]);
    if (a <= 0.0 || b <= 0.0 || c <= 0.0)
        return 0.0;
    double best = 180.0;
    // 顶点 0 处内角（对边 a）
    {
        const double cosA = (b * b + c * c - a * a) / (2.0 * b * c);
        best = std::min(best, std::acos(std::max(-1.0, std::min(1.0, cosA))) * 180.0 / kPi);
    }
    {
        const double cosB = (a * a + c * c - b * b) / (2.0 * a * c);
        best = std::min(best, std::acos(std::max(-1.0, std::min(1.0, cosB))) * 180.0 / kPi);
    }
    {
        const double cosC = (a * a + b * b - c * c) / (2.0 * a * b);
        best = std::min(best, std::acos(std::max(-1.0, std::min(1.0, cosC))) * 180.0 / kPi);
    }
    return best;
}

// 从约束边构建闭合多边形（顶点索引序列）。v1 仅支持单环（每点度 2）。
bool buildConstraintPolygon(const std::vector<std::pair<int, int>>& constraints,
                            std::vector<int>& poly, std::string* error)
{
    std::unordered_map<int, std::vector<int>> adj;
    for (const auto& e : constraints) {
        adj[e.first].push_back(e.second);
        adj[e.second].push_back(e.first);
    }
    for (const auto& kv : adj) {
        if (kv.second.size() != 2) {
            if (error) *error = "constraints not a simple cycle (branch/void unsupported in v1)";
            return false;
        }
    }
    if (adj.empty()) {
        if (error) *error = "empty constraint set";
        return false;
    }
    const int start = constraints[0].first;
    int prev = -1, cur = start;
    do {
        poly.push_back(cur);
        const auto& nbrs = adj[cur];
        const int next = (nbrs[0] == prev) ? nbrs[1] : nbrs[0];
        prev = cur;
        cur = next;
    } while (cur != start);
    if (static_cast<int>(poly.size()) != static_cast<int>(adj.size())) {
        if (error) *error = "constraint polygon not closed";
        return false;
    }
    return true;
}

// 射线法：三角形质心是否在约束多边形内
bool centroidInPolygon(const Mesh2D& mesh, const Tri2D& tri, const std::vector<int>& poly)
{
    const Point2D c = {
        (mesh.points[tri.v[0]].x + mesh.points[tri.v[1]].x + mesh.points[tri.v[2]].x) / 3.0,
        (mesh.points[tri.v[0]].y + mesh.points[tri.v[1]].y + mesh.points[tri.v[2]].y) / 3.0,
    };
    bool inside = false;
    for (int i = 0, j = static_cast<int>(poly.size()) - 1;
         i < static_cast<int>(poly.size()); j = i++) {
        const Point2D& a = mesh.points[poly[i]];
        const Point2D& b = mesh.points[poly[j]];
        if ((a.y > c.y) != (b.y > c.y)) {
            const double xint = a.x + (c.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (c.x < xint)
                inside = !inside;
        }
    }
    return inside;
}

// 查找无向边 (a,b) 在约束集合中的索引（-1 表示非约束边）
int findConstraintIndex(const std::vector<std::pair<int, int>>& constraints, int a, int b)
{
    for (int i = 0; i < static_cast<int>(constraints.size()); ++i) {
        if ((constraints[i].first == a && constraints[i].second == b)
            || (constraints[i].first == b && constraints[i].second == a))
            return i;
    }
    return -1;
}

// 点 p 是否落在约束子段 (a,b) 的直径圆内（Ruppert "encroachment"）。
// 直径圆测试：p 看 (a,b) 的夹角为钝角 ⟺ dot(a-p, b-p) < 0。
// 注意不能用"点到线段的距离 < 半长"：当 p 恰在子段端点时距离为 0 会误判为
// encroach，导致对同一子段无限分裂（向端点收敛）。
inline bool inDiametralCircle(const Point2D& p, const Point2D& a, const Point2D& b)
{
    return (a.x - p.x) * (b.x - p.x) + (a.y - p.y) * (b.y - p.y) < 0.0;
}

} // namespace

/// 按尺寸和角度要求细化二维约束三角网格。
RuppertResult ruppertRefine(Mesh2D& mesh,
                            std::vector<std::pair<int, int>>& constraints,
                            const RuppertParams& params)
{
    RuppertResult res;
    int startTri = 0;
    std::unordered_set<int> skipped;   // 已判定"无法继续改善"的三角形（如圆心与现有点重合）

    // ---- 0) 删除域外三角形 ----
    // CDT 的 Delaunay 三角化会填满凸包，包含凹域（如 L 形缺口）外的三角形。
    // 细化前按"质心在约束多边形内"剔除，否则会在域外插入 Steiner 点。
    {
        std::vector<int> poly;
        if (!buildConstraintPolygon(constraints, poly, &res.error))
            return res;
        std::vector<int> toDelete;
        for (int t = 0; t < static_cast<int>(mesh.tris.size()); ++t)
            if (!centroidInPolygon(mesh, mesh.tris[t], poly))
                toDelete.push_back(t);
        std::sort(toDelete.begin(), toDelete.end(), std::greater<int>());
        for (const int t : toDelete)
            mesh.removeTri(t);
        std::string verr;
        if (!mesh.validate(&verr)) {
            res.error = "mesh invalid after removing outside triangles: " + verr;
            return res;
        }
    }

    for (int iter = 0; iter < params.maxIterations; ++iter) {
        res.iterations = iter + 1;

        // ---- A) 分裂超长约束边（大小函数：先保证边长上界） ----
        bool splitAny = false;
        for (int i = 0; i < static_cast<int>(constraints.size()); ++i) {
            const auto& e = constraints[i];
            if (dist2D(mesh.points[e.first], mesh.points[e.second]) > params.maxEdgeLength) {
                if (!splitConstraint(i, mesh, constraints, startTri, &startTri, &res.error)) {
                    res.error = "splitConstraint failed: " + res.error;
                    return res;
                }
                ++res.insertedPoints;
                splitAny = true;
                break;   // constraints 已变化，重新扫描
            }
        }
        if (splitAny) {
            std::string verr;
            if (!mesh.validate(&verr)) {
                res.error = "mesh invalid after splitConstraint: " + verr;
                return res;
            }
            continue;
        }

        // ---- B) 找最坏三角形（质量 + 大小函数：边长超限也算坏） ----
        int badTri = -1;
        double worstScore = 0.0;
        Point2D cc;
        double badLmin = 0.0;
        bool badBySize = false;   // 因边长超限（而非半径边比）而坏
        for (int t = 0; t < static_cast<int>(mesh.tris.size()); ++t) {
            if (skipped.count(t) != 0)
                continue;   // 已判定无法改善
            const Tri2D& tri = mesh.tris[t];
            double R = 0.0;
            Point2D c;
            if (!circumcircle(mesh.points[tri.v[0]], mesh.points[tri.v[1]],
                              mesh.points[tri.v[2]], c, R))
                continue;
            const double l01 = dist2D(mesh.points[tri.v[0]], mesh.points[tri.v[1]]);
            const double l12 = dist2D(mesh.points[tri.v[1]], mesh.points[tri.v[2]]);
            const double l20 = dist2D(mesh.points[tri.v[2]], mesh.points[tri.v[0]]);
            const double lmin = std::min(l01, std::min(l12, l20));
            const double lmax = std::max(l01, std::max(l12, l20));
            if (lmin <= 0.0)
                continue;
            // 评分：质量分 R/Lmin、大小分 最大边长，各自归一后取更高者
            const double ratio = R / lmin;
            double score = 0.0;
            if (ratio > params.radiusEdgeRatio)
                score = ratio / params.radiusEdgeRatio;
            if (lmax > params.maxEdgeLength)
                score = std::max(score, lmax / params.maxEdgeLength);
            if (score > worstScore) {
                worstScore = score;
                badTri = t;
                cc = c;
                badLmin = lmin;
                badBySize = (lmax > params.maxEdgeLength);
            }
        }
        if (badTri < 0 || worstScore <= 1.0) {
            res.converged = true;
            break;
        }

        // ---- C) 决策插入点 ----
        // 大小函数优先：超长边（> maxEdgeLength）→ 细分最长边（约束边则分裂，
        // 否则插中点），保证边长上界。若最长边中点与现有点过近（它已贴着细化过
        // 的边界），降级到质量路径（C2 插外接圆心），而非终止细化。
        bool didInsert = false;
        if (badBySize) {
            const Tri2D& ttri = mesh.tris[badTri];
            const double l01 = dist2D(mesh.points[ttri.v[0]], mesh.points[ttri.v[1]]);
            const double l12 = dist2D(mesh.points[ttri.v[1]], mesh.points[ttri.v[2]]);
            const double l20 = dist2D(mesh.points[ttri.v[2]], mesh.points[ttri.v[0]]);
            int lu, lv;
            if (l01 >= l12 && l01 >= l20) { lu = ttri.v[0]; lv = ttri.v[1]; }
            else if (l12 >= l20) { lu = ttri.v[1]; lv = ttri.v[2]; }
            else { lu = ttri.v[2]; lv = ttri.v[0]; }
            const int ci = findConstraintIndex(constraints, lu, lv);
            if (ci >= 0) {
                if (!splitConstraint(ci, mesh, constraints, 0, &startTri, &res.error)) {
                    res.error = "splitConstraint(oversize edge) failed: " + res.error;
                    return res;
                }
                ++res.insertedPoints;
                std::string verr;
                if (!mesh.validate(&verr)) {
                    res.error = "mesh invalid after splitConstraint(oversize edge): " + verr;
                    return res;
                }
                continue;
            }
            const Point2D mid = midPoint(mesh.points[lu], mesh.points[lv]);
            double nearest = std::numeric_limits<double>::max();
            for (const Point2D& p : mesh.points)
                nearest = std::min(nearest, dist2D(mid, p));
            if (nearest >= 0.5 * badLmin) {
            if (!insertPoint(mesh, mid, constraints, startTri, nullptr, &startTri, &res.error)) {
                res.error = "insertPoint(edge midpoint) failed: " + res.error;
                return res;
            }
                ++res.insertedPoints;
                std::string verr;
                if (!mesh.validate(&verr)) {
                    res.error = "mesh invalid after insertPoint(edge midpoint): " + verr;
                    return res;
                }
                didInsert = true;
            }
        }
        if (didInsert)
            continue;

        // ---- C2) 质量坏 → Shewchuk 2002 Algorithm 1 ----
        // 规则：外接圆心 encroach 某约束子段（圆心在子段直径圆内）→ 分裂该子段；
        // 否则直接插入外接圆心（无论圆心在坏三角形内外——Ruppert 引理保证圆心
        // 在三角形外时必落于其最短边的直径圆内，插入是合理的）。
        bool encroached = false;
        for (int i = 0; i < static_cast<int>(constraints.size()); ++i) {
            const auto& e = constraints[i];
            if (inDiametralCircle(cc, mesh.points[e.first], mesh.points[e.second])) {
                if (!splitConstraint(i, mesh, constraints, 0, &startTri, &res.error)) {
                    res.error = "splitConstraint(encroached) failed: " + res.error;
                    return res;
                }
                ++res.insertedPoints;
                encroached = true;
                break;
            }
        }
        if (encroached) {
            std::string verr;
            if (!mesh.validate(&verr)) {
                res.error = "mesh invalid after splitConstraint(encroached): " + verr;
                return res;
            }
            continue;
        }

        // 圆心与现有点过近（< 0.5×最短边）→ Ruppert 引理被违反，网格已达数值极限
        double nearest = std::numeric_limits<double>::max();
        for (const Point2D& p : mesh.points)
            nearest = std::min(nearest, dist2D(cc, p));
        if (nearest < 0.5 * badLmin) {
            // 圆心与现有点重合/过近：该三角形无法继续改善（网格局部饱和），
            // 标记跳过，继续处理其他坏三角形
            skipped.insert(badTri);
            continue;
        }

        if (!insertPoint(mesh, cc, constraints, startTri, nullptr, &startTri, &res.error)) {
            // cc 在域外（walk 定位失败）：这是"圆心跨出边界但不在任何子段直径圆内"
            // 的数值边缘情况。分裂距 cc 最近的约束子段来改进边界。
            int best = -1;
            double bestD = std::numeric_limits<double>::max();
            for (int i = 0; i < static_cast<int>(constraints.size()); ++i) {
                const auto& e = constraints[i];
                const double d = dist2D(cc, midPoint(mesh.points[e.first],
                                                     mesh.points[e.second]));
                if (d < bestD) { bestD = d; best = i; }
            }
            if (best < 0) {
                res.error = "insertPoint(circumcenter) failed and no constraint to split: "
                            + res.error;
                return res;
            }
            if (!splitConstraint(best, mesh, constraints, 0, &startTri, &res.error)) {
                res.error = "splitConstraint(cc-outside) failed: " + res.error;
                return res;
            }
            ++res.insertedPoints;
            std::string verr;
            if (!mesh.validate(&verr)) {
                res.error = "mesh invalid after splitConstraint(cc-outside): " + verr;
                return res;
            }
            continue;
        }
        ++res.insertedPoints;
        std::string verr;
        if (!mesh.validate(&verr)) {
            res.error = "mesh invalid after insertPoint (iter " + std::to_string(res.iterations)
                        + "): " + verr;
            return res;
        }
    }

    if (!res.converged)
        res.error = "ruppertRefine: iteration limit reached";

    // ---- 统计 ----
    res.triCount = static_cast<int>(mesh.tris.size());
    res.minAngleDeg = 180.0;
    res.maxEdgeLen = 0.0;
    for (const Tri2D& tri : mesh.tris) {
        res.minAngleDeg = std::min(res.minAngleDeg, minAngleOfTriangle(mesh, tri));
        for (int e = 0; e < 3; ++e)
            res.maxEdgeLen = std::max(res.maxEdgeLen,
                                      dist2D(mesh.points[tri.v[e]], mesh.points[tri.v[(e + 1) % 3]]));
    }
    for (const auto& e : constraints)
        res.maxEdgeLen = std::max(res.maxEdgeLen,
                                  dist2D(mesh.points[e.first], mesh.points[e.second]));
    return res;
}

} // namespace mesher
