#include "Delaunay2D.h"

#include "GeomPredicates.h"

#include <algorithm>
#include <random>
#include <set>
#include <unordered_set>

namespace mesher {

namespace {

// 顶点对折叠键（与 Mesh2D 半边表同一约定）
inline uint64_t edgeKey(int a, int b)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}

// 把约束边（方向任意）转为两个方向的查找键集合
std::unordered_set<uint64_t> makeBlockedSet(const std::vector<std::pair<int, int>>& edges)
{
    std::unordered_set<uint64_t> s;
    s.reserve(edges.size() * 2);
    for (const auto& e : edges) {
        s.insert(edgeKey(e.first, e.second));
        s.insert(edgeKey(e.second, e.first));
    }
    return s;
}

// 步行定位：从 start 出发，沿"点在有向边右侧"跨越邻居，直到 p 在三角形三边内侧。
// 返回包含 p 的三角形 id；p 落在网格覆盖区外或步数超限返回 -1。
int locateTriangle(const Mesh2D& mesh, const Point2D& p, int start)
{
    // start 可能因 cavity 删除（swap-erase）而悬空，回退到 0
    if (start < 0 || start >= static_cast<int>(mesh.tris.size()))
        start = 0;
    int t = start;
    const int maxSteps = 1000000;
    for (int step = 0; step < maxSteps; ++step) {
        const Tri2D& tri = mesh.tris[t];
        bool inside = true;
        for (int e = 0; e < 3; ++e) {
            if (orient2d(mesh.points[tri.v[e]], mesh.points[tri.v[(e + 1) % 3]], p) < 0) {
                const int nb = tri.n[e];
                if (nb < 0)
                    return -1;   // 越界：p 在网格覆盖区外
                t = nb;
                inside = false;
                break;
            }
        }
        if (inside)
            return t;
    }
    return -1;
}

} // namespace

bool insertPoint(Mesh2D& mesh, const Point2D& p,
                 const std::vector<std::pair<int, int>>& blockedEdges,
                 int startTri, int* outNewVertex, int* outTri,
                 std::string* error)
{
    if (mesh.tris.empty()) {
        if (error) *error = "insertPoint: empty mesh";
        return false;
    }

    const int newV = mesh.addPoint(p);
    if (outNewVertex) *outNewVertex = newV;
    if (outTri) *outTri = -1;

    // 1) 定位包含 p 的三角形（walk 在约束 Delaunay 网格上可能被约束边干扰而失败，
    //    先换起点重试，再暴力搜索回退）
    int t = locateTriangle(mesh, p, startTri);
    if (t < 0)
        t = locateTriangle(mesh, p, 0);
    if (t < 0) {
        for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
            const Tri2D& tri = mesh.tris[i];
            if (orient2dDet(mesh.points[tri.v[0]], mesh.points[tri.v[1]], p) >= 0.0
                && orient2dDet(mesh.points[tri.v[1]], mesh.points[tri.v[2]], p) >= 0.0
                && orient2dDet(mesh.points[tri.v[2]], mesh.points[tri.v[0]], p) >= 0.0) {
                t = i;
                break;
            }
        }
    }
    if (t < 0) {
        mesh.points.pop_back();
        if (outNewVertex) *outNewVertex = -1;
        if (error) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "insertPoint: point (%.6f, %.6f) outside region (tris=%zu, startTri=%d)",
                          p.x, p.y, mesh.tris.size(), startTri);
            *error = buf;
        }
        return false;
    }

    // 2) BFS 收集 cavity：外接圆包含 p 的连通三角形集，约束边不可跨越
    const std::unordered_set<uint64_t> blocked = makeBlockedSet(blockedEdges);
    std::vector<char> inCavity(mesh.tris.size(), 0);
    std::vector<int> cavity;
    std::vector<int> queue;
    queue.push_back(t);
    inCavity[t] = 1;
    while (!queue.empty()) {
        const int ct = queue.back();
        queue.pop_back();
        cavity.push_back(ct);
        const Tri2D& tri = mesh.tris[ct];
        for (int e = 0; e < 3; ++e) {
            const int nb = tri.n[e];
            if (nb < 0 || inCavity[nb])
                continue;
            if (blocked.count(edgeKey(tri.v[e], tri.v[(e + 1) % 3])) != 0)
                continue;   // 约束边：cavity 不穿越
            const Tri2D& ntri = mesh.tris[nb];
            // 共圆（incircle==0）视为"在圆内"：共圆退化下若不删除会产生裂缝
            //（缺凸包边），标准 Bowyer-Watson 对一般位置严格 >0，此处放宽到 >=0
            if (incircle(mesh.points[ntri.v[0]], mesh.points[ntri.v[1]],
                         mesh.points[ntri.v[2]], p) >= 0) {
                inCavity[nb] = 1;
                queue.push_back(nb);
            }
        }
    }

    // 3) 收集 cavity 边界边（邻居不在 cavity 内的边），按 (p,a,b) 逆时针定向
    struct Edge { int a, b; };
    std::vector<Edge> boundary;
    boundary.reserve(cavity.size() + 4);
    for (const int ct : cavity) {
        const Tri2D& tri = mesh.tris[ct];
        for (int e = 0; e < 3; ++e) {
            const int nb = tri.n[e];
            if (nb >= 0 && inCavity[nb])
                continue;
            const Point2D& pa = mesh.points[tri.v[e]];
            const Point2D& pb = mesh.points[tri.v[(e + 1) % 3]];
            if (orient2d(p, pa, pb) > 0)
                boundary.push_back({ tri.v[e], tri.v[(e + 1) % 3] });
            else
                boundary.push_back({ tri.v[(e + 1) % 3], tri.v[e] });
        }
    }

    // 4) 删除 cavity（从大到小，避免 swap-erase 干扰待删集合）
    std::sort(cavity.begin(), cavity.end(), std::greater<int>());
    for (const int ct : cavity)
        mesh.removeTri(ct);

    // 5) 新三角形 (newV, a, b)，addTri 自动与相邻三角形互连
    int newTri = -1;
    for (const Edge& e : boundary)
        newTri = mesh.addTri(newV, e.a, e.b);

    if (outTri) *outTri = newTri;
    return true;
}

Delaunay2DResult delaunay2D(Mesh2D& mesh, std::vector<Point2D> points)
{
    Delaunay2DResult res;

    // ---- 去重（精确重合点会让 Bowyer-Watson 产生退化）----
    // 保持输入顺序：排序去重会改变点索引，破坏调用方对"第 i 个输入点 → 索引 i+3"的预期
    {
        std::set<std::pair<double, double>> seen;
        std::vector<Point2D> unique;
        unique.reserve(points.size());
        for (const Point2D& p : points) {
            if (seen.insert({ p.x, p.y }).second)
                unique.push_back(p);
        }
        points = std::move(unique);
    }
    if (points.size() < 3) {
        res.error = "delaunay2D: need at least 3 distinct points";
        return res;
    }

    // ---- 归一化：平移到 bbox 中心、缩放到半尺寸 1（super 尺度可控，数值稳定） ----
    double minX = points[0].x, maxX = points[0].x;
    double minY = points[0].y, maxY = points[0].y;
    for (const Point2D& p : points) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    const double cx = 0.5 * (minX + maxX), cy = 0.5 * (minY + maxY);
    const double scale = std::max(maxX - minX, maxY - minY) * 0.5;
    for (Point2D& p : points) {
        p.x = (p.x - cx) / scale;
        p.y = (p.y - cy) / scale;
    }

    // ---- 破坏共圆退化（符号扰动）----
    // 四点共圆（如正方形顶点）时，Bowyer-Watson 的 cavity 会把"内部顶点"的关联
    // 三角形全部吞掉，导致凸包边丢失（网格出现洞）。施加确定性极小扰动使输入处于
    // 一般位置即可恢复正确性。2D 原型接受 ~1e-8 的坐标误差（输出几何几乎不变）；
    // 3D 阶段将改为不改变坐标的精确符号扰动。
    {
        std::mt19937 pertRng(20260917u + 7);
        std::uniform_real_distribution<double> pert(-1e-8, 1e-8);
        for (Point2D& p : points) {
            p.x += pert(pertRng);
            p.y += pert(pertRng);
        }
    }

    // ---- super 三角形：包裹 [-1,1]^2 的点集，顶点先于输入点插入 ----
    const int s0 = mesh.addPoint({ 0.0, 100.0 });
    const int s1 = mesh.addPoint({ -100.0, -100.0 });
    const int s2 = mesh.addPoint({ 100.0, -100.0 });
    mesh.addTri(s0, s1, s2);

    // ---- 逐个插入（按输入顺序：保证第 i 个输入点的网格索引为 i+3，
    //      调用方据此组织约束边；walk 用 remembered 起点，性能不受影响） ----
    int startTri = 0;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        std::string err;
        if (!insertPoint(mesh, points[i], {}, startTri, nullptr, &startTri, &err)) {
            std::string verr;
            res.error = "delaunay2D: step " + std::to_string(res.insertSteps)
                        + " (point #" + std::to_string(i) + "): " + err
                        + " | mesh valid=" + (mesh.validate(&verr) ? "yes" : ("no: " + verr));
            return res;
        }
        ++res.insertSteps;
        std::string verr;
        if (!mesh.validate(&verr)) {
            res.error = "delaunay2D: mesh invalid after step " + std::to_string(res.insertSteps)
                        + ": " + verr;
            return res;
        }
    }

    // ---- 剔除 super 相关三角形 ----
    std::vector<int> toDelete;
    for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
        const Tri2D& tri = mesh.tris[i];
        if (tri.v[0] == s0 || tri.v[0] == s1 || tri.v[0] == s2
            || tri.v[1] == s0 || tri.v[1] == s1 || tri.v[1] == s2
            || tri.v[2] == s0 || tri.v[2] == s1 || tri.v[2] == s2)
            toDelete.push_back(i);
    }
    std::sort(toDelete.begin(), toDelete.end(), std::greater<int>());
    for (const int id : toDelete)
        mesh.removeTri(id);

    // ---- 还原坐标（super 顶点无三角形引用，还原后成为孤立点，可忽略） ----
    for (Point2D& p : mesh.points) {
        p.x = p.x * scale + cx;
        p.y = p.y * scale + cy;
    }

    // ---- 收集有效三角形 ----
    for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
        const Tri2D& tri = mesh.tris[i];
        if (tri.v[0] != s0 && tri.v[0] != s1 && tri.v[0] != s2
            && tri.v[1] != s0 && tri.v[1] != s1 && tri.v[1] != s2
            && tri.v[2] != s0 && tri.v[2] != s1 && tri.v[2] != s2)
            res.triIds.push_back(i);
    }

    std::string err;
    if (!mesh.validate(&err)) {
        res.error = "mesh invalid after delaunay2D: " + err;
        return res;
    }
    res.ok = true;
    return res;
}

} // namespace mesher
