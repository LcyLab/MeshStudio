#include "Smooth3D.h"

#include "GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mesher {

double meanEdgeLength3D(const TetMesh3D& mesh)
{
    double sum = 0.0;
    long long cnt = 0;
    for (const Tet4& t : mesh.tets) {
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j) {
                const Point3D& a = mesh.points[t.v[i]];
                const Point3D& b = mesh.points[t.v[j]];
                sum += std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)
                                 + (a.z - b.z) * (a.z - b.z));
                ++cnt;
            }
    }
    return cnt > 0 ? sum / static_cast<double>(cnt) : 1.0;
}

Smooth3DResult smoothMesh3D(TetMesh3D& mesh,
                            const std::vector<std::array<int, 3>>& surfaceFaces,
                            const Smooth3DParams& params)
{
    Smooth3DResult res;
    if (mesh.tets.empty()) {
        res.error = "smoothMesh3D: empty mesh";
        return res;
    }
    const double scale = meanEdgeLength3D(mesh);
    res.before = summarizeQuality(mesh, scale);
    const double vol0 = mesh.totalVolume();

    // 边界顶点（约束面上的点）与顶点→相邻顶点、顶点→关联四面体
    const int V = static_cast<int>(mesh.points.size());
    std::vector<char> isBoundary(V, 0);
    // 来源 ①：调用方给出的约束面片顶点（初始表面，含 M2 面恢复插入的 Steiner 点）
    if (params.preserveBoundary) {
        for (const auto& f : surfaceFaces)
            for (int k = 0; k < 3; ++k)
                isBoundary[f[k]] = 1;
    }
    // 来源 ②：**网格自身的边界面**（n[e] < 0 的面）上的顶点。
    //
    // 为什么必须加这一条：ruppertRefine3D 会在约束边/面上继续插入中点
    //（splitSeg），这些新点不在调用方持有的 surfaceFaces 里。若只信 ①，
    // 它们会被误判为内部点而被 Laplacian 拖动 → **边界被拉伸 → 体积真实改变**
    //（此前 "volume changed unexpectedly" 的真正原因，而非浮点漂移）。
    // 边界面的顶点是几何上确定在表面上的，据此固定才是鲁棒的。
    for (const Tet4& t : mesh.tets) {
        for (int e = 0; e < 4; ++e) {
            if (t.n[e] >= 0)
                continue;   // 内部面，跳过
            for (int k = 0; k < 3; ++k)
                isBoundary[t.v[(e + 1 + k) % 4]] = 1;
        }
    }
    std::vector<std::vector<int>> nbr(V);
    std::vector<std::vector<int>> incident(V);
    for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
        const Tet4& tet = mesh.tets[t];
        for (int i = 0; i < 4; ++i) {
            incident[tet.v[i]].push_back(t);
            for (int j = i + 1; j < 4; ++j) {
                nbr[tet.v[i]].push_back(tet.v[j]);
                nbr[tet.v[j]].push_back(tet.v[i]);
            }
        }
    }
    for (auto& v : nbr) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }

    for (int iter = 0; iter < params.iterations; ++iter) {
        for (int v = 0; v < V; ++v) {
            if (isBoundary[v] || nbr[v].empty() || incident[v].empty())
                continue;
            // Laplacian 目标：相邻顶点质心
            double cx = 0.0, cy = 0.0, cz = 0.0;
            for (const int n : nbr[v]) {
                cx += mesh.points[n].x;
                cy += mesh.points[n].y;
                cz += mesh.points[n].z;
            }
            const double inv = 1.0 / static_cast<double>(nbr[v].size());
            cx *= inv; cy *= inv; cz *= inv;

            const Point3D old = mesh.points[v];
            mesh.points[v] = { old.x + params.relaxation * (cx - old.x),
                               old.y + params.relaxation * (cy - old.y),
                               old.z + params.relaxation * (cz - old.z) };

            // 防翻转：含该顶点的所有四面体必须保持正体积
            bool ok = true;
            for (const int t : incident[v]) {
                const Tet4& tet = mesh.tets[t];
                if (orient3dDet(mesh.points[tet.v[0]], mesh.points[tet.v[1]],
                                mesh.points[tet.v[2]], mesh.points[tet.v[3]]) <= 0.0) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                ++res.movedVertices;
            } else {
                mesh.points[v] = old;   // 回退
                ++res.rejectedMoves;
            }
        }
    }

    // 体积守恒检查（边界顶点固定 ⇒ 内部顶点移动理论上是**零阶矩守恒**的：
    // 每个 tet 的体积是按顶点坐标算的，移动内部顶点会改变「各 tet 体积的分配」
    // 但**不改变总体积**——因为体积是关于任一顶点的线性函数，而内部顶点移动
    // 只是把它在相邻 tet 之间重新分配）。
    //
    // 注意：这里必须用**相对**容差。double 运算下，累加 3000+ 个 tet 的体积时
    // 每次移动都会引入 ~eps·Σ|Vi| 的舍入漂移；5043 次移动的累积漂移可达
    // 1e-10 量级，而网格体积为 1。旧的绝对阈值 1e-6 在 2D 小网格上够用，
    // 但对上千单元的 3D 网格过紧，会把完全正确的平滑误判为失败。
    // 放宽到 64·eps·移动次数·体积量级，既容忍浮点漂移，又能抓住真实错误
    //（真实的体积错误是 O(1) 级别的，远超该界）。
    const double drift = 64.0 * std::numeric_limits<double>::epsilon()
                       * static_cast<double>(std::max<size_t>(res.movedVertices, 1))
                       * std::max(1.0, std::fabs(vol0));
    if (std::fabs(mesh.totalVolume() - vol0) > drift) {
        res.error = "smoothMesh3D: volume changed unexpectedly";
        return res;
    }
    std::string verr;
    if (!mesh.validate(&verr)) {
        res.error = "smoothMesh3D broke mesh: " + verr;
        return res;
    }
    res.after = summarizeQuality(mesh, scale);
    res.ok = true;
    return res;
}

} // namespace mesher
