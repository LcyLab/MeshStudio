#include "SurfaceMesh3D.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace mesher {

namespace {

// ---- 空间哈希格子 ----------------------------------------------------------
// ★ 旧实现（已废弃，勿回退）★ 把三个轴各量化成 21 位整数塞进一个 uint64：
//     const double t = std::floor(v / tol) + 1048576.0;   // 偏移避免负数
//     const long long c = clamp(i, 0, 2097151);
//   有两个致命缺陷：
//   1) 有效坐标范围只有 ±1048575·tol。引擎传 tol = diag·1e-7，于是范围仅
//      ±0.105·diag —— 一个半径 50 的球（diag≈173）里所有 |v|>18 的坐标全部
//      被钳到同一个格子，焊接退化成"全塌成 1 个点"。cube.stl / cube10.stl 能
//      过纯属侥幸（它们只有 0/1/10 两个坐标值，恰好落在钳制边界两侧）。
//   2) 负坐标经 floor+偏移后为负 → 被 clamp 到 0，原点附近的负半轴整段丢失。
//   现在改用「floor(v/tol) 的三元 64 位格子坐标」做精确键，无偏移、无钳制，
//   任意坐标范围（含负）都正确；再配合邻域探测得到真正的容差语义。
struct CellCoord
{
    long long x = 0, y = 0, z = 0;
    bool operator==(const CellCoord& o) const
    {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct CellCoordHash
{
    std::size_t operator()(const CellCoord& c) const noexcept
    {
        std::uint64_t h = 1469598103934665603ull;   // FNV-1a 变体
        const auto mix = [&h](long long v) {
            h ^= static_cast<std::uint64_t>(v);
            h *= 1099511628211ull;
        };
        mix(c.x);
        mix(c.y);
        mix(c.z);
        return static_cast<std::size_t>(h);
    }
};

inline CellCoord cellOf(const Point3D& p, double tol)
{
    return { static_cast<long long>(std::floor(p.x / tol)),
             static_cast<long long>(std::floor(p.y / tol)),
             static_cast<long long>(std::floor(p.z / tol)) };
}

// 27 个邻域偏移，(0,0,0) 放最前：绝大多数输入（STL 三角形汤）的重复顶点
// 坐标完全相同，落在自身格子里，第一次查找就命中，避免无谓的 26 次查找。
const long long kNeighborOffsets[27][3] = {
    {  0,  0,  0 },
    { -1,  0,  0 }, {  1,  0,  0 }, {  0, -1,  0 }, {  0,  1,  0 },
    {  0,  0, -1 }, {  0,  0,  1 },
    { -1, -1,  0 }, { -1,  1,  0 }, {  1, -1,  0 }, {  1,  1,  0 },
    { -1,  0, -1 }, { -1,  0,  1 }, {  1,  0, -1 }, {  1,  0,  1 },
    {  0, -1, -1 }, {  0, -1,  1 }, {  0,  1, -1 }, {  0,  1,  1 },
    { -1, -1, -1 }, { -1, -1,  1 }, { -1,  1, -1 }, { -1,  1,  1 },
    {  1, -1, -1 }, {  1, -1,  1 }, {  1,  1, -1 }, {  1,  1,  1 }
};

// 无向边 key
inline uint64_t edgeKey(int a, int b)
{
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}

} // namespace

WeldResult weldSurfaceMesh(const std::vector<Point3D>& vertices,
                           const std::vector<std::array<int, 3>>& triangles,
                           double tolerance)
{
    WeldResult res;
    const double tol = (tolerance > 0.0) ? tolerance : 1e-9;
    const double tol2 = tol * tol;

    // 格子 -> 落在该格子里的代表顶点索引（可能有多个：距离略大于 tol 的点
    // 会共处一格，必须逐个做真实距离比较，不能只凭格子命中就合并）。
    std::unordered_map<CellCoord, std::vector<int>, CellCoordHash> grid;
    grid.reserve(vertices.size() * 2);
    std::vector<int> remap(vertices.size(), -1);
    res.surface.vertices.reserve(vertices.size());

    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        const Point3D& p = vertices[i];
        const CellCoord c = cellOf(p, tol);

        // ★ 邻域探测 ★：只在自身格子里查是不够的 —— 距离小于 tol 的两点
        // 完全可能被格边界分开（例如 x=0.99·tol 与 x=1.01·tol）。逐面离散的
        // B-Rep 表面（STEP/IGES/BRep 经 BRepMesh 三角化）在共享棱上的顶点就是
        // 这种"近似相等但跨格"的情形，漏合并会让一条棱被 4 个面共享，
        // isWatertight 直接判不水密。所以扫 3×3×3。
        int found = -1;
        for (const auto& off : kNeighborOffsets) {
            const auto it = grid.find({ c.x + off[0], c.y + off[1], c.z + off[2] });
            if (it == grid.end())
                continue;
            for (const int cand : it->second) {
                const Point3D& q = res.surface.vertices[cand];
                const double dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
                if (dx * dx + dy * dy + dz * dz <= tol2) {
                    found = cand;
                    break;
                }
            }
            if (found >= 0)
                break;
        }

        if (found >= 0) {
            remap[i] = found;
            ++res.mergedVertices;
        } else {
            const int nv = static_cast<int>(res.surface.vertices.size());
            res.surface.vertices.push_back(p);
            grid[c].push_back(nv);
            remap[i] = nv;
        }
    }

    res.surface.triangles.reserve(triangles.size());
    for (const auto& t : triangles) {
        const int a = remap[t[0]], b = remap[t[1]], c = remap[t[2]];
        if (a < 0 || b < 0 || c < 0)
            continue;
        if (a == b || b == c || a == c) {
            ++res.droppedTriangles;   // 焊接后退化
            continue;
        }
        res.surface.triangles.push_back({ a, b, c });
    }
    return res;
}

bool isWatertight(const SurfaceMesh3D& surface, std::string* error)
{
    std::unordered_map<uint64_t, int> edgeCount;
    edgeCount.reserve(surface.triangles.size() * 3);
    for (const auto& t : surface.triangles) {
        for (int k = 0; k < 3; ++k)
            ++edgeCount[edgeKey(t[k], t[(k + 1) % 3])];
    }
    int bad = 0;
    for (const auto& kv : edgeCount)
        if (kv.second != 2)
            ++bad;
    if (bad != 0) {
        if (error) {
            *error = "surface not watertight: " + std::to_string(bad)
                     + " edge(s) not shared by exactly 2 triangles";
        }
        return false;
    }
    return true;
}

double enclosedVolume(const SurfaceMesh3D& surface)
{
    double v6 = 0.0;
    for (const auto& t : surface.triangles) {
        const Point3D& a = surface.vertices[t[0]];
        const Point3D& b = surface.vertices[t[1]];
        const Point3D& c = surface.vertices[t[2]];
        // a · (b × c)
        v6 += a.x * (b.y * c.z - b.z * c.y)
            - a.y * (b.x * c.z - b.z * c.x)
            + a.z * (b.x * c.y - b.y * c.x);
    }
    return v6 / 6.0;
}

} // namespace mesher
