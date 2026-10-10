// 文件职责：体网格输入检查：拒绝退化、非流形、自交或不闭合表面，先保证约束边界可用再进行体生成。
#include "VolumeBoundaryValidation.h"
#include "geometry/predicates/GeomPredicates.h"
#include "SurfaceMeshValidation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mesher {
namespace {
/// 将有序节点对编码为边查找键。
uint64_t edgeKey(int a, int b)
{
    if (a > b) std::swap(a,b);
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
           static_cast<uint32_t>(b);
}

struct TriangleHash {
    /// 将三角形节点编号组合为边界面哈希键。
    size_t operator()(const std::array<int,3>& triangle) const
    {
        size_t hash = 0;
        for (int vertex : triangle)
            hash ^= std::hash<int>{}(vertex) + 0x9e3779b9U + (hash << 6) + (hash >> 2);
        return hash;
    }
};

struct EdgeUse {
    int a = -1, b = -1;
    int firstTriangle = -1, secondTriangle = -1;
    int count = 0;
    int directionSum = 0;
};

/// 用三个投影的方向符号检查三维三角形是否退化。
bool zeroArea(const Point3D& a, const Point3D& b, const Point3D& c)
{
    // 精确符号的三个投影同时共线才退化，不用模型尺度阈值丢弃薄片。
    return orient2d({a.x,a.y},{b.x,b.y},{c.x,c.y}) == 0 &&
           orient2d({a.y,a.z},{b.y,b.z},{c.y,c.z}) == 0 &&
           orient2d({a.z,a.x},{b.z,b.x},{c.z,c.x}) == 0;
}

struct RelativePoint { long double x, y, z; };
/// 计算点相对指定原点的坐标。
RelativePoint relative(const Point3D& point, const Point3D& origin, long double scale)
{
    return {(static_cast<long double>(point.x)-origin.x)/scale,
            (static_cast<long double>(point.y)-origin.y)/scale,
            (static_cast<long double>(point.z)-origin.z)/scale};
}
}

/// 体网格入口的只读验收：闭合、朝向一致、顶点流形、逐壳非零体积。
VolumeBoundaryValidationResult validateVolumeBoundary(const SurfaceMesh3D& surface,
                                                       bool checkIntersections)
{
    VolumeBoundaryValidationResult result;
    auto fail = [&](std::string error) {
        result.error = std::move(error);
        return result;
    };
    if (surface.vertices.empty() || surface.triangles.empty())
        return fail("empty volume boundary");
    if (surface.vertices.size() > static_cast<size_t>(std::numeric_limits<int>::max()) ||
        surface.triangles.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
        return fail("volume boundary exceeds supported vertex or triangle index range");
    for (size_t vertex = 0; vertex < surface.vertices.size(); ++vertex) {
        const auto& point = surface.vertices[vertex];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            return fail("non-finite volume boundary vertex " + std::to_string(vertex));
    }

    std::unordered_map<std::array<int,3>,int,TriangleHash> triangles;
    triangles.reserve(surface.triangles.size());
    std::unordered_map<uint64_t,size_t> edgeIndices;
    edgeIndices.reserve(surface.triangles.size()*3);
    std::vector<EdgeUse> edges;
    std::vector<std::vector<int>> incident(surface.vertices.size());
    for (int triangle = 0; triangle < static_cast<int>(surface.triangles.size()); ++triangle) {
        const auto& t = surface.triangles[triangle];
        for (int vertex : t) {
            if (vertex < 0 || vertex >= static_cast<int>(surface.vertices.size()))
                return fail("volume boundary triangle " + std::to_string(triangle) +
                            " has invalid vertex " + std::to_string(vertex));
        }
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0])
            return fail("volume boundary triangle " + std::to_string(triangle) +
                        " repeats a vertex");
        auto sorted = t;
        std::sort(sorted.begin(),sorted.end());
        const auto inserted = triangles.emplace(sorted,triangle);
        if (!inserted.second)
            return fail("duplicate volume boundary triangles " +
                        std::to_string(inserted.first->second) + " and " + std::to_string(triangle));
        if (zeroArea(surface.vertices[t[0]],surface.vertices[t[1]],surface.vertices[t[2]]))
            return fail("zero-area volume boundary triangle " + std::to_string(triangle) +
                        " at vertices " + std::to_string(t[0]) + ", " +
                        std::to_string(t[1]) + ", " + std::to_string(t[2]));
        for (int k = 0; k < 3; ++k) {
            incident[t[k]].push_back(triangle);
            const int a = t[k], b = t[(k+1)%3];
            const auto edge = edgeIndices.emplace(edgeKey(a,b),edges.size());
            if (edge.second) edges.push_back({std::min(a,b),std::max(a,b)});
            auto& use = edges[edge.first->second];
            if (use.count == 0) use.firstTriangle = triangle;
            if (use.count == 1) use.secondTriangle = triangle;
            ++use.count;
            use.directionSum += a < b ? 1 : -1;
        }
    }

    std::vector<std::vector<int>> neighbors(surface.triangles.size());
    for (const auto& edge : edges) {
        const std::string description = "edge (vertex " + std::to_string(edge.a) +
                                       ", vertex " + std::to_string(edge.b) + ")";
        if (edge.count != 2)
            return fail("volume boundary " + description + " at triangle " +
                        std::to_string(edge.firstTriangle) + " has " +
                        std::to_string(edge.count) + " incident triangles; expected 2");
        if (edge.directionSum != 0)
            return fail("inconsistent winding at volume boundary " + description +
                        " between triangle " + std::to_string(edge.firstTriangle) +
                        " and triangle " + std::to_string(edge.secondTriangle));
        neighbors[edge.firstTriangle].push_back(edge.secondTriangle);
        neighbors[edge.secondTriangle].push_back(edge.firstTriangle);
    }

    for (int vertex = 0; vertex < static_cast<int>(incident.size()); ++vertex) {
        if (incident[vertex].empty()) continue;
        // 每片在顶点对面的边构成 link；一个流形顶点的 link 必须是单一环。
        std::unordered_map<int,std::vector<int>> link;
        for (int triangle : incident[vertex]) {
            const auto& t = surface.triangles[triangle];
            int a = -1, b = -1;
            for (int node : t) if (node != vertex) {
                if (a < 0) a = node; else b = node;
            }
            link[a].push_back(b);
            link[b].push_back(a);
        }
        for (const auto& entry : link) {
            if (entry.second.size() != 2)
                return fail("non-manifold volume boundary vertex " + std::to_string(vertex) +
                            " near triangle " + std::to_string(incident[vertex].front()) +
                            ": link node " + std::to_string(entry.first) + " does not have degree 2");
        }
        std::unordered_set<int> visited;
        std::vector<int> pending{link.begin()->first};
        while (!pending.empty()) {
            const int node = pending.back(); pending.pop_back();
            if (!visited.insert(node).second) continue;
            for (int neighbor : link.at(node)) pending.push_back(neighbor);
        }
        if (visited.size() != link.size())
            return fail("non-manifold volume boundary vertex " + std::to_string(vertex) +
                        " near triangle " + std::to_string(incident[vertex].front()) +
                        ": link contains multiple disconnected cycles");
    }

    if (checkIntersections && !validateSurfaceIntersections(surface,&result.error)) return result;

    std::vector<bool> visited(surface.triangles.size(),false);
    long double totalVolume = 0;
    for (int first = 0; first < static_cast<int>(surface.triangles.size()); ++first) {
        if (visited[first]) continue;
        std::vector<int> shell, pending{first};
        visited[first] = true;
        while (!pending.empty()) {
            const int triangle = pending.back(); pending.pop_back();
            shell.push_back(triangle);
            for (int neighbor : neighbors[triangle]) if (!visited[neighbor]) {
                visited[neighbor] = true;
                pending.push_back(neighbor);
            }
        }
        ++result.shellCount;
        const Point3D& origin = surface.vertices[surface.triangles[first][0]];
        long double scale = 0;
        for (int triangle : shell) for (int vertex : surface.triangles[triangle]) {
            const auto& point = surface.vertices[vertex];
            scale = std::max({scale,std::abs(static_cast<long double>(point.x)-origin.x),
                              std::abs(static_cast<long double>(point.y)-origin.y),
                              std::abs(static_cast<long double>(point.z)-origin.z)});
        }
        if (!(scale > 0) || !std::isfinite(scale))
            return fail("invalid volume scale for shell at triangle " + std::to_string(first));
        long double volume6 = 0, correction = 0;
        for (int triangle : shell) {
            const auto& t = surface.triangles[triangle];
            const auto a = relative(surface.vertices[t[0]],origin,scale);
            const auto b = relative(surface.vertices[t[1]],origin,scale);
            const auto c = relative(surface.vertices[t[2]],origin,scale);
            const long double determinant = a.x*(b.y*c.z-b.z*c.y) -
                                            a.y*(b.x*c.z-b.z*c.x) + a.z*(b.x*c.y-b.y*c.x);
            const long double adjusted = determinant-correction;
            const long double sum = volume6+adjusted;
            correction = (sum-volume6)-adjusted;
            volume6 = sum;
        }
        if (volume6 == 0 || !std::isfinite(volume6))
            return fail("zero or non-finite volume for shell at triangle " + std::to_string(first));
        const long double volume = (volume6/6)*scale*scale*scale;
        if (!std::isfinite(volume) || volume == 0)
            return fail("unrepresentable volume for shell at triangle " + std::to_string(first));
        totalVolume += volume;
    }
    result.signedVolume = static_cast<double>(totalVolume);
    if (!std::isfinite(result.signedVolume)) return fail("non-finite total volume of boundary shells");
    result.ok = true;
    return result;
}

} // namespace mesher
