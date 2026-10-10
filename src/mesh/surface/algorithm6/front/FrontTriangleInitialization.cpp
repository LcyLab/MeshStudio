/** @file FrontTriangleInitialization.cpp
 * @brief 初始前沿三角运行状态。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontTriangleInitialization.h"
#include "mesh/surface/algorithm6/geometry/SurfaceMetric.h"
#include "FrontTriangleOrdering.h"
#include <algorithm>
#include <cmath>
#include <set>
namespace mesher::gmshReference {
/// 由初始面网格建立前沿三角形、半径和排序快照。
FrontTriangleInitializationResult initializeFrontTriangles(const FaceMesh& mesh,
    const NodeSizeState& nodes,bool extendBoundarySize,FrontMesh& output,const RunControl& control)
{
    FrontTriangleInitializationResult result;
    const auto cancelled = [&] { return control.keepGoing && !control.keepGoing(); };
    if(cancelled()) {
        result.status = {MeshingStatus::Cancelled,"front triangle initialization cancelled"}; return result;
    }
    if(nodes.nodes.size() > control.maximumNodes) {
        result.status = {MeshingStatus::BudgetExceeded,"front node budget exceeded"}; return result;
    }
    if(nodes.indices.size() != nodes.nodes.size()) {
        result.status = {MeshingStatus::InvalidTopology,"front node index size mismatch"}; return result;
    }
    for(std::size_t i = 0; i < nodes.nodes.size(); ++i) {
        const auto& node = nodes.nodes[i]; const auto found = nodes.indices.find(node.vertex);
        if(found == nodes.indices.end() || found->second != i) {
            result.status = {MeshingStatus::InvalidTopology,"front node index mismatch"}; return result;
        }
        if(!std::isfinite(node.sizes.meshSize) || !std::isfinite(node.sizes.backgroundSize)) {
            result.status = {MeshingStatus::InvalidInput,"nonfinite front node size"}; return result;
        }
    }
    FrontMesh prepared;
    prepared.triangles.reserve(mesh.triangles.size());
    std::set<FrontTriangleKey,FrontTriangleKeyLess> ordered;
    for(const auto& ids : mesh.triangles) {
        if(cancelled()) {
            result.status = {MeshingStatus::Cancelled,"front triangle initialization cancelled"}; return result;
        }
        std::array<Point3D,3> points;
        std::array<NodeSize,3> sizes;
        for(int i = 0; i < 3; ++i) {
            const auto vertex = mesh.vertices.find(ids[i]); const auto index = nodes.indices.find(ids[i]);
            if(vertex == mesh.vertices.end() || index == nodes.indices.end()) {
                result.status = {MeshingStatus::InvalidTopology,"front triangle references missing vertex or node"}; return result;
            }
            points[i] = vertex->second.point; sizes[i] = nodes.nodes[index->second].sizes;
        }
        FrontTriangle triangle; triangle.vertices = ids;
        // 保留原版字面常数和加法顺序，不能改成除以3或先各自加权。
        triangle.meanSizes.meshSize = .3333333333 * (sizes[0].meshSize + sizes[1].meshSize + sizes[2].meshSize);
        triangle.meanSizes.backgroundSize = .3333333333 * (sizes[0].backgroundSize + sizes[1].backgroundSize + sizes[2].backgroundSize);
        triangle.targetSize = extendBoundarySize ? std::min(triangle.meanSizes.meshSize,triangle.meanSizes.backgroundSize) : triangle.meanSizes.backgroundSize;
        if(!std::isfinite(triangle.meanSizes.meshSize) || !std::isfinite(triangle.meanSizes.backgroundSize)) {
            result.status = {MeshingStatus::NonConverged,"nonfinite mean front triangle size"}; return result;
        }
        const auto circle = computePhysicalCircumradius(points);
        if(!circle.status.complete()) { result.status = circle.status; return result; }
        triangle.radius = circle.radius / triangle.targetSize;
        if(!std::isfinite(triangle.radius)) {
            result.status = {MeshingStatus::NonConverged,"nonfinite normalized front triangle radius"}; return result;
        }
        const int index = static_cast<int>(prepared.triangles.size());
        prepared.triangles.push_back(triangle);
        if(!ordered.insert(makeFrontTriangleKey(triangle,index)).second) result.rejectedDuplicateTriangles.push_back(index);
    }
    for(const auto& key : ordered) prepared.orderedTriangles.push_back(key.triangle);
    struct Edge { std::array<std::uint64_t,2> vertices; int triangle,localEdge; };
    std::vector<Edge> edges; edges.reserve(3 * ordered.size());
    for(int index : prepared.orderedTriangles) {
        const auto& triangle = prepared.triangles[index];
        for(int edge = 0; edge < 3; ++edge) {
            std::array<std::uint64_t,2> ids{triangle.vertices[(edge+2)%3],triangle.vertices[edge]};
            if(ids[1] < ids[0]) std::swap(ids[0],ids[1]);
            edges.push_back({ids,index,edge});
        }
    }
    // 原生connectTris只比较身份边键，非流形重复边仍按同一次std::sort后的成对序连接。
    std::sort(edges.begin(),edges.end(),[](const Edge& a,const Edge& b) { return a.vertices < b.vertices; });
    for(std::size_t i = 0; i + 1 < edges.size(); ++i) {
        const auto& a = edges[i]; const auto& b = edges[i+1];
        if(a.vertices == b.vertices && a.triangle != b.triangle) {
            prepared.triangles[a.triangle].neighbors[a.localEdge] = b.triangle;
            prepared.triangles[b.triangle].neighbors[b.localEdge] = a.triangle;
            ++i;
        }
    }
    if(cancelled()) {
        result.status = {MeshingStatus::Cancelled,"front triangle initialization cancelled"}; return result;
    }
    output = std::move(prepared);
    return result;
}
}
