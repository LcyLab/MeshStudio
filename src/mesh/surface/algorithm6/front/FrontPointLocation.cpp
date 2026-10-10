/** @file FrontPointLocation.cpp
 * @brief 原前沿包含片定位和重心尺寸插值。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontPointLocation.h"
#include "geometry/predicates/GeomPredicates.h"
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); }
/// 查询前沿三角形三个顶点对应的节点尺寸记录。
std::array<const InitializedNodeSize*,3> triangleNodes(const FrontTriangle& triangle,const NodeSizeState& nodes) {
    std::array<const InitializedNodeSize*,3> result;
    for(int i = 0; i < 3; ++i) {
        const auto found = nodes.indices.find(triangle.vertices[i]);
        if(found == nodes.indices.end() || found->second >= nodes.nodes.size() || nodes.nodes[found->second].vertex != triangle.vertices[i])
            throw OperationFailure{{MeshingStatus::InvalidTopology,"front triangle node index mismatch"}};
        result[i] = &nodes.nodes[found->second];
    }
    return result;
}
/// 检查操作状态或前置条件，失败时抛出本模块的明确错误。
void require(OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; }
}
/// 计算候选 UV 在指定三角形中的重心坐标。
FrontPointCoordinates mapFrontPointToTriangle(const FrontTriangle& triangle,const NodeSizeState& nodes,Point2D point,double tolerance)
{
    FrontPointCoordinates result;
    try {
        if(!finite(point) || !std::isfinite(tolerance)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite front inverse-map input"}};
        const auto n = triangleNodes(triangle,nodes); const auto a = n[0]->parameters,b = n[1]->parameters,c = n[2]->parameters;
        if(!finite(a) || !finite(b) || !finite(c)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite front triangle parameters"}};
        const double m00 = b.x-a.x,m01 = c.x-a.x,m10 = b.y-a.y,m11 = c.y-a.y;
        const double r0 = point.x-a.x,r1 = point.y-a.y;
        const double norm = std::pow(m00,2)+std::pow(m11,2)+std::pow(m01,2)+std::pow(m10,2);
        const double determinant = m00*m11-m10*m01;
        if(!std::isfinite(norm) || !std::isfinite(determinant)) throw OperationFailure{{MeshingStatus::NonConverged,"front inverse-map system overflow"}};
        result.singularSystem = norm == 0. || std::abs(determinant)/norm < 1.e-16;
        if(!result.singularSystem) {
            const double inverse = 1./determinant;
            result.local = {r0*m11-m01*r1,m00*r1-m10*r0};
            result.local.x *= inverse; result.local.y *= inverse;
        }
        if(!finite(result.local)) throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite front inverse-map result"}};
        const auto uv = result.local;
        result.inside = uv.x >= -tolerance && uv.y >= -tolerance && uv.x <= 1.+tolerance && uv.y <= 1.+tolerance && 1.-uv.x-uv.y > -tolerance;
    } catch(const OperationFailure& error) { result.status = error.status; }
    return result;
}
/// 检查搜索线段与前沿边的相交关系。
bool intersectFrontSearchSegments(Point2D a,Point2D b,Point2D c,Point2D d)
{
    if(!finite(a) || !finite(b) || !finite(c) || !finite(d)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite front search segment"}};
    double s0 = orient2dValue(a,b,c),s1 = orient2dValue(a,b,d);
    if(!std::isfinite(s0) || !std::isfinite(s1)) throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite front search orientation"}};
    if(s0*s1 > 0.) return false;
    s0 = orient2dValue(c,d,a); s1 = orient2dValue(c,d,b);
    if(!std::isfinite(s0) || !std::isfinite(s1)) throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite front search orientation"}};
    return !(s0*s1 > 0.);
}
/// 沿前沿邻接定位包含候选点的三角形。
FrontTriangleSearch findContainingFrontTriangle(const FrontMesh& mesh,const NodeSizeState& nodes,
    const FrontTriangleSet& all,int start,Point2D point,bool force,const RunControl& control)
{
    FrontTriangleSearch result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"front triangle search cancelled"}}; };
        auto triangle = [&](int index)->const FrontTriangle& {
            if(index < 0 || std::size_t(index) >= mesh.triangles.size()) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid front search triangle"}};
            return mesh.triangles[index];
        };
        auto visit = [&](int index) {
            check(); result.visitedTriangles.push_back(index);
            result.coordinates = mapFrontPointToTriangle(triangle(index),nodes,point); require(result.coordinates.status);
            result.coordinateTests.push_back({index,result.coordinates});
            if(result.coordinates.inside) { result.triangle = index; return true; } return false;
        };
        check(); int current = start;
        if(visit(current)) return result;
        std::size_t iteration = 0;
        while(true) {
            check(); const auto& t = triangle(current); const auto n = triangleNodes(t,nodes);
            const Point2D center{(n[0]->parameters.x+n[1]->parameters.x+n[2]->parameters.x)*(1./3.),
                                 (n[0]->parameters.y+n[1]->parameters.y+n[2]->parameters.y)*(1./3.)};
            int edge = 0;
            for(; edge < 3; ++edge) if(intersectFrontSearchSegments(n[(edge+2)%3]->parameters,n[edge]->parameters,point,center)) break;
            if(edge == 3) { result.noCrossedEdge = true; break; }
            result.crossedEdges.push_back(edge); current = t.neighbors[edge];
            if(current == -1) break;
            if(visit(current)) return result;
            if(iteration++ > all.size()) { result.iterationLimit = true; break; }
        }
        if(force) for(const auto& key : all) {
            check(); const auto& t = triangle(key.triangle); if(t.deleted) continue;
            result.forceScannedTriangles.push_back(key.triangle);
            result.coordinates = mapFrontPointToTriangle(t,nodes,point); require(result.coordinates.status);
            result.coordinateTests.push_back({key.triangle,result.coordinates});
            if(result.coordinates.inside) { result.triangle = key.triangle; return result; }
        }
        check();
    } catch(const OperationFailure& error) { result.status = error.status; }
    return result;
}
/// 按重心权重插值候选点的节点尺寸。
OperationStatus interpolateNodeMeshSize(const FrontTriangle& triangle,const NodeSizeState& nodes,Point2D local,double& output)
{
    try {
        if(!finite(local)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite front size weights"}};
        const auto n = triangleNodes(triangle,nodes);
        for(const auto* node : n) if(!std::isfinite(node->sizes.meshSize)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite front mesh size"}};
        const double size = (1.-local.x-local.y)*n[0]->sizes.meshSize+local.x*n[1]->sizes.meshSize+local.y*n[2]->sizes.meshSize;
        if(!std::isfinite(size)) throw OperationFailure{{MeshingStatus::NonConverged,"front mesh size interpolation overflow"}};
        output = size; return {};
    } catch(const OperationFailure& error) { return error.status; }
}
}
