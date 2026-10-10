/** @file FrontPointPlacement.cpp
 * @brief 度量候选点与普通CAD圆修正。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontPointPlacement.h"
#include <algorithm>
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); }
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
/// 计算向量叉积，确定有向关系。
Point3D cross(Point3D a,Point3D b)
{ return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
/// 计算向量的欧氏长度。
double norm(Point3D p) { return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z); }
/// 将非零三维向量归一化。
void normalize(Point3D& p) {
    const double length = norm(p);
    if(length != 0.) { const double inverse = 1./length; p.x *= inverse; p.y *= inverse; p.z *= inverse; }
}
/// 计算参数平面中的三角形外心，退化时返回无效标记。
Point2D planarCircumcenter(const std::array<Point2D,3>& p) {
    const double d = 2.*(p[0].y*(p[1].x-p[2].x)+p[1].y*(p[2].x-p[0].x)+p[2].y*(p[0].x-p[1].x));
    if(d == 0.) return {-99999.,-99999.};
    const double a = p[0].x*p[0].x+p[0].y*p[0].y;
    const double b = p[1].x*p[1].x+p[1].y*p[1].y;
    const double c = p[2].x*p[2].x+p[2].y*p[2].y;
    return {(a*(p[2].y-p[1].y)+b*(p[0].y-p[2].y)+c*(p[1].y-p[0].y))/d,
            (a*(p[1].x-p[2].x)+b*(p[2].x-p[0].x)+c*(p[0].x-p[1].x))/d};
}
/// 查询调用方是否已请求取消。
bool cancelled(const RunControl& control) { return control.keepGoing && !control.keepGoing(); }
}
/// 根据活动边及局部曲面度量计算前沿候选点。
FrontPointPlacementResult computeFrontPoint(const FrontTriangle& triangle,int edge,const NodeSizeState& nodes,
    bool extend,const FrontMetricSource& source,FrontPointCandidate& output,const RunControl& control)
{
    FrontPointPlacementResult result;
    if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front point computation cancelled"}; return result; }
    if(edge < 0 || edge > 2 || !source) { result.status = {MeshingStatus::InvalidInput,"invalid front edge or missing metric source"}; return result; }
    std::array<Point2D,3> p; std::array<NodeSize,3> sizes;
    for(int i = 0; i < 3; ++i) {
        const auto index = nodes.indices.find(triangle.vertices[i]);
        if(index == nodes.indices.end() || index->second >= nodes.nodes.size() || nodes.nodes[index->second].vertex != triangle.vertices[i]) {
            result.status = {MeshingStatus::InvalidTopology,"front triangle node index mismatch"}; return result;
        }
        const auto& node = nodes.nodes[index->second]; p[i] = node.parameters; sizes[i] = node.sizes;
        if(!finite(p[i]) || !std::isfinite(sizes[i].meshSize) || !std::isfinite(sizes[i].backgroundSize)) {
            result.status = {MeshingStatus::InvalidInput,"nonfinite front node data"}; return result;
        }
    }
    FrontPointCandidate candidate;
    candidate.planarCenter = planarCircumcenter(p);
    candidate.metricQuery = {(p[0].x+p[1].x+p[2].x)*(1./3.),(p[0].y+p[1].y+p[2].y)*(1./3.)};
    if(!finite(candidate.metricQuery)) { result.status = {MeshingStatus::NonConverged,"front metric query overflow"}; return result; }
    if(control.maximumGeometryEvaluations == 0) { result.status = {MeshingStatus::BudgetExceeded,"front metric query budget exceeded"}; return result; }
    ++result.geometryEvaluations;
    try { candidate.metric = source(candidate.metricQuery); }
    catch(const OperationFailure& error) { result.status = error.status; return result; }
    catch(const std::exception& error) { result.status = {MeshingStatus::GeometryQueryFailed,error.what()}; return result; }
    catch(...) { result.status = {MeshingStatus::GeometryQueryFailed,"front metric query threw an unknown exception"}; return result; }
    const auto circle = computeMetricCircumcircle(p,candidate.metric);
    if(!circle.status.complete()) { result.status = circle.status; return result; }
    candidate.metricCenter = circle.center;
    const int a = (edge+2)%3, b = edge;
    candidate.edgeMidpoint = {.5*(p[a].x+p[b].x),.5*(p[a].y+p[b].y)};
    double du = circle.center.x-candidate.edgeMidpoint.x, dv = circle.center.y-candidate.edgeMidpoint.y;
    const double dx = du, dy = dv;
    candidate.directionNorm = std::sqrt(du*du+dv*dv);
    du /= candidate.directionNorm; dv /= candidate.directionNorm;
    const auto& m = candidate.metric;
    candidate.metricRatio = std::sqrt(du*du*m.gUU+2*dv*du*m.gUV+dv*dv*m.gVV);
    candidate.edgeMeanSizes = {.5*(sizes[a].meshSize+sizes[b].meshSize),.5*(sizes[a].backgroundSize+sizes[b].backgroundSize)};
    candidate.targetSize = extend ? std::min(candidate.edgeMeanSizes.meshSize,candidate.edgeMeanSizes.backgroundSize) : candidate.edgeMeanSizes.backgroundSize;
    candidate.distanceToCenter = std::sqrt(dx*m.gUU*dx+2*dx*m.gUV*dy+dy*m.gVV*dy);
    candidate.height = std::min(candidate.targetSize*std::sqrt(3.)*.5,candidate.distanceToCenter);
    candidate.fastPoint = {candidate.edgeMidpoint.x+candidate.height*du/candidate.metricRatio,
                           candidate.edgeMidpoint.y+candidate.height*dv/candidate.metricRatio};
    candidate.point = candidate.fastPoint; candidate.geometryEvaluations = result.geometryEvaluations;
    if(!finite(candidate.fastPoint) || !std::isfinite(candidate.height) || !std::isfinite(candidate.distanceToCenter) ||
       !std::isfinite(candidate.targetSize) || !std::isfinite(candidate.edgeMeanSizes.meshSize) || !std::isfinite(candidate.edgeMeanSizes.backgroundSize)) {
        result.status = {MeshingStatus::NonConverged,"nonfinite native front candidate"}; return result;
    }
    if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front point computation cancelled"}; return result; }
    output = candidate; return result;
}
/// 通过圆与曲面求交修正候选点的位置。
FrontPointPlacementResult correctFrontPointOnSurface(const FrontTriangle& triangle,int edge,const FaceMesh& mesh,
    const FrontPointCandidate& input,const SurfaceFunction& surface,FrontPointCandidate& output,const RunControl& control)
{
    FrontPointPlacementResult result; result.geometryEvaluations = input.geometryEvaluations;
    if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front point correction cancelled"}; return result; }
    if(edge < 0 || edge > 2 || !surface || !finite(input.fastPoint) || !std::isfinite(input.height)) {
        result.status = {MeshingStatus::InvalidInput,"invalid front surface correction input"}; return result;
    }
    if(input.geometryEvaluations > control.maximumGeometryEvaluations) {
        result.status = {MeshingStatus::BudgetExceeded,"front correction prior query budget exceeded"}; return result;
    }
    std::array<Point3D,3> p;
    for(int i = 0; i < 3; ++i) {
        const auto found = mesh.vertices.find(triangle.vertices[i]);
        if(found == mesh.vertices.end()) { result.status = {MeshingStatus::InvalidTopology,"front correction references missing physical vertex"}; return result; }
        p[i] = found->second.point;
        if(!finite(p[i])) { result.status = {MeshingStatus::InvalidInput,"nonfinite front physical vertex"}; return result; }
    }
    FrontPointCandidate candidate = input;
    candidate.point = input.fastPoint; candidate.skippedDegenerateNormal = false;
    candidate.correctionAttempted = candidate.intersectionFound = candidate.retainedFastPoint = false;
    const auto& a = p[(edge+2)%3]; const auto& b = p[edge]; const auto& c = p[(edge+1)%3];
    candidate.circle.center = {(a.x+b.x)*.5,(a.y+b.y)*.5,(a.z+b.z)*.5};
    const Point3D edgeVector{b.x-a.x,b.y-a.y,b.z-a.z};
    const Point3D offset{c.x-candidate.circle.center.x,c.y-candidate.circle.center.y,c.z-candidate.circle.center.z};
    auto normal = cross(edgeVector,offset); candidate.normalNorm = norm(normal);
    if(!std::isfinite(candidate.normalNorm)) { result.status = {MeshingStatus::NonConverged,"front physical normal overflow"}; return result; }
    if(candidate.normalNorm < 1.e-12) {
        candidate.skippedDegenerateNormal = candidate.retainedFastPoint = true;
        if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front point correction cancelled"}; return result; }
        output = candidate; return result;
    }
    auto transverse = cross(normal,edgeVector); normalize(normal); normalize(transverse);
    candidate.circle.cosineAxis = transverse; candidate.circle.sineAxis = normal; candidate.circle.radius = input.height;
    candidate.correctionAttempted = true;
    RunControl remaining = control; remaining.maximumGeometryEvaluations -= input.geometryEvaluations;
    result.intersection = intersectCircleWithSurface(candidate.circle,surface,{input.fastPoint.x,input.fastPoint.y,0.},remaining);
    const auto& intersection = *result.intersection;
    result.geometryEvaluations += intersection.residualEvaluations;
    candidate.geometryEvaluations = result.geometryEvaluations;
    if(intersection.intersectionFound) {
        candidate.intersectionFound = true; candidate.point = {intersection.parameters[0],intersection.parameters[1]};
    }
    else if(intersection.status.code == MeshingStatus::NonConverged || intersection.status.code == MeshingStatus::GeometryQueryFailed) {
        // 普通CAD原生返回true，失败参数仍是原快候选；诊断保留在intersection中。
        candidate.retainedFastPoint = true;
    }
    else { result.status = intersection.status; return result; }
    if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front point correction cancelled"}; return result; }
    output = candidate; return result;
}
}
