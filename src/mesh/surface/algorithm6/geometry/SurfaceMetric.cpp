// 文件职责：曲面度量计算：从几何导数构造局部长度度量，供初始合法化和前沿候选判定使用。
#include "SurfaceMetric.h"
#include <cmath>

namespace mesher::gmshReference {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point2D point) { return std::isfinite(point.x) && std::isfinite(point.y); }
/// 检查输入坐标或数值是否有限。
bool finite(Point3D point) { return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z); }
/// 计算三维向量点积。
double inner(Point3D a, Point3D b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
/// 将两数之差表示为保留低位残差的浮点展开。
Point3D difference(Point3D a, Point3D b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
/// 计算三维向量叉积。
Point3D vectorProduct(Point3D a, Point3D b)
{ return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

/// 将非零向量归一化并报告退化情况。
double normalizeVector(Point3D& vector)
{
    const double length = std::sqrt(inner(vector, vector));
    if(length != 0.) {
        const double inverse = 1. / length;
        vector.x *= inverse; vector.y *= inverse; vector.z *= inverse;
    }
    return length;
}
} // namespace

/// 由曲面一阶导数计算局部度量。
MetricEvaluation evaluateSurfaceMetric(Point3D du, Point3D dv)
{
    MetricEvaluation result;
    if(!finite(du) || !finite(dv)) {
        result.status = {MeshingStatus::GeometryQueryFailed, "nonfinite surface derivative"};
        return result;
    }
    result.metric = {inner(du, du), inner(dv, du), inner(dv, dv)};
    const auto& metric = result.metric;
    if(!std::isfinite(metric.gUU) || !std::isfinite(metric.gUV) || !std::isfinite(metric.gVV))
        result.status = {MeshingStatus::GeometryQueryFailed, "surface metric overflow"};
    result.singularDerivatives = metric.gUU * metric.gVV - metric.gUV * metric.gUV <= 0.;
    return result;
}

/// 在局部曲面度量下计算三角形外心和半径。
MetricCircumcircle computeMetricCircumcircle(const std::array<Point2D, 3>& triangle, SurfaceMetric metric)
{
    MetricCircumcircle result;
    if(!finite(triangle[0]) || !finite(triangle[1]) || !finite(triangle[2]) ||
       !std::isfinite(metric.gUU) || !std::isfinite(metric.gUV) || !std::isfinite(metric.gVV)) {
        result.status = {MeshingStatus::InvalidInput, "nonfinite metric circumcircle input"};
        return result;
    }
    const auto& origin = triangle[0];
    const double a = metric.gUU, b = metric.gUV, d = metric.gVV;
    double matrix[2][2], rhs[2];
    for(int row = 0; row < 2; ++row) {
        const auto& other = triangle[row + 1];
        matrix[row][0] = 2. * a * (origin.x - other.x) + 2. * b * (origin.y - other.y);
        matrix[row][1] = 2. * d * (origin.y - other.y) + 2. * b * (origin.x - other.x);
        rhs[row] = a * (origin.x * origin.x - other.x * other.x) +
                   d * (origin.y * origin.y - other.y * other.y) +
                   2. * b * (origin.x * origin.y - other.x * other.y);
    }
    // 保留 sys2x2 的范数和绝对归一阈值，不换成 robust orient 或新条件数规则。
    const double norm = std::pow(matrix[0][0], 2) + std::pow(matrix[1][1], 2) +
                        std::pow(matrix[0][1], 2) + std::pow(matrix[1][0], 2);
    const double determinant = matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1];
    if(!std::isfinite(norm) || !std::isfinite(determinant) || !std::isfinite(rhs[0]) || !std::isfinite(rhs[1])) {
        result.status = {MeshingStatus::GeometryQueryFailed, "metric circumcircle system overflow"};
        return result;
    }
    result.singularLinearSystem = norm == 0. || std::abs(determinant) / norm < 1.e-16;
    if(!result.singularLinearSystem) {
        const double inverse = 1. / determinant;
        result.center.x = (rhs[0] * matrix[1][1] - matrix[0][1] * rhs[1]) * inverse;
        result.center.y = (matrix[0][0] * rhs[1] - matrix[1][0] * rhs[0]) * inverse;
    }
    const double deltaU = result.center.x - origin.x, deltaV = result.center.y - origin.y;
    result.radiusSquared = deltaU * deltaU * a + deltaV * deltaV * d + 2. * deltaU * deltaV * b;
    if(!finite(result.center) || !std::isfinite(result.radiusSquared))
        result.status = {MeshingStatus::GeometryQueryFailed, "metric circumcircle result overflow"};
    return result;
}

/// 从三维顶点计算三角形的物理外接圆半径。
PhysicalCircumradius computePhysicalCircumradius(const std::array<Point3D, 3>& triangle)
{
    PhysicalCircumradius result;
    if(!finite(triangle[0]) || !finite(triangle[1]) || !finite(triangle[2])) {
        result.status = {MeshingStatus::InvalidInput, "nonfinite physical circumcircle input"};
        return result;
    }
    const auto firstEdge = difference(triangle[1], triangle[0]);
    const auto secondEdge = difference(triangle[2], triangle[0]);
    auto tangent = firstEdge;
    auto normal = vectorProduct(firstEdge, secondEdge);
    auto transverse = vectorProduct(normal, tangent);
    normalizeVector(tangent); normalizeVector(transverse); normalizeVector(normal);
    const Point2D second{inner(firstEdge, tangent), inner(firstEdge, transverse)};
    const Point2D third{inner(secondEdge, tangent), inner(secondEdge, transverse)};
    const double denominator = 2. * (second.y * third.x - third.y * second.x);
    Point2D projectedCenter;
    result.degenerateTriangle = denominator == 0.;
    if(result.degenerateTriangle) projectedCenter = {-99999., -99999.};
    else {
        const double secondNorm = second.x * second.x + second.y * second.y;
        const double thirdNorm = third.x * third.x + third.y * third.y;
        projectedCenter.x = (secondNorm * -third.y + thirdNorm * second.y) / denominator;
        projectedCenter.y = (secondNorm * third.x - thirdNorm * second.x) / denominator;
    }
    result.center = {triangle[0].x + projectedCenter.x * tangent.x + projectedCenter.y * transverse.x,
                     triangle[0].y + projectedCenter.x * tangent.y + projectedCenter.y * transverse.y,
                     triangle[0].z + projectedCenter.x * tangent.z + projectedCenter.y * transverse.z};
    const auto offset = difference(triangle[0], result.center);
    result.radius = std::sqrt(inner(offset, offset));
    if(!finite(result.center) || !std::isfinite(result.radius))
        result.status = {MeshingStatus::GeometryQueryFailed, "physical circumcircle overflow"};
    return result;
}
} // namespace mesher::gmshReference
