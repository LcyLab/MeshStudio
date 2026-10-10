#pragma once
/** @file SurfaceMetric.h
 * @brief 第一基本形式及原生外接圆构造；不混用旧 SurfaceChart 的 Cholesky 因子。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/data/SurfaceTypes.h"
#include <array>

namespace mesher::gmshReference {
struct SurfaceMetric { double gUU = 0., gUV = 0., gVV = 0.; };
struct MetricEvaluation {
    OperationStatus status;
    SurfaceMetric metric;
    bool singularDerivatives = false;
};
struct MetricCircumcircle {
    OperationStatus status;
    Point2D center;
    double radiusSquared = 0.; // 物理长度平方。
    bool singularLinearSystem = false;
};
struct PhysicalCircumradius {
    OperationStatus status;
    Point3D center;
    double radius = 0.;
    bool degenerateTriangle = false;
};

/** buildMetric：直接对 CAD 的 du/dv 作内积。读取导数，不查询或修改 CAD。
 * 允许周期极点的奇异导数并标记；非有限输入/溢出明确失败。
 */
MetricEvaluation evaluateSurfaceMetric(Point3D du, Point3D dv);

/** circumCenterMetric 和 Numeric::sys2x2；不移动 UV 原点以避免改变基线舍入。
 * 原生 |det|/norm<1e-16 置外心为 (0,0)，同时标记奇异，不冒充正常外心。
 */
MetricCircumcircle computeMetricCircumcircle(const std::array<Point2D, 3>& triangle, SurfaceMetric metric);

/** Numeric::circumCenterXYZ：投影至物理三角平面再构造圆。
 * 退化保留原生 circumCenterXY 的 -99999 哨兵行为并显式标记。
 * 返回物理长度，不除以 meshSize 或最短边。
 */
PhysicalCircumradius computePhysicalCircumradius(const std::array<Point3D, 3>& triangle);
} // namespace mesher::gmshReference
