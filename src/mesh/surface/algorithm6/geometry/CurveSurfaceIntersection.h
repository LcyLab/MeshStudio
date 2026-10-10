#pragma once
/** @file CurveSurfaceIntersection.h
 * @brief 原生有限差分 Newton 和圆/曲面修正；不使用解析导数或线搜索。
 * @date 2026-10-07
 */
#include "geometry/numerics/LinearSystem3.h"
#include "mesh/surface/algorithm6/data/SurfaceTypes.h"

namespace mesher::gmshReference {
using CurveFunction = std::function<Point3D(double)>;
using SurfaceFunction = std::function<Point3D(Point2D)>;
enum class IntersectionStop {
    InitialResidual, NewtonStep, ExactZeroResidualExit, ParameterNormLimit,
    IterationLimit, NonfiniteStep, InvalidQuery, Cancelled, BudgetExceeded
};
struct CurveSurfaceIntersectionResult {
    OperationStatus status;
    Vector3 parameters{}; // u、v、曲线参数 t；失败保留原初值。
    bool intersectionFound = false;
    IntersectionStop stop = IntersectionStop::IterationLimit;
    int iterations = 0;
    std::size_t residualEvaluations = 0;
    double lastResidualNorm = 0.;
};
struct Circle3 {
    Point3D center;
    Point3D cosineAxis;
    Point3D sineAxis;
    double radius = 0.; // 原生有符号半径；允许负值，不对epsilon取绝对值。
};

/** Numeric::newton_fd：100 次、扰动 1e-4*|xj|（零值取 1e-4）、
 * 原生 QR、relax=1、dx.norm()<1e-6、x.norm()>1e6 停止。
 * 精确零残差 break 后原生返回 false，不改变此行为。
 * parameters 成功才更新；正常未收敛和非法查询分开；每次残差求值计一个预算单位。
 * 成功表示原生步长条件通过，不保证最终曲线/曲面残差已达目标。
 * 非有限 QR 步长提前返回 NonfiniteStep；原生可能继续无效算术直到 100 次。
 * 此失败诊断差异单独记录，不将迭代次数判为等价。
 * CAD回调携带OperationFailure时保留取消/预算等状态，不吞成普通求值失败。
 */
CurveSurfaceIntersectionResult solveCurveSurfaceIntersection(const CurveFunction& curve,
    const SurfaceFunction& surface, Vector3 initialParameters, const RunControl& control = {});

/** intersectCurveSurface/curveFunctorCircle：先检查残差<radius*1e-8，
 * 再调用上述 Newton；轴向量由前沿布局模块提供，不在这里重构几何方向。
 * 失败保留原参数，普通 CAD 上调用方按 Gmsh 保留未修正候选。
 */
CurveSurfaceIntersectionResult intersectCircleWithSurface(const Circle3& circle,
    const SurfaceFunction& surface, Vector3 initialParameters, const RunControl& control = {});
} // namespace mesher::gmshReference
