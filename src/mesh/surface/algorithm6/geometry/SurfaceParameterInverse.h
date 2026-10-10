#pragma once
/** @file SurfaceParameterInverse.h
 * @brief 原通用曲面参数反求；几何和SVD分别由独立源与数值模块负责。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/SurfaceTypes.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include <array>

namespace mesher::gmshReference {
struct SurfaceParameterInverseSource {
    SurfaceParameterBounds bounds;
    std::function<Point3D(Point2D)> point;
    std::function<std::array<Point3D,2>(Point2D)> derivatives;
};
struct SurfaceParameterInverseOptions {
    double modelLength=1., relaxation=1.;
    bool onSurface=true, convergenceTestXYZ=false, newtonConvergenceTestXYZ=false;
};
enum class SurfaceParameterInverseStop { InitialPoint, ParameterStep, OffSurfaceLast, RelaxationLimit, ControlFailure };
struct SurfaceParameterInverseQuery {
    int kind=0; // 0=point，1=firstDer；保留实际请求序。
    Point2D parameters;
    std::array<Point3D,2> values{};
};
using SurfaceParameterInverseObserver=std::function<void(const SurfaceParameterInverseQuery&)>;
struct SurfaceParameterInverseResult {
    OperationStatus status;
    Point2D parameters;
    SurfaceParameterInverseStop stop=SurfaceParameterInverseStop::ControlFailure;
    bool converged=false;
    std::size_t geometryEvaluations=0, relaxationPasses=0, seeds=0, steps=0;
};
/** 对应GFace::XYZtoUV：原81种子序、SVD更新、越界/参数步长/旧点XYZ检查、relax*=.75。
 * 正常未收敛仍返回原最后UV，status.complete且converged=false；调用方显式处理。
 * 不夹紧UV、增加最终残差检查或以QR代替伪逆。source不管理OCC直接投影分流。
 * 取消/预算/几何异常明确返回失败；每次point/firstDer计一个源请求，跨缩步累计。
 */
SurfaceParameterInverseResult invertSurfaceParameters(Point3D target,const SurfaceParameterInverseSource& source,
    const SurfaceParameterInverseOptions& options={},const RunControl& control={},const SurfaceParameterInverseObserver& observer={});
}
