// 文件职责：曲线与曲面求交：用有限差分 Newton 修正前沿候选，保留原停止规则及失败诊断。
#include "CurveSurfaceIntersection.h"
#include <cmath>
#include <exception>

namespace mesher::gmshReference {
namespace {
/// 按固定累加顺序计算三维向量长度。
double nativeNorm(const Vector3& vector)
{
    double squared = 0.;
    for(double component : vector) squared += component * component;
    return std::sqrt(squared);
}

/// 检查输入坐标或数值是否有限。
bool finite(const Point3D& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

class IntersectionSolver {
public:
    /// 保存曲线、曲面及初始参数，初始化求交状态。
    IntersectionSolver(const CurveFunction& curve, const SurfaceFunction& surface,
                       Vector3 initialParameters, const RunControl& control)
        : curve_(curve), surface_(surface), parameters_(initialParameters), control_(control)
    {
        result_.parameters = initialParameters;
        stop(IntersectionStop::IterationLimit, MeshingStatus::NonConverged, "native Newton iteration limit");
    }

    /// 迭代求解曲线与曲面的交点，并报告停止原因。
    CurveSurfaceIntersectionResult solve(bool checkInitialResidual, double epsilon)
    {
        if(!curve_ || !surface_) {
            stop(IntersectionStop::InvalidQuery, MeshingStatus::InvalidInput, "missing curve or surface evaluator");
            return result_;
        }
        for(double parameter : parameters_) {
            if(!std::isfinite(parameter)) {
                stop(IntersectionStop::InvalidQuery, MeshingStatus::InvalidInput, "nonfinite intersection parameter");
                return result_;
            }
        }
        Vector3 residual{};
        if(checkInitialResidual) {
            if(!evaluateResidual(residual)) return result_;
            result_.lastResidualNorm = nativeNorm(residual);
            if(result_.lastResidualNorm < epsilon) {
                succeed(IntersectionStop::InitialResidual);
                return result_;
            }
        }
        for(int iteration = 0; iteration < 100; ++iteration) {
            if(nativeNorm(parameters_) > 1.e6) {
                stop(IntersectionStop::ParameterNormLimit, MeshingStatus::NonConverged, "native Newton parameter norm limit");
                return result_;
            }
            result_.iterations = iteration + 1;
            if(!evaluateResidual(residual)) return result_;
            result_.lastResidualNorm = nativeNorm(residual);
            if(residual[0] == 0. && residual[1] == 0. && residual[2] == 0.) {
                stop(IntersectionStop::ExactZeroResidualExit, MeshingStatus::NonConverged, "native exact-zero break returns false");
                return result_;
            }
            Matrix3 jacobian{};
            for(int column = 0; column < 3; ++column) {
                double delta = 1.e-4 * std::abs(parameters_[column]);
                if(delta == 0.) delta = 1.e-4;
                parameters_[column] += delta;
                Vector3 perturbedResidual{};
                if(!evaluateResidual(perturbedResidual)) return result_;
                for(int row = 0; row < 3; ++row)
                    jacobian[row][column] = (perturbedResidual[row] - residual[row]) / delta;
                // 保留原生 +=h/-=h 的舍入轨迹，不恢复保存的原值。
                parameters_[column] -= delta;
            }
            const auto solved = solveColumnPivotedQr(jacobian, residual);
            if(!solved.finiteResult) {
                stop(IntersectionStop::NonfiniteStep, MeshingStatus::NonConverged, "nonfinite native Newton step");
                return result_;
            }
            for(int component = 0; component < 3; ++component)
                parameters_[component] -= solved.solution[component];
            if(nativeNorm(solved.solution) < 1.e-6) {
                // 原生只判步长；不能在这里增加最终残差门槛。
                succeed(IntersectionStop::NewtonStep);
                return result_;
            }
        }
        return result_;
    }

private:
    /// 查询曲线和曲面位置，计算求交残差并检查预算。
    bool evaluateResidual(Vector3& residual)
    {
        if(control_.keepGoing && !control_.keepGoing()) {
            stop(IntersectionStop::Cancelled, MeshingStatus::Cancelled, "intersection cancelled");
            return false;
        }
        if(result_.residualEvaluations >= control_.maximumGeometryEvaluations) {
            stop(IntersectionStop::BudgetExceeded, MeshingStatus::BudgetExceeded, "intersection evaluation budget exceeded");
            return false;
        }
        ++result_.residualEvaluations;
        try {
            const auto surfacePoint = surface_({parameters_[0], parameters_[1]});
            const auto curvePoint = curve_(parameters_[2]);
            residual = {surfacePoint.x - curvePoint.x, surfacePoint.y - curvePoint.y, surfacePoint.z - curvePoint.z};
            if(finite(surfacePoint) && finite(curvePoint) && std::isfinite(residual[0]) &&
               std::isfinite(residual[1]) && std::isfinite(residual[2])) return true;
            stop(IntersectionStop::InvalidQuery, MeshingStatus::GeometryQueryFailed, "nonfinite curve or surface query");
        }
        catch(const OperationFailure& error) {
            const auto reason = error.status.code == MeshingStatus::Cancelled ? IntersectionStop::Cancelled :
                error.status.code == MeshingStatus::BudgetExceeded ? IntersectionStop::BudgetExceeded : IntersectionStop::InvalidQuery;
            result_.stop = reason; result_.status = error.status;
        }
        catch(const std::exception& error) {
            stop(IntersectionStop::InvalidQuery, MeshingStatus::GeometryQueryFailed, error.what());
        }
        catch(...) {
            stop(IntersectionStop::InvalidQuery, MeshingStatus::GeometryQueryFailed, "curve or surface query threw an unknown exception");
        }
        return false;
    }

    /// 保存求交停止原因和失败状态。
    void stop(IntersectionStop reason, MeshingStatus status, const char* message)
    {
        result_.stop = reason;
        result_.status = {status, message};
    }
    /// 发布求交成功状态和最终参数。
    void succeed(IntersectionStop reason)
    {
        result_.stop = reason;
        result_.status = {};
        result_.parameters = parameters_;
        result_.intersectionFound = true;
    }

    const CurveFunction& curve_;
    const SurfaceFunction& surface_;
    Vector3 parameters_;
    const RunControl& control_;
    CurveSurfaceIntersectionResult result_;
};
} // namespace

/// 迭代求解参数曲线与曲面的交点。
CurveSurfaceIntersectionResult solveCurveSurfaceIntersection(const CurveFunction& curve, const SurfaceFunction& surface,
    Vector3 initialParameters, const RunControl& control)
{
    return IntersectionSolver(curve, surface, initialParameters, control).solve(false, 0.);
}

/// 构造局部圆并求其与曲面的交点。
CurveSurfaceIntersectionResult intersectCircleWithSurface(const Circle3& circle, const SurfaceFunction& surface,
    Vector3 initialParameters, const RunControl& control)
{
    if(!finite(circle.center) || !finite(circle.cosineAxis) || !finite(circle.sineAxis) ||
       !std::isfinite(circle.radius)) {
        CurveSurfaceIntersectionResult result;
        result.parameters = initialParameters;
        result.status = {MeshingStatus::InvalidInput, "invalid circle for surface intersection"};
        result.stop = IntersectionStop::InvalidQuery;
        return result;
    }
    const CurveFunction curve = [&](double t) {
        const double cosine = std::cos(t), sine = std::sin(t);
        return Point3D{circle.center.x + (circle.cosineAxis.x * cosine + circle.sineAxis.x * sine) * circle.radius,
                       circle.center.y + (circle.cosineAxis.y * cosine + circle.sineAxis.y * sine) * circle.radius,
                       circle.center.z + (circle.cosineAxis.z * cosine + circle.sineAxis.z * sine) * circle.radius};
    };
    return IntersectionSolver(curve, surface, initialParameters, control).solve(true, circle.radius * 1.e-8);
}
}
