// 文件职责：曲面参数反求：通过调用方提供的位置和导数迭代 UV，返回收敛与查询失败状态。
#include "SurfaceParameterInverse.h"
#include "geometry/numerics/SingularMatrixInverse3.h"
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
/// 计算两点之间的欧氏距离。
double distance(Point3D target,Point3D point) {
    return std::sqrt(std::pow(target.x-point.x,2)+std::pow(target.y-point.y,2)+std::pow(target.z-point.z,2));
}
class SurfaceInverseSolver {
public:
    /// 保存曲面查询源、范围、目标点和运行控制。
    SurfaceInverseSolver(Point3D target,const SurfaceParameterInverseSource& source,const SurfaceParameterInverseOptions& options,
        const RunControl& control,const SurfaceParameterInverseObserver& observer)
        : target_(target),source_(source),options_(options),control_(control),observer_(observer) {}
    /// 在参数范围内迭代反求目标三维点的曲面 UV。
    SurfaceParameterInverseResult run() {
        try {
            validate();
            const auto& b=source_.bounds;
            const double precision=options_.onSurface ? 1.e-8 : 1.e-3;
            const int maximumIterations=options_.onSurface ? 25 : 10;
            const double tolerance=precision*(std::pow(b.uMax-b.uMin,2)+std::pow(b.vMax-b.vMin,2));
            if(!std::isfinite(tolerance)) throw OperationFailure{{MeshingStatus::NonConverged,"surface inverse tolerance overflow"}};
            std::array<double,9> us{{.5,.6,.4,.7,.3,.8,.2,1.,0.}},vs=us;
            for(int i=0;i<9;++i) { us[i]=b.uMin+us[i]*(b.uMax-b.uMin);vs[i]=b.vMin+vs[i]*(b.vMax-b.vMin); }
            const bool testXYZ=options_.convergenceTestXYZ || options_.newtonConvergenceTestXYZ;
            double relaxation=options_.relaxation;
            while(true) {
                ++result_.relaxationPasses;
                // 原递归每轮重新初始化new，但本轮81种子共享它；保持特殊大tol行为。
                Point2D next{};
                for(int i=0;i<9;++i) for(int j=0;j<9;++j) {
                    ++result_.seeds;
                    result_.parameters={us[i],vs[j]};
                    double error=1.;int iteration=1;
                    auto point=evaluatePoint(result_.parameters);
                    double xyzError=distance(target_,point);
                    if(xyzError < 1.e-8*options_.modelLength) return succeed(SurfaceParameterInverseStop::InitialPoint);
                    while(error > tolerance && iteration < maximumIterations) {
                        point=evaluatePoint(result_.parameters);
                        const auto derivatives=evaluateDerivatives(result_.parameters);
                        ++result_.steps;
                        const Matrix3 matrix{{{{derivatives[0].x,derivatives[0].y,derivatives[0].z}},
                            {{derivatives[1].x,derivatives[1].y,derivatives[1].z}},{{0.,0.,0.}}}};
                        const auto inverse=computeSingularMatrixPseudoInverse(matrix);
                        if(!inverse.finiteResult) throw OperationFailure{{MeshingStatus::NonConverged,"surface inverse SVD failed"}};
                        const auto& jac=inverse.inverse;
                        const double dx=target_.x-point.x,dy=target_.y-point.y,dz=target_.z-point.z;
                        const auto uv=result_.parameters;
                        next.x=uv.x+relaxation*(jac[0][0]*dx+jac[1][0]*dy+jac[2][0]*dz);
                        next.y=uv.y+relaxation*(jac[0][1]*dx+jac[1][1]*dy+jac[2][1]*dz);
                        if(!std::isfinite(next.x) || !std::isfinite(next.y))
                            throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite surface inverse step"}};
                        // 两轴均越界才停止，break不发布next；单轴越界继续原步长判断。
                        if((next.x>b.uMax+tolerance || next.x<b.uMin-tolerance) &&
                            (next.y>b.vMax+tolerance || next.y<b.vMin-tolerance)) break;
                        error=std::pow(next.x-uv.x,2)+std::pow(next.y-uv.y,2);
                        xyzError=distance(target_,point); // 原版使用更新前P。
                        ++iteration;result_.parameters=next;
                    }
                    if(iteration<maximumIterations && error<=tolerance && next.x<=b.uMax && next.y<=b.vMax && next.x>=b.uMin && next.y>=b.vMin) {
                        if(!(options_.onSurface && xyzError>1.e-4*options_.modelLength && testXYZ))
                            return succeed(SurfaceParameterInverseStop::ParameterStep);
                    }
                }
                if(!options_.onSurface) { result_.stop=SurfaceParameterInverseStop::OffSurfaceLast;return result_; }
                if(relaxation<1.e-3) { result_.stop=SurfaceParameterInverseStop::RelaxationLimit;return result_; }
                relaxation*=.75;
            }
        }
        catch(const OperationFailure& error) { result_.status=error.status; }
        catch(const std::exception& error) { result_.status={MeshingStatus::GeometryQueryFailed,error.what()}; }
        catch(...) { result_.status={MeshingStatus::GeometryQueryFailed,"surface inverse source threw unknown exception"}; }
        result_.converged=false;result_.stop=SurfaceParameterInverseStop::ControlFailure;return result_;
    }
private:
    /// 校验曲面反求的目标点、参数范围及查询接口。
    void validate() {
        const auto& b=source_.bounds;
        if(!finite(target_) || !source_.point || !source_.derivatives || !std::isfinite(b.uMin) || !std::isfinite(b.uMax) ||
            !std::isfinite(b.vMin) || !std::isfinite(b.vMax) || b.uMin>b.uMax || b.vMin>b.vMax ||
            !std::isfinite(options_.modelLength) || options_.modelLength<0. || !std::isfinite(options_.relaxation) || options_.relaxation<=0.)
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid surface inverse source, target, bounds or options"}};
    }
    /// 检查取消及查询预算，并记录一次几何查询。
    void consume() {
        if(control_.keepGoing && !control_.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"surface inverse cancelled"}};
        if(result_.geometryEvaluations >= control_.maximumGeometryEvaluations)
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"surface inverse geometry budget exceeded"}};
        ++result_.geometryEvaluations;
    }
    /// 查询指定 UV 对应的曲面点并检查坐标。
    Point3D evaluatePoint(Point2D uv) {
        consume();const auto value=source_.point(uv);
        if(!finite(value)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite surface inverse point"}};
        if(observer_) observer_({0,uv,{{value,{}}}});
        return value;
    }
    /// 查询曲面一阶导数并检查数值。
    std::array<Point3D,2> evaluateDerivatives(Point2D uv) {
        consume();const auto values=source_.derivatives(uv);
        if(!finite(values[0]) || !finite(values[1])) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite surface inverse derivatives"}};
        if(observer_) observer_({1,uv,values});
        return values;
    }
    /// 记录反求收敛和具体停止原因。
    SurfaceParameterInverseResult succeed(SurfaceParameterInverseStop stop) {
        result_.converged=true;result_.stop=stop;return result_;
    }
    Point3D target_;
    const SurfaceParameterInverseSource& source_;
    const SurfaceParameterInverseOptions& options_;
    const RunControl& control_;
    const SurfaceParameterInverseObserver& observer_;
    SurfaceParameterInverseResult result_;
};
}
/// 在指定参数范围内反求三维点的曲面 UV。
SurfaceParameterInverseResult invertSurfaceParameters(Point3D target,const SurfaceParameterInverseSource& source,
    const SurfaceParameterInverseOptions& options,const RunControl& control,const SurfaceParameterInverseObserver& observer) {
    return SurfaceInverseSolver(target,source,options,control,observer).run();
}
}
