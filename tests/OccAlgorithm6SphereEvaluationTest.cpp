/** @file OccAlgorithm6SphereEvaluationTest.cpp
 * @brief 独立球面前沿求值的SDK数值回归及变换/反向/极点验证。
 * @date 2026-10-09
 */
#include "cad/occ/OccAlgorithm6Adapter.h"
#include "geometry/numerics/NativeSinCos.h"
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp_Ax1.hxx>
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
namespace {
double distance(Point3D a,Point3D b) {return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);}
}
int main() {
    // 大参数约减和非有限输入不能破坏浮点栈或后续CAD求值。
    for(double angle:{0.,-0.,1.e30,-1.e30}) {
        const auto value=gmshReference::evaluateNativeSinCos(angle);
        if(!std::isfinite(value.sine) || !std::isfinite(value.cosine) ||
           std::abs(value.sine*value.sine+value.cosine*value.cosine-1)>1e-14) {
            std::cerr<<"FAIL native sincos range reduction\n";return 1;
        }
    }
    for(double angle:{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        const auto value=gmshReference::evaluateNativeSinCos(angle);
        if(!std::isnan(value.sine) || !std::isnan(value.cosine)) {
            std::cerr<<"FAIL nonfinite native sincos\n";return 1;
        }
    }
    // 官方4.15.2 Windows64 SDK的实际Newton查询；无需SDK或参考目录运行。
    const Point2D uv{5.749675923930807,1.1998216467524552};
    const Point3D expected{0.31214317166198075,-0.18436452240025997,0.9319744434612589};
    OccAlgorithm6Adapter unit(BRepPrimAPI_MakeSphere(1).Shape());
    const auto source=unit.buildFaceInput(0);const auto actual=source.front.surface(uv);
#if defined(_MSC_VER) && defined(_M_X64)
    if(actual.x!=expected.x || actual.y!=expected.y || actual.z!=expected.z) {
        std::cerr<<"FAIL sphere front Value must reproduce paired native sincos rounding\n";return 1;
    }
#endif
    for(bool transformed:{false,true})for(bool reversed:{false,true}) {
        auto shape=BRepPrimAPI_MakeSphere(3).Shape();
        if(transformed) {
            gp_Trsf pose;pose.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(1,2,3)),.47);
            pose.SetTranslationPart(gp_Vec(12,-7,3));shape.Move(TopLoc_Location(pose));
        }
        if(reversed)shape.Reverse();
        OccAlgorithm6Adapter adapter(shape);const auto face=adapter.buildFaceInput(0);
        for(Point2D q:{uv,Point2D{0,1.5707963267948966},Point2D{6.283185307179586,-1.5707963267948966},
                      Point2D{-17294.495494989726,-70.06109968369002},uv}) {
            const auto p=face.front.surface(q);const auto callback=face.front.surfacePoint(q);
            if(distance(p,adapter.geometry().faceSurfacePoint(0,q))>1e-13 || !callback.succeeded ||
               distance(p,callback.point)!=0) {std::cerr<<"FAIL transformed sphere point\n";return 1;}
            const auto metric=face.front.metric(q);const auto direct=face.initialMetric(q);
            if(!direct.status.complete() || std::abs(metric.gUU-direct.metric.gUU)>1e-12 ||
               std::abs(metric.gUV-direct.metric.gUV)>1e-12 || std::abs(metric.gVV-direct.metric.gVV)>1e-12) {
                std::cerr<<"FAIL sphere derivatives/metric\n";return 1;
            }
        }
    }
    OccAlgorithm6Adapter cylinder(BRepPrimAPI_MakeCylinder(2,4).Shape());
    const auto model=cylinder.buildModelInput();
    for(const auto& face:model.faces)for(Point2D q:{Point2D{.73,.81},Point2D{1.8,.37}}) {
        if(distance(face.front.surface(q),cylinder.geometry().faceSurfacePoint(face.chart.entity-1,q))!=0)
            {std::cerr<<"FAIL nonspherical source changed\n";return 1;}
    }
    std::cout<<"PASS independent sphere front rounding, transformed/reversed frames, poles and nonspherical source\n";
}
