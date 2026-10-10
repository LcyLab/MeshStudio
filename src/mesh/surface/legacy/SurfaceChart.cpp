// 文件职责：默认表面路线的局部度量：把 CAD 导数换算为物理长度，并估计几何限制需要的尺寸。
#include "SurfaceChart.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mesher {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
/// 计算向量的欧氏长度。
double norm(Point3D p) { return std::hypot(p.x,p.y,p.z); }
/// 计算向量点积。
double dot(Point3D a,Point3D b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
/// 计算向量叉积，确定有向关系。
Point3D cross(Point3D a,Point3D b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
/// 读取统一缩放比例。
Point3D scale(Point3D p,double s) { return {p.x*s,p.y*s,p.z*s}; }
/// 计算两个点的坐标差。
Point3D sub(Point3D a,Point3D b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
}

/// 将原 UV 坐标映射到局部物理长度度量平面。
Point2D SurfaceMetric::project(Point2D p) const {
    const double u=p.x-origin.x,v=p.y-origin.y;
    return {xx*u+xy*v,yy*v};
}
/// 将局部度量平面坐标还原为原 UV 参数。
Point2D SurfaceMetric::unproject(Point2D p) const {
    const double v=p.y/yy;
    return {origin.x+(p.x-xy*v)/xx,origin.y+v};
}
/// 由曲面导数构造局部第一基本形式及其分解。
bool surfaceMetricAt(const GeometrySource& geo,int face,Point2D uv,
                     SurfaceMetric& metric,std::string* error) {
    Point3D du,dv; geo.faceDerivatives(face,uv,du,dv);
    const double x=norm(du),area=norm(cross(du,dv));
    if (!finite(du)||!finite(dv)||!std::isfinite(x)||!std::isfinite(area)||!(x>0 && area>0)) {
        if (error) *error="singular/non-finite surface metric";
        return false;
    }
    SurfaceMetric m; m.origin=uv; m.xx=x; m.xy=dot(du,dv)/x; m.yy=area/x;
    if (!std::isfinite(m.xy)||!std::isfinite(m.yy)||!(m.yy>0)) {
        if (error) *error="singular/non-finite surface metric";
        return false;
    }
    metric=m; return true;
}

/// 结合曲率和几何偏差要求估计局部目标尺寸。
SurfaceSizeSample surfaceSizeAt(const GeometrySource& geo,int face,Point2D uv,
                               double target,const SurfaceCriteria& criteria) {
    SurfaceSizeSample result; result.target=target;
    if (!std::isfinite(uv.x)||!std::isfinite(uv.y)||!std::isfinite(target)||!(target>0)||
        !std::isfinite(criteria.maxDeviation)||!(criteria.maxDeviation>0)||
        !std::isfinite(criteria.maxNormalAngleDeg)||
        !(criteria.maxNormalAngleDeg>0 && criteria.maxNormalAngleDeg<=180)) {
        result.error="invalid surface sizing input/criteria"; return result;
    }
    SurfaceParameterBounds bounds;
    const bool bounded=geo.faceParameterBounds(face,bounds);
    if (bounded && (!std::isfinite(bounds.uMin)||!std::isfinite(bounds.uMax)||
        !std::isfinite(bounds.vMin)||!std::isfinite(bounds.vMax)||
        !(bounds.uMax>bounds.uMin && bounds.vMax>bounds.vMin))) {
        result.error="invalid CAD surface parameter bounds"; return result;
    }
    if (bounded) {
        // 参数曲线端点可能因舍入略超出曲面范围；只钳制这段舍入误差，不向域外延伸导数差分。
        const double uTol=64*std::numeric_limits<double>::epsilon()*
            std::max({1.0,std::abs(bounds.uMin),std::abs(bounds.uMax)});
        const double vTol=64*std::numeric_limits<double>::epsilon()*
            std::max({1.0,std::abs(bounds.vMin),std::abs(bounds.vMax)});
        if (uv.x<bounds.uMin-uTol || uv.x>bounds.uMax+uTol ||
            uv.y<bounds.vMin-vTol || uv.y>bounds.vMax+vTol) {
            result.error="surface sizing sample outside parameter bounds"; return result;
        }
        uv.x=std::clamp(uv.x,bounds.uMin,bounds.uMax);
        uv.y=std::clamp(uv.y,bounds.vMin,bounds.vMax);
    }
    bool singular=false;
    const auto normal=[&](Point2D p) {
        const Point3D raw=geo.faceNormal(face,p);
        const double len=norm(raw);
        if (!finite(raw)||!std::isfinite(len)) result.error="non-finite CAD sizing normal";
        else if (!(len>0)) singular=true;
        return len>0 && std::isfinite(len)?scale(raw,1/len):Point3D{};
    };
    normal(uv);
    if (!result.error.empty()) return result;
    if (singular) { result.valid=true; result.singular=true; return result; }
    SurfaceMetric m;
    if (!surfaceMetricAt(geo,face,uv,m,&result.error)) return result;
    const double eps=std::cbrt(std::numeric_limits<double>::epsilon());
    const double hu=eps*std::max({1.0,std::abs(uv.x),bounded?bounds.uMax-bounds.uMin:0.0});
    const double hv=eps*std::max({1.0,std::abs(uv.y),bounded?bounds.vMax-bounds.vMin:0.0});
    const double ua=bounded?std::max(bounds.uMin,uv.x-hu):uv.x-hu;
    const double ub=bounded?std::min(bounds.uMax,uv.x+hu):uv.x+hu;
    const double va=bounded?std::max(bounds.vMin,uv.y-hv):uv.y-hv;
    const double vb=bounded?std::min(bounds.vMax,uv.y+hv):uv.y+hv;
    if (!(ub>ua && vb>va)) { result.error="unresolved CAD sizing derivative interval"; return result; }
    const Point3D nu=scale(sub(normal({ub,uv.y}),normal({ua,uv.y})),1/(ub-ua));
    const Point3D nv=scale(sub(normal({uv.x,vb}),normal({uv.x,va})),1/(vb-va));
    if (!result.error.empty()) return result;
    if (singular) { result.valid=true; result.singular=true; return result; }
    // 将法向的 UV 导数转换为正交物理切基下的导数 Dn·L^-1。
    const Point3D kx=scale(nu,1/m.xx);
    const Point3D ky=scale(sub(nv,scale(kx,m.xy)),1/m.yy);
    const double a=dot(kx,kx),b=dot(kx,ky),d=dot(ky,ky);
    const double curvature=std::sqrt(std::max(0.0,(a+d+std::hypot(a-d,2*b))/2));
    if (!std::isfinite(curvature)) { result.error="non-finite CAD sizing curvature"; return result; }
    result.maxCurvature=curvature;
    if (curvature>0) {
        const double radians=criteria.maxNormalAngleDeg*std::acos(-1.0)/180;
        const double hn=2*std::sin(radians/2)/curvature;
        const double hd=std::sqrt(8*criteria.maxDeviation/curvature);
        result.curvatureSpan=std::min(hn,hd);
        result.target=std::min(target,result.curvatureSpan);
    }
    if (!(result.target>0) || !std::isfinite(result.target)) {
        result.error="invalid curvature-derived surface size"; return result;
    }
    result.valid=true; return result;
}

} // namespace mesher
