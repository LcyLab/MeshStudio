// 文件职责：局部几何缓存：复用同一面的原参数位置、导数和棱上 UV；不缓存尺寸或质量结论。
#include "SurfaceEvaluationCache.h"
#include <cmath>
#include <stdexcept>
namespace mesher {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
}
/// 绑定一个不可变 CAD 查询源，建立单面采样缓存。
SurfaceEvaluationCache::SurfaceEvaluationCache(const GeometrySource& g,int f,MeshObservation o):
    source(g),face(f),derivativeNormal(g.normalUsesOrientedDerivatives()),observation(o) {}
/// 按精确 UV 键查询或创建曲面采样缓存项。
SurfaceEvaluationCache::Sample& SurfaceEvaluationCache::sample(Point2D uv) const {
    if (!std::isfinite(uv.x)||!std::isfinite(uv.y)) throw std::invalid_argument("non-finite geometry cache UV");
    const auto inserted=samples.try_emplace({uv.x,uv.y});
    if (inserted.second) observation.count(MeshCounter::GeometryCacheSamples,1);
    return inserted.first->second;
}
/// 计算指定面在 UV 参数处的三维位置。
Point3D SurfaceEvaluationCache::facePoint(int f,Point2D uv) const {
    if (f!=face) return source.facePoint(f,uv);
    auto& s=sample(uv);
    if (s.hasPoint) { observation.count(MeshCounter::GeometryCacheHits,1); return s.point; }
    const auto p=source.facePoint(f,uv); if (finite(p)) { s.point=p; s.hasPoint=true; } return p;
}
/// 计算面的参数导数，供局部度量和法向计算使用。
void SurfaceEvaluationCache::faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const {
    if (f!=face) { source.faceDerivatives(f,uv,du,dv); return; }
    auto& s=sample(uv);
    if (s.hasD1) { observation.count(MeshCounter::GeometryCacheHits,1); du=s.du; dv=s.dv; return; }
    source.faceDerivatives(f,uv,du,dv);
    if (finite(du)&&finite(dv)) { s.du=du; s.dv=dv; s.hasD1=true; }
}
/// 计算指定面参数处的单位法向。
Point3D SurfaceEvaluationCache::faceNormal(int f,Point2D uv) const {
    if (f!=face) return source.faceNormal(f,uv);
    auto& s=sample(uv);
    if (s.hasNormal) { observation.count(MeshCounter::GeometryCacheHits,1); return s.normal; }
    Point3D n;
    if (derivativeNormal) {
        Point3D u,v; faceDerivatives(f,uv,u,v);
        n={u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x};
        if (faceReversed(f)) n={-n.x,-n.y,-n.z};
        // 与 OCC 原法向的零长度判断相同，不另行归一化或改容差。
        if (std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z)==0) n={0,0,0};
    } else n=source.faceNormal(f,uv);
    if (finite(n)) { s.normal=n; s.hasNormal=true; } return n;
}
/// 计算边上节点在指定面分支中的 UV 坐标。
Point2D SurfaceEvaluationCache::edgeUV(int f,const EdgeUse& u,double t) const {
    if (f!=face) return source.edgeUV(f,u,t);
    if (!std::isfinite(t)) throw std::invalid_argument("non-finite geometry cache parameter");
    const auto key=std::make_tuple(u.edge,u.branch,u.reversed,t); const auto found=trims.find(key);
    if (found!=trims.end()) { observation.count(MeshCounter::GeometryCacheHits,1); return found->second; }
    const auto uv=source.edgeUV(f,u,t);
    if (std::isfinite(uv.x)&&std::isfinite(uv.y)) { trims.emplace(key,uv); observation.count(MeshCounter::GeometryCacheSamples,1); }
    return uv;
}
/// 查询底层曲面的参数范围。
bool SurfaceEvaluationCache::faceParameterBounds(int f,SurfaceParameterBounds& b) const {
    if (f!=face) return source.faceParameterBounds(f,b);
    if (!hasBounds) { bounded=source.faceParameterBounds(f,parameterBounds); hasBounds=true; }
    else observation.count(MeshCounter::GeometryCacheHits,1);
    b=parameterBounds; return bounded;
}
/// 判断 CAD 面是否相对底层曲面反向。
bool SurfaceEvaluationCache::faceReversed(int f) const {
    if (f!=face) return source.faceReversed(f);
    if (!hasReversed) { isReversed=source.faceReversed(f); hasReversed=true; }
    else observation.count(MeshCounter::GeometryCacheHits,1);
    return isReversed;
}
} // namespace mesher
