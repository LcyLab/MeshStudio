// 文件职责：表面质量决策：评价形状改善，并保护尖角和最小新增边限制；不直接修改拓扑。
#include "SurfaceQualityPolicy.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace mesher {
/// 等边为 1；退化或非有限输入为 0，长度尺度不改变指标。
double surfaceShapeQuality(Point3D a,Point3D b,Point3D c) {
    const Point3D u{b.x-a.x,b.y-a.y,b.z-a.z},v{c.x-a.x,c.y-a.y,c.z-a.z};
    const double scale=std::max(std::hypot(u.x,u.y,u.z),std::hypot(v.x,v.y,v.z));
    if(!(scale>0)||!std::isfinite(scale))return 0;
    const Point3D x{u.x/scale,u.y/scale,u.z/scale},y{v.x/scale,v.y/scale,v.z/scale};
    const double area2=std::hypot(x.y*y.z-x.z*y.y,x.z*y.x-x.x*y.z,x.x*y.y-x.y*y.x);
    const auto norm2=[](Point3D p){return p.x*p.x+p.y*p.y+p.z*p.z;};
    const double denominator=norm2(x)+norm2(y)+norm2({y.x-x.x,y.y-x.y,y.z-x.z});
    const double q=2*std::sqrt(3.)*area2/denominator;
    return std::isfinite(q)?std::clamp(q,0.,1.):0;
}
/// 将浮点展开按指定符号相加。
void SurfaceShapeScore::add(double radiusEdge,double shape) {
    ++count;worstRadiusEdge=std::max(worstRadiusEdge,radiusEdge);
    minimumShape=std::min(minimumShape,shape);sumShape+=shape;
}
/// 相同片数，最差 radius/edge 与最低 q 均不恶化，平均 q 严格改善。
bool SurfaceShapeScore::improves(const SurfaceShapeScore& before) const {
    if(!(count>0&&count==before.count&&std::isfinite(worstRadiusEdge)&&std::isfinite(sumShape)&&
        worstRadiusEdge<=before.worstRadiusEdge*(1+1e-12)&&minimumShape>=before.minimumShape-1e-12&&
        sumShape>before.sumShape+1e-6*count))return false;
    return true;
}
/// 校验尺寸和半径边长比，建立表面质量判断规则。
SurfaceQualityPolicy::SurfaceQualityPolicy(double minSize,double radiusEdgeRatio):floor(minSize),radiusEdgeLimit(radiusEdgeRatio) {
    if(!(minSize>0)||!std::isfinite(minSize)||!(radiusEdgeRatio>.5)||!std::isfinite(radiusEdgeRatio))throw std::invalid_argument("invalid surface quality policy");
    angleLimit=std::asin(1/(2*radiusEdgeRatio))*180/std::acos(-1.);
}
/// 仅传入 CAD 角点的有效内角；凹角不保护，近似曲线采样角不当作 CAD 尖角。
void SurfaceQualityPolicy::protectCorner(int vertex,Point3D p,double angle) {
    if(vertex>=0 && std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
       std::isfinite(angle)&&angle>0 && angle<angleLimit)corners[vertex]=angle;
}
/// 保留短边 s 时，新边 >= floor 导致 R/s >= floor/(2s)。仅排除已违反此必要条件的插点。
SurfaceQualityReason SurfaceQualityPolicy::decision(std::array<int,3> vertices,double shortestEdge,double localFloor) const {
    if(localFloor<0 || !std::isfinite(localFloor))throw std::invalid_argument("invalid local surface quality floor");
    for(int v:vertices)if(corners.count(v))return SurfaceQualityReason::ProtectedCorner;
    const double minimum=localFloor>0?std::min(floor,localFloor):floor;
    return shortestEdge<minimum/(2*radiusEdgeLimit)?SurfaceQualityReason::MinimumSize:SurfaceQualityReason::Eligible;
}
/// 将表面质量判定原因转换为报告名称。
const char* surfaceQualityReasonName(SurfaceQualityReason reason) {
    switch(reason){case SurfaceQualityReason::Eligible:return "unclassified";case SurfaceQualityReason::ProtectedCorner:return "protected CAD corner";
        case SurfaceQualityReason::MinimumSize:return "minimum size conflict";case SurfaceQualityReason::NoCandidate:return "no accepted local candidate";
        case SurfaceQualityReason::BudgetLimit:return "quality work budget exhausted";}return "unclassified";
}
/// 域内面片连接到另一条平行裁剪段时，允许协调窄带分点试验。
bool narrowBoundarySplitTrialAllowed(Point3D a,Point3D b,Point3D p,Point3D q,Point3D mid,double minimum,double target){
    const auto finite=[](Point3D v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
    const auto sub=[](Point3D x,Point3D y){return Point3D{x.x-y.x,x.y-y.y,x.z-y.z};};
    const auto len=[](Point3D v){return std::hypot(v.x,v.y,v.z);};
    const auto dot=[](Point3D x,Point3D y){return x.x*y.x+x.y*y.y+x.z*y.z;};
    if(!finite(a)||!finite(b)||!finite(p)||!finite(q)||!finite(mid)||!(minimum>0)||!std::isfinite(minimum)||
        !(target>.5)||!std::isfinite(target))return false;
    const auto ab=sub(b,a),pq=sub(q,p);const double base=len(ab),other=len(pq);
    if(!(base>0&&other>0)||!std::isfinite(base)||!std::isfinite(other))return false;
    const Point3D axis{ab.x/base,ab.y/base,ab.z/base},parallel{pq.x/other,pq.y/other,pq.z/other};
    if(std::abs(dot(axis,parallel))<.95)return false;
    const auto ap=sub(p,a);const double along=dot(ap,axis);
    if(along<-1e-8*base||along>base*(1+1e-8))return false;
    const Point3D transverse{ap.x-along*axis.x,ap.y-along*axis.y,ap.z-along*axis.z};
    const double width=len(transverse);
    // 直角带的 R/短边 = sqrt(base^2+width^2)/(2*width)；宽度须容纳常规/固定来源下限。
    if(!(width>=minimum)||base<=width*std::sqrt(4*target*target-1)*(1+1e-6))return false;
    return std::min({len(sub(mid,a)),len(sub(mid,b)),len(sub(mid,p))})>=minimum;
}
/// 细侧坏片对面的粗 CAD 段分点，协调错开的两侧节点；midpoint 位于 p-q。
bool narrowBoundaryAlignmentTrialAllowed(Point3D a,Point3D b,Point3D p,Point3D q,Point3D mid,double minimum,double target){
    const auto finite=[](Point3D v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
    const auto sub=[](Point3D x,Point3D y){return Point3D{x.x-y.x,x.y-y.y,x.z-y.z};};
    const auto len=[](Point3D v){return std::hypot(v.x,v.y,v.z);};
    const auto dot=[](Point3D x,Point3D y){return x.x*y.x+x.y*y.y+x.z*y.z;};
    if(!finite(a)||!finite(b)||!finite(p)||!finite(q)||!finite(mid)||!(minimum>0)||!std::isfinite(minimum)||
        !(target>.5)||!std::isfinite(target))return false;
    const auto ab=sub(b,a),pq=sub(q,p);const double base=len(ab),other=len(pq);
    if(!(base>0&&other>base*1.5)||!std::isfinite(base)||!std::isfinite(other))return false;
    const Point3D axis{ab.x/base,ab.y/base,ab.z/base},parallel{pq.x/other,pq.y/other,pq.z/other};
    if(std::abs(dot(axis,parallel))<.95)return false;
    const auto ap=sub(p,a);const double along=dot(ap,axis);
    const Point3D transverse{ap.x-along*axis.x,ap.y-along*axis.y,ap.z-along*axis.z};
    if(len(transverse)<minimum)return false;
    const double projected=dot(sub(mid,a),axis);
    if(projected<-base*1e-6||projected>base*(1+1e-6))return false;
    return std::min({len(sub(mid,p)),len(sub(mid,q)),len(sub(mid,a)),len(sub(mid,b))})>=minimum;
}
/// 检查普通裁剪段被坏片对顶点侵入，候选分点不恶化最差子片、改善总和并保持新增边下限。
bool qualityBoundarySplitAllowed(Point3D a,Point3D b,Point3D p,Point3D mid,double minimum,double target){
    const auto finite=[](Point3D v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
    const auto sub=[](Point3D x,Point3D y){return Point3D{x.x-y.x,x.y-y.y,x.z-y.z};};
    const auto len=[](Point3D v){return std::hypot(v.x,v.y,v.z);};
    if(!finite(a)||!finite(b)||!finite(p)||!finite(mid)||!std::isfinite(minimum)||!(minimum>0)||
        !std::isfinite(target)||!(target>.5))return false;
    const double scale=std::max({len(sub(b,a)),len(sub(p,a)),len(sub(p,b))});
    if(!(scale>0)||!std::isfinite(scale))return false;
    const auto normalized=[&](Point3D v){return Point3D{v.x/scale,v.y/scale,v.z/scale};};
    const auto dot=[](Point3D x,Point3D y){return x.x*y.x+x.y*y.y+x.z*y.z;};
    if(dot(normalized(sub(p,a)),normalized(sub(p,b)))>=0)return false;
    // 旧的 p-a / p-b 仍是同一条边；尺寸下限只约束本次新增的三条边。
    if(std::min({len(sub(mid,a)),len(sub(mid,b)),len(sub(mid,p))})<minimum)return false;
    // 保留短边若已违反新边下限的质量必要条件，继续切分无法达到目标。
    if(std::min(len(sub(p,a)),len(sub(p,b)))<minimum/(2*target))return false;
    const auto quality=[&](Point3D x,Point3D y,Point3D z){
        const auto u=normalized(sub(y,x)),v=normalized(sub(z,x));
        const double twiceArea=std::hypot(u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x);
        const double s0=len(u),s1=len(v),s2=len(normalized(sub(z,y)));
        return twiceArea>0?s0*s1*s2/(2*twiceArea*std::min({s0,s1,s2})):INFINITY;
    };
    const double before=quality(a,b,p),first=quality(a,mid,p),second=quality(mid,b,p);
    return std::isfinite(before)&&before>target&&std::isfinite(first)&&std::isfinite(second)&&
        std::max(first,second)<=before*(1+1e-8) && first+second<2*before*(1-1e-6);
}
}
