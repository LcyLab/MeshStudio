// 文件职责：局部细化候选：构造偏心点、外心和等距分边点；只返回位置，不提交连接修改。
#include "SurfaceRefinementCandidates.h"
#include "geometry/predicates/GeomPredicates.h"
#include <algorithm>
#include <cmath>
namespace mesher {
/// 沿原UV线段求到两个CAD端点等距的分点，保持拓扑边分支；不是全局投影或弧长中点。
bool surfaceBalancedEdgePoint(const GeometrySource& geometry,int face,Point2D a,Point2D b,
                              SurfaceEdgeSplitPoint& output,std::string* error) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    const auto finite2=[](Point2D p){return std::isfinite(p.x)&&std::isfinite(p.y);};
    const auto finite3=[](Point3D p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
    const auto distance=[](Point3D p,Point3D q){return std::hypot(p.x-q.x,p.y-q.y,p.z-q.z);};
    if(!finite2(a)||!finite2(b)||(a.x==b.x&&a.y==b.y))return fail("invalid surface edge parameters");
    const auto pa=geometry.facePoint(face,a),pb=geometry.facePoint(face,b);
    const double chord=distance(pa,pb);
    if(!finite3(pa)||!finite3(pb)||!(chord>0)||!std::isfinite(chord))return fail("collapsed or non-finite surface edge");
    double lo=0,hi=1;SurfaceEdgeSplitPoint candidate;
    for(int iteration=0;iteration<48;++iteration) {
        const double t=lo+(hi-lo)/2;
        candidate.uv={a.x*(1-t)+b.x*t,a.y*(1-t)+b.y*t};
        if(!finite2(candidate.uv)||(candidate.uv.x==a.x&&candidate.uv.y==a.y)||
            (candidate.uv.x==b.x&&candidate.uv.y==b.y))return fail("surface edge split cannot advance in UV");
        candidate.point=geometry.facePoint(face,candidate.uv);
        if(!finite3(candidate.point))return fail("non-finite surface edge split point");
        const double left=distance(candidate.point,pa),right=distance(candidate.point,pb);
        if(!std::isfinite(left)||!std::isfinite(right))return fail("non-finite surface edge split distance");
        candidate.fraction=t;candidate.iterations=iteration+1;
        if(std::abs(left-right)<=chord*1e-10){output=candidate;return true;}
        if(left<right)lo=t;else hi=t;
    }
    return fail("surface edge split balance iteration limit");
}
/// 返回靠近最短边的偏心点及度量外心；minimumNewEdge 约束偏心点到基边端点的度量长度。
std::vector<Point2D> surfaceRefinementCandidates(const std::array<Point2D,3>& uv,
                                                const SurfaceMetric& metric,double target,double minimumNewEdge) {
    std::vector<Point2D> result;
    if(!(target>0) || !std::isfinite(target) || !(minimumNewEdge>=0) || !std::isfinite(minimumNewEdge) ||
        !(metric.xx>0) || !(metric.yy>0))return result;
    std::array<Point2D,3> p;for(int i=0;i<3;++i){p[i]=metric.project(uv[i]);if(!std::isfinite(p[i].x)||!std::isfinite(p[i].y))return result;}
    if(orient2d(p[0],p[1],p[2])==0)return result;
    Point2D center;double radius;
    if(!circumcircle(p[0],p[1],p[2],center,radius))return result;
    int edge=0;double shortest=dist2D(p[0],p[1]);
    for(int k=1;k<3;++k){const double len=dist2D(p[k],p[(k+1)%3]);if(len<shortest){shortest=len;edge=k;}}
    if(!(shortest>0))return result;
    const auto a=p[edge],b=p[(edge+1)%3],opposite=p[(edge+2)%3];
    const Point2D mid{a.x+(b.x-a.x)/2,a.y+(b.y-a.y)/2};
    const double sign=orient2d(a,b,opposite)>0?1:-1;
    const double nx=sign*(a.y-b.y)/shortest,ny=sign*(b.x-a.x)/shortest;
    const double circumHeight=(center.x-mid.x)*nx+(center.y-mid.y)*ny;
    // 最短边对应等边高度，目标尺寸较大时仍避免在远处放置细化点。
    // 已有基边可小于常规尺寸；新两腰保持下限，避免等边偏心点必然被新增边验收拒绝。
    const double side=std::max(std::min(target,shortest),minimumNewEdge*(1+1e-8));
    const double height=std::sqrt(std::max(0.,side*side-shortest*shortest/4));
    const auto add=[&](Point2D q){auto u=metric.unproject(q);if(std::isfinite(u.x)&&std::isfinite(u.y))result.push_back(u);};
    if(circumHeight>height && height>0)add({mid.x+height*nx,mid.y+height*ny});
    add(center);return result;
}
}
