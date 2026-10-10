// 文件职责：旧表面框架的边界布点：积分尺寸加权曲线长度并反解采样参数，实际几何误差由调用方验收。
#include "SurfaceReferenceBoundary.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace mesher {
/// 自适应积分 |C'(t)|/h(t)，反解等累计度量参数，包含两个端点。
bool sampleReferenceCurve(double first,double last,
    const std::function<Point3D(double)>& derivative,const std::function<double(double)>& sizeAt,
    std::vector<double>& parameters,std::string* error,ReferenceCurveSamplingOptions options,
    const std::function<bool()>& keepGoing) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(!std::isfinite(first)||!std::isfinite(last)||!(last>first)||!derivative||!sizeAt||
        !std::isfinite(options.relativeTolerance)||!(options.relativeTolerance>0)||options.relativeTolerance>=.1||
        options.maxDepth<1||options.maxDepth>48||options.maxEvaluations<5||options.maxSegments<1)
        return fail("invalid reference curve sampling inputs");
    std::map<double,double> density;int evaluations=0;std::string failure;
    const auto at=[&](double t) {
        const auto found=density.find(t);if(found!=density.end())return found->second;
        if(evaluations>=options.maxEvaluations){failure="reference curve integration budget exceeded";return 0.;}
        if(evaluations%64==0&&keepGoing&&!keepGoing()){failure="surface meshing cancelled during reference curve integration";return 0.;}
        ++evaluations;const auto d=derivative(t);const double h=sizeAt(t),speed=std::hypot(d.x,d.y,d.z);
        if(!std::isfinite(d.x)||!std::isfinite(d.y)||!std::isfinite(d.z)||!std::isfinite(speed)||
            !std::isfinite(h)||!(h>0)){failure="non-finite/invalid reference curve derivative or size";return 0.;}
        const double value=speed/h;if(!std::isfinite(value)){failure="non-finite reference curve density";return 0.;}
        density[t]=value;return value;
    };
    struct Span {double a,b,fa,fb,area;};std::vector<Span> spans;
    const auto integral=[](double a,double b,double fa,double fm,double fb){return (b-a)*(fa/6+2*fm/3+fb/6);};
    const double middle=first+(last-first)/2;
    const double fa=at(first),fm=at(middle),fb=at(last);
    if(!failure.empty()){if(error)*error=failure;return false;}
    const double initial=integral(first,last,fa,fm,fb);
    const double tolerance=options.relativeTolerance*std::max(1.,initial);
    std::function<bool(double,double,double,double,double,double,double,int)> refine;
    refine=[&](double a,double b,double left,double mid,double right,double old,double allowed,int depth) {
        const double m=a+(b-a)/2,l=a+(m-a)/2,r=m+(b-m)/2;
        if(!(a<l&&l<m&&m<r&&r<b)){failure="reference curve integration parameter stagnation";return false;}
        const double fl=at(l),fr=at(r);if(!failure.empty())return false;
        const double i0=integral(a,m,left,fl,mid),i1=integral(m,b,mid,fr,right);
        if(!std::isfinite(i0)||!std::isfinite(i1)){failure="non-finite reference curve integral";return false;}
        const double interpolationError=std::abs(i0-(i0+i1)/2-(b-a)*(left-right)/8);
        if(std::abs(i0+i1-old)<=15*allowed&&interpolationError<=allowed) {
            spans.push_back({a,m,left,mid,i0});spans.push_back({m,b,mid,right,i1});return true;
        }
        if(depth>=options.maxDepth){failure="reference curve integration depth exceeded";return false;}
        return refine(a,m,left,fl,mid,i0,allowed/2,depth+1)&&refine(m,b,mid,fr,right,i1,allowed/2,depth+1);
    };
    if(!refine(first,last,fa,fm,fb,initial,tolerance,0)){if(error)*error=failure;return false;}
    double total=0;for(const auto& s:spans)total+=s.area;
    if(!std::isfinite(total)||total<0)return fail("invalid reference total curve metric");
    // 整数附近积分舍入不应凭空增加整条曲线的一个分段。
    const double rounded=std::round(total);
    if(std::abs(total-rounded)<=options.relativeTolerance*std::max(1.,total))total=rounded;
    const double count=std::max(1.,std::ceil(total));
    if(count>options.maxSegments)return fail("reference curve segment budget exceeded");
    const int segments=int(count);std::vector<double> result;result.reserve(size_t(segments)+1);result.push_back(first);
    size_t slot=0;double cumulative=0;
    // 单调 Hermite 累计函数用于叶区间反解，端点斜率限制避免负密度过冲。
    for(int k=1;k<segments;++k) {
        if(k%64==0&&keepGoing&&!keepGoing())return fail("surface meshing cancelled during reference curve inversion");
        const double target=total*k/segments;
        while(slot+1<spans.size()&&cumulative+spans[slot].area<target){cumulative+=spans[slot].area;++slot;}
        const auto s=spans[slot];if(!(s.area>0))return fail("unresolved zero reference curve metric interval");
        double left=(s.b-s.a)*s.fa/s.area,right=(s.b-s.a)*s.fb/s.area;
        const double slope=std::hypot(left,right);if(slope>3){left*=3/slope;right*=3/slope;}
        const double fraction=std::clamp((target-cumulative)/s.area,0.,1.);
        double lo=0,hi=1;
        for(int iteration=0;iteration<52;++iteration) {
            const double x=(lo+hi)/2,x2=x*x,x3=x2*x;
            const double value=-2*x3+3*x2+left*(x3-2*x2+x)+right*(x3-x2);
            if(value<fraction)lo=x;else hi=x;
        }
        const double parameter=s.a+(s.b-s.a)*(lo+hi)/2;
        if(!(parameter>result.back()&&parameter<last))return fail("reference curve inverse parameter stagnation");
        result.push_back(parameter);
    }
    result.push_back(last);parameters=std::move(result);return true;
}
}
