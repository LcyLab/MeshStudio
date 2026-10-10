#include "mesh/surface/legacy/SurfaceRefinementCandidates.h"
#include "mesh/surface/legacy/SurfaceTangentChart.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace mesher;
struct PoleSphere : GeometrySource {
    double radius=500,uScale=1,uOffset=0;
    bool singular=false,throws=false;
    mutable bool outsideBounds=false;
    void checked(Point2D p) const {
        const double pi=std::acos(-1.0);outsideBounds|=p.x<uOffset||p.x>uOffset+pi*uScale||p.y<-pi/2||p.y>pi/2;
    }
    int vertexCount() const override {return 0;}
    Point3D vertexPoint(int) const override {return {};}
    int edgeCount() const override {return 0;}
    std::pair<int,int> edgeVertices(int) const override {return {};}
    std::pair<double,double> edgeRange(int) const override {return {};}
    Point3D edgePoint(int,double) const override {return {};}
    int faceCount() const override {return 1;}
    FaceTopology faceTopology(int) const override {return {};}
    Point2D edgeUV(int,const EdgeUse&,double) const override {return {};}
    Point3D facePoint(int,Point2D p) const override {
        checked(p);if(throws)throw std::runtime_error("CAD point query failed");
        p.x=(p.x-uOffset)/uScale;
        return {radius*std::cos(p.x)*std::cos(p.y),radius*std::sin(p.x)*std::cos(p.y),radius*std::sin(p.y)};
    }
    Point3D faceNormal(int f,Point2D p) const override {return facePoint(f,p);}
    void faceDerivatives(int,Point2D p,Point3D& u,Point3D& v) const override {
        checked(p);p.x=(p.x-uOffset)/uScale;
        u={-radius*std::sin(p.x)*std::cos(p.y)/uScale,radius*std::cos(p.x)*std::cos(p.y)/uScale,0};
        v={-radius*std::cos(p.x)*std::sin(p.y),-radius*std::sin(p.x)*std::sin(p.y),radius*std::cos(p.y)};
        if(singular)u=v={};
    }
    bool faceParameterBounds(int,SurfaceParameterBounds& b) const override {
        const double pi=std::acos(-1.0);b={uOffset,uOffset+pi*uScale,-pi/2,pi/2};return true;
    }
};
struct NonlinearPlane : PoleSphere {
    bool invalid=false;
    Point3D facePoint(int,Point2D p) const override {
        if(throws)throw std::runtime_error("CAD point query failed");
        return {p.x,invalid?std::numeric_limits<double>::quiet_NaN():
            p.y+.9*std::sin(2*std::acos(-1.)*p.y)/(2*std::acos(-1.)),0};
    }
};
int main() {
    SurfaceMetric metric;metric.origin={0,0};metric.xx=1;metric.xy=0;metric.yy=1;
    const std::array<Point2D,3> tri{{{0,0},{.025,0},{.2,.2}}};
    auto candidates=surfaceRefinementCandidates(tri,metric,.025);
    bool ok=!candidates.empty() && candidates.size()<=2;
    NonlinearPlane plane;SurfaceEdgeSplitPoint split;std::string splitError;
    ok=surfaceBalancedEdgePoint(plane,0,{0,0},{0,.5},split,&splitError)&&ok;
    ok=ok&&std::abs(split.point.y-.25)<1e-9&&split.fraction<.3&&split.iterations<=48;
    SurfaceEdgeSplitPoint reverse;
    ok=surfaceBalancedEdgePoint(plane,0,{0,.5},{0,0},reverse,&splitError)&&ok;
    ok=ok&&std::hypot(reverse.uv.x-split.uv.x,reverse.uv.y-split.uv.y)<1e-9;
    const auto saved=split;plane.invalid=true;
    ok=ok&&!surfaceBalancedEdgePoint(plane,0,{0,0},{0,.5},split,&splitError)&&
        split.fraction==saved.fraction&&split.uv.y==saved.uv.y;
    plane.invalid=false;
    ok=ok&&!surfaceBalancedEdgePoint(plane,0,{0,0},{0,0},split,&splitError);
    plane.throws=true;bool edgePropagated=false;
    try{surfaceBalancedEdgePoint(plane,0,{0,0},{0,.5},split,&splitError);}catch(const std::runtime_error&){edgePropagated=true;}
    ok=ok&&edgePropagated;
    const auto floorCandidates=surfaceRefinementCandidates({{{0,0},{.008,0},{.04,.004}}},metric,.01,.01);
    ok=ok&&!floorCandidates.empty();
    if(!floorCandidates.empty())ok=ok&&std::hypot(floorCandidates[0].x,floorCandidates[0].y)>=.01&&
        std::hypot(floorCandidates[0].x-.008,floorCandidates[0].y)>=.01;
    ok=ok&&surfaceRefinementCandidates(tri,metric,.025,-.1).empty()&&
        surfaceRefinementCandidates(tri,metric,.025,std::numeric_limits<double>::quiet_NaN()).empty();
    if(ok) {const auto p=metric.project(candidates[0]);ok=std::abs(p.x-.0125)<1e-12 && p.y>0 && p.y<.05;}
    SurfaceMetric sheared;sheared.origin={0,0};sheared.xx=.01;sheared.xy=2;sheared.yy=10;
    std::array<Point2D,3> uv;for(int i=0;i<3;++i)uv[i]=sheared.unproject(tri[i]);
    auto equivalent=surfaceRefinementCandidates(uv,sheared,.025);
    ok=ok && equivalent.size()==candidates.size();
    for(size_t i=0;i<candidates.size() && i<equivalent.size();++i){const auto p=sheared.project(equivalent[i]);ok=ok && std::hypot(p.x-candidates[i].x,p.y-candidates[i].y)<1e-10;}
    std::array<Point2D,3> line{{{0,0},{1,0},{2,0}}};ok=ok && surfaceRefinementCandidates(line,metric,1).empty();
    // 凹球最差片的实际 UV；球极点对应多个参数位置，线性 D1 映射会越界。
    PoleSphere sphere;
    const std::array<Point2D,3> polar{{{2.1213119287661297,1.556084353753006},
        {2.5648854069206899,1.5325526647372461},{std::acos(-1.0),std::acos(-1.0)/2}}};
    Point2D seed{};for(auto p:polar){seed.x+=p.x/3;seed.y+=p.y/3;}
    SurfaceMetric polarMetric;std::string error;
    bool polarValid=surfaceMetricAt(sphere,0,seed,polarMetric,&error);
    auto polarCandidates=surfaceRefinementCandidates(polar,polarMetric,43.301270197882189);
    std::array<Point3D,3> xyz;for(int i=0;i<3;++i)xyz[i]=sphere.facePoint(0,polar[i]);
    const auto tangent=surfaceTangentRefinementCandidates(sphere,0,polar,xyz,43.301270197882189);
    polarCandidates.insert(polarCandidates.end(),tangent.uv.begin(),tangent.uv.end());
    bool interior=false;
    for(auto p:polarCandidates) {
        std::cout<<"pole candidate UV "<<p.x<<' '<<p.y<<'\n';
        interior=interior || (p.x>0 && p.x<std::acos(-1.0) && p.y>-std::acos(-1.0)/2 && p.y<std::acos(-1.0)/2);
    }
    // 两种候选都可能确实越过裁剪边；图表不能把不合法的点夹到边界伪造成功。
    SurfaceTangentChart chart;
    bool chartValid=surfaceTangentChartAt(sphere,0,polar,xyz,chart,&error);
    const Point2D validUV{2.5,1.55};
    const auto inverse=invertSurfaceTangentChart(sphere,0,chart,chart.project(sphere.facePoint(0,validUV)));
    const auto outside=invertSurfaceTangentChart(sphere,0,chart,chart.project(sphere.facePoint(0,{3.3,1.55})));
    ok=ok && polarValid && chartValid && !interior && !tangent.refusals.empty() && inverse.valid &&
        std::hypot(inverse.uv.x-validUV.x,inverse.uv.y-validUV.y)<1e-8 && !outside.valid && outside.iterations<=20;
    sphere.outsideBounds=false;
    for(double scale:{.001,1000.})for(double stretch:{.01,1000.}){
        sphere.radius=500*scale;sphere.uScale=stretch;sphere.uOffset=2*std::acos(-1.0);
        auto branch=polar;for(auto& p:branch)p.x=sphere.uOffset+p.x*stretch;
        for(int i=0;i<3;++i)xyz[i]=sphere.facePoint(0,branch[i]);
        const Point2D seedUV{sphere.uOffset+validUV.x*stretch,validUV.y};
        SurfaceTangentChart local;ok=surfaceTangentChartAt(sphere,0,branch,xyz,local,&error)&&ok;
        const auto back=invertSurfaceTangentChart(sphere,0,local,local.project(sphere.facePoint(0,seedUV)));
        ok=ok&&back.valid&&std::hypot((back.uv.x-seedUV.x)/stretch,back.uv.y-seedUV.y)<1e-8&&!sphere.outsideBounds;
    }
    sphere.radius=500;sphere.uScale=1;sphere.uOffset=0;sphere.outsideBounds=false;
    sphere.singular=true;
    const auto failed=invertSurfaceTangentChart(sphere,0,chart,chart.project(sphere.facePoint(0,validUV)));
    ok=ok&&!failed.valid&&failed.error.find("singular")!=std::string::npos&&!sphere.outsideBounds;
    sphere.singular=false;
    ok=ok&&!invertSurfaceTangentChart(sphere,0,chart,{std::numeric_limits<double>::quiet_NaN(),0}).valid;
    sphere.throws=true;bool propagated=false;
    try{invertSurfaceTangentChart(sphere,0,chart,{0,0});}catch(const std::runtime_error&){propagated=true;}
    ok=ok&&propagated;
    if(!ok)std::cerr<<"FAIL bounded pole chart inverse or honest trim rejection\n";
    std::cout<<(ok?"PASS":"FAIL")<<" physical off-center candidates and UV shear invariance\n";return ok?0:1;
}
