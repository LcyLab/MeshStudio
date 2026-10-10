#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include "CadSurfaceQualityReport.h"
#include "geometry/predicates/GeomPredicates.h"
#include "mesh/validation/SurfaceMeshValidation.h"
#include "mesh/surface/legacy/SurfaceChart.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace mesher;

namespace {

struct PlanarPatch : GeometrySource {
    bool hole;
    double gap;
    explicit PlanarPatch(bool hasHole,double width=1) : hole(hasHole),gap(width) {}
    int vertexCount() const override { return hole ? 8 : 4; }
    Point3D vertexPoint(int i) const override {
        const Point3D p[] = {{0,0,0},{4,0,0},{4,4,0},{0,4,0},
                            {gap,gap,0},{gap,4-gap,0},{4-gap,4-gap,0},{4-gap,gap,0}};
        return p[i];
    }
    int edgeCount() const override { return hole ? 8 : 4; }
    std::pair<int,int> edgeVertices(int i) const override {
        static const int e[][2] = {{0,1},{1,2},{2,3},{3,0},
                                   {4,5},{5,6},{6,7},{7,4}};
        return {e[i][0], e[i][1]};
    }
    std::pair<double,double> edgeRange(int) const override { return {0,1}; }
    Point3D edgePoint(int i, double t) const override {
        const auto [a,b] = edgeVertices(i);
        const Point3D p=vertexPoint(a), q=vertexPoint(b);
        return {p.x+(q.x-p.x)*t,p.y+(q.y-p.y)*t,0};
    }
    int faceCount() const override { return 1; }
    FaceTopology faceTopology(int) const override {
        FaceTopology f;
        f.loops.push_back({{0,false,0},{1,false,0},{2,false,0},{3,false,0}});
        if (hole) f.loops.push_back({{4,false,0},{5,false,0},{6,false,0},{7,false,0}});
        return f;
    }
    Point2D edgeUV(int, const EdgeUse& use, double t) const override {
        const Point3D p=edgePoint(use.edge,t);
        return {p.x,p.y};
    }
    Point3D facePoint(int, Point2D uv) const override { return {uv.x,uv.y,0}; }
    Point3D faceNormal(int, Point2D) const override { return {0,0,1}; }
};
struct SharpPatch final : PlanarPatch {
    SharpPatch():PlanarPatch(false) {}
    int vertexCount() const override { return 3; }
    int edgeCount() const override { return 3; }
    Point3D vertexPoint(int v) const override {
        const Point3D p[]={{0,0,0},{4,0,0},{0.1,0.01,0}};
        return p[v];
    }
    std::pair<int,int> edgeVertices(int e) const override { return {e,(e+1)%3}; }
    FaceTopology faceTopology(int) const override { return {{{{0,false,0},{1,false,0},{2,false,0}}}}; }
};
// 五边形没有尖角；短边需要质量布点，不能因贴边直接豁免。
struct ShortBoundaryPatch : PlanarPatch {
    ShortBoundaryPatch():PlanarPatch(false) {}
    int vertexCount() const override { return 5; }
    int edgeCount() const override { return 5; }
    Point3D vertexPoint(int v) const override {
        const Point3D p[]={{0,0,0},{.025,.005,0},{4,0,0},{4,4,0},{0,4,0}};return p[v];
    }
    std::pair<int,int> edgeVertices(int e) const override { return {e,(e+1)%5}; }
    FaceTopology faceTopology(int) const override { return {{{{0,false,0},{1,false,0},{2,false,0},{3,false,0},{4,false,0}}}}; }
};
struct ShearedShortBoundary final:ShortBoundaryPatch {
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {auto p=edgePoint(use.edge,t);return {(p.x-2*p.y)/10,p.y/3};}
    Point3D facePoint(int,Point2D p) const override{return {10*p.x+6*p.y,3*p.y,0};}
    void faceDerivatives(int,Point2D,Point3D& du,Point3D& dv) const override{du={10,0,0};dv={6,3,0};}
};
struct NonuniformShortBoundary final:ShortBoundaryPatch {
    Point3D edgePoint(int e,double t) const override{return PlanarPatch::edgePoint(e,t*t*t);}
};
// 两侧初始必要采样密度不同的窄环：不能用新增边 min 把邻近尺寸协调锁死。
struct NarrowRing final:PlanarPatch {
    NarrowRing():PlanarPatch(false){}
    int vertexCount() const override{return 322;}
    int edgeCount() const override{return vertexCount();}
    Point3D vertexPoint(int v) const override{
        const double pi=std::acos(-1.);
        const double angle=v<=256?pi*v/256:pi*(321-v)/64;
        const double radius=v<=256?22:21.5;return {radius*std::cos(angle),radius*std::sin(angle),0};
    }
    std::pair<int,int> edgeVertices(int e) const override{return {e,(e+1)%edgeCount()};}
    FaceTopology faceTopology(int) const override{FaceTopology f;std::vector<EdgeUse> loop;
        for(int e=0;e<edgeCount();++e)loop.push_back({e,false,0});f.loops.push_back(loop);return f;}
};
struct NarrowFrame final:PlanarPatch {
    NarrowFrame():PlanarPatch(true){}
    Point3D vertexPoint(int v) const override{
        const Point3D p[]={{0,0,0},{88,0,0},{88,66,0},{0,66,0},
            {.5,.5,0},{.5,65.5,0},{87.5,65.5,0},{87.5,.5,0}};return p[v];}
};
struct PhaseNarrowStrip final:PlanarPatch {
    PhaseNarrowStrip():PlanarPatch(false){}
    int vertexCount() const override{return 194;}
    int edgeCount() const override{return vertexCount();}
    Point3D vertexPoint(int v) const override{return v<=128?Point3D{0,44.*v/128,0}:Point3D{.5,44.*(193-v)/64,0};}
    std::pair<int,int> edgeVertices(int e) const override{return {e,(e+1)%edgeCount()};}
    FaceTopology faceTopology(int) const override{FaceTopology t;std::vector<EdgeUse> loop;
        for(int e=0;e<edgeCount();++e)loop.push_back({e,false,0});t.loops.push_back(loop);return t;}
};
// Tiny trim-axis drift must not form an unrelated column for every long-side node.
struct NoisyRectangle final : PlanarPatch {
    NoisyRectangle():PlanarPatch(false){}
    Point3D vertexPoint(int v) const override {const Point3D p[]={{0,0,0},{0.1,0,0},{0.1,100,0},{0,100,0}};return p[v];}
    Point3D edgePoint(int e,double t) const override {
        auto p=PlanarPatch::edgePoint(e,t);
        if(e==1 || e==3)p.x+=1e-7*std::sin(5*t)*(1-t)*t;
        return p;
    }
};

// 相邻 CAD 面可要求共享长边有密集节点；本面仍不能把整片短边当作插点禁令。
struct DenseTrimRectangle final : PlanarPatch {
    int segments;double width,height;
    DenseTrimRectangle(int count=16,double w=4,double h=8):PlanarPatch(false),segments(count),width(w),height(h){}
    int vertexCount() const override{return 2*(segments+1);}
    int edgeCount() const override{return vertexCount();}
    Point3D vertexPoint(int v) const override {
        return v<=segments?Point3D{0,height*v/segments,0}:Point3D{width,height*(vertexCount()-1-v)/segments,0};
    }
    std::pair<int,int> edgeVertices(int e) const override{return {e,(e+1)%edgeCount()};}
    FaceTopology faceTopology(int) const override{FaceTopology t;std::vector<EdgeUse> loop;
        for(int e=0;e<edgeCount();++e)loop.push_back({e,false,0});t.loops.push_back(loop);return t;}
};
// 两个相邻宽面共享一条长边，各面采用不同的 UV 坐标系。
struct SharedDenseRectangles : PlanarPatch {
    DenseTrimRectangle upper{64,11.85,4.652};
    SharedDenseRectangles():PlanarPatch(false){}
    int vertexCount() const override{return 258;}
    Point3D vertexPoint(int v) const override{
        if(v<130)return upper.vertexPoint(v);const int k=v-130;
        return k<64?Point3D{0,-4.652*(k+1)/64,0}:Point3D{11.85,-4.652*(128-k)/64,0};
    }
    int edgeCount() const override{return 259;}
    std::pair<int,int> edgeVertices(int e) const override{
        return e<130?upper.edgeVertices(e):std::pair<int,int>{e==130?0:e-1,e==258?129:e};
    }
    int faceCount() const override{return 2;}
    FaceTopology faceTopology(int face) const override{
        if(!face)return upper.faceTopology(0);FaceTopology t;std::vector<EdgeUse> loop;
        for(int e=130;e<259;++e)loop.push_back({e,false,0});loop.push_back({129,false,0});t.loops.push_back(loop);return t;
    }
    Point2D edgeUV(int face,const EdgeUse& use,double t) const override{
        auto p=edgePoint(use.edge,t);return face?Point2D{2*p.x+p.y+10,3*p.y+100}:Point2D{p.x,p.y};
    }
    Point3D facePoint(int face,Point2D uv) const override{
        if(!face)return {uv.x,uv.y,0};const double y=(uv.y-100)/3;return {(uv.x-y-10)/2,y,0};
    }
    void faceDerivatives(int face,Point2D,Point3D& du,Point3D& dv) const override{
        du=face?Point3D{.5,0,0}:Point3D{1,0,0};dv=face?Point3D{-1./6,1./3,0}:Point3D{0,1,0};
    }
};
struct FailureReuseAssembly final:SharedDenseRectangles {
    int vertexCount() const override{return 261;}
    int edgeCount() const override{return 262;}
    int faceCount() const override{return 3;}
    Point3D vertexPoint(int v) const override{
        if(v<258)return SharedDenseRectangles::vertexPoint(v);
        const Point3D p[]={{100,100,0},{100.25,100,0},{100.125,100.015625,0}};return p[v-258];}
    std::pair<int,int> edgeVertices(int e) const override{
        return e<259?SharedDenseRectangles::edgeVertices(e):std::pair<int,int>{258+e-259,258+(e-258)%3};}
    FaceTopology faceTopology(int f) const override{return f<2?SharedDenseRectangles::faceTopology(f):FaceTopology{{{{259},{260},{261}}}};}
    Point2D edgeUV(int f,const EdgeUse& use,double t) const override{
        if(f<2)return SharedDenseRectangles::edgeUV(f,use,t);auto p=edgePoint(use.edge,t);return {p.x-100,p.y-100};}
    Point3D facePoint(int f,Point2D uv) const override{return f<2?SharedDenseRectangles::facePoint(f,uv):Point3D{100+uv.x,100+uv.y,0};}
    void faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const override{
        if(f<2)SharedDenseRectangles::faceDerivatives(f,uv,du,dv);else{du={1,0,0};dv={0,1,0};}}
};

// 平面上的弯曲窄条：UV 度量正常，粗边界弦仍会造成映射面片翻折。
struct WarpedStrip : PlanarPatch {
    WarpedStrip():PlanarPatch(false) {}
    Point2D corner(int v) const {
        const Point2D p[]={{0,0},{0.02,0},{0.02,4},{0,4}};
        return p[v];
    }
    Point3D vertexPoint(int v) const override { return facePoint(0,corner(v)); }
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {
        const auto ends=edgeVertices(use.edge);
        const auto a=corner(ends.first),b=corner(ends.second);
        return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};
    }
    Point3D edgePoint(int e,double t) const override { return facePoint(0,edgeUV(0,{e,false,0},t)); }
    Point3D facePoint(int,Point2D uv) const override { return {uv.x+0.2*uv.y*uv.y,uv.y,0}; }
    void faceDerivatives(int,Point2D uv,Point3D& du,Point3D& dv) const override {
        du={1,0,0}; dv={0.4*uv.y,1,0};
    }
};

struct TorusStrip final : WarpedStrip {
    Point2D corner(int v) const {
        const Point2D p[]={{0,0},{2.12664,0},{2.12664,4.5},{0,4.5}};
        return p[v];
    }
    Point3D vertexPoint(int v) const override { return facePoint(0,corner(v)); }
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {
        const auto ends=edgeVertices(use.edge);
        const auto a=corner(ends.first),b=corner(ends.second);
        return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};
    }
    Point3D facePoint(int,Point2D p) const override {
        const double r=0.2,R=6.1;
        return {(R+r*std::cos(p.x))*std::cos(p.y),
                (R+r*std::cos(p.x))*std::sin(p.y),r*std::sin(p.x)};
    }
    Point3D faceNormal(int,Point2D p) const override {
        return {-std::cos(p.x)*std::cos(p.y),-std::cos(p.x)*std::sin(p.y),-std::sin(p.x)};
    }
    void faceDerivatives(int,Point2D p,Point3D& du,Point3D& dv) const override {
        du={-0.2*std::sin(p.x)*std::cos(p.y),-0.2*std::sin(p.x)*std::sin(p.y),0.2*std::cos(p.x)};
        dv={-(6.1+0.2*std::cos(p.x))*std::sin(p.y),(6.1+0.2*std::cos(p.x))*std::cos(p.y),0};
    }
};

struct SeparatePatches final : GeometrySource {
    PlanarPatch flat{false};
    WarpedStrip curved;
    mutable int flatBuilds=0;
    const GeometrySource& patch(int f) const { return f==0?static_cast<const GeometrySource&>(flat):curved; }
    int vertexCount() const override { return 8; }
    Point3D vertexPoint(int v) const override {
        auto p=patch(v/4).vertexPoint(v%4); if (v<4) p.x+=100; return p;
    }
    int edgeCount() const override { return 8; }
    std::pair<int,int> edgeVertices(int e) const override {
        const auto v=patch(e/4).edgeVertices(e%4); return {v.first+4*(e/4),v.second+4*(e/4)};
    }
    std::pair<double,double> edgeRange(int) const override { return {0,1}; }
    Point3D edgePoint(int e,double t) const override {
        auto p=patch(e/4).edgePoint(e%4,t); if (e<4) p.x+=100; return p;
    }
    int faceCount() const override { return 2; }
    FaceTopology faceTopology(int f) const override {
        auto topology=patch(f).faceTopology(0);
        for (auto& loop:topology.loops) for (auto& u:loop) u.edge+=4*f;
        return topology;
    }
    Point2D edgeUV(int f,const EdgeUse& u,double t) const override {
        return patch(f).edgeUV(0,{u.edge-4*f,u.reversed,u.branch},t);
    }
    Point3D facePoint(int f,Point2D uv) const override {
        auto p=patch(f).facePoint(0,uv); if (f==0) p.x+=100; return p;
    }
    Point3D faceNormal(int f,Point2D uv) const override { return patch(f).faceNormal(0,uv); }
    void faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const override {
        if (f==0 && uv.x==2 && uv.y==2) ++flatBuilds; // Chart-center builds, excluding boundary sizing queries.
        patch(f).faceDerivatives(0,uv,du,dv);
    }
};

struct CrossingSheets : GeometrySource {
    bool coplanar;
    explicit CrossingSheets(bool flat):coplanar(flat) {}
    int vertexCount() const override { return 8; }
    Point3D vertexPoint(int i) const override {
        static const Point2D p[]={{-1,-1},{1,-1},{1,1},{-1,1}};
        return facePoint(i/4,p[i%4]);
    }
    int edgeCount() const override { return 8; }
    std::pair<int,int> edgeVertices(int e) const override { return {e,(e/4)*4+(e+1)%4}; }
    std::pair<double,double> edgeRange(int) const override { return {0,1}; }
    Point3D edgePoint(int e,double t) const override {
        auto [a,b]=edgeVertices(e); auto p=vertexPoint(a),q=vertexPoint(b);
        return {p.x+(q.x-p.x)*t,p.y+(q.y-p.y)*t,p.z+(q.z-p.z)*t};
    }
    int faceCount() const override { return 2; }
    FaceTopology faceTopology(int f) const override {
        return {{{{4*f,false,0},{4*f+1,false,0},{4*f+2,false,0},{4*f+3,false,0}}}};
    }
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {
        static const Point2D p[]={{-1,-1},{1,-1},{1,1},{-1,1}};
        const auto a=p[use.edge%4],b=p[(use.edge+1)%4];
        return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};
    }
    Point3D facePoint(int f,Point2D p) const override {
        if (f==0) return {p.x,p.y,0};
        return coplanar ? Point3D{p.x+0.5,p.y,0} : Point3D{p.x,0,p.y};
    }
    Point3D faceNormal(int f,Point2D) const override {
        return f==0 || coplanar ? Point3D{0,0,1} : Point3D{0,-1,0};
    }
};

// 两个不相交的球面片相距 0.05；错开的采样使粗弦面片可能穿越。
struct NearSpherePatches final : CrossingSheets {
    NearSpherePatches():CrossingSheets(false) {}
    Point3D edgePoint(int e,double t) const override { return facePoint(e/4,edgeUV(e/4,{e,false,0},t)); }
    Point3D facePoint(int f,Point2D p) const override {
        const double r=12-0.05*f,u=0.6*p.x+0.12*f,v=0.5*p.y;
        return {r*std::cos(u)*std::cos(v),r*std::sin(u)*std::cos(v),r*std::sin(v)};
    }
    Point3D faceNormal(int f,Point2D p) const override {
        const double u=0.6*p.x+0.12*f,v=0.5*p.y;
        return {std::cos(u)*std::cos(v),std::sin(u)*std::cos(v),std::sin(v)};
    }
    void faceDerivatives(int f,Point2D p,Point3D& du,Point3D& dv) const override {
        const double r=12-0.05*f,u=0.6*p.x+0.12*f,v=0.5*p.y;
        du={-0.6*r*std::sin(u)*std::cos(v),0.6*r*std::cos(u)*std::cos(v),0};
        dv={-0.5*r*std::cos(u)*std::sin(v),-0.5*r*std::sin(u)*std::sin(v),0.5*r*std::cos(v)};
    }
};

struct JoinedCurvedPatches final : GeometrySource {
    int vertexCount() const override { return 6; }
    Point3D vertexPoint(int v) const override {
        const Point3D p[]={{-1,0,0},{0,0,0},{0,1,0},{-1,1,0},{1,0,0},{1,1,0}};
        return p[v];
    }
    int edgeCount() const override { return 7; }
    std::pair<int,int> edgeVertices(int e) const override {
        const int ends[][2]={{0,1},{1,2},{2,3},{3,0},{1,4},{4,5},{5,2}};
        return {ends[e][0],ends[e][1]};
    }
    std::pair<double,double> edgeRange(int) const override { return {0,1}; }
    Point3D edgePoint(int e,double t) const override {
        const auto [a,b]=edgeVertices(e);
        const auto p=vertexPoint(a),q=vertexPoint(b);
        return facePoint(0,{p.x+(q.x-p.x)*t,p.y+(q.y-p.y)*t});
    }
    int faceCount() const override { return 2; }
    FaceTopology faceTopology(int f) const override {
        if (f==0) return {{{{0,false,0},{1,false,0},{2,false,0},{3,false,0}}}};
        return {{{{4,false,0},{5,false,0},{6,false,0},{1,true,0}}}};
    }
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {
        const auto p=edgePoint(use.edge,t); return {p.x,p.y};
    }
    Point3D facePoint(int,Point2D p) const override {
        return {p.x,p.y,0.2*std::sin(3.141592653589793*p.x)*std::sin(3.141592653589793*p.y)};
    }
    Point3D faceNormal(int,Point2D p) const override {
        const double pi=3.141592653589793;
        return {-0.2*pi*std::cos(pi*p.x)*std::sin(pi*p.y),
                -0.2*pi*std::sin(pi*p.x)*std::cos(pi*p.y),1};
    }
};

bool testPatch(bool hole,double gap=1) {
    PlanarPatch patch(hole,gap);
    SurfaceCriteria criteria;
    criteria.targetSize = 1.0;
    criteria.minSize = 0.1;
    criteria.maxDeviation = 0.05;
    const SurfaceMeshResult r = generateCadSurfaceMesh(patch,criteria);
    if (!r.ok) { std::cerr << r.error << '\n'; return false; }
    if (r.mesh.triangles.empty() || r.triangleFaceIds.size()!=r.mesh.triangles.size()) {
        std::cerr << "missing triangles or metadata for hole=" << hole << '\n'; return false;
    }
    double area=0;
    for (const auto& t:r.mesh.triangles) {
        const auto a=r.mesh.vertices[t[0]], b=r.mesh.vertices[t[1]], c=r.mesh.vertices[t[2]];
        area += 0.5*std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x));
        if (hole && (a.x+b.x+c.x)/3.0 > gap && (a.x+b.x+c.x)/3.0 < 4-gap &&
            (a.y+b.y+c.y)/3.0 > gap && (a.y+b.y+c.y)/3.0 < 4-gap) {
            std::cerr << "triangle in hole: ("<<a.x<<","<<a.y<<") ("<<b.x<<","<<b.y<<") ("<<c.x<<","<<c.y<<")\n"; return false;
        }
    }
    const double expected=hole?16-(4-2*gap)*(4-2*gap):16;
    if (std::abs(area-expected)>=1e-6)
        std::cerr << "area " << area << " expected " << expected
                  << " triangles " << r.mesh.triangles.size() << '\n';
    if (gap<criteria.minSize && r.diagnostics.empty()) {
        std::cerr << "narrow feature/minSize conflict was not reported\n"; return false;
    }
    return std::abs(area-expected)<1e-6;
}

bool testSharedContact() {
    SurfaceMesh3D surface;
    surface.vertices={{5.2353036576337422,0.058867291449144474,23.574276558697687},
        {5.8281275065656866,-0.086326395253268587,23.198741801148405},
        {5.0899409891380509,0.10371560103998972,23.586338509108195},
        {5.7303344195260602,-0.068570835362671903,23.314349313723362}};
    surface.triangles={{0,1,2},{1,0,3}};
    std::string error;
    bool ok=validateSurfaceIntersections(surface,&error);
    if (!ok) std::cerr<<"valid shared CAD edge rejected: "<<error<<'\n';
    surface.vertices.push_back({5.8282820130588746,-0.091225344559930061,23.273782412360848});
    surface.triangles={{0,1,2},{1,3,4}};
    if (!validateSurfaceIntersections(surface,&error)) {
        std::cerr<<"valid shared CAD vertex rejected: "<<error<<'\n'; ok=false;
    }
    surface.vertices={{0,0,0},{1,0,0},{0,1,0},{1,-1,0}};
    surface.triangles={{0,1,2},{1,0,3}};
    ok=validateSurfaceIntersections(surface,&error)&&ok;
    surface.vertices[3]={0.2,0.2,0};
    if (validateSurfaceIntersections(surface,&error)) {
        std::cerr<<"overlapping coplanar neighbors accepted\n"; ok=false;
    }
    surface.vertices={{0,0,0},{2,0,0},{0,2,0},{1,1,-1},{1,1,1}};
    surface.triangles={{0,1,2},{0,3,4}};
    if (validateSurfaceIntersections(surface,&error)) {
        std::cerr<<"shared vertex hid an interior crossing\n"; ok=false;
    }
    return ok;
}

// Analytical sphere: curvature and physical sizing must be invariant under
// unit changes and UV shear; bounded adapters must never receive extrapolation.
bool testSurfaceSizing() {
    struct SphereQueries final : PlanarPatch {
        double radius=2,shear=0;
        bool zeroNormal=false,badNormal=false,badDerivatives=false;
        mutable bool queriedOutside=false;
        SphereQueries():PlanarPatch(false) {}
        bool faceParameterBounds(int,SurfaceParameterBounds& b) const override {
            b={-1,1,-1,1}; return true;
        }
        Point3D faceNormal(int,Point2D p) const override {
            queriedOutside|=p.x < -1 || p.x > 1 || p.y < -1 || p.y > 1;
            if (badNormal) return {std::numeric_limits<double>::quiet_NaN(),0,1};
            if (zeroNormal) return {};
            const double u=p.x+shear*p.y,v=p.y;
            return {std::cos(u)*std::cos(v),std::sin(u)*std::cos(v),std::sin(v)};
        }
        void faceDerivatives(int,Point2D p,Point3D& du,Point3D& dv) const override {
            const double u=p.x+shear*p.y,v=p.y;
            du={-radius*std::sin(u)*std::cos(v),radius*std::cos(u)*std::cos(v),0};
            dv={shear*du.x-radius*std::cos(u)*std::sin(v),
                shear*du.y-radius*std::sin(u)*std::sin(v),radius*std::cos(v)};
            if (badDerivatives) du={};
        }
    } geo;
    SurfaceCriteria c; c.maxNormalAngleDeg=10; c.maxDeviation=1;
    bool ok=true;
    for (double scale:{1.0,1000.0}) for (double shear:{0.0,20.0}) {
        geo.radius=2*scale; geo.shear=shear; c.maxDeviation=scale;
        const auto s=surfaceSizeAt(geo,0,{0.4,0.2},100*scale,c);
        const double expected=4*scale*std::sin(5*std::acos(-1.0)/180);
        if (!s.valid || std::abs(s.maxCurvature*geo.radius-1)>1e-6 ||
            std::abs(s.target/expected-1)>1e-6) {
            std::cerr<<"physical curvature size changed with units/UV shear\n"; ok=false;
        }
        SurfaceMetric metric; std::string error;
        if (!surfaceMetricAt(geo,0,{0.4,0.2},metric,&error)) { ok=false; continue; }
        const Point2D uv{0.43,0.27},mapped=metric.project(uv),back=metric.unproject(mapped);
        Point3D du,dv; geo.faceDerivatives(0,metric.origin,du,dv);
        const double tangent=std::hypot(du.x*0.03+dv.x*0.07,
            du.y*0.03+dv.y*0.07,du.z*0.03+dv.z*0.07);
        if (std::abs(std::hypot(mapped.x,mapped.y)/tangent-1)>1e-12 ||
            std::hypot(back.x-uv.x,back.y-uv.y)>1e-12) {
            std::cerr<<"local metric does not preserve physical tangent length\n"; ok=false;
        }
    }
    geo.radius=2; geo.shear=0; c.maxDeviation=0.0001;
    const auto deviation=surfaceSizeAt(geo,0,{1,1},100,c);
    if (!deviation.valid || geo.queriedOutside ||
        std::abs(deviation.target/std::sqrt(8*c.maxDeviation*geo.radius)-1)>1e-5) {
        std::cerr<<"bounded/deviation sizing failed\n"; ok=false;
    }
    geo.zeroNormal=true;
    const auto singular=surfaceSizeAt(geo,0,{0,0},1,c);
    if (!singular.valid || !singular.singular || singular.target!=1) ok=false;
    geo.zeroNormal=false; geo.badNormal=true;
    if (surfaceSizeAt(geo,0,{0,0},1,c).valid) ok=false;
    geo.badNormal=false; geo.badDerivatives=true;
    if (surfaceSizeAt(geo,0,{0,0},1,c).valid) ok=false;
    geo.badDerivatives=false;
    const double nan=std::numeric_limits<double>::quiet_NaN();
    c.maxNormalAngleDeg=nan;
    if (surfaceSizeAt(geo,0,{0,0},1,c).valid) {
        std::cerr<<"non-finite sizing criterion accepted\n"; ok=false;
    }
    c.maxNormalAngleDeg=30;
    if (surfaceSizeAt(geo,0,{0,0},nan,c).valid ||
        surfaceSizeAt(geo,0,{nan,0},1,c).valid) ok=false;
    geo.queriedOutside=false;
    if (!surfaceSizeAt(geo,0,{std::nextafter(1.0,2.0),1},1,c).valid || geo.queriedOutside) {
        std::cerr<<"parameter-bound roundoff was not clamped\n"; ok=false;
    }
    if (surfaceSizeAt(geo,0,{2,0},1,c).valid || geo.queriedOutside) {
        std::cerr<<"out-of-bounds sizing queried CAD\n"; ok=false;
    }
    struct BoundedPlane final : PlanarPatch {
        mutable bool outside=false;
        BoundedPlane():PlanarPatch(false) {}
        bool faceParameterBounds(int,SurfaceParameterBounds& b) const override {
            b={0,4,0,4}; return true;
        }
        Point3D facePoint(int,Point2D p) const override {
            outside|=p.x<0 || p.x>4 || p.y<0 || p.y>4;
            return {p.x,p.y,0};
        }
    } plane;
    const auto flat=surfaceSizeAt(plane,0,{0,4},1,c);
    if (!flat.valid || flat.target!=1 || flat.maxCurvature!=0 || plane.outside) {
        std::cerr<<"default bounded geometry derivatives extrapolated\n"; ok=false;
    }
    if (!ok) std::cerr<<"surface metric/sizing contract failed\n";
    return ok;
}
}

int main(int argc,char** argv) {
    if(argc==2&&std::string(argv[1])=="--narrow-frame"){
        SurfaceCriteria c;c.targetSize=c.maxSize=3.90333514021862;c.minSize=c.maxDeviation=.390333514021862;c.shapeOptimizationSweeps=0;
        NarrowFrame geometry;const auto r=generateCadSurfaceMesh(geometry,c);
        if(!r.ok){std::cerr<<r.error<<'\n';return 1;}
        const auto q=cadquality::analyze(r,c.maxRadiusEdgeRatio,c.minSize);
        std::cout<<"narrow frame bad="<<q.bad<<" worst="<<q.worst<<" work="<<r.work.totalWork<<'\n';
        if(q.invalid||q.bad)return 1;
        PhaseNarrowStrip phase;const auto s=generateCadSurfaceMesh(phase,c);
        if(!s.ok){std::cerr<<s.error<<'\n';return 1;}
        const auto sq=cadquality::analyze(s,c.maxRadiusEdgeRatio,c.minSize);
        std::cout<<"phase narrow strip bad="<<sq.bad<<" worst="<<sq.worst<<" work="<<s.work.totalWork<<'\n';
        if(sq.invalid||sq.bad)return 1;return 0;
    }
    if(argc==2&&std::string(argv[1])=="--quality-reuse"){
        struct Observer:MeshingObserver {uint64_t reused=0;
            void stageBegin(MeshStage,MeshEntityRef) noexcept override{}
            void stageEnd(MeshStage,MeshEntityRef) noexcept override{}
            void addCount(MeshCounter c,MeshEntityRef,std::uint64_t n) noexcept override{if(c==MeshCounter::QualityReusedFaces)reused+=n;}} observer;
        SurfaceCriteria c;c.targetSize=c.maxSize=3.90333514021862;c.minSize=c.maxDeviation=.390333514021862;c.shapeOptimizationSweeps=0;
        FailureReuseAssembly geometry;c.reuseFailedQualityPasses=false;const auto old=generateCadSurfaceMesh(geometry,c);
        c.reuseFailedQualityPasses=true;const auto next=generateCadSurfaceMesh(geometry,c,nullptr,{},&observer);
        if(!old.ok||!next.ok){std::cerr<<old.error<<' '<<next.error<<'\n';return 1;}
        std::cout<<"quality reuse="<<observer.reused<<" passes="<<old.work.schedulingPasses<<'/'<<next.work.schedulingPasses
            <<" points="<<old.mesh.vertices.size()<<'/'<<next.mesh.vertices.size()<<" triangles="<<old.mesh.triangles.size()<<'/'<<next.mesh.triangles.size()<<'\n';
        if(!observer.reused||old.mesh.triangles!=next.mesh.triangles||old.triangleFaceIds!=next.triangleFaceIds||
            old.mesh.vertices.size()!=next.mesh.vertices.size()||old.triangleBodyIds!=next.triangleBodyIds||
            old.triangleCornerUV.size()!=next.triangleCornerUV.size()||old.boundaryEdges.size()!=next.boundaryEdges.size())return 1;
        for(size_t v=0;v<old.mesh.vertices.size();++v){auto a=old.mesh.vertices[v],b=next.mesh.vertices[v];if(a.x!=b.x||a.y!=b.y||a.z!=b.z)return 1;}
        for(size_t t=0;t<old.triangleCornerUV.size();++t)for(int k=0;k<3;++k){auto a=old.triangleCornerUV[t][k],b=next.triangleCornerUV[t][k];if(a.x!=b.x||a.y!=b.y)return 1;}
        for(size_t e=0;e<old.boundaryEdges.size();++e){auto a=old.boundaryEdges[e],b=next.boundaryEdges[e];if(a.a!=b.a||a.b!=b.b||a.cadEdge!=b.cadEdge)return 1;}
        std::cout<<"PASS same-version failed face reuse, changed shared faces recomputed; reused="<<observer.reused<<'\n';return 0;
    }
    if(argc==2&&std::string(argv[1])=="--narrow-quality"){
        SurfaceCriteria c;c.targetSize=c.maxSize=3.90333514021862;c.minSize=c.maxDeviation=.390333514021862;
        c.minSize=.7;c.shapeOptimizationSweeps=0;NarrowRing geometry;const auto r=generateCadSurfaceMesh(geometry,c);
        if(!r.ok){std::cerr<<"narrow ring failed: "<<r.error<<'\n';return 1;}
        const auto q=cadquality::analyze(r,c.maxRadiusEdgeRatio,c.minSize);
        std::cout<<"narrow ring bad="<<q.bad<<" worst="<<q.worst<<" points="<<r.mesh.vertices.size()<<'\n';
        if(q.invalid||q.bad)return 1;
        for(int v=0;v<geometry.vertexCount();++v){auto p=geometry.vertexPoint(v);
            if(!std::any_of(r.mesh.vertices.begin(),r.mesh.vertices.end(),[&](auto x){return x.x==p.x&&x.y==p.y&&x.z==p.z;}))return 1;}
        std::cout<<"PASS narrow ring source coordination preserves all original CAD vertices\n";return 0;
    }
    if(argc==2 && std::string(argv[1])=="--shape-quality"){
        SurfaceCriteria c;c.targetSize=.8;c.minSize=.08;c.maxDeviation=.08;
        PlanarPatch geometry(true);
        c.shapeOptimizationSweeps=0;const auto before=generateCadSurfaceMesh(geometry,c);
        c.shapeOptimizationSweeps=4;const auto after=generateCadSurfaceMesh(geometry,c);
        if(!before.ok||!after.ok){std::cerr<<"shape comparison failed: "<<before.error<<' '<<after.error<<'\n';return 1;}
        const auto a=cadquality::analyze(before,c.maxRadiusEdgeRatio,c.minSize),b=cadquality::analyze(after,c.maxRadiusEdgeRatio,c.minSize);
        std::cout<<"shape P5 angle "<<a.p5Angle<<" -> "<<b.p5Angle<<", P5 q "<<a.p5Shape<<" -> "<<b.p5Shape
            <<", P50 q "<<a.p50Shape<<" -> "<<b.p50Shape<<'\n';
        if(b.bad>a.bad||b.worst>a.worst*(1+1e-10)||after.mesh.triangles.size()!=before.mesh.triangles.size()||
            b.p5Shape<a.p5Shape-1e-10 ||
            (b.p5Angle<a.p5Angle+3 && b.p5Shape<a.p5Shape*1.1))return 1;
        if(after.boundaryEdges.size()!=before.boundaryEdges.size())return 1;
        for(size_t i=0;i<before.boundaryEdges.size();++i){auto x=before.boundaryEdges[i],y=after.boundaryEdges[i];
            if(x.a!=y.a||x.b!=y.b||x.cadEdge!=y.cadEdge)return 1;}
        c.shapeOptimizationSweeps=-1;if(generateCadSurfaceMesh(geometry,c).ok)return 1;
        std::cout<<"PASS bounded shape optimization preserves constraints and improves already acceptable mesh\n";return 0;
    }
    if(argc==2 && std::string(argv[1])=="--shared-boundary"){
        SurfaceCriteria c;c.targetSize=c.maxSize=3.90333514021862;c.minSize=c.maxDeviation=.390333514021862;
        SharedDenseRectangles geometry;const auto r=generateCadSurfaceMesh(geometry,c);
        if(!r.ok){std::cerr<<"shared boundary synchronization failed: "<<r.error<<'\n';return 1;}
        for(int face=0;face<2;++face)if(!std::any_of(r.diagnostics.begin(),r.diagnostics.end(),[&](const auto& d){
            return d.find("face "+std::to_string(face)+" shared boundary locally synchronized:")!=std::string::npos;})){
            std::cerr<<"shared boundary face was not synchronized: "<<face<<'\n';for(auto d:r.diagnostics)std::cerr<<d<<'\n';return 1;}
        for(auto edge:r.boundaryEdges)if(edge.cadEdge==129){int count=0,direction=0;bool faces[2]={false,false};
            for(size_t t=0;t<r.mesh.triangles.size();++t){auto tri=r.mesh.triangles[t];for(int k=0;k<3;++k){
                if(tri[k]==edge.a&&tri[(k+1)%3]==edge.b){++count;++direction;faces[r.triangleFaceIds[t]]=true;}
                if(tri[k]==edge.b&&tri[(k+1)%3]==edge.a){++count;--direction;faces[r.triangleFaceIds[t]]=true;}}}
            if(count!=2||direction!=0||!faces[0]||!faces[1]){std::cerr<<"shared boundary lost incidence or orientation\n";return 1;}}
        for(size_t t=0;t<r.mesh.triangles.size();++t)for(int k=0;k<3;++k){auto p=geometry.facePoint(r.triangleFaceIds[t],r.triangleCornerUV[t][k]);
            auto q=r.mesh.vertices[r.mesh.triangles[t][k]];if(std::hypot(p.x-q.x,p.y-q.y,p.z-q.z)>1e-10)return 1;}
        std::cout<<"PASS shared CAD node IDs, opposite incidence and separate UV charts\n";return 0;
    }
    if(argc>=2 && std::string(argv[1])=="--dense-boundary"){
        // 宽平面共享的密边界远小于常规 min，仍应能建立局部质量过渡。
        SurfaceCriteria c;c.targetSize=c.maxSize=3.90333514021862;c.minSize=c.maxDeviation=.390333514021862;
        if(argc==3&&std::string(argv[2])=="--without-shape")c.shapeOptimizationSweeps=0;
        const DenseTrimRectangle geometry{64,11.85,4.652};
        const auto dense=generateCadSurfaceMesh(geometry,c);
        if(!dense.ok){std::cerr<<"dense boundary generation failed: "<<dense.error<<'\n';return 1;}
        const auto report=cadquality::analyze(dense,c.maxRadiusEdgeRatio,c.minSize);
        for(int v=0;v<geometry.vertexCount();++v){const auto original=geometry.vertexPoint(v);
            if(!std::any_of(dense.mesh.vertices.begin(),dense.mesh.vertices.end(),[&](const auto& p){
                return p.x==original.x && p.y==original.y && p.z==original.z;})){
                std::cerr<<"local size transition lost original CAD vertex\n";return 1;}}
        for(const auto& e:dense.boundaryEdges){const auto ends=geometry.edgeVertices(e.cadEdge);
            const auto a=geometry.vertexPoint(ends.first),b=geometry.vertexPoint(ends.second);
            const double original=std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
            const auto x=dense.mesh.vertices[e.a],y=dense.mesh.vertices[e.b];
            if(original<c.minSize && std::hypot(x.x-y.x,x.y-y.y,x.z-y.z)<original*(1-1e-8)){
                std::cerr<<"local size transition subdivided mandatory short CAD segment\n";return 1;}}
        std::cout<<"preserved original CAD vertices and mandatory short boundary segments\n";
        if(report.invalid || report.bad){
            std::cerr<<"mandatory dense boundary lacks local size transition: "<<dense.error
                <<" bad="<<report.bad<<" worst="<<report.worst<<" points="<<dense.mesh.vertices.size()<<'\n';
            cadquality::writeCsv(std::cerr,report,dense);
            for(const auto& d:dense.diagnostics)std::cerr<<d<<'\n';return 1;}
        std::cout.precision(17);std::cout<<"PASS dense boundary: points="<<dense.mesh.vertices.size()
            <<" triangles="<<dense.mesh.triangles.size()<<" bad="<<report.bad<<" worst="<<report.worst<<'\n';
        return 0;
    }
    {
        SurfaceCriteria c;c.targetSize=c.maxSize=3.90333514021862;c.minSize=c.maxDeviation=.390333514021862;
        const DenseTrimRectangle geometry{128,6,44};
        const auto dense=generateCadSurfaceMesh(geometry,c);
        if(!dense.ok){std::cerr<<"dense CAD rectangle failed: "<<dense.error<<'\n';return 1;}
        const auto report=cadquality::analyze(dense,c.maxRadiusEdgeRatio,c.minSize);
        if(report.invalid || report.bad){
            std::cerr<<"dense rectangle caps unresolved: bad="<<report.bad<<" worst="<<report.worst<<'\n';
            cadquality::writeCsv(std::cerr,report,dense);
            for(const auto& d:dense.diagnostics)std::cerr<<d<<'\n';return 1;}
        for(const auto& e:dense.boundaryEdges){const auto ends=geometry.edgeVertices(e.cadEdge);
            const auto a=geometry.vertexPoint(ends.first),b=geometry.vertexPoint(ends.second);
            const double original=std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
            const auto x=dense.mesh.vertices[e.a],y=dense.mesh.vertices[e.b];
            if(original<c.minSize && std::hypot(x.x-y.x,x.y-y.y,x.z-y.z)<original*(1-1e-8)){
                std::cerr<<"dense cap optimization subdivided protected short CAD edge\n";return 1;}}
    }
    {
        SurfaceCriteria c;c.targetSize=c.maxSize=4;c.minSize=.6;c.maxDeviation=.4;
        const auto dense=generateCadSurfaceMesh(DenseTrimRectangle{},c);
        const auto report=cadquality::analyze(dense,c.maxRadiusEdgeRatio,c.minSize);
        if(!dense.ok || report.invalid || report.bad || dense.measuredWorstRadiusEdgeRatio>c.maxRadiusEdgeRatio*(1+1e-8)){
            std::cerr<<"existing short trim incorrectly blocks admissible quality insertion: "<<dense.error
                <<" worst="<<dense.measuredWorstRadiusEdgeRatio<<'\n';
            cadquality::writeCsv(std::cerr,report,dense);
            for(const auto& d:dense.diagnostics)std::cerr<<d<<'\n';
            return 1;}
        for(const auto& e:dense.boundaryEdges){const auto ends=DenseTrimRectangle{}.edgeVertices(e.cadEdge);
            const auto originalA=DenseTrimRectangle{}.vertexPoint(ends.first),originalB=DenseTrimRectangle{}.vertexPoint(ends.second);
            if(std::hypot(originalA.x-originalB.x,originalA.y-originalB.y)<c.minSize &&
                std::hypot(dense.mesh.vertices[e.a].x-dense.mesh.vertices[e.b].x,dense.mesh.vertices[e.a].y-dense.mesh.vertices[e.b].y)<.5*(1-1e-8)){
                std::cerr<<"existing short CAD edge unnecessarily subdivided\n";return 1;}
        }
    }
    {
        SurfaceCriteria c;c.targetSize=c.maxSize=1;c.minSize=.005;c.maxDeviation=.01;
        const auto shortEdge=generateCadSurfaceMesh(ShortBoundaryPatch{},c);
        if(!shortEdge.ok || shortEdge.measuredWorstRadiusEdgeRatio>c.maxRadiusEdgeRatio*(1+1e-8)) {
            std::cerr<<"ordinary short boundary lost quality refinement: "<<shortEdge.error
                <<" quality="<<shortEdge.measuredWorstRadiusEdgeRatio<<'\n';
            for(const auto& d:shortEdge.diagnostics)std::cerr<<d<<'\n';return 1;
        }
        const auto sheared=generateCadSurfaceMesh(ShearedShortBoundary{},c);
        std::cout<<"short-boundary physical radius-edge normal="<<shortEdge.measuredWorstRadiusEdgeRatio
            <<" sheared="<<sheared.measuredWorstRadiusEdgeRatio<<'\n';
        if(!sheared.ok || sheared.measuredWorstRadiusEdgeRatio>c.maxRadiusEdgeRatio*(1+1e-8) ||
            std::abs(sheared.measuredWorstRadiusEdgeRatio-shortEdge.measuredWorstRadiusEdgeRatio)>.1) {
            std::cerr<<"UV shear changed physical short-boundary quality: "<<sheared.error
                <<" ratios="<<shortEdge.measuredWorstRadiusEdgeRatio<<','<<sheared.measuredWorstRadiusEdgeRatio<<'\n';return 1;
        }
        c.minSize=.1;
        const auto limited=generateCadSurfaceMesh(ShortBoundaryPatch{},c);
        if(!limited.ok)return 1;
        for(const auto& edge:limited.boundaryEdges)if(edge.cadEdge!=0) {
            const auto a=limited.mesh.vertices[edge.a],b=limited.mesh.vertices[edge.b];
            if(std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)<c.minSize*(1-1e-8)) {
                std::cerr<<"quality-only boundary subdivision crossed regular minimum size\n";return 1;
            }
        }
        c.minSize=.02;
        const auto nonuniform=generateCadSurfaceMesh(NonuniformShortBoundary{},c);
        if(!nonuniform.ok)return 1;
        for(auto edge:nonuniform.boundaryEdges){auto a=nonuniform.mesh.vertices[edge.a],b=nonuniform.mesh.vertices[edge.b];
            if(std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)<c.minSize*(1-1e-8)) {
                std::cerr<<"parameter midpoint crossed physical minimum size\n";return 1;
            }
        }
    }
    {
        struct BoundarySize final:MeshSizeField {double sizeAt(const Point3D& p) const override {
            return p.x==0 || p.x==4 || p.y==0 || p.y==4?1:2;
        }} field;
        SurfaceCriteria c;c.targetSize=c.maxSize=2;c.growthRatio=2;c.maxVertices=1000;c.maxIterations=10000;
        const auto square=generateCadSurfaceMesh(PlanarPatch{false},c,&field);
        if(!square.ok || square.mesh.vertices.size()>25) {std::cerr<<"regular Cartesian seed wasted one extra node per cell: "<<square.error<<" points="<<square.mesh.vertices.size()<<'\n';return 1;}
    }
    {
        SurfaceCriteria c;c.targetSize=c.maxSize=2;c.minSize=c.maxDeviation=0.2;c.maxVertices=1000;c.maxIterations=20000;
        const auto noisy=generateCadSurfaceMesh(NoisyRectangle{},c);
        if(!noisy.ok || noisy.mesh.vertices.size()>300) {std::cerr<<"trim axis drift caused rectangular Cartesian explosion: "<<noisy.error<<'\n';return 1;}
    }
    const double n=134217728.0;
    const double m=134217729.0;
    const bool predicates=orient2d({n+1,n},{n,n-1},{0,0})==-1 &&
        incircle({5*m,0},{3*m,4*m},{-5*m,0},{-3*m,-4*m})==0;
    if (!predicates) std::cerr << "2D cancellation predicates failed\n";
    SurfaceCriteria c; c.targetSize=1; c.maxDeviation=0.01;
    const auto crossing=generateCadSurfaceMesh(CrossingSheets(false),c);
    const auto overlapping=generateCadSurfaceMesh(CrossingSheets(true),c);
    const bool intersections=!crossing.ok && !overlapping.ok &&
        crossing.error.find("intersect")!=std::string::npos &&
        overlapping.error.find("intersect")!=std::string::npos;
    if (!intersections) std::cerr << "crossing CAD sheets accepted\n";
    bool ok=testSurfaceSizing()&&testSharedContact()&&predicates&&intersections;
    SurfaceCriteria nearCriteria; nearCriteria.targetSize=20; nearCriteria.maxDeviation=2;
    nearCriteria.maxIterations=50000;
    const auto nearby=generateCadSurfaceMesh(NearSpherePatches{},nearCriteria);
    if (!nearby.ok) std::cerr<<"nonintersecting near sphere patches failed: "<<nearby.error<<'\n';
    const bool nearbyProtected=nearby.ok && nearby.work.intersectionRefinements>0 &&
        nearby.measuredMaxDeviation<=nearCriteria.maxDeviation*(1+1e-8) &&
        nearby.measuredMaxNormalAngleDeg<=nearCriteria.maxNormalAngleDeg*(1+1e-8);
    if (!nearbyProtected) std::cerr<<"near sphere protection did not satisfy criteria\n";
    ok=ok&&nearbyProtected;
    SurfaceCriteria stripCriteria;
    stripCriteria.targetSize=20; stripCriteria.maxDeviation=2;
    stripCriteria.maxIterations=1000;
    auto torusCriteria=stripCriteria; torusCriteria.maxIterations=5000;
    const auto torus=generateCadSurfaceMesh(TorusStrip(),torusCriteria);
    if (!torus.ok || torus.measuredMaxNormalAngleDeg>30*(1+1e-8)) {
        std::cerr<<"thin torus normal refinement failed: "<<torus.error<<'\n'; ok=false;
    }
    const auto strip=generateCadSurfaceMesh(WarpedStrip(),stripCriteria);
    if (!strip.ok || strip.boundaryEdges.size()<4 || strip.measuredMaxDeviation>stripCriteria.maxDeviation*(1+1e-8)) {
        std::cerr<<"curved narrow strip boundary protection failed: "<<strip.error<<'\n'; ok=false;
    }
    double stripArea=0;
    for (size_t i=0;i<strip.mesh.triangles.size();++i) {
        const auto& tri=strip.mesh.triangles[i];
        const auto a=strip.mesh.vertices[tri[0]],b=strip.mesh.vertices[tri[1]],d=strip.mesh.vertices[tri[2]];
        const double twice=(b.x-a.x)*(d.y-a.y)-(b.y-a.y)*(d.x-a.x);
        if (!(twice>0)) { std::cerr<<"inverted narrow strip facet\n"; ok=false; }
        const auto& uv=strip.triangleCornerUV[i];
        stripArea+=((uv[1].x-uv[0].x)*(uv[2].y-uv[0].y)-
                    (uv[1].y-uv[0].y)*(uv[2].x-uv[0].x))/2;
    }
    if (strip.ok && std::abs(stripArea-0.08)>1e-10) {
        std::cerr<<"narrow strip UV domain area not preserved\n"; ok=false;
    }
    SeparatePatches separate;
    const auto separateResult=generateCadSurfaceMesh(separate,stripCriteria);
    if (!separateResult.ok || separate.flatBuilds!=1) {
        std::cerr<<"unrelated CAD face rebuilt "<<separate.flatBuilds<<" times: "<<separateResult.error<<'\n'; ok=false;
    }
    if (separateResult.ok) {
        std::vector<bool> used(separateResult.mesh.vertices.size(),false);
        for (const auto& t:separateResult.mesh.triangles) for (int v:t) used[v]=true;
        for (const auto& e:separateResult.boundaryEdges) { used[e.a]=true; used[e.b]=true; }
        for (bool referenced:used) if (!referenced) { std::cerr<<"discarded face node exported\n"; ok=false; }
        if (separateResult.work.faceBuilds<2 ||
            separateResult.work.allocatedVertices<int(separateResult.mesh.vertices.size())) {
            std::cerr<<"local rebuilding work statistics inconsistent\n"; ok=false;
        }
    }
    SurfaceCriteria joinedCriteria; joinedCriteria.targetSize=1;
    joinedCriteria.maxDeviation=0.01; joinedCriteria.maxIterations=10000;
    const auto joined=generateCadSurfaceMesh(JoinedCurvedPatches(),joinedCriteria);
    bool shared=joined.ok;
    for (const auto& edge:joined.boundaryEdges) if (edge.cadEdge==1) {
        int incident=0; bool left=false,right=false;
        for (size_t i=0;i<joined.mesh.triangles.size();++i) {
            const auto& t=joined.mesh.triangles[i];
            for (int k=0;k<3;++k) if ((t[k]==edge.a && t[(k+1)%3]==edge.b) ||
                                      (t[k]==edge.b && t[(k+1)%3]==edge.a)) {
                ++incident; left|=joined.triangleFaceIds[i]==0; right|=joined.triangleFaceIds[i]==1;
            }
        }
        shared=shared && incident==2 && left && right;
    }
    if (!shared) std::cerr << "joined curved CAD patches failed: "<<joined.error<<'\n';
    ok=shared&&ok;
    ok=testPatch(false)&&ok;
    ok=testPatch(true)&&ok;
    ok=testPatch(true,0.05)&&ok;
    const auto sharp=generateCadSurfaceMesh(SharpPatch(),c);
    if (!sharp.ok || sharp.diagnostics.empty()) { std::cerr << "sharp feature case failed: "<<sharp.error<<'\n'; ok=false; }
    const auto cancelled=generateCadSurfaceMesh(PlanarPatch(false),c,nullptr,[](int,int){return false;});
    if (cancelled.ok || cancelled.error.find("cancelled")==std::string::npos) ok=false;
    c.maxVertices=4;
    const auto budget=generateCadSurfaceMesh(PlanarPatch(false),c);
    if (budget.ok || budget.error.find("budget")==std::string::npos) ok=false;
    c.maxVertices=5;
    const auto recursiveBudget=generateCadSurfaceMesh(PlanarPatch(false),c);
    if (recursiveBudget.ok || recursiveBudget.mesh.vertices.size()>5) ok=false;
    struct InvalidField final : MeshSizeField {
        double sizeAt(const Point3D&) const override { return std::numeric_limits<double>::quiet_NaN(); }
    } invalidField;
    c.maxVertices=200000;
    struct SmallField final : MeshSizeField {
        double sizeAt(const Point3D&) const override { return 0.4; }
    } smallField;
    const auto uniform=generateCadSurfaceMesh(PlanarPatch(false),c);
    SurfaceCriteria capped=c; capped.targetSize=2; capped.minSize=0.04; capped.maxSize=0.42;
    const auto limited=generateCadSurfaceMesh(PlanarPatch(false),capped);
    if (!limited.ok || limited.measuredMaxEdgeLength>capped.maxSize*(1+1e-8)) {
        std::cerr<<"explicit maximum size not enforced: "<<limited.error<<'\n'; ok=false;
    }
    capped.minSize=0.5;
    const auto invalidRange=generateCadSurfaceMesh(PlanarPatch(false),capped);
    if (invalidRange.ok || invalidRange.error.find("criteria")==std::string::npos) {
        std::cerr<<"minimum larger than explicit maximum accepted\n"; ok=false;
    }
    capped.minSize=0.04; capped.maxSize=std::numeric_limits<double>::quiet_NaN();
    if (generateCadSurfaceMesh(PlanarPatch(false),capped).ok) { std::cerr<<"invalid maximum accepted\n"; ok=false; }
    capped.maxSize=.42; capped.targetSize=std::numeric_limits<double>::infinity();
    if(generateCadSurfaceMesh(PlanarPatch(false),capped).ok) {std::cerr<<"non-finite target hidden by maximum\n";ok=false;}
    const auto sized=generateCadSurfaceMesh(PlanarPatch(false),c,&smallField);
    if (!uniform.ok || !sized.ok || sized.mesh.triangles.size()<=uniform.mesh.triangles.size() ||
        sized.measuredMaxEdgeLength>0.42) {
        std::cerr << "3D scalar size field was not respected\n"; ok=false;
    }
    const auto field=generateCadSurfaceMesh(PlanarPatch(false),c,&invalidField);
    if (field.ok || field.error.find("size field")==std::string::npos) ok=false;
    std::cout << (ok?"PASS":"FAIL") << " predicates, holes, narrow/sharp features, intersections and guards\n";
    return ok?0:1;
}
