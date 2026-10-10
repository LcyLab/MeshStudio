#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include "mesh/surface/legacy/SurfaceFrontalDelaunay.h"
#include "mesh/surface/legacy/SurfaceChart.h"
#include "mesh/planar/Delaunay2D.h"
#include "mesh/planar/Constrained2D.h"
#include "CadSurfaceQualityReport.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace mesher;

namespace {
// 同一物理平面采用两套参数，检测是否误把 UV 等边当作物理等边。
struct Plane final : GeometrySource {
    bool sheared=false,hole=false,nonlinear=false;
    static double physicalY(double v){return v+.9*std::sin(2*std::acos(-1.)*v)/(2*std::acos(-1.));}
    static double parameterY(double y){double lo=0,hi=12;for(int i=0;i<64;++i){const double mid=(lo+hi)/2;if(physicalY(mid)<y)lo=mid;else hi=mid;}return (lo+hi)/2;}
    int vertexCount() const override {return hole?8:4;}
    Point3D vertexPoint(int i) const override {
        const Point3D p[]={{0,0,0},{16,0,0},{16,12,0},{0,12,0},{6,4,0},{6,8,0},{10,8,0},{10,4,0}};return p[i];
    }
    int edgeCount() const override {return vertexCount();}
    std::pair<int,int> edgeVertices(int e) const override {return {e,e<4?(e+1)%4:4+(e-3)%4};}
    std::pair<double,double> edgeRange(int) const override {return {0,1};}
    Point3D edgePoint(int e,double t) const override {
        const auto ends=edgeVertices(e);const auto a=vertexPoint(ends.first),b=vertexPoint(ends.second);
        return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0};
    }
    int faceCount() const override {return 1;}
    FaceTopology faceTopology(int) const override {
        FaceTopology topology{{{{0},{1},{2},{3}}}};if(hole)topology.loops.push_back({{4},{5},{6},{7}});return topology;
    }
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {
        const auto p=edgePoint(use.edge,t);if(nonlinear)return {p.x,parameterY(p.y)};
        return sheared?Point2D{(p.x-2*p.y)/10,p.y/3}:Point2D{p.x,p.y};
    }
    Point3D facePoint(int,Point2D uv) const override {
        if(nonlinear)return {uv.x,physicalY(uv.y),0};
        return sheared?Point3D{10*uv.x+6*uv.y,3*uv.y,0}:Point3D{uv.x,uv.y,0};
    }
    void faceDerivatives(int,Point2D uv,Point3D& du,Point3D& dv) const override {
        if(nonlinear){du={1,0,0};dv={0,1+.9*std::cos(2*std::acos(-1.)*uv.y),0};return;}
        du=sheared?Point3D{10,0,0}:Point3D{1,0,0};dv=sheared?Point3D{6,3,0}:Point3D{0,1,0};
    }
    Point3D faceNormal(int,Point2D) const override {return {0,0,1};}
};

struct Cylinder final : GeometrySource {
    const double width=2*std::acos(-1.);
    Point2D uvVertex(int i) const {const Point2D p[]={{0,0},{width,0},{width,8},{0,8}};return p[i];}
    int vertexCount() const override {return 4;}
    Point3D vertexPoint(int i) const override {return facePoint(0,uvVertex(i));}
    int edgeCount() const override {return 4;}
    std::pair<int,int> edgeVertices(int e) const override {return {e,(e+1)%4};}
    std::pair<double,double> edgeRange(int) const override {return {0,1};}
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {
        const auto a=uvVertex(use.edge),b=uvVertex((use.edge+1)%4);return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};
    }
    Point3D edgePoint(int e,double t) const override {return facePoint(0,edgeUV(0,{e},t));}
    int faceCount() const override {return 1;}
    FaceTopology faceTopology(int) const override {return {{{{0},{1},{2},{3}}}};}
    Point3D facePoint(int,Point2D p) const override {return {2*std::cos(p.x/2),2*std::sin(p.x/2),p.y};}
    void faceDerivatives(int,Point2D p,Point3D& du,Point3D& dv) const override {
        du={-std::sin(p.x/2),std::cos(p.x/2),0};dv={0,0,1};
    }
    Point3D faceNormal(int,Point2D p) const override {return {std::cos(p.x/2),std::sin(p.x/2),0};}
    bool faceParameterBounds(int,SurfaceParameterBounds& bounds) const override {bounds={0,width,0,8};return true;}
};

struct ShapeSummary {int inside=0,nearEquilateral=0;double fraction=0,meanShape=0;};
ShapeSummary measure(const SurfaceMeshResult& result,bool cylinder=false,bool hole=false) {
    ShapeSummary summary;
    for(size_t index=0;index<result.mesh.triangles.size();++index) {
        const auto ids=result.mesh.triangles[index];
        const auto a=result.mesh.vertices[ids[0]],b=result.mesh.vertices[ids[1]],c=result.mesh.vertices[ids[2]];
        if(cylinder) {
            const auto uv=result.triangleCornerUV[index];
            if(std::min({uv[0].x,uv[1].x,uv[2].x})<1||std::max({uv[0].x,uv[1].x,uv[2].x})>2*std::acos(-1.)-1||
               std::min({a.z,b.z,c.z})<1||std::max({a.z,b.z,c.z})>7)continue;
        } else if(std::min({a.x,b.x,c.x})<2 || std::max({a.x,b.x,c.x})>14 ||
                  std::min({a.y,b.y,c.y})<2 || std::max({a.y,b.y,c.y})>10)continue;
        // 内部统计离全部真实边界至少两单位；洞边也是真实边界。
        if(hole) {
            const auto distance=[](Point3D p){return std::hypot(std::max({6-p.x,0.,p.x-10}),std::max({4-p.y,0.,p.y-8}));};
            if(std::min({distance(a),distance(b),distance(c)})<2)continue;
        }
        const auto q=cadquality::measure(a,b,c);++summary.inside;summary.meanShape+=q.shape;
        const auto length=[](auto p,auto v){return std::hypot(p.x-v.x,p.y-v.y,p.z-v.z);};
        const double sides[]={length(a,b),length(b,c),length(c,a)};bool near=q.valid;
        for(int k=0;k<3;++k) {
            const double x=sides[k],y=sides[(k+1)%3],z=sides[(k+2)%3];
            const double angle=std::acos(std::clamp((x*x+y*y-z*z)/(2*x*y),-1.,1.))*180/std::acos(-1.);
            near=near&&angle>=50&&angle<=70;
        }
        summary.nearEquilateral+=near;
    }
    if(summary.inside){summary.fraction=double(summary.nearEquilateral)/summary.inside;summary.meanShape/=summary.inside;}
    return summary;
}

// 独立准备已满足几何尺度的边界，直接停止在前沿出口；不调用硬细化或末尾优化。
SurfaceMeshResult frontalOnly(const GeometrySource& geometry,const SurfaceCriteria& criteria) {
    SurfaceMeshResult result;Mesh2D mesh;SurfaceDomainState domain;std::vector<Point2D> boundary;
    std::vector<std::pair<int,int>> constraints;const auto topology=geometry.faceTopology(0);
    Point2D center{};int corners=0;
    for(const auto& loop:topology.loops)for(auto use:loop){const auto uv=geometry.edgeUV(0,use,0);center.x+=uv.x;center.y+=uv.y;++corners;}
    center.x/=corners;center.y/=corners;const auto size=surfaceSizeAt(geometry,0,center,criteria.targetSize,criteria);
    if(!size.valid){result.error=size.error;return result;}
    for(const auto& loop:topology.loops) {
        const int first=int(boundary.size());
        for(auto use:loop) {
            double length=0;auto previous=geometry.edgePoint(use.edge,0);
            for(int k=1;k<=32;++k){const auto next=geometry.edgePoint(use.edge,k/32.);length+=std::hypot(next.x-previous.x,next.y-previous.y,next.z-previous.z);previous=next;}
            const int count=std::max(1,int(std::ceil(length/size.target)));
            for(int k=0;k<count;++k) {
                const double t=use.reversed?1-k/double(count):k/double(count);
                boundary.push_back(geometry.edgeUV(0,use,t));result.mesh.vertices.push_back(geometry.edgePoint(use.edge,t));
            }
        }
        for(int v=first;v<int(boundary.size());++v)constraints.push_back({v+3,v+1==int(boundary.size())?first+3:v+4});
    }
    const auto initial=delaunay2D(mesh,boundary,true);if(!initial.ok){result.error=initial.error;return result;}
    std::vector<int> global(mesh.points.size(),-1);for(int v=0;v<int(boundary.size());++v)global[v+3]=v;
    if(!enforceConstraints(mesh,constraints,&result.error)||!domain.initialize(mesh,constraints,&result.error))return result;
    SurfaceFrontalStatistics stats;int work=0;
    if(!generateSurfaceFrontalPoints(mesh,global,result.mesh.vertices,domain,constraints,geometry,0,criteria,
        [&](Point3D){return criteria.targetSize;},[&](Point3D p){const int v=int(result.mesh.vertices.size());result.mesh.vertices.push_back(p);return v;},
        work,stats,&result.error))return result;
    for(int t=0;t<int(mesh.tris.size());++t)if(domain.values()[t]) {
        const auto tri=mesh.tris[t];result.mesh.triangles.push_back({global[tri.v[0]],global[tri.v[1]],global[tri.v[2]]});
        result.triangleCornerUV.push_back({mesh.points[tri.v[0]],mesh.points[tri.v[1]],mesh.points[tri.v[2]]});
    }
    result.ok=true;return result;
}
}

int main(int argc,char** argv) {
    SurfaceCriteria criteria;criteria.targetSize=1;criteria.minSize=.1;criteria.maxSize=1.05;
    criteria.maxDeviation=.001;criteria.shapeOptimizationSweeps=0;
    criteria.pointPlacement=SurfacePointPlacement::FrontalDelaunay;
    if(argc==2)criteria.pointPlacement=std::string(argv[1])=="reference"?SurfacePointPlacement::ReferenceFrontal:SurfacePointPlacement::PhysicalReferenceFrontal;
    bool ok=true;ShapeSummary reference;
    const auto rawPlane=frontalOnly(Plane(),criteria);const auto rawPlaneShape=measure(rawPlane);
    std::cout<<"frontal-only plane interior="<<rawPlaneShape.inside<<" near-equilateral="<<rawPlaneShape.fraction<<'\n';
    ok=ok&&rawPlane.ok&&rawPlaneShape.inside>=50&&rawPlaneShape.fraction>=.7;
    Plane nonlinear;nonlinear.nonlinear=true;
    const auto rawNonlinear=frontalOnly(nonlinear,criteria);const auto nonlinearShape=measure(rawNonlinear);
    std::cout<<"frontal-only nonlinear plane ok="<<rawNonlinear.ok<<" interior="<<nonlinearShape.inside<<" near-equilateral="<<nonlinearShape.fraction<<'\n';
    if(!rawNonlinear.ok)std::cerr<<rawNonlinear.error<<'\n';
    ok=ok&&rawNonlinear.ok&&nonlinearShape.inside>=50&&nonlinearShape.fraction>=.7&&std::abs(nonlinearShape.fraction-rawPlaneShape.fraction)<.1;
    for(bool shear:{false,true}) {
        Plane plane;plane.sheared=shear;const auto result=generateCadSurfaceMesh(plane,criteria);
        if(!result.ok){std::cerr<<"FAIL plane generation: "<<result.error<<'\n';return 1;}
        const auto summary=measure(result);
        std::cout<<"plane shear="<<shear<<" vertices="<<result.mesh.vertices.size()<<" triangles="<<result.mesh.triangles.size()
                 <<" interior="<<summary.inside<<" near-equilateral="<<summary.fraction<<" mean-q="<<summary.meanShape
                 <<" frontal="<<result.work.frontalInserted<<" additional="<<result.work.insertedSurfacePoints-result.work.frontalInserted<<'\n';
        ok=ok&&summary.inside>=50&&summary.fraction>=.7;
        ok=ok&&result.measuredMaxDeviation<=criteria.maxDeviation&&result.measuredMaxEdgeLength<=criteria.maxSize*(1+1e-8);
        ok=ok&&result.work.frontalInserted>0&&result.work.qualityFlips==0&&result.work.qualityMoves==0;
        if(!shear)reference=summary;
        else ok=ok&&std::abs(summary.fraction-reference.fraction)<.1;
    }
    if(criteria.pointPlacement==SurfacePointPlacement::PhysicalReferenceFrontal) {
        // 物理平面仅换参数；放宽参数对应点偏差，单独检查完整管线会否覆盖物理连接。
        auto local=criteria;local.maxDeviation=.3;
        const auto result=generateCadSurfaceMesh(nonlinear,local);const auto summary=measure(result);
        std::cout<<"complete nonlinear plane ok="<<result.ok<<" interior="<<summary.inside<<" near-equilateral="<<summary.fraction
            <<" triangles="<<result.mesh.triangles.size()<<" flips="<<result.work.delaunayFlips<<" hard-edge="<<result.work.refinementEdgeViolations<<'\n';
        if(!result.ok)std::cerr<<result.error<<'\n';
        ok=ok&&result.ok&&summary.inside>=50&&summary.fraction>=.7&&std::abs(summary.fraction-reference.fraction)<.1&&
            result.measuredMaxEdgeLength<=local.maxSize*(1+1e-8)&&result.measuredMaxDeviation<=local.maxDeviation;
    }
    Plane holed;holed.hole=true;const auto hole=generateCadSurfaceMesh(holed,criteria);double area=0;
    if(!hole.ok){std::cerr<<"FAIL holed domain: "<<hole.error<<'\n';return 1;}
    for(auto ids:hole.mesh.triangles) {
        const auto a=hole.mesh.vertices[ids[0]],b=hole.mesh.vertices[ids[1]],c=hole.mesh.vertices[ids[2]];
        area+=std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/2;
        const double x=(a.x+b.x+c.x)/3,y=(a.y+b.y+c.y)/3;
        ok=ok&&!(x>6&&x<10&&y>4&&y<8);
    }
    std::cout<<"holed plane area="<<area<<" with-hole-boundary="<<measure(hole).fraction
             <<" far-from-all-boundaries="<<measure(hole,false,true).fraction<<'\n';
    ok=ok&&std::abs(area-176)<1e-8&&measure(hole,false,true).inside>0&&measure(hole,false,true).fraction>=.7;
    const auto cancelled=generateCadSurfaceMesh(holed,criteria,nullptr,[](int,int){return false;});
    ok=ok&&!cancelled.ok&&cancelled.error.find("cancelled")!=std::string::npos;
    SurfaceCriteria limited=criteria;limited.maxIterations=80;
    const auto budget=generateCadSurfaceMesh(Plane(),limited);
    ok=ok&&!budget.ok&&budget.error.find("budget")!=std::string::npos;
    criteria.targetSize=.5;criteria.minSize=.05;criteria.maxSize=.525;criteria.maxDeviation=.005;criteria.maxNormalAngleDeg=20;
    const auto rawCylinder=frontalOnly(Cylinder(),criteria);const auto rawCylinderShape=measure(rawCylinder,true);
    std::cout<<"frontal-only cylinder interior="<<rawCylinderShape.inside<<" near-equilateral="<<rawCylinderShape.fraction<<'\n';
    ok=ok&&rawCylinder.ok&&rawCylinderShape.inside>=50&&rawCylinderShape.fraction>=.7;
    const auto curved=generateCadSurfaceMesh(Cylinder(),criteria);
    if(!curved.ok){std::cerr<<"FAIL cylinder generation: "<<curved.error<<'\n';return 1;}
    const auto curvedShape=measure(curved,true);
    std::cout<<"cylinder vertices="<<curved.mesh.vertices.size()<<" triangles="<<curved.mesh.triangles.size()
             <<" interior="<<curvedShape.inside<<" near-equilateral="<<curvedShape.fraction<<" mean-q="<<curvedShape.meanShape
             <<" deviation="<<curved.measuredMaxDeviation<<'\n';
    std::cout<<"cylinder frontal="<<curved.work.frontalInserted<<" additional="<<curved.work.insertedSurfacePoints-curved.work.frontalInserted
             <<" hard edge="<<curved.work.refinementEdgeViolations<<" deviation="<<curved.work.refinementDeviationViolations
             <<" normal="<<curved.work.refinementNormalViolations<<" quality="<<curved.work.refinementQualityViolations
             <<" flips="<<curved.work.delaunayFlips<<" after-legalize="<<curved.work.violationsAfterLegalization<<'\n';
    for(const auto& message:curved.diagnostics)std::cout<<message<<'\n';
    ok=ok&&curvedShape.inside>=50&&curvedShape.fraction>=.7&&curved.measuredMaxDeviation<=criteria.maxDeviation&&
        curved.measuredMaxNormalAngleDeg<=criteria.maxNormalAngleDeg;
    if(!ok)std::cerr<<"FAIL generation without final smoothing must yield mostly near-equilateral interior triangles\n";
    return ok?0:1;
}
