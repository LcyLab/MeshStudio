#include "mesh/surface/legacy/SurfaceFrontalDelaunay.h"
#include "mesh/planar/Delaunay2D.h"
#include "mesh/planar/Constrained2D.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace mesher;
namespace {
struct Plane : GeometrySource {
    enum class Fault {None,Point,Normal,Fold};Fault fault=Fault::None;
    int vertexCount() const override {return 4;}
    Point3D vertexPoint(int i) const override {const Point3D p[]={{0,0,0},{4,0,0},{4,4,0},{0,4,0}};return p[i];}
    int edgeCount() const override {return 4;}
    std::pair<int,int> edgeVertices(int e) const override {return {e,(e+1)%4};}
    std::pair<double,double> edgeRange(int) const override {return {0,1};}
    Point3D edgePoint(int e,double t) const override {const auto a=vertexPoint(e),b=vertexPoint((e+1)%4);return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0};}
    int faceCount() const override {return 1;}
    FaceTopology faceTopology(int) const override {return {{{{0},{1},{2},{3}}}};}
    Point2D edgeUV(int,const EdgeUse& use,double t) const override {const auto p=edgePoint(use.edge,t);return {p.x,p.y};}
    Point3D facePoint(int,Point2D p) const override {
        if(fault==Fault::Point)return {std::numeric_limits<double>::quiet_NaN(),0,0};
        return {p.x,fault==Fault::Fold?-p.y:p.y,0};
    }
    Point3D faceNormal(int,Point2D) const override {return fault==Fault::Normal?Point3D{0,0,std::numeric_limits<double>::quiet_NaN()}:Point3D{0,0,1};}
    void faceDerivatives(int,Point2D,Point3D& u,Point3D& v) const override {u={1,0,0};v={0,1,0};}
};
struct Fixture {
    Mesh2D mesh;std::vector<int> global;std::vector<Point3D> xyz;
    std::vector<std::pair<int,int>> constraints;SurfaceDomainState domain;
    Fixture() {
        Plane plane;std::vector<Point2D> boundary;
        for(int e=0;e<4;++e)for(int k=0;k<4;++k){const auto p=plane.edgePoint(e,k/4.);boundary.push_back({p.x,p.y});xyz.push_back(p);}
        const auto initial=delaunay2D(mesh,boundary,true);if(!initial.ok)throw std::runtime_error(initial.error);
        global.assign(mesh.points.size(),-1);
        for(int v=0;v<int(boundary.size());++v){global[v+3]=v;constraints.push_back({v+3,3+(v+1)%int(boundary.size())});}
        std::string error;if(!enforceConstraints(mesh,constraints,&error)||!domain.initialize(mesh,constraints,&error))throw std::runtime_error(error);
    }
    bool valid() {std::string error;return global.size()==mesh.points.size()&&mesh.validate(&error)&&domain.verify(mesh,constraints,&error);}
};
std::vector<std::array<int,3>> connections(const Mesh2D& mesh) {
    std::vector<std::array<int,3>> result;for(auto tri:mesh.tris)result.push_back({tri.v[0],tri.v[1],tri.v[2]});return result;
}
bool run(Fixture& f,const Plane& plane,SurfaceCriteria c,int& work,SurfaceFrontalStatistics& stats,std::string& error,MeshingProgress progress={}) {
    return generateSurfaceFrontalPoints(f.mesh,f.global,f.xyz,f.domain,f.constraints,plane,0,c,
        [](Point3D){return 1.;},[&](Point3D p){const int v=int(f.xyz.size());f.xyz.push_back(p);return v;},work,stats,&error,progress);
}
}
int main() {
    SurfaceCriteria c;c.targetSize=1;c.minSize=.1;c.maxDeviation=.001;Plane plane;bool ok=true;
    {
        Fixture f;SurfaceFrontalStatistics stats;int work=0;std::string error;
        const bool generated=run(f,plane,c,work,stats,error);
        ok=ok&&generated&&stats.inserted>0&&work==stats.attempted&&f.valid();
        std::cout<<"direct frontal inserted="<<stats.inserted<<" rejected="<<stats.rejected<<'\n';
    }
    {
        Fixture f;SurfaceFrontalStatistics stats;int work=0;std::string error;
        const bool generated=run(f,plane,c,work,stats,error,[&](int,int){return stats.attempted==0;});
        ok=ok&&!generated&&error.find("cancelled during frontal")!=std::string::npos&&stats.attempted>0&&stats.inserted>0&&f.valid();
        std::cout<<"cancel after frontal insertion: "<<error<<'\n';
    }
    {
        Fixture f;SurfaceFrontalStatistics stats;int work=0;std::string error;auto limited=c;limited.maxIterations=1;
        const bool generated=run(f,plane,limited,work,stats,error);
        ok=ok&&!generated&&error=="surface frontal generation budget exceeded"&&work==1&&stats.attempted==1&&f.valid();
    }
    for(auto fault:{Plane::Fault::Point,Plane::Fault::Normal,Plane::Fault::Fold}) {
        Fixture f;const auto before=connections(f.mesh);const auto beforeGlobal=f.global;const auto beforeDomain=f.domain.values();
        const auto points=f.mesh.points.size(),physical=f.xyz.size();SurfaceFrontalStatistics stats;int work=0;std::string error;
        Plane invalid;invalid.fault=fault;const bool generated=run(f,invalid,c,work,stats,error);
        const bool unchanged=f.valid()&&connections(f.mesh)==before&&f.global==beforeGlobal&&f.domain.values()==beforeDomain&&
            f.mesh.points.size()==points&&f.xyz.size()==physical;
        if(fault==Plane::Fault::Fold)ok=ok&&generated&&stats.inserted==0&&stats.rejected>0&&stats.refusals.size()<=8&&unchanged;
        else ok=ok&&!generated&&error.find("non-finite")!=std::string::npos&&unchanged;
        std::cout<<"fault="<<int(fault)<<" ok="<<generated<<" rejected="<<stats.rejected<<" unchanged="<<unchanged<<" error="<<error<<'\n';
    }
    {
        // ABC 的最短边 BC 邻接另一张未满足尺寸的片，不能把它当作活动边。
        Fixture f;f.mesh.clear();f.xyz={{0,0,0},{4,0,0},{2,1,0},{4,4,0}};f.global={0,1,2,3};
        for(auto p:f.xyz)f.mesh.addPoint({p.x,p.y});f.mesh.addTri(0,1,2);f.mesh.addTri(2,1,3);
        f.constraints={{0,1},{1,3},{3,2},{2,0}};std::string error;
        ok=ok&&f.domain.initialize(f.mesh,f.constraints,&error);
        SurfaceFrontalStatistics stats;int work=0;const bool generated=run(f,plane,c,work,stats,error);
        bool active=false;
        if(!stats.samples.empty()) {
            const auto sample=stats.samples.front();
            active=sample.source==SurfaceFrontalEdgeSource::ProtectedConstraint&&
                sample.baseUV[0].x==2&&sample.baseUV[0].y==1&&sample.baseUV[1].x==0&&sample.baseUV[1].y==0;
        }
        ok=ok&&generated&&active&&stats.samples.size()<=8&&f.valid();
        std::cout<<"active edge identity="<<active<<'\n';
    }
    for(int position:{1,2})for(double invalidSize:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        Fixture f;f.mesh.clear();f.xyz={{0,0,0},{4,0,0},{0,4,0}};f.global={0,1,2};
        for(auto p:f.xyz)f.mesh.addPoint({p.x,p.y});f.mesh.addTri(0,1,2);f.constraints={{0,1},{1,2},{2,0}};
        std::string error;ok=ok&&f.domain.initialize(f.mesh,f.constraints,&error);const auto before=connections(f.mesh);
        const auto bad=f.xyz[position];SurfaceFrontalStatistics stats;int work=0;
        const bool generated=generateSurfaceFrontalPoints(f.mesh,f.global,f.xyz,f.domain,f.constraints,plane,0,c,
            [&](Point3D p){return p.x==bad.x&&p.y==bad.y?invalidSize:1.;},
            [&](Point3D p){const int v=int(f.xyz.size());f.xyz.push_back(p);return v;},work,stats,&error);
        ok=ok&&!generated&&error.find("invalid frontal target size")!=std::string::npos&&connections(f.mesh)==before&&f.xyz.size()==3&&f.valid();
        std::cout<<"invalid target position="<<position<<" rejected="<<!generated<<" untouched="<<(f.xyz.size()==3)<<'\n';
    }
    {
        Fixture f;const size_t live=f.xyz.size();f.xyz.resize(live+16);size_t active=live;
        auto limited=c;limited.maxVertices=int(f.xyz.size());SurfaceFrontalStatistics stats;int work=0;std::string error;
        const bool generated=generateSurfaceFrontalPoints(f.mesh,f.global,f.xyz,f.domain,f.constraints,plane,0,limited,
            [](Point3D){return 1.;},[&](Point3D p){const int v=int(active++);f.xyz.at(v)=p;return v;},work,stats,&error,
            [&](int,int){return stats.attempted==0;},{},[&]{return active;});
        ok=ok&&!generated&&error.find("cancelled during frontal")!=std::string::npos&&stats.inserted==1&&active==live+1&&f.valid();
        std::cout<<"full pool with reusable slots inserted="<<stats.inserted<<" error="<<error<<'\n';
    }
    if(!ok)std::cerr<<"FAIL direct frontal guard/rollback coverage\n";
    return ok?0:1;
}
