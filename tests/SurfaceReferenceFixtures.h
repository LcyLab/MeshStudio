#pragma once
#include "mesh/surface/legacy/SurfaceReferenceFront.h"
#include "mesh/planar/Delaunay2D.h"
#include "mesh/planar/Constrained2D.h"
#include <cmath>
#include <stdexcept>

namespace referencefixture {
using namespace mesher;
struct AnalyticSurface : GeometrySource {
    // 0=平面，1=仿射剪切平面，2=非线性平面，3=R2圆柱，4=R8球面。
    int shape=0;
    explicit AnalyticSurface(int s=0):shape(s){}
    int vertexCount() const override {return 0;}
    Point3D vertexPoint(int) const override {return {};}
    int edgeCount() const override {return 0;}
    std::pair<int,int> edgeVertices(int) const override {return {-1,-1};}
    std::pair<double,double> edgeRange(int) const override {return {0,1};}
    Point3D edgePoint(int,double) const override {return {};}
    int faceCount() const override {return 1;}
    FaceTopology faceTopology(int) const override {return {};}
    Point2D edgeUV(int,const EdgeUse&,double) const override {return {};}
    Point3D facePoint(int,Point2D p) const override {
        if(shape==1)return {10*p.x+6*p.y,3*p.y,0};
        if(shape==2)return {p.x,p.y+.9*std::sin(2*std::acos(-1.)*p.y)/(2*std::acos(-1.)),0};
        if(shape==3)return {2*std::cos(p.x),2*std::sin(p.x),p.y};
        if(shape==4)return {8*std::cos(p.y)*std::cos(p.x),8*std::cos(p.y)*std::sin(p.x),8*std::sin(p.y)};
        return {p.x,p.y,0};
    }
    Point3D faceNormal(int,Point2D p) const override {
        if(shape==3)return {std::cos(p.x),std::sin(p.x),0};
        if(shape==4){const auto q=facePoint(0,p);return {q.x/8,q.y/8,q.z/8};}
        return {0,0,1};
    }
    void faceDerivatives(int,Point2D p,Point3D& u,Point3D& v) const override {
        if(shape==1){u={10,0,0};v={6,3,0};}
        else if(shape==2){u={1,0,0};v={0,1+.9*std::cos(2*std::acos(-1.)*p.y),0};}
        else if(shape==3){u={-2*std::sin(p.x),2*std::cos(p.x),0};v={0,0,1};}
        else if(shape==4){u={-8*std::cos(p.y)*std::sin(p.x),8*std::cos(p.y)*std::cos(p.x),0};v={-8*std::sin(p.y)*std::cos(p.x),-8*std::sin(p.y)*std::sin(p.x),8*std::cos(p.y)};}
        else {u={1,0,0};v={0,1,0};}
    }
    Point2D coordinates(Point3D p) const {
        if(shape==1)return {(p.x-2*p.y)/10,p.y/3};
        if(shape==2){double lo=0,hi=12;for(int k=0;k<64;++k){const double v=(lo+hi)/2;if(facePoint(0,{p.x,v}).y<p.y)lo=v;else hi=v;}return {p.x,(lo+hi)/2};}
        if(shape==3)return {std::atan2(p.y,p.x),p.z};
        if(shape==4)return {std::atan2(p.y,p.x),std::asin(p.z/8)};
        return {p.x,p.y};
    }
};
struct Fixture {
    Mesh2D mesh;SurfaceDomainState domain;
    std::vector<int> global;
    std::vector<Point3D> xyz;
    std::vector<SurfaceReferenceSize> sizes;
    std::vector<std::pair<int,int>> constraints;
    void initialize(const std::vector<Point2D>& boundaryUV,const std::vector<Point3D>& boundaryXYZ,
        const std::vector<std::pair<int,int>>& edges,const std::vector<SurfaceReferenceSize>& boundarySizes) {
        xyz=boundaryXYZ;const auto result=delaunay2D(mesh,boundaryUV,true);if(!result.ok)throw std::runtime_error(result.error);
        global.assign(mesh.points.size(),-1);sizes.resize(mesh.points.size());
        for(int k=0;k<int(boundaryUV.size());++k){global[k+3]=k;sizes[k+3]=boundarySizes[k];}
        for(auto e:edges)constraints.push_back({e.first+3,e.second+3});
        std::string error;if(!enforceConstraints(mesh,constraints,&error)||!domain.initialize(mesh,constraints,&error))throw std::runtime_error(error);
    }
    void rectangle(const AnalyticSurface& geometry,bool hole=false) {
        std::vector<Point2D> uv;std::vector<Point3D> positions;std::vector<std::pair<int,int>> edges;
        const Point3D corners[]={{0,0,0},{16,0,0},{16,12,0},{0,12,0},{6,4,0},{6,8,0},{10,8,0},{10,4,0}};
        for(int loop=0;loop<(hole?2:1);++loop) {
            const int first=int(uv.size());
            for(int e=0;e<4;++e) {
                const auto a=corners[4*loop+e],b=corners[4*loop+(e+1)%4];
                const int count=int(std::hypot(b.x-a.x,b.y-a.y));
                for(int k=0;k<count;++k){const Point3D p{a.x+(b.x-a.x)*k/count,a.y+(b.y-a.y)*k/count,0};positions.push_back(p);uv.push_back(geometry.coordinates(p));}
            }
            for(int k=first;k<int(uv.size());++k)edges.push_back({k,k+1==int(uv.size())?first:k+1});
        }
        initialize(uv,positions,edges,std::vector<SurfaceReferenceSize>(uv.size(),{1,1}));
    }
    bool run(const GeometrySource& geometry,SurfaceReferenceStatistics& stats,std::string& error,
        SurfaceCriteria criteria={},MeshingProgress progress={},SurfaceReferenceOptions options={},
        const std::function<double(Point2D,Point3D)>& maximumEdgeAt={}) {
        int work=0;
        return generateSurfaceReferenceFront(mesh,global,xyz,domain,constraints,geometry,0,criteria,sizes,
            [](Point2D,Point3D){return 1.;},[&](Point3D p){const int v=int(xyz.size());xyz.push_back(p);return v;},work,stats,&error,options,progress,{}, {},maximumEdgeAt);
    }
};
}
