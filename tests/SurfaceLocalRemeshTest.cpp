#include "mesh/surface/legacy/SurfaceLocalRemesh.h"
#include "geometry/predicates/GeomPredicates.h"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <algorithm>
using namespace mesher;
Mesh2D fan(){Mesh2D m;for(auto p:std::vector<Point2D>{{0,0},{2,0},{2,2},{0,2},{.05,.3}})m.addPoint(p);
    for(int i=0;i<4;++i)m.addTri(4,i,(i+1)%4);return m;}
bool same(const Mesh2D& a,const Mesh2D& b){if(a.points.size()!=b.points.size()||a.tris.size()!=b.tris.size())return false;
    for(int i=0;i<int(a.tris.size());++i){for(int k=0;k<3;++k)if(a.tris[i].v[k]!=b.tris[i].v[k]||a.tris[i].n[k]!=b.tris[i].n[k])return false;
        if(a.triangleVersion(i)!=b.triangleVersion(i))return false;}
    for(int i=0;i<int(a.points.size());++i)if(a.exactPoint(a.points[i])!=b.exactPoint(b.points[i]))return false;return true;}
int main(){
    {
        Mesh2D accepted;std::vector<int> ids;std::vector<std::pair<int,int>> trim;std::string error;
        const std::vector<std::array<int,3>> triangles{{10,11,12},{10,12,13}};
        const std::vector<std::array<Point2D,3>> uv{{Point2D{0,0},{1,0},{1,1}},{Point2D{0,0},{1,1},{0,1}}};
        if(!rebuildAcceptedSurfaceChart(triangles,uv,accepted,ids,trim,&error)||accepted.points.size()!=4||trim.size()!=4||
            ids!=std::vector<int>{10,11,12,13})return 1;
        SurfaceDomainState domain;if(!domain.initialize(accepted,trim,&error)||!domain.verify(accepted,trim,&error))return 1;
        auto saved=accepted;const auto oldIds=ids;const auto oldTrim=trim;
        auto invalid=uv;invalid[0][2]=invalid[0][1];
        if(rebuildAcceptedSurfaceChart(triangles,invalid,accepted,ids,trim,&error)||!same(saved,accepted)||ids!=oldIds||trim!=oldTrim)return 1;
        invalid=uv;invalid[0][0].x=std::numeric_limits<double>::quiet_NaN();
        if(rebuildAcceptedSurfaceChart(triangles,invalid,accepted,ids,trim,&error)||!same(saved,accepted)||ids!=oldIds||trim!=oldTrim)return 1;
    }
    const std::vector<std::pair<int,int>> boundary{{0,1},{1,2},{2,3},{3,0}};std::string error;
    {
        for(double offset:{0.,-.05}){
            auto local=fan();auto trim=boundary;SurfaceDomainState tags;tags.initialize(local,trim,&error);
            int added=-1;MeshEditDelta edit;
            if(!trySplitSurfaceBoundaryEdge(local,0,1,{1,offset},trim,tags,&added,&edit,&error) ||
                added!=5 || local.tris.size()!=5 || local.points.size()!=6 || trim.size()!=5 ||
                local.points[4].x!=.05 || local.points[4].y!=.3 || !local.hasEdgeAny(0,added) ||
                !local.hasEdgeAny(added,1) || local.hasEdgeAny(0,1) || !local.validate(&error) || !tags.verify(local,trim,&error)){
                std::cerr<<"boundary split lost accepted interior layout: "<<error<<'\n';return 1;}
        }
        auto local=fan(),saved=local;auto trim=boundary;SurfaceDomainState tags;tags.initialize(local,trim,&error);
        int added=99;
        if(trySplitSurfaceBoundaryEdge(local,0,1,{1,0},trim,tags,&added,nullptr,&error,{},
            [](const Mesh2D&,int,const MeshEditDelta&){return false;}) || added!=-1 || trim!=boundary ||
            !same(local,saved) || !tags.verify(local,trim,&error))return 1;
        bool exception=false;try{trySplitSurfaceBoundaryEdge(local,0,1,{1,0},trim,tags,&added,nullptr,&error,{},
            [](const Mesh2D&,int,const MeshEditDelta&)->bool{throw std::runtime_error("CAD veto exception");});}
        catch(const std::runtime_error&){exception=true;}
        if(!exception || !same(local,saved) || trim!=boundary || !tags.verify(local,trim,&error))return 1;
        for(auto p:std::vector<Point2D>{{1,5},{.05,.3},{std::numeric_limits<double>::quiet_NaN(),0}})
            if(trySplitSurfaceBoundaryEdge(local,0,1,p,trim,tags,&added,nullptr,&error) || !same(local,saved) || trim!=boundary)return 1;
        Mesh2D annulus;for(auto p:std::vector<Point2D>{{-1,-1},{3,-1},{3,3},{-1,3},{0,0},{2,0},{2,2},{0,2}})annulus.addPoint(p);
        for(int i=0;i<4;++i){const int j=(i+1)%4;annulus.addTri(i,j,4+j);annulus.addTri(i,4+j,4+i);}
        annulus.addTri(4,5,6);annulus.addTri(4,6,7);
        std::vector<std::pair<int,int>> loops;for(int i=0;i<4;++i){loops.push_back({i,(i+1)%4});loops.push_back({4+i,4+(i+1)%4});}
        SurfaceDomainState regions;regions.initialize(annulus,loops,&error);
        if(!trySplitSurfaceBoundaryEdge(annulus,4,5,{1,.1},loops,regions,&added,nullptr,&error) ||
            std::count(regions.values().begin(),regions.values().end(),true)!=9 || annulus.tris.size()!=12 ||
            !annulus.validate(&error) || !regions.verify(annulus,loops,&error)){
            std::cerr<<"boundary split mixed hole and surface labels: "<<error<<'\n';return 1;}
    }
    const auto cost=[](std::array<int,3>){return 1.;};
    auto mesh=fan();SurfaceDomainState domain;bool ok=domain.initialize(mesh,boundary,&error);
    MeshEditDelta delta;ok=tryRemeshSurfaceVertex(mesh,4,boundary,domain,cost,&delta,&error)&&ok;
    ok=ok&&mesh.tris.size()==2&&mesh.points.size()==5&&mesh.exactPoint({.05,.3})==4&&mesh.validate(&error)&&domain.verify(mesh,boundary,&error);
    for(auto tri:mesh.tris)for(int v:tri.v)ok=ok&&v!=4;
    auto protectedMesh=fan(),original=protectedMesh;SurfaceDomainState protectedDomain;protectedDomain.initialize(protectedMesh,boundary,&error);
    ok=ok&&!tryRemeshSurfaceVertex(protectedMesh,0,boundary,protectedDomain,cost,nullptr,&error)&&same(protectedMesh,original);
    auto vetoed=fan(),before=vetoed;SurfaceDomainState vetoDomain;vetoDomain.initialize(vetoed,boundary,&error);
    ok=ok&&!tryRemeshSurfaceVertex(vetoed,4,boundary,vetoDomain,cost,nullptr,&error,{},[](const Mesh2D&,const MeshEditDelta&){return false;})&&same(vetoed,before)&&vetoDomain.verify(vetoed,boundary,&error);
    bool propagated=false;try{tryRemeshSurfaceVertex(vetoed,4,boundary,vetoDomain,cost,nullptr,&error,{},
        [](const Mesh2D&,const MeshEditDelta&)->bool{throw std::runtime_error("CAD check failed");});}catch(const std::runtime_error&){propagated=true;}
    ok=ok&&propagated&&same(vetoed,before)&&vetoDomain.verify(vetoed,boundary,&error);
    ok=ok&&!tryRemeshSurfaceVertex(vetoed,4,boundary,vetoDomain,[](std::array<int,3>){return std::numeric_limits<double>::infinity();},
        nullptr,&error)&&same(vetoed,before);
    const std::vector<int> incomplete{0,1,2},duplicate{0,1,2,2},stale{0,1,2,9},complete{0,1,2,3};
    for(const auto* slots:{&incomplete,&duplicate,&stale})
        ok=ok&&!tryRemeshSurfaceVertex(vetoed,4,boundary,vetoDomain,cost,nullptr,&error,{},{},slots)&&same(vetoed,before);
    ok=ok&&tryRemeshSurfaceVertex(vetoed,4,boundary,vetoDomain,cost,nullptr,&error,{},{},&complete)&&
        same(vetoed,mesh)&&vetoDomain.verify(vetoed,boundary,&error);
    Mesh2D concave;for(auto p:std::vector<Point2D>{{2,0},{.5,.5},{0,2},{-2,0},{0,-2},{0,0}})concave.addPoint(p);
    std::vector<std::pair<int,int>> trim;for(int i=0;i<5;++i){concave.addTri(5,i,(i+1)%5);trim.push_back({i,(i+1)%5});}
    const auto area=[](const Mesh2D& m){double sum=0;for(auto t:m.tris)sum+=orient2d(m.points[t.v[0]],m.points[t.v[1]],m.points[t.v[2]])/2;return sum;};
    const double originalArea=area(concave);SurfaceDomainState concaveDomain;concaveDomain.initialize(concave,trim,&error);
    ok=ok&&tryRemeshSurfaceVertex(concave,5,trim,concaveDomain,cost,nullptr,&error)&&concave.tris.size()==3&&
        std::abs(area(concave)-originalArea)<1e-12&&concaveDomain.verify(concave,trim,&error)&&concave.validate(&error);
    std::cout<<(ok?"PASS":"FAIL")<<" protected surface fan remesh and veto/exception rollback\n";return ok?0:1;
}
