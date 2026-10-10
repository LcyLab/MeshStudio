#include "mesh/surface/legacy/SurfaceDomainState.h"
#include "mesh/planar/Delaunay2D.h"
#include "mesh/planar/Constrained2D.h"
#include "geometry/predicates/GeomPredicates.h"
#include <iostream>
#include <stdexcept>
using namespace mesher;
namespace {
void require(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
bool flip(Mesh2D& mesh,const SurfaceDomainState& domain,bool tag) {
    for (int t=0;t<int(mesh.tris.size());++t) {
        if (domain.values()[t]!=tag) continue;
        const auto tri=mesh.tris[t];
        for (int k=0;k<3;++k) {
            const int n=tri.n[k];
            if (n<0 || domain.values()[n]!=tag) continue;
            const int a=tri.v[k],b=tri.v[(k+1)%3],c=tri.v[(k+2)%3];
            int d=-1; for (int v:mesh.tris[n].v) if (v!=a && v!=b) d=v;
            if (d<0 || mesh.hasEdgeAny(c,d) ||
                orient2d(mesh.points[c],mesh.points[d],mesh.points[a])*orient2d(mesh.points[c],mesh.points[d],mesh.points[b])>=0) continue;
            mesh.removeTri(std::max(t,n)); mesh.removeTri(std::min(t,n));
            auto add=[&](int x,int y,int z) {
                if (orient2d(mesh.points[x],mesh.points[y],mesh.points[z])<0) std::swap(y,z);
                mesh.addTri(x,y,z);
            };
            add(c,d,a); add(d,c,b); return true;
        }
    }
    return false;
}
void scenario(const std::vector<Point2D>& points,const std::vector<std::pair<int,int>>& rings,
              const std::vector<Point2D>& insertions) {
    Mesh2D mesh; require(delaunay2D(mesh,points,true).ok,"initial mesh");
    std::string error; require(enforceConstraints(mesh,rings,&error),"recover rings");
    SurfaceDomainState domain; require(domain.initialize(mesh,rings,&error),"initial domain");
    for (Point2D p:insertions) {
        MeshEditDelta delta;
        require(insertPoint(mesh,p,rings,0,nullptr,nullptr,&error,nullptr,{},&delta,&domain),"guarded insertion");
        require(domain.apply(mesh,delta,&error) && domain.verify(mesh,rings,&error),"local tags equal full BFS after insert/moves");
    }
    {
        Mesh2DTransaction tx(mesh); SurfaceDomainState::Edit labels(domain);
        for (bool tag:{true,false,true}) {
            if (!flip(mesh,domain,tag)) throw std::runtime_error("same-domain flip fixture points="+
                std::to_string(points.size())+" tag="+std::to_string(tag));
            require(labels.consume(tx.pendingDelta(),&error) && domain.verify(mesh,rings,&error),"each staged flip equals BFS");
        }
        require(labels.validate(mesh,tx.pendingDelta().changedSlots,&error) && tx.commit(&error),"paired local validation");
        labels.commit();
    }
    const auto stableTags=domain.values(); const auto stableMesh=mesh;
    try {
        Mesh2DTransaction tx(mesh); SurfaceDomainState::Edit labels(domain);
        require(flip(mesh,domain,true) && labels.consume(tx.pendingDelta(),&error),"cancelled flip fixture");
        throw std::runtime_error("cancel fixture");
    } catch (const std::runtime_error& e) { require(std::string(e.what())=="cancel fixture","cancel setup succeeded"); }
    require(domain.values()==stableTags && domain.verify(mesh,rings,&error) && mesh.tris.size()==stableMesh.tris.size(),"paired cancellation rollback");
    MeshEditDelta rejected; int vertex=99,triangle=99;
    const auto edge=rings.front(); const auto a=mesh.points[edge.first],b=mesh.points[edge.second];
    require(!insertPoint(mesh,{(a.x+b.x)/2,(a.y+b.y)/2},rings,0,&vertex,&triangle,&error,nullptr,{},&rejected,&domain) &&
        vertex==-1 && triangle==-1 && domain.values()==stableTags,"boundary insert rejected without tag mutation");
    bool different=false;
    for (int t=0;t<int(mesh.tris.size());++t) for (int n:mesh.tris[t].n) if (n>=0 && domain.values()[t]!=domain.values()[n]) {
        different=true; require(!domain.uniform({t,n},&error),"cross-domain edit rejected");
    }
    require(different,"fixture has both domain tags");
    const auto before=domain.values();
    // 即使调用方给出不满足继承前提的报告，标签也不留下半次更新。
    int inside=-1,outside=-1;
    for (int t=0;t<int(before.size());++t) (before[t]?inside:outside)=t;
    MeshEditDelta invalid; invalid.initialTriangles=mesh.tris.size(); invalid.initialPoints=mesh.points.size();
    invalid.events={{MeshEditEvent::Kind::Removed,inside,int(mesh.tris.size())-1},
                    {MeshEditEvent::Kind::Removed,outside,int(mesh.tris.size())-2}};
    require(!domain.apply(mesh,invalid,&error) && domain.values()==before,"invalid delta tag rollback");
}
}
int main() { try {
    scenario({{0,0},{6,0},{6,4},{0,4},{1,1},{2,1},{2,3},{1,3},{4,1},{5,1},{5,3},{4,3}},
        {{3,4},{4,5},{5,6},{6,3},{7,8},{8,9},{9,10},{10,7},{11,12},{12,13},{13,14},{14,11}},
        {{0.5,1.5},{2.8,2.3},{1.4,2.1},{4.3,1.7},{5.5,3.4}});
    scenario({{0,0},{2,0},{2,1},{1,1},{1,2},{0,2}},
        {{3,4},{4,5},{5,6},{6,7},{7,8},{8,3}},{{0.3,0.6},{1.4,1.3},{1.2,1.6},{0.7,1.7}});
    std::cout<<"PASS local domain inheritance, holes, concavity, exterior and failure rollback\n";
    return 0;
} catch (const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<'\n'; return 1; } }
