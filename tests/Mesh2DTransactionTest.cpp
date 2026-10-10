#include "mesh/data/Mesh2DTransaction.h"
#include "mesh/planar/Delaunay2D.h"
#include <iostream>
#include <limits>
using namespace mesher;
namespace mesher {
struct Mesh2DTestAccess { static void forceVisitWrap(Mesh2D& m) { m.m_visitEpoch=std::numeric_limits<std::uint64_t>::max(); } };
}
namespace {
Mesh2D square() {
    Mesh2D m; for (auto p:std::vector<Point2D>{{0,0},{1,0},{1,1},{0,1}}) m.addPoint(p);
    m.addTri(0,1,2); m.addTri(0,2,3); return m;
}
bool same(const Mesh2D& a,const Mesh2D& b) {
    if (a.points.size()!=b.points.size() || a.tris.size()!=b.tris.size()) return false;
    for (size_t i=0;i<a.points.size();++i) if (a.points[i].x!=b.points[i].x || a.points[i].y!=b.points[i].y) return false;
    for (size_t i=0;i<a.tris.size();++i) {
        if (a.triangleVersion(int(i))!=b.triangleVersion(int(i))) return false;
        for (int k=0;k<3;++k) if (a.tris[i].v[k]!=b.tris[i].v[k] || a.tris[i].n[k]!=b.tris[i].n[k]) return false;
    }
    for (int x=0;x<int(a.points.size());++x) for (int y=0;y<int(a.points.size());++y)
        if (a.findHalfedge(x,y)!=b.findHalfedge(x,y)) return false;
    return a.validate() && b.validate();
}
}
int main() {
    {
        Mesh2D mesh;const auto initial=delaunay2D(mesh,{{0,0},{2,0},{2,2},{0,2}},true);
        const auto before=mesh;int vertex=-1,tri=-1;std::string error;bool visited=false;
        const bool accepted=insertPoint(mesh,{.5,.5},{},0,&vertex,&tri,&error,nullptr,{},nullptr,nullptr,
            [&](const Mesh2D&,int,const MeshEditDelta&){visited=true;return false;});
        if(!initial.ok || accepted || !visited || vertex!=-1 || mesh.points.size()!=before.points.size() ||
            !same(mesh,before) || mesh.exactPoint({.5,.5})>=0 || !mesh.validate(&error)) {
            std::cerr<<"physical insertion veto did not restore original topology\n";return 1;
        }
        try {
            insertPoint(mesh,{.5,.5},{},0,&vertex,&tri,&error,nullptr,{},nullptr,nullptr,
                [](const Mesh2D&,int,const MeshEditDelta&)->bool{throw std::runtime_error("cancel candidate");});
            std::cerr<<"candidate exception was hidden\n";return 1;
        } catch(const std::runtime_error&) {}
        if(!same(mesh,before) || mesh.exactPoint({.5,.5})>=0)return 1;
        bool circleVisited=false;MeshEditDelta circleDelta;
        const bool invalidCircle=insertPoint(mesh,{.5,.5},{},0,&vertex,&tri,&error,nullptr,{},&circleDelta,nullptr,{},
            [&](const Mesh2D&,const Tri2D&,Point2D){circleVisited=true;return 2;});
        if(invalidCircle||!circleVisited||vertex!=-1||tri!=-1||!circleDelta.changedSlots.empty()||
           error.find("invalid circle predicate")==std::string::npos||!same(mesh,before)) {
            std::cerr<<"invalid physical circle predicate changed mesh or published output\n";return 1;
        }
        try {
            insertPoint(mesh,{.5,.5},{},0,&vertex,&tri,&error,nullptr,{},&circleDelta,nullptr,{},
                [](const Mesh2D&,const Tri2D&,Point2D)->int{throw std::runtime_error("circle cancelled");});
            std::cerr<<"circle predicate exception was hidden\n";return 1;
        } catch(const std::runtime_error&) {}
        if(vertex!=-1||tri!=-1||!circleDelta.changedSlots.empty()||!same(mesh,before)||mesh.exactPoint({.5,.5})>=0)return 1;
    }
    {
        auto m=square();auto before=m;const auto version=m.triangleVersion(0);
        if(!m.relocatePoint(0,{-0.1,0},{0,1}) || m.exactPoint({0,0})>=0 || m.exactPoint({-0.1,0})!=0 ||
           m.triangleVersion(0)==version || !m.validate()) {std::cerr<<"FAIL relocation index/version update\n";return 1;}
        auto moved=m;
        if(m.relocatePoint(0,{1,0},{0,1}) || !same(m,moved)) {std::cerr<<"FAIL duplicate relocation changed mesh\n";return 1;}
        {Mesh2DTransaction tx(m);if(m.relocatePoint(0,{0,0},{0,1})) {std::cerr<<"FAIL relocation in unsupported topology transaction\n";return 1;}}
        if(!same(m,moved))return 1;
    }
    for (int removal:{0,1}) {
        auto m=square(); const auto before=m;
        { Mesh2DTransaction tx(m); m.removeTri(removal); m.addPoint({0.2,0.3}); }
        if (!same(m,before) || m.exactPoint({0.2,0.3})>=0) { std::cerr<<"FAIL deletion/new point rollback\n"; return 1; }
    }
    auto m=square(); const auto before=m;
    {
        Mesh2DTransaction tx(m); m.removeTri(0); m.removeTri(0); const int p=m.addPoint({0.5,0.5});
        m.addTri(0,1,p); m.addTri(1,2,p); m.addTri(2,3,p); m.addTri(3,0,p);
        if(tx.pendingDelta().removedTriangles!=std::vector<std::array<int,3>>{{0,1,2},{0,2,3}}){
            std::cerr<<"FAIL cavity lost original triangles after slot reuse\n";return 1;}
        if (!tx.validate() || !m.validate()) return 1;
    }
    if (!same(m,before)) { std::cerr<<"FAIL repeated moves and additions rollback\n"; return 1; }
    {
        Mesh2DTransaction tx(m); m.addTri(0,1,3);
        if (tx.validate() || tx.commit()) { std::cerr<<"FAIL duplicate directed edge accepted\n"; return 1; }
    }
    if (!same(m,before)) return 1;
    {
        Mesh2DTransaction tx(m); m.removeTri(0);
        if (tx.commit(nullptr,nullptr,{{0,1}})) { std::cerr<<"FAIL missing protected edge accepted\n"; return 1; }
    }
    if (!same(m,before)) return 1;
    // 连续删中间槽位、尾片反复搬移，并在同一槽位新建后抛异常。
    Mesh2D fan;
    for (auto p:std::vector<Point2D>{{0,0},{2,0},{3,1},{2,2},{0,2},{-1,1},{1,1}}) fan.addPoint(p);
    for (int i=0;i<6;++i) fan.addTri(i,(i+1)%6,6);
    const auto fanBefore=fan;
    try {
        Mesh2DTransaction tx(fan); fan.removeTri(0); fan.removeTri(1); fan.removeTri(0);
        fan.addPoint({1.1,1.2}); fan.addTri(0,1,7); fan.removeTri(int(fan.tris.size())-1);
        throw std::runtime_error("cancelled fixture");
    } catch (const std::runtime_error&) {}
    if (!same(fan,fanBefore) || fan.exactPoint({1.1,1.2})>=0) { std::cerr<<"FAIL repeated middle moves exception rollback\n"; return 1; }
    MeshEditDelta delta;
    {
        Mesh2DTransaction tx(m); m.removeTri(0); m.removeTri(0); const int p=m.addPoint({0.5,0.5});
        m.addTri(0,1,p); m.addTri(1,2,p); m.addTri(2,3,p); m.addTri(3,0,p);
        if (!tx.commit(nullptr,&delta,{{0,1},{1,2},{2,3},{3,0}})) return 1;
    }
    if (!m.validate() || delta.moves.size()!=1 || delta.initialTriangles!=2 || delta.createdSlots.size()!=4 ||
        m.triangleVersion(0)<=before.triangleVersion(0) || m.exactPoint({-0.0,0})!=0) return 1;
    const auto epoch=m.beginCavityVisit(); m.markCavity(0,epoch); if (!m.cavityVisited(0,epoch)) return 1;
    const auto next=m.beginCavityVisit(); if (m.cavityVisited(0,next)) return 1;
    Mesh2DTestAccess::forceVisitWrap(m); if (m.beginCavityVisit()!=1 || m.cavityVisited(0,1)) return 1;
    std::cout<<"PASS local transaction rollback, moves, constraints, exact point index and visit wrap\n";
}
