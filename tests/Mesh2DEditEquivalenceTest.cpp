#include "mesh/planar/Delaunay2D.h"
#include "mesh/planar/Constrained2D.h"
#include "mesh/data/Mesh2DTransaction.h"
#include "Mesh2DReference.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace mesher;
using namespace mesher::testing;
namespace {
bool same(const Mesh2D& a,const Mesh2D& b) {
    if (a.points.size()!=b.points.size() || a.tris.size()!=b.tris.size()) return false;
    for (size_t i=0;i<a.points.size();++i) if (a.points[i].x!=b.points[i].x || a.points[i].y!=b.points[i].y) return false;
    for (size_t i=0;i<a.tris.size();++i) for (int k=0;k<3;++k) {
        if (a.tris[i].v[k]!=b.tris[i].v[k] || a.tris[i].n[k]!=b.tris[i].n[k]) return false;
        if (a.findHalfedge(a.tris[i].v[k],a.tris[i].v[(k+1)%3])!=b.findHalfedge(b.tris[i].v[k],b.tris[i].v[(k+1)%3])) return false;
    }
    return a.validate() && b.validate();
}
void require(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
struct Counts : MeshingObserver {
    uint64_t copies=0,logged=0;
    void stageBegin(MeshStage,MeshEntityRef) noexcept override {}
    void stageEnd(MeshStage,MeshEntityRef) noexcept override {}
    void addCount(MeshCounter c,MeshEntityRef,uint64_t n) noexcept override {
        if (c==MeshCounter::WholeMeshCopies) copies+=n;
        if (c==MeshCounter::TransactionTrianglesLogged) logged+=n;
    }
};
}
int main() { try {
    Mesh2D mesh;
    require(delaunay2D(mesh,{{0,0},{8,0},{8,6},{0,6},{2,2},{3,2},{3,4},{2,4},{6,0},{6,6}},true).ok,"initial triangulation");
    auto reference=mesh;
    std::vector<std::pair<int,int>> boundary{{3,4},{4,5},{5,6},{6,3},{7,8},{8,9},{9,10},{10,7},{11,12}};
    std::string a,b; MeshEditDelta delta;
    require(enforceConstraints(mesh,boundary,&a,{},&delta)==enforceConstraintsReference(reference,boundary,&b),"recovery result equivalence");
    require(a==b && same(mesh,reference),"recovery exact equivalence");
    const LinearMetric2D metric{2,0.7,0.5};
    for (Point2D p:std::vector<Point2D>{{1,1},{4.5,3},{7,2},{2.5,3},{9,3},{0,0},{0,3},{1.25,0.8},{std::numeric_limits<double>::infinity(),1}}) {
        int av=99,at=99,bv=99,bt=99; a.clear(); b.clear(); const auto before=mesh;
        const bool ok=insertPoint(mesh,p,boundary,0,&av,&at,&a,&metric,{},&delta);
        const bool ref=insertPointReference(reference,p,boundary,0,&bv,&bt,&b,&metric);
        require(ok==ref && av==bv && at==bt && a==b && same(mesh,reference),"point edit exact equivalence");
        if (!ok) {
            require(same(mesh,before),"point edit rollback");
            for (size_t t=0;t<mesh.tris.size();++t) require(mesh.triangleVersion(int(t))==before.triangleVersion(int(t)),"rollback versions");
            require(mesh.exactPoint(p)==before.exactPoint(p),"rollback point index");
        }
    }
    Mesh2D crossed;
    require(delaunay2D(crossed,{{0,0},{2,0},{2,2},{0,2}},true).ok,"crossing input");
    auto copy=crossed; const auto before=crossed;
    const auto existing=crossed.hasEdgeAny(3,5)?std::pair<int,int>{3,5}:std::pair<int,int>{4,6};
    const auto other=existing.first==3?std::pair<int,int>{4,6}:std::pair<int,int>{3,5}; a.clear(); b.clear();
    require(!enforceConstraints(crossed,{other,existing},&a) && !enforceConstraintsReference(copy,{other,existing},&b),"batch crossing rejection");
    require(a==b && same(crossed,copy) && same(crossed,before),"batch rollback equivalence");
    // 一个大网格中的小 cavity 应只记账局部区域。
    Mesh2D large; std::vector<Point2D> grid;
    for (int y=0;y<24;++y) for (int x=0;x<24;++x) grid.push_back({double(x),double(y)});
    require(delaunay2D(large,grid,true).ok,"large input");
    Counts counts;
    require(insertPoint(large,{11.3,12.4},{},0,nullptr,nullptr,&a,nullptr,{&counts}),"large local insert");
    require(large.validate() && counts.copies==0 && counts.logged<large.tris.size()/4,"local journal bounded by edit");
    // 默认二维路径还原坐标后，精确索引必须按最终坐标重新建立。
    Mesh2D normalized; require(delaunay2D(normalized,{{20,30},{28,30},{28,36},{20,36}}).ok,"normalized input");
    for (int v=0;v<int(normalized.points.size());++v) require(normalized.exactPoint(normalized.points[v])==v,"restored point index");
    std::cout<<"PASS local/copy edit equivalence, failure atomicity and bounded journal\n";
    return 0;
} catch (const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<'\n'; return 1; } }
