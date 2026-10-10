#include "mesh/surface/legacy/SurfaceRefinementQueue.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <stdexcept>
using namespace mesher;
namespace {
void require(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
Mesh2D square() {
    Mesh2D mesh; for (auto p:std::vector<Point2D>{{0,0},{1,0},{1,1},{0,1}}) mesh.addPoint(p);
    mesh.addTri(0,1,2); mesh.addTri(0,2,3); return mesh;
}
SurfaceTriangleTicket ticket(const Mesh2D& mesh,int slot,int priority,double score,uint64_t criteria=1) {
    const auto tri=mesh.tris[slot]; std::array<int,3> vertices{tri.v[0],tri.v[1],tri.v[2]}; std::sort(vertices.begin(),vertices.end());
    return {7,slot,mesh.triangleVersion(slot),criteria,priority,score,vertices};
}
}
int main() { try {
    auto mesh=square(); std::string error; SurfaceTriangleTicket next;
    SurfaceRefinementQueue queue(7,1);
    require(queue.offer(ticket(mesh,1,1,8),&error) && queue.offer(ticket(mesh,0,2,1.1),&error),"offer");
    require(queue.pop(mesh,next) && next.slot==0,"hard geometry before quality");
    require(queue.pop(mesh,next) && next.slot==1 && !queue.pop(mesh,next),"pop remaining");
    require(queue.offer(ticket(mesh,0,2,1.1),&error) && queue.offer(ticket(mesh,1,2,1.9),&error),"same hard priority");
    require(queue.pop(mesh,next) && next.slot==1 && queue.pop(mesh,next) && next.slot==0,"hard/inversion class uses violation ratio");
    // 同分按稳定顶点键；插入次序不影响决胜。
    require(queue.offer(ticket(mesh,1,2,1.5),&error) && queue.offer(ticket(mesh,0,2,1.5),&error),"tie offer");
    require(queue.pop(mesh,next) && next.slot==0,"stable vertex tie");
    queue.resetCriteria(2); require(!queue.pop(mesh,next),"criteria change invalidates old tickets");
    require(queue.offer(ticket(mesh,0,2,2,2),&error),"new criteria ticket");
    MeshEditDelta delta;
    { Mesh2DTransaction tx(mesh); mesh.removeTri(0); require(tx.commit(&error,&delta),"move transaction"); }
    queue.invalidate(delta);
    require(!queue.pop(mesh,next),"old slot version cannot target moved triangle");
    require(queue.offer(ticket(mesh,0,2,2,2),&error) && queue.pop(mesh,next) && next.vertexKey==std::array<int,3>{0,2,3},"moved bad triangle requeued");
    // 局部邻接版本变化使旧票失效，即使调用方漏了失效通知。
    require(queue.offer(ticket(mesh,0,2,2,2),&error),"stale offer");
    { Mesh2DTransaction tx(mesh); mesh.removeTri(0); mesh.addTri(0,2,3); require(tx.commit(&error),"slot reuse"); }
    require(!queue.pop(mesh,next),"version checked during lazy pop");
    for (int i=0;i<600;++i) {
        require(queue.offer(ticket(mesh,0,2,2+i,2),&error),"replace active entry");
    }
    require(queue.activeSize()==1 && queue.heapSize()<=131 && queue.compactions()>0,"heap history bounded");
    auto bad=ticket(mesh,0,2,std::numeric_limits<double>::infinity(),2);
    require(!queue.offer(bad,&error),"non-finite score rejected");
    bad=ticket(mesh,0,2,2,1); require(!queue.offer(bad,&error),"old criteria offer rejected");
    std::cout<<"PASS queue priorities, stable ties, criteria/slot versions, moves and compaction\n";
    return 0;
} catch (const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<'\n'; return 1; } }
