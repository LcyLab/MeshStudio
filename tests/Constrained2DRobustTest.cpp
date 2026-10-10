#include "mesh/planar/Constrained2D.h"
#include "mesh/planar/Delaunay2D.h"
#include "geometry/predicates/GeomPredicates.h"

#include <cmath>
#include <iostream>

using namespace mesher;
namespace {
int failures=0;
void check(bool ok,const char* name) {
    if (!ok) { ++failures; std::cerr<<"FAIL "<<name<<'\n'; }
}
bool same(const Mesh2D& a,const Mesh2D& b) {
    if (a.points.size()!=b.points.size() || a.tris.size()!=b.tris.size()) return false;
    for (size_t i=0;i<a.points.size();++i)
        if (a.points[i].x!=b.points[i].x || a.points[i].y!=b.points[i].y) return false;
    for (size_t i=0;i<a.tris.size();++i) for (int k=0;k<3;++k)
        if (a.tris[i].v[k]!=b.tris[i].v[k] || a.tris[i].n[k]!=b.tris[i].n[k]) return false;
    return a.validate() && b.validate();
}
void chain() {
    Mesh2D mesh;
    check(delaunay2D(mesh,{{0,0},{2,0},{2,2},{0,2},{1,0},{1,1}},true).ok,"chain input");
    std::string error;
    check(enforceConstraints(mesh,{{3,4}},&error),"collinear constraint chain accepted");
    check(mesh.hasEdgeAny(3,7) && mesh.hasEdgeAny(7,4) && mesh.validate(),"chain subsegments retained");
}
void narrowPhoneTrim() {
    // Exact five-node boundary captured before frontal insertion on phone face 818.
    const std::vector<Point2D> points{{1.8476831394100088,1.0709160122657002},
        {-.10043545125949294,1.0709160122656998},{-2.0485539999999958,1.0709160122657},
        {-2.0485539999999958,1.0749999999999993},{1.8476831394059996,1.0749999999999993}};
    Mesh2D mesh;const LinearMetric2D metric{1,0,1};
    check(delaunay2D(mesh,points,true,&metric).ok,"phone narrow initial triangulation");
    std::string error;const bool recovered=enforceConstraints(mesh,{{3,4},{4,5},{5,6},{6,7},{7,3}},&error);
    check(recovered,"phone narrow convex trim must retain full boundary");
    if(!recovered)std::cerr<<error<<" triangles="<<mesh.tris.size()<<'\n';
    check(mesh.hasEdgeAny(6,7),"phone narrow top hull edge remains present");
    for(const auto& tri:mesh.tris)for(int i=3;i<int(mesh.points.size());++i) {
        if(i==tri.v[0]||i==tri.v[1]||i==tri.v[2])continue;
        check(incircleMetric(mesh.points[tri.v[0]],mesh.points[tri.v[1]],mesh.points[tri.v[2]],mesh.points[i],1,0,1)<=0,
            "narrow hull reconstruction retains empty circles");
    }
    for(int offset=0;offset<5;++offset) {
        std::vector<Point2D> order;for(int i=0;i<5;++i)order.push_back(points[(i+offset)%5]);
        order.push_back({0,1.073});Mesh2D reordered;const LinearMetric2D shear{2,.7,.5};
        check(delaunay2D(reordered,order,true,&shear).ok,"narrow hull with interior point and reordered metric input");
        for(const auto& tri:reordered.tris)for(int i=3;i<int(reordered.points.size());++i) {
            if(i==tri.v[0]||i==tri.v[1]||i==tri.v[2])continue;
            check(incircleMetric(reordered.points[tri.v[0]],reordered.points[tri.v[1]],reordered.points[tri.v[2]],reordered.points[i],2,.7,.5)<=0,
                "real hull seed and interior cavity preserve metric empty circles");
        }
        for(size_t i=0;i<order.size();++i)check(reordered.points[i+3].x==order[i].x&&reordered.points[i+3].y==order[i].y,"real hull preserves input index and coordinate");
    }
}
void recoveryRollback() {
    Mesh2D mesh;
    check(delaunay2D(mesh,{{0,0},{2,0},{2,2},{0,2}},true).ok,"crossed input");
    const auto before=mesh;
    std::string error;
    const std::pair<int,int> existing=mesh.hasEdgeAny(3,5)?std::pair<int,int>{3,5}:std::pair<int,int>{4,6};
    const std::pair<int,int> other=existing.first==3?std::pair<int,int>{4,6}:std::pair<int,int>{3,5};
    check(!enforceConstraints(mesh,{other,existing},&error),"crossed constraints rejected");
    check(same(mesh,before),"failed recovery preserves original topology");
}
void insertionRollback() {
    Mesh2D mesh;
    for (Point2D p:std::vector<Point2D>{{0,0},{2,0},{2,2},{0,2}}) mesh.addPoint(p);
    mesh.addTri(0,1,2); mesh.addTri(0,2,3);
    const auto before=mesh;
    std::string error;
    int vertex=99,tri=99;
    check(!insertPoint(mesh,{1,1},{{0,2}},0,&vertex,&tri,&error),"insertion on locked edge rejected");
    check(vertex==-1 && tri==-1 && same(mesh,before),"locked-edge rejection is atomic");
    mesh=before;
    check(!insertPoint(mesh,{0,0},{},0,&vertex,&tri,&error),"duplicate insertion rejected");
    check(vertex==-1 && tri==-1 && same(mesh,before),"duplicate rejection is atomic");
    mesh=before;
    check(insertPoint(mesh,{1,1},{},0,&vertex,&tri,&error) && mesh.validate(),"unlocked interior edge insertion");
    check(std::abs(mesh.totalArea()-4)<1e-12,"interior edge insertion preserves area");
}
void zeroArea() {
    Mesh2D mesh;
    mesh.addPoint({0,0}); mesh.addPoint({1,0}); mesh.addPoint({2,0});
    const int id=mesh.addTri(0,1,2);
    check(id<0 || !mesh.validate(),"zero-area triangle cannot pass validation");
}
void metricPredicates() {
    // 符号由输入 double 的精确有理数行列式独立核对；三个点几乎共线。
    const Point2D a{4.6152349002839541,1.046229259294956};
    const Point2D b{4.6253662549562975,1.0462383604545014};
    const Point2D c{4.6182971725902995,1.0462320101836311};
    const Point2D p{4.6082555226207838,1.0462229896072692};
    for (const auto m:std::vector<LinearMetric2D>{{1,0,1},{0.5,0.25,2},{1,1e8,0.25}}) {
        check(incircleMetric(a,b,c,p,m.xx,m.xy,m.yy)==1,"near-collinear metric circle exact sign");
        check(incircleMetric(c,a,b,p,m.xx,m.xy,m.yy)==1 &&
              incircleMetric(a,c,b,p,m.xx,m.xy,m.yy)==1,"metric circle permutation normalization");
    }
    check(incircleMetric({0,0},{1,0},{1,1},{0,1},1,0,1)==0,"identity metric retains cocircularity");
    check(incircleMetric({0,0},{1,0},{1,1},{0,1},2,1,1)==1,"shear metric changes circle sign");
    const double m=134217729.0;
    check(incircleMetric({5*m,0},{3*m,4*m},{-5*m,0},{-3*m,-4*m},2,0,2)==0,
          "scaled large-integer cocircular expansion");
    Mesh2D mesh; const LinearMetric2D linear{2,0.7,0.5};
    const std::vector<Point2D> points{{0,0},{2,0},{2,2},{0,2},{0.7,0.9}};
    check(delaunay2D(mesh,points,true,&linear).ok,"metric triangulation");
    for (size_t i=0;i<points.size();++i)
        check(mesh.points[i+3].x==points[i].x && mesh.points[i+3].y==points[i].y,"metric keeps input UV unchanged");
    for (const auto& tri:mesh.tris) for (size_t i=3;i<mesh.points.size();++i) {
        if (int(i)==tri.v[0] || int(i)==tri.v[1] || int(i)==tri.v[2]) continue;
        check(incircleMetric(mesh.points[tri.v[0]],mesh.points[tri.v[1]],mesh.points[tri.v[2]],mesh.points[i],
                             linear.xx,linear.xy,linear.yy)<=0,"metric triangulation has empty circles");
    }
    const auto before=mesh; const LinearMetric2D invalid{0,0,1};
    int vertex=99,tri=99; std::string error;
    check(!insertPoint(mesh,{0.4,0.5},{},0,&vertex,&tri,&error,&invalid) &&
          vertex==-1 && tri==-1 && same(mesh,before),"invalid metric insertion is atomic");
    check(insertPoint(mesh,{0.4,0.5},{},0,&vertex,&tri,&error,&linear) && mesh.validate(),"metric insertion succeeds");
    for (size_t i=0;i<before.points.size();++i)
        check(mesh.points[i].x==before.points[i].x && mesh.points[i].y==before.points[i].y,"metric insertion preserves existing UV");
}
void domains() {
    Mesh2D mesh;
    check(delaunay2D(mesh,{{0,0},{6,0},{6,4},{0,4},
        {1,1},{2,1},{2,3},{1,3},{4,1},{5,1},{5,3},{4,3},{3,0},{3,4}},true).ok,"double-hole input");
    std::vector<std::pair<int,int>> boundary;
    for (int ring=0;ring<3;++ring) for (int k=0;k<4;++k)
        boundary.push_back({3+4*ring+k,3+4*ring+(k+1)%4});
    auto constraints=boundary;
    constraints.push_back({15,16}); // Internal feature must not create a hole.
    std::string error;
    check(enforceConstraints(mesh,constraints,&error),"double-hole and internal feature recovered");
    std::vector<bool> inside;
    const bool ok=classifyConstrainedDomain(mesh,boundary,inside,&error);
    check(ok,"double-hole domain classified");
    double area=0;
    if (ok) for (size_t t=0;t<mesh.tris.size();++t) if (inside[t]) {
        const auto tri=mesh.tris[t];
        const auto a=mesh.points[tri.v[0]],b=mesh.points[tri.v[1]],c=mesh.points[tri.v[2]];
        area+=std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/2;
        const double x=(a.x+b.x+c.x)/3,y=(a.y+b.y+c.y)/3;
        check(!(y>1 && y<3 && ((x>1 && x<2)||(x>4 && x<5))),"hole contains no domain triangle");
    }
    check(std::abs(area-20)<1e-12,"double-hole domain area is 20");
    inside.assign(2,true);
    check(!classifyConstrainedDomain(mesh,{{15,16}},inside,&error),"open boundary rejected");
    check(inside.size()==2 && inside[0] && inside[1],"failed classification preserves output");

    Mesh2D concave;
    check(delaunay2D(concave,{{0,0},{2,0},{2,1},{1,1},{1,2},{0,2}},true).ok,"concave input");
    boundary.clear();
    for (int k=0;k<6;++k) boundary.push_back({3+k,3+(k+1)%6});
    check(enforceConstraints(concave,boundary,&error),"concave boundary recovered");
    check(classifyConstrainedDomain(concave,boundary,inside,&error),"concave domain classified");
    area=0;
    if (inside.size()==concave.tris.size()) for (size_t t=0;t<concave.tris.size();++t) if (inside[t]) {
        const auto tri=concave.tris[t];
        const auto a=concave.points[tri.v[0]],b=concave.points[tri.v[1]],c=concave.points[tri.v[2]];
        area+=std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/2;
    }
    check(std::abs(area-3)<1e-12,"concave domain area is 3");
}
}
int main() {
    chain(); narrowPhoneTrim(); recoveryRollback(); insertionRollback(); zeroArea(); metricPredicates(); domains();
    std::cout<<"failures="<<failures<<'\n';
    return failures?1:0;
}
