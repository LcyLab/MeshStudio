#include "mesh/volume/Constrained3D.h"
#include "mesh/volume/Delaunay3D.h"
#include "mesh/volume/VolumeDomain3D.h"
#include "mesh/volume/Flips3D.h"
#include "geometry/predicates/GeomPredicates.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

// 先用旧入口复现：闭合凹形 L 棱柱不能按顶点凸包填充。
int main(int argc,char**)
{
    using namespace mesher;
    std::vector<Point3D> points;
    const double xy[6][2]={{0,0},{2,0},{2,1},{1,1},{1,2},{0,2}};
    for(int z=0;z<2;++z) for(auto& p:xy) points.push_back({p[0],p[1],double(z)});
    std::vector<std::array<int,3>> faces;
    for(auto tri:std::vector<std::array<int,3>>{{0,1,3},{1,2,3},{0,3,5},{3,4,5}}) {
        faces.push_back({tri[0]+4,tri[2]+4,tri[1]+4});
        faces.push_back({tri[0]+10,tri[1]+10,tri[2]+10});
    }
    for(int i=0;i<6;++i) {
        int j=(i+1)%6;
        faces.push_back({i+4,j+4,j+10}); faces.push_back({i+4,j+10,i+10});
    }
    TetMesh3D mesh;
    auto d=delaunay3D(mesh,points);
    std::string error;
    auto r=enforceSurface3D(mesh,faces,&error);
    if(argc==1) {
        auto recovered=recoverVolumeBoundary(mesh,faces);
        auto domain=extractVolumeDomain(mesh,faces);
        error=recovered.error+domain.error;
        r.ok=recovered.ok && domain.ok;
    }
    bool ok=d.ok && r.ok && std::abs(mesh.totalVolume()-3.0)<1e-10;
    std::cout<<(ok?"PASS":"FAIL")<<" concave prism volume: "<<mesh.totalVolume()
             <<" (expected 3), "<<error<<'\n';
    if(argc>1) return ok?0:1;
    int failures=ok?0:1;
    auto check=[&](bool valid,const char* name) {std::cout<<(valid?"PASS ":"FAIL ")<<name<<'\n';if(!valid) ++failures;};
    // 三片扇叶围绕穿越目标面的中央边，面恢复必须构造真实 c-d-e 面。
    TetMesh3D fan;fan.points={{0,0,-1},{0,0,1},{1,0,0},{-0.5,1,0},{-0.5,-1,0}};
    auto add=[&](TetMesh3D& m,int a,int b,int c,int d) {
        if(orient3d(m.points[a],m.points[b],m.points[c],m.points[d])<0) std::swap(c,d);
        m.addTet(a,b,c,d);
    };
    add(fan,0,1,2,3);add(fan,0,1,3,4);add(fan,0,1,4,2);
    const double fanVolume=fan.totalVolume();
    TetMesh3D flip=fan;
    check(flip32(flip,0,1,2,&error) && flip.hasFaceAny(2,3,4) && flip.validate(&error) &&
          std::abs(flip.totalVolume()-fanVolume)<1e-12,"3-to-2 preserves fan volume and leaf face");
    TetMesh3D reordered;reordered.points=fan.points;
    add(reordered,0,1,2,3);add(reordered,2,4,1,0);add(reordered,3,4,0,1);
    check(reordered.validate(&error) && flip32(reordered,0,1,2,&error) &&
          reordered.validate(&error) && reordered.hasFaceAny(2,3,4),"3-to-2 independent of tetrahedron vertex order");
    std::vector<std::array<int,3>> pyramid={{2,3,4},{0,3,2},{0,4,3},{0,2,4}};
    const auto missing=measureVolumeBoundaryCoverage(fan,pyramid);
    check(!missing.ok && !missing.missingFaces.empty(),"edge existence does not certify missing face");
    auto recovery=recoverVolumeBoundary(fan,pyramid);
    auto carved=extractVolumeDomain(fan,pyramid);
    check(recovery.ok && recovery.recoveredFlips==1 && carved.ok &&
          std::abs(fan.totalVolume()-fanVolume/2)<1e-12,"face recovered before half-pyramid parity extraction");
    TetMesh3D bipyramid;bipyramid.points={{1,0,0},{-0.5,1,0},{-0.5,-1,0},{0,0,-1},{0,0,1}};
    add(bipyramid,3,0,1,2);add(bipyramid,4,0,1,2);
    const std::vector<std::array<int,3>> wedge={{3,4,0},{3,1,4},{3,0,1},{4,1,0}};
    const auto wedgeRecovery=recoverVolumeBoundary(bipyramid,wedge);
    const auto wedgeDomain=extractVolumeDomain(bipyramid,wedge);
    check(wedgeRecovery.ok && wedgeRecovery.recoveredFlips==1 && wedgeDomain.ok &&
          std::abs(bipyramid.totalVolume()-1.0/3)<1e-12,"safe 2-to-3 recovers two missing wedge faces before domain extraction");

    // 构造一致的 Freudenthal 网格，独立验证空腔/不连通实体的域分类。
    auto grid=[&](int nx,int ny,int nz) {
        TetMesh3D m;
        for(int z=0;z<=nz;++z) for(int y=0;y<=ny;++y) for(int x=0;x<=nx;++x) m.addPoint({double(x),double(y),double(z)});
        auto id=[&](int x,int y,int z) {return (z*(ny+1)+y)*(nx+1)+x;};
        for(int z=0;z<nz;++z) for(int y=0;y<ny;++y) for(int x=0;x<nx;++x) {
            std::array<int,3> p={0,1,2};
            do {
                int a[3]={x,y,z},b[3]={x,y,z};++a[p[0]];++b[p[0]];++b[p[1]];
                add(m,id(x,y,z),id(a[0],a[1],a[2]),id(b[0],b[1],b[2]),id(x+1,y+1,z+1));
            } while(std::next_permutation(p.begin(),p.end()));
        }
        return m;
    };
    auto boundary=[](const TetMesh3D& m) {
        std::vector<std::array<int,3>> f;
        for(auto& t:m.tets) for(int e=0;e<4;++e) if(t.n[e]<0) f.push_back({t.v[(e+1)%4],t.v[(e+2)%4],t.v[(e+3)%4]});
        return f;
    };
    auto hollow=grid(3,3,3),shell=hollow;
    for(int t=int(shell.tets.size())-1;t>=0;--t) {
        Point3D c{};for(int v:shell.tets[t].v) {c.x+=shell.points[v].x/4;c.y+=shell.points[v].y/4;c.z+=shell.points[v].z/4;}
        if(c.x>1 && c.x<2 && c.y>1 && c.y<2 && c.z>1 && c.z<2) shell.removeTet(t);
    }
    auto shellFaces=boundary(shell);auto h=extractVolumeDomain(hollow,shellFaces);
    check(h.ok && std::abs(hollow.totalVolume()-26)<1e-10,"nested shell cavity remains empty");
    auto disconnected=grid(3,1,1),parts=disconnected;
    for(int t=int(parts.tets.size())-1;t>=0;--t) {
        double x=0;for(int v:parts.tets[t].v) x+=parts.points[v].x/4;
        if(x>1 && x<2) parts.removeTet(t);
    }
    auto partFaces=boundary(parts);auto p=extractVolumeDomain(disconnected,partFaces);
    check(p.ok && std::abs(disconnected.totalVolume()-2)<1e-10,"disconnected closed volumes preserve separating exterior");
    auto invalid=grid(1,1,1);const double original=invalid.totalVolume();
    auto bad=boundary(invalid);bad.pop_back();auto rejected=extractVolumeDomain(invalid,bad);
    check(!rejected.ok && invalid.totalVolume()==original,"invalid barrier failure is atomic");
    TetMesh3D folded;folded.points={{0,0,0},{1,0,0},{0,1,0},{0,0,1},{1,1,1}};
    const std::vector<std::array<int,3>> foldedFaces={{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
    for(auto f:foldedFaces) add(folded,4,f[0],f[1],f[2]);
    check(!extractVolumeDomain(folded,foldedFaces).ok,"folded overlapping fan is rejected despite positive cell volumes");
    TetMesh3D translated;translated.points={{1e6,1e6,1e6},{1e6+1,1e6,1e6},{1e6,1e6+1,1e6},{1e6,1e6,1e6+0.01}};
    add(translated,0,1,2,3);
    check(translated.validate(&error),"positive translated thin tetrahedron is not false degenerate");
    return failures?1:0;
}
