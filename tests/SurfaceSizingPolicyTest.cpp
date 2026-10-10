#include "mesh/surface/legacy/SurfaceSizingPolicy.h"
#include <cmath>
#include <iostream>
using namespace mesher;
int main() {
    const std::vector<Point3D> points{{0,0,0},{.07,0,0},{0,.1,0},{0,2,0},{4,0,0},{4.1,0,0},{4,1,0}};
    const std::vector<std::array<int,3>> tris{{0,1,2},{1,2,3},{4,5,6}};
    SurfaceCriteria c;c.targetSize=1;c.minSize=.39;
    SurfaceSizingPolicy policy;std::string error;
    bool ok=policy.initialize(points,tris,{{0,1}},{{points[0],.07,8},{points[1],.07,8}},c,&error);
    ok=ok && std::abs(policy.target(0)-.07)<1e-12 && std::abs(policy.floor(0)-.035)<1e-12 &&
        std::abs(policy.floor(2)-.05)<1e-12 &&
        policy.target(2)<.39 && policy.target(3)>policy.target(2) && policy.target(4)==1 && policy.floor(4)==.39;
    const double first=policy.target(0);
    ok=ok && policy.append({{0,.02},{2,.08}},&error) && policy.target(7)>=first && policy.floor(7)>=.035 &&
        policy.target(0)==first;
    const double last=policy.target(7);ok=ok && !policy.append({{99,.1}},&error) && policy.target(7)==last;
    // 略短于常规 min 的首次必要采样同样需要局部余量，不能只支持极小段。
    ok=ok && policy.initialize(points,tris,{{0,1}},{{points[0],.34,8},{points[1],.34,8}},c,&error) &&
        std::abs(policy.floor(0)-.17)<1e-12 && std::abs(policy.target(0)-.34)<1e-12;
    // 平行非相邻边之间的有效域内边给出窄带宽度；共享角点不是宽度证据。
    const std::vector<Point3D> strip{{0,0,0},{0,1,0},{.1,0,0},{.1,1,0}};
    c.minSize=.02;
    ok=ok && policy.initialize(strip,{{0,2,1},{1,2,3}},{{0,1},{2,3}}, {},c,&error) &&
        policy.target(0)<=.1+1e-12 && policy.target(3)<=.1+1e-12 && policy.floor(0)==.02;
    // 几何靠近但图表不相连的两块域不互相加密。
    ok=ok && policy.initialize(points,tris,{{0,1},{4,5}},{{points[0],.07,8},{points[1],.07,8}},c,&error) &&
        policy.target(4)==1;
    std::cout<<(ok?"PASS":"FAIL")<<" coordinated immutable-source sizing and narrow-domain witnesses "<<error<<'\n';
    return ok?0:1;
}
