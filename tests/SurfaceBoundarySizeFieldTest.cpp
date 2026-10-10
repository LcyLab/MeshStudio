#include "mesh/surface/legacy/SurfaceBoundarySizeField.h"
#include <iostream>
using namespace mesher;
int main() {
    std::vector<Point3D> points{{0,0,0},{.025,0,0},{0,.1,0},{0,1,0},{0,.1,.01},{.02,.1,.01},{1,1,0}};
    std::vector<std::array<int,3>> tris{{0,1,2},{1,2,3},{4,5,6}};
    SurfaceBoundarySizeField field;
    std::string error;
    bool ok=field.initialize(points,tris,{{0,1}},1,.01,1.3,&error) &&
        field.at(2)<.08 && field.at(3)>field.at(2) && field.at(4)==1;
    const auto value=field.candidateSize({0,1,2});ok=ok && value<.08 && value>=.01;
    // A quality subdivision must not become a new, smaller geometric source.
    const std::vector<std::pair<int,double>> anchors{{0,.025},{1,.025}};
    ok=ok && field.initialize(points,tris,{{0,1}},1,.01,1.3,&error,anchors);
    const double oldFar=field.at(3);
    points.push_back({.0125,0,0});
    tris={{0,7,2},{7,1,2},{1,2,3},{4,5,6}};
    ok=ok && field.initialize(points,tris,{{0,7},{7,1}},1,.01,1.3,&error,anchors)
        && field.at(0)==.025 && field.at(1)==.025 && field.at(3)==oldFar;
    // New adjacency can shorten a valid domain path; restore grading locally.
    ok=ok && field.append({{0,.05},{3,.05}},&error) && field.at(3)<=field.at(8)+.3*.05+1e-12;
    const double before=field.at(3);
    ok=ok && !field.append({{99,.1}},&error) && field.at(3)==before;
    // 密度场保持常规 min；另一标量场仅在固定几何源附近降低允许下限。
    SurfaceBoundarySizeField regular,floors;
    const std::vector<Point3D> gradedPoints{{0,0,0},{.07,0,0},{0,.1,0},{0,2,0},{4,0,0},{4.1,0,0},{4,1,0}};
    const std::vector<std::array<int,3>> gradedTriangles{{0,1,2},{1,2,3},{4,5,6}};
    ok=ok && regular.initialize(gradedPoints,gradedTriangles,{{0,1}},1,.39,1.3,&error,{{0,.07},{1,.07}})
        && floors.initialize(gradedPoints,gradedTriangles,{{0,1}},.39,.035,1.3,&error,{{0,.035},{1,.035}})
        && regular.at(0)==.39 && floors.at(0)==.035 && floors.at(2)<.07 && floors.at(3)==.39 && floors.at(4)==.39;
    ok=ok && floors.append({{0,.02},{2,.08}},&error) && floors.at(7)>=.035 && floors.at(0)==.035;
    std::cout<<(ok?"PASS":"FAIL")<<" boundary size distance grading and disconnected-domain barrier "<<error<<'\n';return ok?0:1;
}
