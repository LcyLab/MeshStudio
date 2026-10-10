#include "mesh/surface/algorithm6/initial/DivideAndConquerTriangulation.h"
#include "geometry/predicates/GeomPredicates.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok,const char* name) { if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    DivideAndConquerTriangulation output;
    auto status = buildDivideAndConquerTriangulation({{20,{1.,0.}},{21,{0.,0.}}},output);
    check(status.complete() && output.sortedPoints.size() == 2 && output.sortedPoints[0].localId == 20 && output.triangles.empty(),"less than three retains unsorted record");
    const std::vector<InitialPoint> triangle{{7,{2.,0.}},{8,{0.,0.}},{9,{1.,1.}}};
    status = buildDivideAndConquerTriangulation(triangle,output);
    check(status.complete() && output.hullPointCount == 3 && output.triangles == std::vector<std::array<int,3>>{{0,1,2}} &&
          output.sortedPoints[0].localId == 8 && output.sortedPoints[1].localId == 9,"native sorted clockwise triple and output order");
    const std::vector<InitialPoint> square{{0,{0.,0.}},{1,{1.,0.}},{2,{1.,1.}},{3,{0.,1.}}};
    status = buildDivideAndConquerTriangulation(square,output);
    check(status.complete() && output.triangles.size() == 2 && output.hullPointCount == 4,"exact cocircular square has two triangles");
    std::vector<InitialPoint> points{{-1,{-3.,-3.}},{-2,{-3.,3.}},{-3,{3.,-3.}},{-4,{3.,3.}}};
    for(int i = 0; i < 19; ++i) points.push_back({i,{std::sin(double(i)*2.345),std::cos(double(i)*1.234)}});
    status = buildDivideAndConquerTriangulation(points,output);
    bool valid = status.complete() && output.hullPointCount == 4 && output.triangles.size() == 2*points.size()-6;
    std::map<std::pair<int,int>,int> edges;
    if(valid) for(const auto& t : output.triangles) {
        const auto a = output.sortedPoints[t[0]].coordinates, b = output.sortedPoints[t[1]].coordinates, c = output.sortedPoints[t[2]].coordinates;
        valid &= orient2d(a,b,c) < 0;
        for(int k = 0; k < int(output.sortedPoints.size()); ++k)
            if(k != t[0] && k != t[1] && k != t[2]) valid &= incircle(a,b,c,output.sortedPoints[k].coordinates) <= 0;
        for(int j = 0; j < 3; ++j) ++edges[std::minmax(t[j],t[(j+1)%3])];
    }
    for(const auto& e : edges) valid &= e.second == 1 || e.second == 2;
    check(valid,"all positions form clockwise planar Delaunay before constraints");
    const auto priorCount = output.sortedPoints.size();
    auto duplicate = points; duplicate.push_back(points[5]);
    status = buildDivideAndConquerTriangulation(duplicate,output);
    check(status.code == MeshingStatus::InvalidInput && output.sortedPoints.size() == priorCount,"duplicate coordinates explicitly fail atomically");
    auto nonfinite = points; nonfinite[0].coordinates.x = std::numeric_limits<double>::infinity();
    status = buildDivideAndConquerTriangulation(nonfinite,output);
    check(status.code == MeshingStatus::InvalidInput && output.sortedPoints.size() == priorCount,"nonfinite input fails atomically");
    RunControl control; control.maximumNodes = points.size()-1;
    status = buildDivideAndConquerTriangulation(points,output,control);
    check(status.code == MeshingStatus::BudgetExceeded && output.sortedPoints.size() == priorCount,"position budget is not convergence");
    control = {}; int calls = 0; control.keepGoing = [&] { return ++calls < 15; };
    status = buildDivideAndConquerTriangulation(points,output,control);
    check(status.code == MeshingStatus::Cancelled && output.sortedPoints.size() == priorCount,"mid-merge cancellation keeps previous result");
    std::cout << "divide/conquer failures=" << failures << '\n'; return failures ? 1 : 0;
}
