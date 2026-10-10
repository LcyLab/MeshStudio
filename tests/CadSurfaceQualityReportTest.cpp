#include "CadSurfaceQualityReport.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

int main() {
    using namespace cadquality;
    const auto equilateral=measure({0,0,0},{1,0,0},{.5,std::sqrt(3.)/2,0});
    const auto needle=measure({0,0,0},{1,0,0},{.5,.01,0});
    const auto flat=measure({0,0,0},{1,0,0},{2,0,0});
    bool ok=equilateral.valid && std::abs(equilateral.radiusEdge-1/std::sqrt(3.))<1e-12 &&
        std::abs(equilateral.minAngleDeg-60)<1e-10 && needle.valid &&
        std::abs(needle.minAngleDeg-std::atan2(.01,.5)*180/std::acos(-1.))<1e-10 &&
        needle.radiusEdge>25 && !flat.valid;
    ok=ok && std::abs(equilateral.shape-1)<1e-12 && needle.shape<.04;
    ok=ok&&std::abs(equilateral.maxAngleDeg-60)<1e-10&&
        std::abs(needle.maxAngleDeg-(180-2*needle.minAngleDeg))<1e-10;
    const auto scaled=measure({0,0,0},{1000,0,0},{500,10,0});
    ok=ok && std::abs(scaled.radiusEdge-needle.radiusEdge)<1e-12;
    mesher::SurfaceMeshResult r;
    r.mesh.vertices={{0,0,0},{1,0,0},{.5,.01,0},{.5,std::sqrt(3.)/2,0}};
    r.mesh.triangles={{0,1,2},{0,1,3}};
    r.triangleFaceIds={5,6};r.triangleBodyIds={1,2};r.boundaryEdges={{0,1,8}};
    const auto report=analyze(r,1.4142135623730951,.02);
    ok=ok && report.total==2 && report.bad==1 && report.byFace.at(5).bad==1 &&
        report.residuals.size()==1 && report.residuals[0].cadEdge==8 && report.residuals[0].body==1;
    ok=ok && report.p5Angle<=report.p50Angle && report.p5Shape<=report.p50Shape &&
        report.below30==1 && report.below25==1 && report.byFace.at(6).p5Angle>59;
    ok=ok&&report.nearEquilateral==1&&report.byFace.at(6).nearEquilateral==1&&report.byFace.at(5).nearEquilateral==0;
    std::ostringstream csv;writeCsv(csv,report,r);
    const auto csvText=csv.str();
    ok=ok && csvText.find("min_size,quality_target,u0,v0")!=std::string::npos &&
        std::count(csvText.begin(),csvText.end(),'\n')==2;
    r.triangleFaceIds.pop_back();
    bool rejected=false;try { analyze(r,1.4,.02); }catch(const std::exception&){rejected=true;}
    ok=ok && rejected;
    std::cout<<(ok?"PASS":"FAIL")<<" independent physical quality and residual coverage\n";
    return ok?0:1;
}
