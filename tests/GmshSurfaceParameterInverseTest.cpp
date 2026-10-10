#include "mesh/surface/algorithm6/geometry/SurfaceParameterInverse.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main() {
    int failures=0;
    auto check=[&](bool ok,const char* name) { if(!ok) { ++failures;std::cerr << "FAIL " << name << '\n'; } };
    auto close=[](double x,double y) { return std::abs(x-y)<1.e-12; };
    SurfaceParameterInverseSource source;
    source.bounds={-1.,1.,-1.,1.};
    source.point=[](Point2D uv) { return Point3D{uv.x,uv.y,0.}; };
    source.derivatives=[](Point2D) { return std::array<Point3D,2>{{{1.,0.,0.},{0.,1.,0.}}}; };
    auto r=invertSurfaceParameters({0.,0.,0.},source);
    check(r.status.complete() && r.converged && r.stop==SurfaceParameterInverseStop::InitialPoint && r.geometryEvaluations==1 && r.seeds==1,"initial center uses only one point query");
    r=invertSurfaceParameters({.25,.4,0.},source);
    check(r.status.complete() && r.converged && r.stop==SurfaceParameterInverseStop::ParameterStep && close(r.parameters.x,.25) && close(r.parameters.y,.4) && r.steps==2 && r.geometryEvaluations==5,"plane inverse retains zero second Newton step");
    r=invertSurfaceParameters({.25,.4,1.},source);
    check(r.status.complete() && r.converged && close(r.parameters.x,.25) && close(r.parameters.y,.4),"native warning-only XYZ mismatch accepts parameter convergence");
    SurfaceParameterInverseOptions options;options.convergenceTestXYZ=true;
    r=invertSurfaceParameters({.25,.4,1.},source,options);
    check(r.status.complete() && !r.converged && r.stop==SurfaceParameterInverseStop::RelaxationLimit && r.relaxationPasses>1 && std::isfinite(r.parameters.x),"forced XYZ failure retains final parameters after native relaxation limit");
    options={};options.onSurface=false;
    r=invertSurfaceParameters({3.,4.,0.},source,options);
    check(r.status.complete() && !r.converged && r.stop==SurfaceParameterInverseStop::OffSurfaceLast && r.parameters.x==-1. && r.parameters.y==-1. && r.seeds==81 && r.steps==81,"both axes outside break before parameter publication");
    r=invertSurfaceParameters({.25,3.,0.},source,options);
    check(r.status.complete() && !r.converged && close(r.parameters.x,.25) && r.parameters.y==3. && r.seeds==81,"one outside axis still updates before last-UV return");
    RunControl control;control.maximumGeometryEvaluations=2;
    r=invertSurfaceParameters({.25,.4,0.},source,{},control);
    check(r.status.code==MeshingStatus::BudgetExceeded && r.geometryEvaluations==2 && !r.converged,"budget shared across point and derivative requests");
    control={};control.keepGoing=[] { return false; };
    r=invertSurfaceParameters({.25,.4,0.},source,{},control);
    check(r.status.code==MeshingStatus::Cancelled && r.geometryEvaluations==0,"cancellation before first source request");
    auto bad=source;bad.point=[](Point2D) { return Point3D{std::numeric_limits<double>::quiet_NaN(),0.,0.}; };
    r=invertSurfaceParameters({},bad);
    check(r.status.code==MeshingStatus::GeometryQueryFailed && r.geometryEvaluations==1,"nonfinite surface point exposed");
    bad=source;bad.derivatives=[](Point2D)->std::array<Point3D,2> { throw OperationFailure{{MeshingStatus::Cancelled,"source cancelled"}}; };
    r=invertSurfaceParameters({.25,.4,0.},bad);
    check(r.status.code==MeshingStatus::Cancelled,"source cancellation preserved");
    bad=source;bad.derivatives=[](Point2D) { return std::array<Point3D,2>{}; };
    r=invertSurfaceParameters({.25,.4,0.},bad);
    check(r.status.complete() && r.converged && r.stop==SurfaceParameterInverseStop::ParameterStep && r.parameters.x==0. && r.parameters.y==0.,"zero tangent native accepts zero parameter step without extra rank gate");
    std::cout << "surface inverse failures=" << failures << '\n'; return failures ? 1 : 0;
}
