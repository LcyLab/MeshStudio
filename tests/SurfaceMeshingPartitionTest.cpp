#include "mesh/surface/legacy/SurfaceMeshingPartition.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace mesher;
namespace {
struct SplitPlane : GeometrySource {
    bool compatible=true,separateBodies=false,opposite=false,badProjection=false,hole=false;
    int vertexCount() const override {return hole?10:6;}
    Point3D vertexPoint(int v) const override {const Point3D p[]={{0,0,0},{2,0,0},{4,0,0},{4,3,0},{2,3,0},{0,3,0},{2.5,1,0},{2.5,2,0},{3.5,2,0},{3.5,1,0}};return p[v];}
    int edgeCount() const override {return hole?11:7;}
    std::pair<int,int> edgeVertices(int e) const override {const std::pair<int,int> v[]={{0,1},{1,4},{4,5},{5,0},{1,2},{2,3},{3,4},{6,7},{7,8},{8,9},{9,6}};return v[e];}
    std::pair<double,double> edgeRange(int) const override {return {0,1};}
    Point3D edgePoint(int e,double t) const override {const auto v=edgeVertices(e);const auto a=vertexPoint(v.first),b=vertexPoint(v.second);return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0};}
    int faceCount() const override {return 2;}
    FaceTopology faceTopology(int f) const override {auto topology=f?FaceTopology{{{{4},{5},{6},{1,true}}}}:FaceTopology{{{{0},{1},{2},{3}}}};
        if(f&&hole)topology.loops.push_back({{7},{8},{9},{10}});return topology;}
    Point2D coordinates(int f,Point3D p) const {return f?Point2D{2*p.x+p.y+10,3*p.y+20}:Point2D{p.x,p.y};}
    Point2D edgeUV(int f,const EdgeUse& use,double t) const override {return coordinates(f,edgePoint(use.edge,t));}
    Point3D facePoint(int f,Point2D uv) const override {if(!f)return {uv.x,uv.y,0};const double y=(uv.y-20)/3;return {(uv.x-y-10)/2,y,0};}
    void faceDerivatives(int f,Point2D,Point3D& du,Point3D& dv) const override {du=f?Point3D{.5,0,0}:Point3D{1,0,0};dv=f?Point3D{-1./6,1./3,0}:Point3D{0,1,0};}
    Point3D faceNormal(int f,Point2D) const override {return {0,0,f&&opposite?-1.:1.};}
    int faceBodyId(int f) const override {return separateBodies?f:0;}
    bool commonSurfaceChart(int,int) const override {return compatible;}
    bool faceCoordinates(int f,Point3D p,Point2D,Point2D& uv) const override {uv=coordinates(f,p);if(badProjection)uv.x+=1;return true;}
    CadTrimLocation faceTrimLocation(int f,Point2D uv) const override {const auto p=facePoint(f,uv);const double x0=f?2:0,x1=f?4:2,tol=1e-9;
        if(p.x<x0-tol||p.x>x1+tol||p.y<-tol||p.y>3+tol)return CadTrimLocation::Outside;
        if(hole&&f&&p.x>2.5+tol&&p.x<3.5-tol&&p.y>1+tol&&p.y<2-tol)return CadTrimLocation::Outside;
        if(hole&&f&&p.x>=2.5-tol&&p.x<=3.5+tol&&p.y>=1-tol&&p.y<=2+tol)return CadTrimLocation::Boundary;
        return p.x<=x0+tol||p.x>=x1-tol||p.y<=tol||p.y>=3-tol?CadTrimLocation::Boundary:CadTrimLocation::Inside;}
    bool bounds(Point3D& lo,Point3D& hi) const override {lo={0,0,0};hi={4,3,0};return true;}
};
struct SplitCylinder : GeometrySource {
    const double period=2*std::acos(-1.);
    int vertexCount() const override {return 3;}
    Point3D vertexPoint(int v) const override {return {2,0,4.*v};}
    int edgeCount() const override {return 5;}
    std::pair<int,int> edgeVertices(int e) const override {const std::pair<int,int> v[]={{0,0},{0,1},{1,1},{1,2},{2,2}};return v[e];}
    std::pair<double,double> edgeRange(int e) const override {return e%2?std::pair<double,double>{e==1?0.:4.,e==1?4.:8.}:std::pair<double,double>{0,period};}
    Point3D edgePoint(int e,double t) const override {return e%2?Point3D{2,0,t}:Point3D{2*std::cos(t),2*std::sin(t),2.*e};}
    int faceCount() const override {return 2;}
    FaceTopology faceTopology(int f) const override {return f?FaceTopology{{{{2},{3,false,1},{4,true},{3,true,0}}}}:FaceTopology{{{{0},{1,false,1},{2,true},{1,true,0}}}};}
    Point2D edgeUV(int f,const EdgeUse& use,double t) const override {Point2D uv=use.edge%2?Point2D{use.branch?period:0,t}:Point2D{t,2.*use.edge};
        return f?Point2D{uv.x+10,2*uv.y+20}:uv;}
    Point3D facePoint(int f,Point2D uv) const override {if(f)uv={uv.x-10,(uv.y-20)/2};return {2*std::cos(uv.x),2*std::sin(uv.x),uv.y};}
    void faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const override {if(f)uv={uv.x-10,(uv.y-20)/2};du={-2*std::sin(uv.x),2*std::cos(uv.x),0};dv={0,0,f?.5:1.};}
    Point3D faceNormal(int f,Point2D uv) const override {if(f)uv.x-=10;return {std::cos(uv.x),std::sin(uv.x),0};}
    bool commonSurfaceChart(int,int) const override {return true;}
    bool faceCoordinates(int f,Point3D p,Point2D seed,Point2D& uv) const override {double u=std::atan2(p.y,p.x)+(f?10:0);u+=std::round((seed.x-u)/period)*period;uv={u,f?2*p.z+20:p.z};return true;}
    Point2D facePeriods(int) const override {return {period,0};}
    bool faceParameterBounds(int f,SurfaceParameterBounds& b) const override {b=f?SurfaceParameterBounds{10,10+period,28,36}:SurfaceParameterBounds{0,period,0,4};return true;}
    CadTrimLocation faceTrimLocation(int f,Point2D uv) const override {SurfaceParameterBounds b;faceParameterBounds(f,b);const double tol=1e-8;
        if(uv.x<b.uMin-tol||uv.x>b.uMax+tol||uv.y<b.vMin-tol||uv.y>b.vMax+tol)return CadTrimLocation::Outside;
        return uv.x<=b.uMin+tol||uv.x>=b.uMax-tol||uv.y<=b.vMin+tol||uv.y>=b.vMax-tol?CadTrimLocation::Boundary:CadTrimLocation::Inside;}
    bool bounds(Point3D& lo,Point3D& hi) const override {lo={-2,-2,0};hi={2,2,8};return true;}
};
struct ContactAssembly : SplitPlane {
    int vertexCount() const override {return 9;}
    Point3D vertexPoint(int v) const override {return v<6?SplitPlane::vertexPoint(v):Point3D{4.,v==7?1.:0.,v==8?1.:0.};}
    int edgeCount() const override {return 10;}
    std::pair<int,int> edgeVertices(int e) const override {return e<7?SplitPlane::edgeVertices(e):std::pair<int,int>{e-1,e==9?6:e};}
    Point3D edgePoint(int e,double t) const override {const auto v=edgeVertices(e);const auto a=vertexPoint(v.first),b=vertexPoint(v.second);return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)};}
    int faceCount() const override {return 3;}
    FaceTopology faceTopology(int f) const override {return f<2?SplitPlane::faceTopology(f):FaceTopology{{{{7},{8},{9}}}};}
    Point2D edgeUV(int f,const EdgeUse& use,double t) const override {if(f<2)return SplitPlane::edgeUV(f,use,t);const auto p=edgePoint(use.edge,t);return {p.y,p.z};}
    Point3D facePoint(int f,Point2D uv) const override {return f<2?SplitPlane::facePoint(f,uv):Point3D{4,uv.x,uv.y};}
    int faceBodyId(int f) const override {return f<2?0:1;}
    bool commonSurfaceChart(int a,int b) const override {return a<2&&b<2;}
    CadBodyContact bodyContact(int a,int b) const override {return (a==1&&b==2)||(a==2&&b==1)?CadBodyContact::BoundaryContact:CadBodyContact::None;}
};
struct CircularTrim : SplitPlane {
    bool invalidProjection=false;
    CadTrimLocation faceTrimLocation(int f,Point2D uv) const override {
        const auto base=SplitPlane::faceTrimLocation(f,uv);if(!f||base==CadTrimLocation::Outside)return base;
        const auto p=facePoint(f,uv);const double radius=std::hypot(p.x-3,p.y-1.5);
        return radius<.5-1e-9?CadTrimLocation::Outside:radius<=.5+1e-9?CadTrimLocation::Boundary:base;
    }
    bool closestTrimCoordinates(int f,Point3D p,Point2D,Point2D& uv,double& reported) const override {
        Point3D closest{std::clamp(p.x,f?2.:0.,f?4.:2.),std::clamp(p.y,0.,3.),0};
        if(f&&!invalidProjection){const double radius=std::hypot(closest.x-3,closest.y-1.5);
            if(radius<.5){if(radius>0){closest.x=3+(closest.x-3)*.5/radius;closest.y=1.5+(closest.y-1.5)*.5/radius;}else closest.x=3.5;}}
        uv=coordinates(f,closest);reported=0;return true; // Deliberately wrong distance: the decorator must recompute it.
    }
};
struct CountingTrim : SplitPlane {
    mutable int trimQueries=0;
    CadTrimLocation faceTrimLocation(int f,Point2D uv) const override {++trimQueries;return SplitPlane::faceTrimLocation(f,uv);}
};
SurfaceMeshResult facet(const SurfaceMeshingPartition& partition,std::array<Point2D,3> uv) {
    SurfaceMeshResult result;result.ok=true;result.mesh.triangles={{0,1,2}};result.triangleFaceIds={0};result.triangleCornerUV={uv};
    for(auto p:uv)result.mesh.vertices.push_back(partition.facePoint(0,p));return result;
}
bool near(Point3D a,Point3D b){return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)<1e-9;}
}
int main() {
    SplitPlane geometry;SurfacePartitionPolicy policy;policy.enabled=true;
    SurfaceMeshingPartition merged(geometry,policy);
    if(merged.faceCount()!=1||merged.removedCadEdges()!=std::vector<int>{1}){
        std::cerr<<"FAIL common split plane must become one patch without the internal CAD edge\n";return 1;}
    bool ok=merged.edgeCount()==6&&merged.sourceFaces()==std::vector<std::vector<int>>{{0,1}};
    for(const auto& loop:merged.faceTopology(0).loops)for(auto use:loop)for(double t:{0.,.37,1.})
        ok=ok&&near(merged.facePoint(0,merged.edgeUV(0,use,t)),merged.edgePoint(use.edge,t));
    SurfaceMeshResult::CadSupportSample support;
    ok=ok&&merged.supportAt(0,{3,1},support)&&support.face==1&&near(geometry.facePoint(support.face,support.uv),Point3D{3,1,0});
    ok=ok&&!merged.supportAt(0,{5,1},support);
    for(int mode=0;mode<6;++mode){auto source=geometry;auto protectedPolicy=policy;
        if(mode==0)protectedPolicy.enabled=false;if(mode==1)protectedPolicy.protectedCadEdges={1};
        if(mode==2)source.separateBodies=true;if(mode==3)source.compatible=false;if(mode==4)source.opposite=true;if(mode==5)source.badProjection=true;
        SurfaceMeshingPartition kept(source,protectedPolicy);ok=ok&&kept.faceCount()==2&&kept.removedCadEdges().empty();}
    auto regions=policy;regions.faceRegions={1,2};SurfaceMeshingPartition interface(geometry,regions);ok=ok&&interface.faceCount()==2;
    auto holed=geometry;holed.hole=true;SurfaceMeshingPartition ring(holed,policy);ok=ok&&ring.faceCount()==1&&ring.faceTopology(0).loops.size()==2&&
        !ring.supportAt(0,{3,1.5},support)&&ring.supportAt(0,{3,2.5},support);
    SplitCylinder cylinder;SurfaceMeshingPartition periodic(cylinder,policy);
    std::cout<<"periodic patches="<<periodic.faceCount()<<" removed="<<periodic.removedCadEdges().size()<<'\n';
    for(const auto& diagnostic:periodic.diagnostics())std::cout<<diagnostic<<'\n';
    ok=ok&&periodic.faceCount()==1&&periodic.removedCadEdges()==std::vector<int>{2}&&periodic.faceTopology(0).loops.size()==1;
    if(periodic.faceCount()==1){for(const auto& loop:periodic.faceTopology(0).loops)for(auto use:loop)for(double w:{0.,.4,1.}){const auto r=periodic.edgeRange(use.edge);const double t=r.first+w*(r.second-r.first);
            ok=ok&&near(periodic.facePoint(0,periodic.edgeUV(0,use,t)),periodic.edgePoint(use.edge,t));}
        ok=ok&&periodic.supportAt(0,{1,6},support)&&support.face==1&&near(cylinder.facePoint(support.face,support.uv),periodic.facePoint(0,{1,6}));}
    SurfacePartitionPolicy disabled;SurfaceMeshingPartition originals(cylinder,disabled);SurfaceMeshResult native;native.ok=true;
    native.mesh.triangles={{0,1,2}};native.triangleFaceIds={0};native.triangleCornerUV={{{{cylinder.period,1},{cylinder.period,.5},{cylinder.period-.1,.5}}}};
    for(auto uv:native.triangleCornerUV.front())native.mesh.vertices.push_back(originals.facePoint(0,uv));
    const auto nativeUV=native.triangleCornerUV;const bool decorated=originals.decorateResult(native,{},1);ok=ok&&decorated&&native.triangleFaceIds==std::vector<int>{0};
    for(int k=0;k<3;++k){const bool preserved=native.triangleCornerUV[0][k].x==nativeUV[0][k].x&&native.triangleCornerUV[0][k].y==nativeUV[0][k].y;
        if(!preserved)std::cerr<<"FAIL single-face periodic UV branch changed at corner "<<k<<'\n';ok=ok&&preserved;}
    ContactAssembly assembly;SurfaceMeshingPartition contact(assembly,policy);
    const bool preservedContact=contact.faceCount()==2&&contact.bodyContact(0,1)==CadBodyContact::BoundaryContact;
    if(!preservedContact)std::cerr<<"FAIL patch contact query ignored a non-representative original face\n";ok=ok&&preservedContact;
    SurfaceMeshResult tagged;tagged.ok=true;tagged.mesh.triangles={{0,1,2}};tagged.triangleFaceIds={0};
    tagged.triangleCornerUV={{{{1,1},{3,1},{2,2}}}};tagged.diagnostics={"face 0 CAD edge 1 diagnostic"};
    for(auto uv:tagged.triangleCornerUV.front())tagged.mesh.vertices.push_back(contact.facePoint(0,uv));
    SurfaceRefinementSample splitSample;splitSample.face=0;splitSample.cadEdge=1;splitSample.facet.uv=tagged.triangleCornerUV.front();
    SurfaceRefinementSample singleSample=splitSample;singleSample.face=1;singleSample.cadEdge=-1;
    tagged.refinementSamples={splitSample,singleSample};
    tagged.bodyInteractions.push_back({0,1,0,1,CadBodyContact::BoundaryContact});
    const auto rawTagged=tagged;const bool taggedOK=contact.decorateResult(tagged);
    const bool originalTags=taggedOK&&tagged.refinementSamples[0].partition==0&&tagged.refinementSamples[0].face==-1&&
        tagged.refinementSamples[0].cadEdge==2&&tagged.refinementSamples[1].partition==1&&tagged.refinementSamples[1].face==2&&
        tagged.refinementSamples[1].cadEdge==-1&&tagged.bodyInteractions[0].firstPartition==0&&tagged.bodyInteractions[0].secondPartition==1&&
        tagged.bodyInteractions[0].firstFace==-1&&tagged.bodyInteractions[0].secondFace==2;
    if(!originalTags)std::cerr<<"FAIL snapshot/interaction labels must separate partition IDs from original CAD IDs\n";ok=ok&&originalTags;
    for(int k=0;k<3;++k)ok=ok&&tagged.refinementSamples[0].facet.uv[k].x==rawTagged.refinementSamples[0].facet.uv[k].x&&
        tagged.refinementSamples[0].facet.uv[k].y==rawTagged.refinementSamples[0].facet.uv[k].y;
    ok=ok&&!tagged.diagnostics.empty()&&tagged.diagnostics.front().find("virtual patch")!=std::string::npos&&
        tagged.diagnostics.front().find("retained-edge")!=std::string::npos;
    for(int mode=0;mode<5;++mode){auto malformed=rawTagged;
        if(mode==0)malformed.refinementSamples[0].face=99;if(mode==1)malformed.refinementSamples[0].cadEdge=99;
        if(mode==2)malformed.bodyInteractions[0].firstFace=99;if(mode==3)malformed.bodyInteractions[0].secondFace=99;
        if(mode==4)malformed.boundaryEdges.push_back({0,1,99});
        bool rejected=false;try{rejected=!contact.decorateResult(malformed)&&!malformed.ok&&!malformed.error.empty();}catch(const std::exception&){rejected=false;}
        ok=ok&&rejected&&malformed.refinementSamples.size()==2&&malformed.bodyInteractions.size()==1&&malformed.trianglePartitionIds.empty();
        if(!rejected)std::cerr<<"FAIL invalid diagnostic metadata must return an explicit failure, mode="<<mode<<'\n';
    }
    CircularTrim curvedTrim;SurfaceMeshingPartition trimmed(curvedTrim,policy);
    const auto approximate=facet(trimmed,{{{2.7,1.9},{3.3,1.9},{3,2.5}}});auto accepted=approximate;
    const bool projected=trimmed.decorateResult(accepted,{},.11);
    const bool projectionRecorded=projected&&accepted.triangleCadSupportSamples.size()==1&&accepted.triangleCadSupportSamples[0][3].projectedToTrim&&
        std::abs(accepted.triangleCadSupportSamples[0][3].distance-.1)<1e-9&&
        curvedTrim.faceTrimLocation(accepted.triangleCadSupportSamples[0][3].face,accepted.triangleCadSupportSamples[0][3].uv)==CadTrimLocation::Boundary;
    if(!projectionRecorded)std::cerr<<"FAIL linear facet outside exact trim needs verified closest-trim support and actual distance\n";ok=ok&&projectionRecorded;
    auto tooFar=approximate;const bool bounded=!trimmed.decorateResult(tooFar,{},.05)&&!tooFar.ok&&!tooFar.error.empty();ok=ok&&bounded;
    auto offSurface=facet(merged,{{{1,1},{3,1},{2,2}}});for(auto& p:offSurface.mesh.vertices)p.z=.2;
    const bool exactDistanceChecked=!merged.decorateResult(offSurface,{},.1)&&!offSurface.ok;
    if(!exactDistanceChecked)std::cerr<<"FAIL exact trim membership must still bound physical facet-to-CAD distance\n";ok=ok&&exactDistanceChecked;
    curvedTrim.invalidProjection=true;SurfaceMeshingPartition untrimmedProjection(curvedTrim,policy);auto outside=approximate;
    const bool trueTrimRequired=!untrimmedProjection.decorateResult(outside,{},.11)&&!outside.ok;ok=ok&&trueTrimRequired;
    SurfaceMeshingPartition cancelledPreparation(geometry,policy,[](int,int){return false;});
    const bool prepCancelled=cancelledPreparation.validationError().find("cancelled during partition preparation")!=std::string::npos;
    if(!prepCancelled)std::cerr<<"FAIL partition preparation must propagate cancellation\n";ok=ok&&prepCancelled;
    SurfaceCriteria c;c.targetSize=c.maxSize=.75;c.minSize=.075;c.maxDeviation=.001;c.shapeOptimizationSweeps=0;
    c.pointPlacement=SurfacePointPlacement::FrontalDelaunay;
    auto beforeDecoration=generateCadSurfaceMesh(merged,c);const bool generatedBeforeCancel=beforeDecoration.ok;
    const bool decorCancelled=!merged.decorateResult(beforeDecoration,[](int,int){return false;},c.maxDeviation)&&!beforeDecoration.ok&&
        beforeDecoration.error.find("cancelled during CAD support decoration")!=std::string::npos;
    if(!decorCancelled)std::cerr<<"FAIL successful generation must remain cancellable during support decoration\n";ok=ok&&generatedBeforeCancel&&decorCancelled;
    CountingTrim countingTrim;bool cancelledInsideDecoration=false;
    const auto interrupted=generatePartitionedCadSurfaceMesh(countingTrim,c,policy,nullptr,[&](int,int){
        if(countingTrim.trimQueries<40)return true;cancelledInsideDecoration=true;return false;});
    const bool wrapperCancelled=cancelledInsideDecoration&&countingTrim.trimQueries>=40&&countingTrim.trimQueries<=48&&
        !interrupted.ok&&!interrupted.mesh.triangles.empty()&&interrupted.triangleCadSupportSamples.empty()&&
        interrupted.error.find("cancelled during CAD support decoration")!=std::string::npos;
    if(!wrapperCancelled)std::cerr<<"FAIL wrapper must propagate cancellation after trim support queries have begun\n";ok=ok&&wrapperCancelled;
    const auto generated=generatePartitionedCadSurfaceMesh(geometry,c,policy);ok=ok&&generated.ok;
    bool crosses=false;
    if(generated.ok){ok=ok&&generated.trianglePartitionIds.size()==generated.mesh.triangles.size()&&generated.triangleCadSupportSamples.size()==generated.mesh.triangles.size();
        for(size_t t=0;t<generated.mesh.triangles.size();++t){double lo=1e9,hi=-1e9;for(int v:generated.mesh.triangles[t]){lo=std::min(lo,generated.mesh.vertices[v].x);hi=std::max(hi,generated.mesh.vertices[v].x);}
            crosses|=lo<2-1e-8&&hi>2+1e-8;ok=ok&&generated.triangleFaceIds[t]==-1&&std::isnan(generated.triangleCornerUV[t][0].x);
            if(t<generated.triangleCadSupportSamples.size())for(auto sample:generated.triangleCadSupportSamples[t])ok=ok&&sample.face>=0&&geometry.faceTrimLocation(sample.face,sample.uv)!=CadTrimLocation::Outside;}
        for(auto edge:generated.boundaryEdges)ok=ok&&edge.cadEdge!=1;
    }else std::cerr<<generated.error<<'\n';
    ok=ok&&crosses;
    if(!ok)std::cerr<<"FAIL partition mapping, retained boundaries or CAD support metadata\n";
    return ok?0:1;
}
