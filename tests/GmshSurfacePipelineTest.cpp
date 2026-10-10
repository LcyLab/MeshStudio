/** @file GmshSurfacePipelineTest.cpp
 * @brief 不提供参考输入的BREP完整算法6流水线及失败原子性测试。
 * @date 2026-10-08
 * @modified 2026-10-09
 */
#include "cad/occ/OccAlgorithm6Adapter.h"
#include "mesh/surface/algorithm6/pipeline/MeshAssembly.h"
#include "Algorithm6CadFixtures.h"
#include <BRepTools.hxx>
#include <filesystem>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <map>
#include <algorithm>
#include <cstdlib>
#include <Standard_Failure.hxx>
using namespace mesher; using namespace mesher::gmshReference;
static void check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
static void checkClosedCadMesh(const Algorithm6SurfaceResult& result,const OccCadGeometry& geometry) {
    check(result.status.complete()&&result.surface.ok,"closed CAD generation must complete");
    const auto& surface=result.surface;
    check(surface.triangleCornerUV.size()==surface.mesh.triangles.size(),"periodic corner UV coverage");
    std::map<std::array<int,2>,std::pair<int,int>> incidence;
    std::map<int,Point2D> firstUV;bool distinctBranch=false;
    for(std::size_t i=0;i<surface.mesh.triangles.size();++i) {
        const auto t=surface.mesh.triangles[i];const int face=surface.triangleFaceIds[i];
        check(surface.triangleBodyIds[i]==geometry.faceBodyId(face),"periodic body labels");
        for(int k=0;k<3;++k) {
            const auto uv=surface.triangleCornerUV[i][k];const auto p=geometry.faceSurfacePoint(face,uv),q=surface.mesh.vertices[t[k]];
            check(std::hypot(p.x-q.x,p.y-q.y,p.z-q.z)<1.e-10,"periodic per-corner UV must reproduce physical XYZ");
            if(face==0){const auto saved=firstUV.emplace(t[k],uv);if(!saved.second&&std::abs(saved.first->second.x-uv.x)>3.)distinctBranch=true;}
            auto edge=std::array<int,2>{t[k],t[(k+1)%3]};const int sign=edge[0]<edge[1]?1:-1;std::sort(edge.begin(),edge.end());auto& entry=incidence[edge];++entry.first;entry.second+=sign;
        }
    }
    for(const auto& e:incidence)check(e.second.first==2&&e.second.second==0,"closed periodic surface must have two oppositely oriented incident faces per edge");
    check(distinctBranch,"periodic shared physical node must retain distinct per-corner branches");
}
int main(int argc,char** argv) {
    try {
        const auto shape = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,0,0),gp_Dir(0,0,1)),0,4,0,3).Face();
        OccAlgorithm6Adapter adapter(shape);
        auto input = adapter.buildModelInput();
        SurfaceMeshingOptions options; options.sizeMax=.3; options.sizeFromPoints=false; options.switchOnFailure=false;
        for(int extension : {0,1}) {
            options.extendBoundarySize=extension;
            const auto result=generateSurfaceMesh(input,options);
            if(!result.status.complete()) std::cerr<<result.failedStage<<": "<<result.status.message<<'\n';
            check(result.status.complete()&&result.surface.ok,"CAD-only rectangle pipeline must complete");
            check(result.surface.mesh.triangles.size()>100,"rectangle must consume generated interior triangles");
            check(result.statistics.inserted>0,"front must insert real points");
            check(result.surface.triangleCornerUV.size()==result.surface.mesh.triangles.size(),"corner UV coverage");
            check(result.surface.triangleFaceIds.size()==result.surface.mesh.triangles.size(),"face labels");
            for(int id:result.surface.triangleFaceIds)check(id==0,"standalone face must retain zero-based CAD identity");
            for(int id:result.surface.triangleBodyIds)check(id==adapter.geometry().faceBodyId(0),"CAD body identity");
        }
        auto ordinaryWithoutDomainSampling=input;
        for(auto& curve:ordinaryWithoutDomainSampling.faces[0].trueBoundaryCurves)
            curve.onFace=[](double,int)->Point2D{throw std::runtime_error("ordinary front must not request periodic true-boundary sampling");};
        const auto ordinaryGenerated=generateSurfaceMesh(ordinaryWithoutDomainSampling,options);
        check(ordinaryGenerated.status.complete()&&ordinaryGenerated.statistics.inserted>0,
              "ordinary native front has no true-boundary gate; restored constraints and initial domain retain the trim");
        RunControl cancelled; cancelled.keepGoing=[] {return false;};
        auto result=generateSurfaceMesh(input,options,cancelled);
        check(result.status.code==MeshingStatus::Cancelled&&!result.surface.ok&&result.surface.mesh.triangles.empty(),"cancelled pipeline must not publish");
        RunControl budget; budget.maximumGeometryEvaluations=1;
        result=generateSurfaceMesh(input,options,budget);
        check(result.status.code==MeshingStatus::BudgetExceeded&&!result.surface.ok,"global query budget");
        input.faces.front().chart.surfacePoint=[](Point2D)->Point3D {throw std::runtime_error("injected CAD query failure");};
        result=generateSurfaceMesh(input,options);
        check(result.status.code==MeshingStatus::GeometryQueryFailed&&result.failedFace==1&&!result.surface.ok,"CAD failure must carry face and publish nothing");
        input=adapter.buildModelInput();
        bool keepGoing=true;RunControl midCancel;midCancel.keepGoing=[&] {return keepGoing;};
        result=generateSurfaceMesh(input,options,midCancel,[&](const std::string& stage,int,const BoundaryMesh&,const FaceMesh*) {if(stage=="pre_smoothing")keepGoing=false;});
        check(result.status.code==MeshingStatus::Cancelled&&result.failedStage=="smoothing"&&!result.surface.ok&&result.surface.mesh.triangles.empty(),"mid-generation cancel cannot publish transferred mesh");
        BoundaryMesh boundaryFixture;const auto boundaryBuilt=buildBoundaryMesh(input.boundary,options,input.sizes,boundaryFixture);
        check(boundaryBuilt.status.complete(),"budget fixture boundary");
        budget.maximumGeometryEvaluations=boundaryBuilt.geometryEvaluations+1;
        result=generateSurfaceMesh(input,options,budget);
        check(result.status.code==MeshingStatus::BudgetExceeded&&result.failedStage=="chart"&&!result.surface.ok,"cumulative stage budget must not reset at chart");
        budget={};budget.maximumNodes=20;
        result=generateSurfaceMesh(input,options,budget);
        check(result.status.code==MeshingStatus::BudgetExceeded&&!result.surface.ok,"allocated node cap");
        input.faces.front().chart.boundaryCurves.front().seam=true;
        result=generateSurfaceMesh(input,options);
        check(result.status.code==MeshingStatus::NonConverged&&result.failedFace==1&&!result.surface.ok,"seam must reject ordinary path");
        OccAlgorithm6Adapter cylinder(BRepPrimAPI_MakeCylinder(1.,2.).Shape());
        result=generateSurfaceMesh(cylinder.buildModelInput(),options);
        if(!result.status.complete())std::cerr<<result.failedStage<<": "<<result.status.message<<'\n';
        check(result.status.complete()&&result.surface.ok,"CAD-only cylinder including seam must complete");
        check(result.surface.mesh.triangles.size()>100,"cylinder must contain full surface mesh");
        checkClosedCadMesh(result,cylinder.geometry());
        check(result.faces.size()==std::size_t(cylinder.geometry().faceCount()),"model report must retain all CAD face outcomes");
        for(const auto& face:result.faces)check(face.attempted&&face.status.complete(),"complete cylinder must run every face");
        OccAlgorithm6Adapter cylinderPatch(BRepBuilderAPI_MakeFace(gp_Cylinder(gp_Ax3(gp_Pnt(0,0,0),gp_Dir(0,0,1)),1.),.2,2.2,0.,2.).Face());
        const auto patchInput=cylinderPatch.buildModelInput();
        check(bool(patchInput.faces[0].periodicChart),"periodic support without seam must select native periodic pipeline");
        result=generateSurfaceMesh(patchInput,options);
        check(result.status.complete()&&result.surface.ok,"trimmed cylinder patch must complete through periodic pipeline");
        SurfaceMeshingOptions curvatureOptions=options;curvatureOptions.sizeMax=.6;curvatureOptions.elementsPerCurvatureCircle=32.;
        result=generateSurfaceMesh(cylinder.buildModelInput(),curvatureOptions);
        check(result.status.complete()&&result.surface.mesh.triangles.size()>1000,"CAD curvature sizes must drive the full Algorithm 6 pipeline");
        OccAlgorithm6Adapter sphere(BRepPrimAPI_MakeSphere(1.).Shape());
        result=generateSurfaceMesh(sphere.buildModelInput(),options);
        if(!result.status.complete())std::cerr<<result.failedStage<<": "<<result.status.message<<'\n';
        check(result.status.complete()&&result.surface.ok,"CAD-only sphere including poles must complete");
        checkClosedCadMesh(result,sphere.geometry());
        const auto sphereInput=sphere.buildModelInput();BoundaryMesh sphereBoundary;
        check(buildBoundaryMesh(sphereInput.boundary,options,sphereInput.sizes,sphereBoundary).status.complete(),"periodic random fixture boundary");
        std::size_t randomQueries=0;const auto periodic=sphereInput.faces[0].periodicChart(sphereBoundary,options,{},randomQueries);
        const double span=std::hypot(periodic.boundary.parameterUpper.x-periodic.boundary.parameterLower.x,periodic.boundary.parameterUpper.y-periodic.boundary.parameterLower.y);
        PeriodicInitialPerturbation random;
        std::srand(1);
        for(int block=0;block<2;++block)for(int i=0;i<10;++i){const auto p=random.perturb({0,0},span,options.initialRandomFactor);const double x=options.initialRandomFactor*span*double(std::rand())/double(RAND_MAX),y=options.initialRandomFactor*span*double(std::rand())/double(RAND_MAX);check(p.x==x&&p.y==y,"owned initial perturbation must continue across blocks with native CRT arithmetic");}
        bool cleaned=false;
        result=generateSurfaceMesh(sphere.buildModelInput(),options,{},[&](const std::string& stage,int,const BoundaryMesh&,const FaceMesh* mesh){
            if(stage=="final"){cleaned=true;check(mesh->equivalence.empty(),"final periodic equivalence objects must be released");for(const auto& v:mesh->vertices)check(v.second.origin!=FaceVertexOrigin::PeriodicCopy,"final periodic fake vertices must be released");}
        });
        check(result.status.complete()&&cleaned,"periodic cleanup must be observed after smoothing");
        input=adapter.buildModelInput();
        input.faces[0].finalBoundaryUV=[](Point3D,Point2D)->Point2D{throw std::runtime_error("injected final CAD projection failure");};
        result=generateSurfaceMesh(input,options);
        check(result.status.code==MeshingStatus::GeometryQueryFailed&&result.failedStage=="final_coordinates"&&!result.surface.ok,"final UV failure cannot publish a completed mesh");
        input.faces[0].finalBoundaryUV=[](Point3D,Point2D)->Point2D{throw Standard_Failure("injected OCC final projection failure");};
        bool escaped=false;try{result=generateSurfaceMesh(input,options);}catch(const Standard_Failure&){escaped=true;}
        check(!escaped&&result.status.code==MeshingStatus::GeometryQueryFailed&&result.failedStage=="final_coordinates"&&!result.surface.ok,"native OCC exception must become an explicit atomic generation failure");
        BRepBuilderAPI_MakePolygon polygon;polygon.Add(gp_Pnt(0,0,0));polygon.Add(gp_Pnt(1,0,0));polygon.Add(gp_Pnt(0,1,0));polygon.Close();
        OccAlgorithm6Adapter triangle(BRepBuilderAPI_MakeFace(polygon.Wire()).Face());auto triangleInput=triangle.buildModelInput();
        SurfaceMeshingOptions coarse=options;coarse.sizeMax=10.;
        std::size_t finalQueries=0;const auto project=triangleInput.faces[0].finalBoundaryUV;
        triangleInput.faces[0].finalBoundaryUV=[&](Point3D p,Point2D uv){++finalQueries;return project(p,uv);};
        result=generateSurfaceMesh(triangleInput,coarse);
        check(result.status.complete()&&result.surface.mesh.triangles.size()==1&&finalQueries==3,"direct triangle must also finalize every CAD corner");
        const auto& sizedEdge=triangleInput.boundary.curves[0];
        const auto sizedTriangle=triangle.buildModelInput({{sizedEdge.beginVertex,.1},{sizedEdge.endVertex,.2}});
        const double middle=(sizedEdge.first+sizedEdge.last)*.5;
        check(std::abs(sizedTriangle.sizes.pointSize({1,sizedEdge.entity,middle,0,sizedEdge.point(middle)})-.15)<1.e-14,"CAD edge point size must interpolate its prescribed endpoint sizes");
        std::map<int,double> prescribed;for(int i=1;i<=adapter.geometry().vertexCount();++i)prescribed.emplace(i,.12);
        SurfaceMeshingOptions pointOptions=options;pointOptions.sizeFromPoints=true;
        result=generateSurfaceMesh(adapter.buildModelInput(prescribed),pointOptions);
        check(result.status.complete()&&result.surface.boundaryEdges.size()>80,"explicit point sizes must reach shared boundary and final mesh");
        triangleInput.faces[0].finalBoundaryUV=[](Point3D,Point2D)->Point2D{throw std::runtime_error("direct final CAD projection failure");};
        result=generateSurfaceMesh(triangleInput,coarse);
        check(result.status.code==MeshingStatus::GeometryQueryFailed&&result.failedStage=="final_coordinates"&&!result.surface.ok,"direct corner projection failure must remain atomic");
        keepGoing=true;
        result=generateSurfaceMesh(sphere.buildModelInput(),options,midCancel,[&](const std::string& stage,int,const BoundaryMesh&,const FaceMesh*){if(stage=="pre_smoothing")keepGoing=false;});
        check(result.status.code==MeshingStatus::Cancelled&&result.failedStage=="periodic_smoothing"&&!result.surface.ok,"periodic cancellation must not publish folded mesh");
        input=adapter.buildModelInput();FaceMesh finalMesh;BoundaryMesh finalBoundary;
        result=generateSurfaceMesh(input,options,{},[&](const std::string& stage,int,const BoundaryMesh& b,const FaceMesh* m){if(stage=="final"){finalMesh=*m;finalBoundary=b;}});
        check(result.status.complete(),"assembly fixture generation");
        finalMesh.triangles.push_back(finalMesh.triangles.front());
        SurfaceMeshResult sentinel;sentinel.error="unchanged";
        const auto assembled=assembleSurfaceMesh(finalBoundary,{{&finalMesh,0,input.faces[0].orientedNormal}},sentinel);
        check(assembled.code==MeshingStatus::InvalidTopology&&sentinel.error=="unchanged"&&sentinel.mesh.triangles.empty(),"assembly rejects duplicates atomically");
        FaceMesh zeroArea=finalMesh;zeroArea.triangles.resize(1);zeroArea.triangleCornerUV.resize(1);
        zeroArea.vertices.at(zeroArea.triangles[0][2]).point=zeroArea.vertices.at(zeroArea.triangles[0][1]).point;
        int assemblyFailedFace=-1;
        const auto zeroAreaStatus=assembleSurfaceMesh(finalBoundary,{{&zeroArea,0,input.faces[0].orientedNormal}},sentinel,{},&assemblyFailedFace);
        check(zeroAreaStatus.code==MeshingStatus::InvalidTopology&&zeroAreaStatus.message.find("face=1")!=std::string::npos&&
              zeroAreaStatus.message.find("triangle=0")!=std::string::npos&&assemblyFailedFace==1&&sentinel.error=="unchanged",
              "zero-area assembly failure must name the CAD face and local triangle without publishing");
        auto assemblyFailureInput=adapter.buildModelInput();
        assemblyFailureInput.faces[0].orientedNormal=[](Point2D)->Point3D{throw Standard_Failure("injected assembly source failure");};
        const auto assemblyFailure=generateSurfaceMesh(assemblyFailureInput,options);
        check(assemblyFailure.status.code==MeshingStatus::GeometryQueryFailed&&assemblyFailure.failedFace==1&&
              assemblyFailure.failedStage=="assembly"&&!assemblyFailure.surface.ok&&
              !assemblyFailure.faces[0].current&&assemblyFailure.faces[0].status.code==MeshingStatus::GeometryQueryFailed,
              "late assembly failure must remain in per-face report and publish no complete model");
        if(argc>2)throw std::runtime_error("usage: GmshSurfacePipelineTest [new-fixture-directory]");
        if(argc==2&&!std::filesystem::create_directory(argv[1]))throw std::runtime_error("fixture directory must be new");
        const std::vector<std::pair<std::string,TopoDS_Shape>> fixtures={
            {"periodic-hole",algorithm6fixtures::periodicHole()},
            {"embedded-plane",algorithm6fixtures::embeddedPlane()},
            {"embedded-cylinder",algorithm6fixtures::embeddedCylinder()}};
        for(const auto& fixture:fixtures) {
            if(argc==2)check(BRepTools::Write(fixture.second,(std::filesystem::path(argv[1])/(fixture.first+".brep")).string().c_str()),"write independent CAD fixture");
            OccAlgorithm6Adapter cad(fixture.second);const auto model=cad.buildModelInput();
            if(fixture.first.find("embedded")!=std::string::npos)check(!model.faces[0].chart.embeddedCurves.empty(),"BREP INTERNAL edge must remain an embedded constraint");
            if(fixture.first=="embedded-cylinder")check(model.faces[0].chart.embeddedVertices.size()==1,"embedded curve endpoints must not be duplicated as independent embedded vertices");
            if(fixture.first=="embedded-plane")check(model.faces[0].chart.embeddedVertices.empty(),"embedded edge alone has no independent face-internal vertex");
            for(int extension:{0,1}) {
                auto configured=options;configured.extendBoundarySize=extension;
                const auto generated=generateSurfaceMesh(model,configured);
                if(!generated.status.complete())throw std::runtime_error(fixture.first+" at "+generated.failedStage+": "+generated.status.message);
                for(std::size_t i=0;i<generated.surface.mesh.triangles.size();++i) {
                    const auto& uv=generated.surface.triangleCornerUV[i];const int face=generated.surface.triangleFaceIds[i];
                    const auto location=cad.geometry().faceTrimLocation(face,{(uv[0].x+uv[1].x+uv[2].x)/3,(uv[0].y+uv[1].y+uv[2].y)/3});
                    check(location==CadTrimLocation::Inside||location==CadTrimLocation::Boundary,"embedded/periodic hole centroid must remain in CAD trim");
                }
                if(fixture.first=="embedded-cylinder")checkClosedCadMesh(generated,cad.geometry());
            }
        }
        std::cout<<"CAD-only pipeline and explicit failure controls passed\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
      catch(const Standard_Failure& e) {std::cerr<<"native OCC failure in test harness: "<<e.DynamicType()->Name()<<": "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
}
