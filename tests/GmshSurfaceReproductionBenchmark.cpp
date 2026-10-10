/** @file GmshSurfaceReproductionBenchmark.cpp
 * @brief CAD-only生成/阶段VTP导出及同一独立审计器；无Gmsh调用。
 * @date 2026-10-08
 * @modified 2026-10-09
 */
#include "cad/occ/OccAlgorithm6Adapter.h"
#include "mesh/surface/algorithm6/pipeline/MeshAssembly.h"
#include "mesh/surface/algorithm6/initial/ConstraintRecovery.h"
#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
#include "Algorithm6SurfaceArtifacts.h"
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <STEPControl_Reader.hxx>
#include <chrono>
#include <iostream>
#include <Standard_Failure.hxx>
using namespace mesher;using namespace mesher::gmshReference;using namespace algorithm6artifacts;
namespace fs=std::filesystem;
int main(int argc,char** argv) {
    if(argc==4&&std::string(argv[1])=="freeze-step") {
        try{if(fs::exists(argv[3]))throw std::runtime_error("frozen BREP destination must be new");STEPControl_Reader reader;if(reader.ReadFile(argv[2])!=IFSelect_RetDone||reader.TransferRoots()<=0)throw std::runtime_error("cannot read STEP");const auto shape=reader.OneShape();if(shape.IsNull()||!BRepTools::Write(shape,argv[3]))throw std::runtime_error("cannot freeze BREP");return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
    }
    if(argc<5){std::cerr<<"usage: generate CAD.brep h extension output-dir | audit CAD.brep mesh.vtp output.json\n";return 2;}
    try {
        const auto start=std::chrono::steady_clock::now();
        BRep_Builder builder;TopoDS_Shape shape;if(!BRepTools::Read(shape,argv[2],builder)||shape.IsNull())throw std::runtime_error("cannot read BREP");
        OccAlgorithm6Adapter adapter(shape);
        if(std::string(argv[1])=="audit") {
            const auto loaded=std::chrono::steady_clock::now();const auto mesh=readVtp(argv[3]);const auto auditStart=std::chrono::steady_clock::now();
            const fs::path output(argv[4]);
            auto saveTiming=[&](bool complete){const auto end=std::chrono::steady_clock::now();std::ofstream timing(output.parent_path()/(output.stem().string()+"-timing.json"));timing<<std::setprecision(17)<<"{\"complete\":"<<(complete?"true":"false")<<",\"import_seconds\":"<<std::chrono::duration<double>(loaded-start).count()<<",\"mesh_read_seconds\":"<<std::chrono::duration<double>(auditStart-loaded).count()<<",\"audit_seconds\":"<<std::chrono::duration<double>(end-auditStart).count()<<"}\n";};
            try{audit(adapter.geometry(),mesh,argv[4]);}catch(...){saveTiming(false);throw;}
            saveTiming(true);return 0;
        }
        if(std::string(argv[1])=="diagnose-face"&&argc==7) {
            const int entity=std::stoi(argv[3]);SurfaceMeshingOptions options;options.sizeMax=std::stod(argv[4]);options.sizeFromPoints=false;options.extendBoundarySize=std::stoi(argv[5]);
            const fs::path directory(argv[6]);if(!fs::create_directory(directory))throw std::runtime_error("diagnostic directory must be new");
            const auto input=adapter.buildModelInput();const auto& face=input.faces.at(entity-1);BoundaryMesh boundary;
            const auto built=buildBoundaryMesh(input.boundary,options,input.sizes,boundary);if(!built.status.complete())throw OperationFailure{built.status};
            FaceChart chart;const auto chartResult=buildFaceChart(boundary,face.chart,chart);if(!chartResult.status.complete())throw OperationFailure{chartResult.status};
            std::ofstream out(directory/"chart.json");out<<std::setprecision(17)<<"{\"face\":"<<entity<<",\"body\":"<<face.bodyId<<",\"nodes\":[";
            for(std::size_t i=0;i<chart.vertices.size();++i){if(i)out<<',';const auto& p=chart.vertices[i];const auto& n=boundary.nodes.at(p.physicalId-1);out<<"{\"local\":"<<p.localId<<",\"physical\":"<<p.physicalId<<",\"dimension\":"<<n.dimension<<",\"entity\":"<<n.entity<<",\"parameter\":"<<n.parameter<<",\"uv\":["<<p.uv.x<<','<<p.uv.y<<"],\"xyz\":["<<n.point.x<<','<<n.point.y<<','<<n.point.z<<"]}";}
            out<<"],\"constraints\":[";for(std::size_t i=0;i<chart.constraints.size();++i){if(i)out<<',';const auto& c=chart.constraints[i];out<<'['<<c.curveEntity<<','<<c.from<<','<<c.to<<']';}out<<"]}\n";
            OrdinaryInitialTriangulation initial;const auto initialStatus=buildInitialTriangulation(chart,initial);if(!initialStatus.complete())throw OperationFailure{initialStatus};
            std::vector<InitialConstraint> constraints;const auto mapped=prepareInitialConstraints(chart,initial.mesh,constraints);if(!mapped.complete())throw OperationFailure{mapped};
            WalkTriangulation recovered;const auto recovery=recoverConstraintEdges(initial.mesh,constraints,recovered);
            std::ofstream initialReport(directory/"initial-recovery.json");initialReport<<"{\"status\":"<<int(recovery.status.code)<<",\"decisions\":[";
            for(std::size_t i=0;i<recovery.decisions.size();++i){if(i)initialReport<<',';const auto& d=recovery.decisions[i];initialReport<<"{\"curve\":"<<d.constraint.curveEntity<<",\"from\":"<<initial.mesh.vertices.at(d.constraint.fromVertex).physicalId<<",\"to\":"<<initial.mesh.vertices.at(d.constraint.toVertex).physicalId<<",\"result\":"<<d.nativeResult<<",\"edge\":"<<d.recoveredHalfEdge<<"}";}initialReport<<"]}\n";
            const auto generated=generateFaceMesh(boundary,face,input.boundary.modelCharacteristicLength,input.sizes,options);
            std::ofstream failed(directory/"face-result.json");failed<<"{\"status\":"<<int(generated.status.code)<<",\"stage\":"<<std::quoted(generated.failedStage)<<",\"message\":"<<std::quoted(generated.status.message)<<",\"unrecovered\":[";
            for(std::size_t i=0;i<generated.unrecovered.size();++i){if(i)failed<<',';const auto& e=generated.unrecovered[i];failed<<'['<<e.curveEntity<<','<<e.from<<','<<e.to<<']';}failed<<"]}\n";
            std::cout<<generated.failedStage<<": "<<generated.status.message<<'\n';return 0;
        }
        if(std::string(argv[1])!="generate"||argc<6)throw std::runtime_error("invalid benchmark arguments");
        bool stages=true,profile=false;double pointSize=0.,curvature=0.;
        for(int k=6;k<argc;++k){const std::string flag=argv[k];if(flag=="--no-stages")stages=false;else if(flag=="--profile-stages")profile=true;else if(flag=="--point-size"&&k+1<argc)pointSize=std::stod(argv[++k]);else if(flag=="--curvature"&&k+1<argc)curvature=std::stod(argv[++k]);else throw std::runtime_error("unknown benchmark option");}
        if(!std::isfinite(pointSize)||pointSize<0||!std::isfinite(curvature)||curvature<0)throw std::runtime_error("invalid size source option");
        SurfaceMeshingOptions options;options.sizeMax=std::stod(argv[3]);options.sizeFromPoints=false;options.switchOnFailure=false;options.extendBoundarySize=std::stoi(argv[4]);
        options.sizeFromPoints=pointSize>0;options.elementsPerCurvatureCircle=curvature;
        const fs::path directory(argv[5]);if(!fs::create_directory(directory))throw std::runtime_error("output directory must be new");
        std::map<int,double> pointSizes;if(pointSize>0)for(int i=1;i<=adapter.geometry().vertexCount();++i)pointSizes.emplace(i,pointSize);
        const auto input=adapter.buildModelInput(pointSizes);const auto loaded=std::chrono::steady_clock::now();
        std::map<std::pair<std::string,int>,FaceMesh> snapshots;BoundaryMesh sharedBoundary;
        SurfaceStageObserver observer;
        std::map<std::string,double> phaseSeconds;std::string phase="shared_boundary";auto phaseStart=loaded;double observerSeconds=0;
        if(stages||profile)observer=[&](const std::string& stage,int face,const BoundaryMesh& boundary,const FaceMesh* mesh){
            const auto now=std::chrono::steady_clock::now();if(profile)phaseSeconds[phase]+=std::chrono::duration<double>(now-phaseStart).count();
            if(stages){sharedBoundary=boundary;if(mesh)snapshots[{stage,face}]=*mesh;}
            phase=stage=="boundary"?"face_initialization":stage=="initial"?"front_and_transfer":stage=="pre_smoothing"?"smoothing_and_final_uv":"between_faces_and_assembly";
            phaseStart=std::chrono::steady_clock::now();observerSeconds+=std::chrono::duration<double>(phaseStart-now).count();
        };
        const auto result=generateSurfaceMesh(input,options,{},observer);
        const auto generated=std::chrono::steady_clock::now();
        if(profile) {
            phaseSeconds[phase]+=std::chrono::duration<double>(generated-phaseStart).count();
            std::ofstream phases(directory/"phase-profile.json");phases<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"observer_seconds\":"<<observerSeconds<<",\"generation_seconds\":"<<std::chrono::duration<double>(generated-loaded).count()<<",\"phases\":{";
            bool first=true;for(const auto& entry:phaseSeconds){if(!first)phases<<',';first=false;phases<<std::quoted(entry.first)<<':'<<entry.second;}phases<<"}}\n";
        }
        std::ofstream configuration(directory/"configuration.json");configuration<<std::setprecision(17)<<"{\"algorithm\":6,\"element_order\":1,\"threads\":1,\"size_min\":"<<options.sizeMin<<",\"size_max\":"<<options.sizeMax<<",\"size_factor\":"<<options.sizeFactor<<",\"size_from_points\":"<<(options.sizeFromPoints?"true":"false")<<",\"uniform_point_size\":"<<pointSize<<",\"curvature_circle_elements\":"<<curvature<<",\"boundary_extension\":"<<options.extendBoundarySize<<",\"smoothing\":"<<options.smoothingIterations<<",\"random_factor\":"<<options.initialRandomFactor<<",\"random_seed\":1,\"switch_on_failure\":false,\"maximum_recovery_retries\":"<<options.maximumRecoveryRetries<<",\"stages\":"<<(stages?"true":"false")<<",\"profile_stages\":"<<(profile?"true":"false")<<"}\n";
        std::ofstream report(directory/"generation.json");report<<std::setprecision(17)<<"{\"complete\":"<<(result.status.complete()?"true":"false")<<",\"status\":"<<int(result.status.code)<<",\"failed_face\":"<<result.failedFace<<",\"h\":"<<options.sizeMax<<",\"extension\":"<<options.extendBoundarySize<<",\"smoothing\":1,\"algorithm\":6,\"switch_on_failure\":false,\"import_seconds\":"<<std::chrono::duration<double>(loaded-start).count()<<",\"generation_seconds\":"<<std::chrono::duration<double>(generated-loaded).count()<<",\"inserted\":"<<result.statistics.inserted<<",\"queries\":"<<result.statistics.geometryEvaluations<<",\"boundary_retries\":"<<result.statistics.boundaryRetries<<"}\n";report.close();
        configuration.close();
        std::ofstream faceReport(directory/"faces.json");faceReport<<"[";
        for(std::size_t k=0;k<result.faces.size();++k){if(k)faceReport<<',';const auto& f=result.faces[k];faceReport<<"{\"face\":"<<f.face<<",\"body\":"<<f.bodyId<<",\"attempted\":"<<(f.attempted?"true":"false")<<",\"current\":"<<(f.current?"true":"false")<<",\"status\":"<<int(f.status.code)<<",\"failed_stage\":"<<std::quoted(f.failedStage)<<",\"inserted\":"<<f.statistics.inserted<<",\"queries\":"<<f.statistics.geometryEvaluations<<"}";}faceReport<<"]\n";faceReport.close();
        if(!result.status.complete()){std::ofstream failure(directory/"failure.txt");failure<<result.failedStage<<": "<<result.status.message;throw std::runtime_error(result.failedStage+": "+result.status.message);}
        const auto exportStart=std::chrono::steady_clock::now();
        for(const auto& entry:snapshots) {
            const auto f=std::find_if(input.faces.begin(),input.faces.end(),[&](const auto& face){return face.chart.entity==entry.first.second;});
            writeFaceVtp(entry.second,sharedBoundary,f->bodyId,directory/(entry.first.first+"-face"+std::to_string(entry.first.second)+".vtp"));
        }
        writeVtp(result.surface,directory/"own.vtp");
        // 共享1D离散单独输出；与阶段三角网格同一节点身份和坐标。
        SurfaceMeshResult boundaryOnly;boundaryOnly.mesh.vertices=result.surface.mesh.vertices;boundaryOnly.boundaryEdges=result.surface.boundaryEdges;writeVtp(boundaryOnly,directory/"boundary.vtp");
        const auto auditStart=std::chrono::steady_clock::now();
        auto saveTiming=[&](bool complete){const auto auditEnd=std::chrono::steady_clock::now();std::ofstream timing(directory/"timing.json");
            timing<<std::setprecision(17)<<"{\"complete\":"<<(complete?"true":"false")<<",\"import_seconds\":"<<std::chrono::duration<double>(loaded-start).count()<<",\"generation_seconds\":"<<std::chrono::duration<double>(generated-loaded).count()<<",\"export_seconds\":"<<std::chrono::duration<double>(auditStart-exportStart).count()<<",\"audit_seconds\":"<<std::chrono::duration<double>(auditEnd-auditStart).count()<<"}\n";};
        try{audit(adapter.geometry(),result.surface,directory/"audit.json");}catch(...){saveTiming(false);throw;}
        saveTiming(true);
        std::cout<<"triangles="<<result.surface.mesh.triangles.size()<<" inserted="<<result.statistics.inserted<<" generation_seconds="<<std::chrono::duration<double>(generated-loaded).count()<<'\n';return 0;
    }catch(const OperationFailure& f){std::cerr<<f.status.message<<'\n';return 1;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
     catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
}
