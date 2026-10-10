/** @file Algorithm6AcceptanceProbe.cpp
 * @brief 显式验收诊断：全量相交集合及真实BREP边界恢复，不参与生产生成。
 * @date 2026-10-09
 */
#include "Algorithm6SurfaceArtifacts.h"
#include "Algorithm6CadFixtures.h"
#include "cad/occ/OccAlgorithm6Adapter.h"
#include <BRepTools.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <gp_Circ.hxx>
#include <Standard_Failure.hxx>
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace fs=std::filesystem;
namespace {
void require(bool pass,const char* reason){if(!pass)throw std::runtime_error(reason);}
void selfTest(){
    SurfaceMesh3D mesh;
    mesh.vertices={{0,0,0},{2,0,0},{0,2,0},{.2,.2,0},{1,.2,0},{.2,1,0},
                   {4,0,0},{6,0,0},{4,2,0},{4.2,.2,0},{5,.2,0},{4.2,1,0}};
    mesh.triangles={{{0,1,2}},{{3,4,5}},{{6,7,8}},{{9,10,11}}};
    std::set<std::pair<int,int>> hits;std::string error;
    require(validateSurfaceIntersections(mesh,&error,0,nullptr,[&](int a,int b){hits.insert(std::minmax(a,b));return true;}),"enumeration traversal failed");
    require(hits==std::set<std::pair<int,int>>{{0,1},{2,3}},"all disjoint intersection groups must be collected");
    require(!validateSurfaceIntersections(mesh,&error),"original first-failure check must still reject");
    mesh.triangles={{{0,1,2}},{{6,7,8}}};hits.clear();
    require(validateSurfaceIntersections(mesh,&error,0,nullptr,[&](int a,int b){hits.insert(std::minmax(a,b));return true;})&&hits.empty(),"separated triangles must have no hits");
    std::cout<<"PASS all-hit traversal and original first-failure semantics\n";
}
TopoDS_Shape load(const char* path){TopoDS_Shape shape;BRep_Builder b;require(BRepTools::Read(shape,path,b)&&!shape.IsNull(),"cannot read BREP");return shape;}
void intersections(const char* cad,const char* vtp,const char* output){
    require(!fs::exists(output),"output must be new");
    OccCadGeometry geometry(load(cad),false);
    const auto mesh=algorithm6artifacts::readVtp(vtp);
    std::vector<std::pair<int,int>> errors,contacts;
    std::set<std::pair<int,int>> seen;
    std::string error;
    const bool scanned=validateSurfaceIntersections(mesh.mesh,&error,0,nullptr,[&](int a,int b){
        const auto key=std::make_pair(std::min(a,b),std::max(a,b));
        require(seen.insert(key).second,"duplicate pair in all-hit traversal");
        const int fa=mesh.triangleFaceIds.at(a),fb=mesh.triangleFaceIds.at(b);
        const int ba=geometry.faceBodyId(fa),bb=geometry.faceBodyId(fb);
        bool allowed=false;
        if(ba>=0&&bb>=0&&ba!=bb){const auto contact=geometry.bodyContact(fa,fb);
            require(contact!=CadBodyContact::QueryFailed,"CAD body contact query failed");
            allowed=contact==CadBodyContact::BoundaryContact||contact==CadBodyContact::VolumeOverlap;}
        (allowed?contacts:errors).push_back(key);
        // 仅诊断继续遍历；scanned表示扫描结束，不能表示网格无相交。
        return true;
    });
    if(!scanned)throw std::runtime_error(error);
    std::ofstream out(output);require(bool(out),"cannot write intersection report");
    out<<"{\"scan_complete\":true,\"intersection_free\":"<<(errors.empty()?"true":"false")<<",\"triangles\":"<<mesh.mesh.triangles.size()<<",\"all_geometric_hits\":"<<seen.size()<<",\"invalid_pairs\":[";
    for(std::size_t i=0;i<errors.size();++i){if(i)out<<',';out<<'['<<errors[i].first<<','<<errors[i].second<<']';}
    out<<"],\"accepted_cad_contact_pairs\":[";
    for(std::size_t i=0;i<contacts.size();++i){if(i)out<<',';out<<'['<<contacts[i].first<<','<<contacts[i].second<<']';}
    out<<"]}\n";out.close();require(bool(out),"intersection report write failed");
    std::cout<<"complete scan invalid_pairs="<<errors.size()<<" accepted_contacts="<<contacts.size()<<'\n';
}
void recoveryFixture(const char* output,double offset,double radius,double h,double angle,bool verifyRetry=false){
    const fs::path dir(output);require(fs::create_directory(dir),"fixture directory must be new");
    const gp_Ax2 axis(gp_Pnt(0,0,0),gp_Dir(0,0,1));
    auto outer=BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(axis,1)).Edge()).Wire();
    const gp_Ax2 innerAxis(gp_Pnt(offset*std::cos(angle),offset*std::sin(angle),0),gp_Dir(0,0,1));
    auto inner=BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(innerAxis,radius)).Edge()).Wire();
    inner.Reverse();BRepBuilderAPI_MakeFace face(gp_Pln(gp_Pnt(0,0,0),gp_Dir(0,0,1)),outer);face.Add(inner);
    const auto shape=face.Face();require(BRepCheck_Analyzer(shape).IsValid(),"fixture must be valid OCC CAD");
    const auto cad=dir/"model.brep";require(BRepTools::Write(shape,cad.string().c_str()),"BREP export failed");
    OccAlgorithm6Adapter adapter(load(cad.string().c_str()));
    SurfaceMeshingOptions options;options.sizeMax=h;options.sizeFromPoints=false;options.extendBoundarySize=1;options.switchOnFailure=false;
    RunControl control;control.maximumGeometryEvaluations=2000000;control.maximumNodes=20000;
    std::vector<std::size_t> counts;
    const auto result=generateSurfaceMesh(adapter.buildModelInput(),options,control,[&](const std::string& stage,int,const BoundaryMesh& boundary,const FaceMesh*){
        if(stage=="boundary")counts.push_back(boundary.nodes.size());});
    std::ofstream out(dir/"recovery.json");out<<"{\"cad_valid\":true,\"complete\":"<<(result.status.complete()?"true":"false")<<",\"status\":"<<int(result.status.code)<<",\"failed_face\":"<<result.failedFace<<",\"failed_stage\":"<<std::quoted(result.failedStage)<<",\"message\":"<<std::quoted(result.status.message)<<",\"boundary_retries\":"<<result.statistics.boundaryRetries<<",\"published_triangles\":"<<result.surface.mesh.triangles.size()<<",\"boundary_node_counts\":[";
    for(std::size_t i=0;i<counts.size();++i){if(i)out<<',';out<<counts[i];}out<<"]}\n";
    if(result.status.complete())algorithm6artifacts::writeVtp(result.surface,dir/"own.vtp");
    else require(!result.surface.ok&&result.surface.mesh.triangles.empty(),"failed recovery must not publish mesh");
    if(verifyRetry){
        require(result.status.complete()&&result.statistics.boundaryRetries>0&&counts.size()>1&&counts.back()>counts.front(),
                "valid BREP must naturally refine boundary and rebuild the full face");
        bool keep=true;int visits=0;RunControl cancellation=control;cancellation.keepGoing=[&]{return keep;};
        const auto cancelled=generateSurfaceMesh(adapter.buildModelInput(),options,cancellation,
            [&](const std::string& stage,int,const BoundaryMesh&,const FaceMesh*){if(stage=="boundary"&&++visits==2)keep=false;});
        require(cancelled.statistics.boundaryRetries>0&&cancelled.status.code==MeshingStatus::Cancelled&&
                !cancelled.surface.ok&&cancelled.surface.mesh.triangles.empty(),"cancel after natural refinement must publish no mesh");
        std::ofstream controls(dir/"retry-controls.json");controls<<"{\"verified\":true,\"natural_retry_complete\":true,\"cancel_after_refinement\":true,\"cancelled_boundary_retries\":"<<cancelled.statistics.boundaryRetries<<",\"published_cancelled_triangles\":0}\n";
    }
    std::cout<<"retries="<<result.statistics.boundaryRetries<<" status="<<int(result.status.code)<<" stage="<<result.failedStage<<'\n';
}
}
int main(int argc,char** argv){try{
    if(argc==2&&std::string(argv[1])=="self-test")selfTest();
    else if(argc==5&&std::string(argv[1])=="intersections")intersections(argv[2],argv[3],argv[4]);
    else if((argc==6||argc==7)&&std::string(argv[1])=="recovery-fixture")recoveryFixture(argv[2],std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]),argc==7?std::stod(argv[6]):0);
    else if(argc==3&&std::string(argv[1])=="recovery-test")recoveryFixture(argv[2],.9,.09,4,std::acos(-1.)/7,true);
    else return 2;return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<e.GetMessageString()<<'\n';return 1;}}
