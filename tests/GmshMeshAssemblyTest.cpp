/** @file GmshMeshAssemblyTest.cpp
 * @brief 最终共享身份合并后的整模型边连接、方向及失败原子性回归。
 * @date 2026-10-09
 */
#include "mesh/surface/algorithm6/pipeline/MeshAssembly.h"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace mesher;
using namespace mesher::gmshReference;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static FaceMesh patch(int entity,bool doubleTriangle){
    FaceMesh mesh;mesh.entity=entity;
    const double side=entity==1?1.:-1.;
    const std::vector<Point3D> points={{-1,0,0},{1,0,0},{0,side,doubleTriangle?side:0.},{0,-side,side}};
    for(std::size_t i=0;i<(doubleTriangle?4:3);++i){
        FaceVertex vertex;vertex.id=i+1;vertex.physicalId=i<2?i+1:10*entity+i+1;
        vertex.dimension=1;vertex.point=points[i];mesh.vertices.emplace(vertex.id,vertex);
        mesh.parametricCoordinates.emplace(vertex.id,Point2D{points[i].x,points[i].y});
    }
    mesh.triangles.push_back(entity==1?std::array<std::uint64_t,3>{1,2,3}:std::array<std::uint64_t,3>{2,1,3});
    if(doubleTriangle)mesh.triangles.push_back(entity==1?std::array<std::uint64_t,3>{2,1,4}:std::array<std::uint64_t,3>{1,2,4});
    return mesh;
}
static BoundaryMesh boundary(bool doubleTriangle){
    BoundaryMesh mesh;
    const auto add=[&](std::uint64_t a,std::uint64_t b,std::vector<int> faces){
        BoundaryCurve curve;curve.entity=int(mesh.curves.size())+1;
        curve.incidentFaces=std::move(faces);curve.samples={{a,0},{b,1}};
        mesh.curves.push_back(std::move(curve));
    };
    if(!doubleTriangle)add(1,2,{1,2});
    for(int face:{1,2}){
        add(2,3,{face});add(3,1,{face});
        if(doubleTriangle){add(1,4,{face});add(4,2,{face});}
    }
    return mesh;
}
static SurfaceMeshResult sentinel(){
    SurfaceMeshResult result;result.ok=true;result.error="preserved";
    result.mesh.vertices.push_back({42,-7,9});result.mesh.triangles.push_back({0,0,0});
    return result;
}
static void checkPreserved(const SurfaceMeshResult& result){
    check(result.ok&&result.error=="preserved"&&result.mesh.vertices.size()==1&&
          result.mesh.vertices[0].x==42&&result.mesh.triangles.size()==1,
          "assembly failure must preserve the caller's result");
}
int main(){
    try{
        const auto up=[](Point2D){return Point3D{0,0,1};};
        const auto down=[](Point2D){return Point3D{0,0,-1};};
        auto first=patch(1,false),second=patch(2,false);const auto borders=boundary(false);
        SurfaceMeshResult output;
        auto status=assembleSurfaceMesh(borders,{{&first,0,up},{&second,0,up}},output);
        check(status.complete()&&output.mesh.vertices.size()==4&&output.mesh.triangles.size()==2,
              "opposite edge directions on adjacent CAD faces must remain valid");
        auto preserved=sentinel();int failedFace=-1;
        first=patch(1,true);second=patch(2,true);const auto loops=boundary(true);
        SurfaceMeshResult individual;
        check(assembleSurfaceMesh(loops,{{&first,0,up}},individual).complete(),"first patch is individually manifold");
        check(assembleSurfaceMesh(loops,{{&second,0,down}},individual).complete(),"second patch is individually manifold");
        preserved=sentinel();failedFace=-1;
        status=assembleSurfaceMesh(loops,{{&first,0,up},{&second,0,down}},preserved,{},&failedFace);
        check(status.code==MeshingStatus::InvalidTopology&&failedFace==2&&
              status.message.find("assembled edge")!=std::string::npos,
              "four incidences after shared identity assembly must not publish a complete model");
        checkPreserved(preserved);
        check(assembleSurfaceMesh(loops,{{&first,0,up},{&second,1,down}},output).complete()&&
              output.mesh.vertices.size()==8,"different CAD bodies keep separate physical node identities");
        // 同一面内不同局部节点也可有相同物理身份；局部边计数不能替代合并后的检查。
        auto aliases=first;auto aliasLoops=loops;
        for(const auto& entry:second.vertices){auto vertex=entry.second;vertex.id+=4;aliases.vertices.emplace(vertex.id,vertex);}
        for(const auto& entry:second.parametricCoordinates)
            aliases.parametricCoordinates.emplace(entry.first+4,Point2D{entry.second.x+10,entry.second.y});
        for(auto tri:second.triangles){for(auto& node:tri)node+=4;aliases.triangles.push_back(tri);}
        for(auto& curve:aliasLoops.curves)if(curve.incidentFaces[0]==2){
            curve.incidentFaces[0]=1;for(auto& sample:curve.samples)sample.node+=4;
        }
        preserved=sentinel();failedFace=-1;
        status=assembleSurfaceMesh(aliasLoops,{{&aliases,0,[](Point2D uv){return Point3D{0,0,uv.x>5?-1.:1.};}}},
                                   preserved,{},&failedFace);
        check(status.code==MeshingStatus::InvalidTopology&&failedFace==1&&
              status.message.find("triangle=2")!=std::string::npos,
              "same-face physical aliases must reject the merged nonmanifold edge atomically");
        checkPreserved(preserved);
        first=patch(1,false);second=patch(2,false);preserved=sentinel();failedFace=-1;
        status=assembleSurfaceMesh(borders,{{&first,0,up},{&second,0,down}},preserved,{},&failedFace);
        check(status.code==MeshingStatus::InvalidTopology&&failedFace==2&&
              status.message.find("assembled edge")!=std::string::npos&&
              status.message.find("triangle=0")!=std::string::npos,
              "same directed assembled edge must fail with face and local triangle");
        checkPreserved(preserved);
        RunControl control;control.keepGoing=[] {return false;};preserved=sentinel();
        status=assembleSurfaceMesh(loops,{{&first,0,up},{&second,1,down}},preserved,control);
        check(status.code==MeshingStatus::Cancelled,"assembly cancellation remains explicit");checkPreserved(preserved);
        control={};control.maximumNodes=2;preserved=sentinel();
        status=assembleSurfaceMesh(loops,{{&first,0,up}},preserved,control);
        check(status.code==MeshingStatus::BudgetExceeded,"assembly node budget remains explicit");checkPreserved(preserved);
        std::cout<<"global assembled edge topology and atomic failures passed\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
