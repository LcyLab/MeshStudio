/** @file MeshAssembly.cpp
 * @brief 装配普通/周期面最终身份与逐角UV，独立检查基础拓扑。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "MeshAssembly.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <sstream>
#include <iomanip>
#include <limits>
namespace mesher::gmshReference {
/// 按部件及 CAD 边界身份统一节点，装配各面并校验连接和方向。
OperationStatus assembleSurfaceMesh(const BoundaryMesh& boundary,const std::vector<AssemblyFace>& faces,
    SurfaceMeshResult& output,const RunControl& control,int* failedFace) {
    if(failedFace)*failedFace=-1;
    int currentFace=-1;std::size_t currentTriangle=std::numeric_limits<std::size_t>::max();
    const auto failure=[&](OperationStatus status) {
        if(failedFace)*failedFace=currentFace;
        if(currentFace>=0) {
            std::string context="face="+std::to_string(currentFace);
            if(currentTriangle!=std::numeric_limits<std::size_t>::max())context+=" triangle="+std::to_string(currentTriangle);
            status.message=context+": "+status.message;
        }
        return status;
    };
    try {
        auto check=[&] {if(control.keepGoing&&!control.keepGoing())throw OperationFailure{{MeshingStatus::Cancelled,"assembly cancelled"}};};
        const auto finite=[](Point3D p) {return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
        // 在临时结果中统一节点和标签；连接/方向检查未通过前不覆盖调用方输出。
        SurfaceMeshResult result;
        using Key=std::tuple<int,int,std::uint64_t>;
        std::map<Key,int> ids;
        std::set<std::array<int,3>> triangles;
        std::map<std::array<int,2>,int> cadEdges;
        struct EdgeIncidence {int count=0,direction=0,firstFace=-1;std::size_t firstTriangle=0;};
        std::map<std::array<int,2>,EdgeIncidence> assembledEdges;
        std::size_t queries=0;
        for(const auto& face:faces) {
            currentFace=face.mesh?face.mesh->entity:-1;currentTriangle=std::numeric_limits<std::size_t>::max();
            check();if(!face.mesh||!face.orientedNormal)throw OperationFailure{{MeshingStatus::InvalidInput,"assembly missing face or normal callback"}};
            const auto& mesh=*face.mesh;
            auto index=[&](std::uint64_t id) {
                const auto& v=mesh.vertices.at(id);
                if(!finite(v.point))throw OperationFailure{{MeshingStatus::InvalidTopology,"nonfinite assembly vertex"}};
                // 不同部件分开；CAD 边界按物理身份共享，面内点则按面号和局部身份区分。
                const Key key{face.bodyId,v.dimension<=1?0:mesh.entity,v.dimension<=1?v.physicalId:id};
                const auto found=ids.find(key);if(found!=ids.end()) {
                    const auto p=result.mesh.vertices.at(found->second);
                    if(p.x!=v.point.x||p.y!=v.point.y||p.z!=v.point.z)throw OperationFailure{{MeshingStatus::InvalidTopology,"shared identity has inconsistent coordinates"}};
                    return found->second;
                }
                if(result.mesh.vertices.size()>=control.maximumNodes)throw OperationFailure{{MeshingStatus::BudgetExceeded,"assembly node budget"}};
                const int n=static_cast<int>(result.mesh.vertices.size());ids.emplace(key,n);result.mesh.vertices.push_back(v.point);return n;
            };
            std::map<std::array<std::uint64_t,2>,int> incidence;
            if(!mesh.triangleCornerUV.empty()&&mesh.triangleCornerUV.size()!=mesh.triangles.size())throw OperationFailure{{MeshingStatus::InvalidTopology,"incomplete per-corner UV"}};
            std::size_t triangleIndex=0;
            for(const auto& t:mesh.triangles) {
                currentTriangle=triangleIndex;
                check();std::array<int,3> tri;std::array<Point2D,3> uv;
                for(int k=0;k<3;++k) {
                    tri[k]=index(t[k]);const auto& v=mesh.vertices.at(t[k]);
                    uv[k]=!mesh.triangleCornerUV.empty()?mesh.triangleCornerUV[triangleIndex][k]:(v.dimension==2?v.parameters:mesh.parametricCoordinates.at(t[k]));
                    if(!std::isfinite(uv[k].x)||!std::isfinite(uv[k].y))throw OperationFailure{{MeshingStatus::InvalidTopology,"nonfinite corner UV"}};
                    auto edge=std::array<std::uint64_t,2>{t[k],t[(k+1)%3]};std::sort(edge.begin(),edge.end());++incidence[edge];
                }
                auto sorted=tri;std::sort(sorted.begin(),sorted.end());
                if(sorted[0]==sorted[1]||sorted[1]==sorted[2]||!triangles.insert(sorted).second)throw OperationFailure{{MeshingStatus::InvalidTopology,"duplicate or collapsed final triangle"}};
                const auto a=result.mesh.vertices[tri[0]],b=result.mesh.vertices[tri[1]],c=result.mesh.vertices[tri[2]];
                Point3D n{(b.y-a.y)*(c.z-a.z)-(b.z-a.z)*(c.y-a.y),(b.z-a.z)*(c.x-a.x)-(b.x-a.x)*(c.z-a.z),(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)};
                if(!finite(n)||!(std::hypot(n.x,n.y,n.z)>0)) {
                    std::ostringstream details;details<<std::setprecision(17)<<"zero-area final triangle; nodes="<<t[0]<<','<<t[1]<<','<<t[2]<<" xyz=";
                    for(const auto p:{a,b,c})details<<'['<<p.x<<','<<p.y<<','<<p.z<<']';
                    throw OperationFailure{{MeshingStatus::InvalidTopology,details.str()}};
                }
                if(queries>=control.maximumGeometryEvaluations)throw OperationFailure{{MeshingStatus::BudgetExceeded,"assembly normal-query budget"}};
                ++queries;const auto normal=face.orientedNormal({(uv[0].x+uv[1].x+uv[2].x)/3,(uv[0].y+uv[1].y+uv[2].y)/3});
                const double dot=n.x*normal.x+n.y*normal.y+n.z*normal.z;
                if(!finite(normal)||!std::isfinite(dot)||dot==0)throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"ambiguous CAD orientation"}};
                if(dot<0){std::swap(tri[1],tri[2]);std::swap(uv[1],uv[2]);}
                // 局部身份可能在跨面/周期物理身份合并后共用一条边；检查最终节点及最终绕向。
                for(int k=0;k<3;++k){
                    auto edge=std::array<int,2>{tri[k],tri[(k+1)%3]};
                    const int direction=edge[0]<edge[1]?1:-1;std::sort(edge.begin(),edge.end());
                    auto& use=assembledEdges[edge];
                    if(use.count==0){use.firstFace=mesh.entity;use.firstTriangle=triangleIndex;}
                    ++use.count;use.direction+=direction;
                    if(use.count>2||(use.count==2&&use.direction!=0)){
                        std::ostringstream details;details<<"invalid assembled edge nodes="<<edge[0]<<','<<edge[1]
                            <<" incidence="<<use.count<<" direction_sum="<<use.direction
                            <<" first_face="<<use.firstFace<<" first_triangle="<<use.firstTriangle;
                        throw OperationFailure{{MeshingStatus::InvalidTopology,details.str()}};
                    }
                }
                result.mesh.triangles.push_back(tri);result.triangleCornerUV.push_back(uv);
                result.triangleFaceIds.push_back(mesh.entity-1);result.triangleBodyIds.push_back(face.bodyId);
                ++triangleIndex;
            }
            currentTriangle=std::numeric_limits<std::size_t>::max();
            std::set<std::array<std::uint64_t,2>> boundarySegments;
            for(const auto& curve:boundary.curves) {
                if(std::find(curve.incidentFaces.begin(),curve.incidentFaces.end(),mesh.entity)==curve.incidentFaces.end()||curve.meshDegenerate)continue;
                for(std::size_t k=1;k<curve.samples.size();++k) {
                    auto edge=std::array<std::uint64_t,2>{curve.samples[k-1].node,curve.samples[k].node};std::sort(edge.begin(),edge.end());
                    // 仅网格中存在的曲线才参与该面；embedded可有两侧关联。
                    const auto found=incidence.find(edge);
                    if(found==incidence.end())throw OperationFailure{{MeshingStatus::InvalidTopology,"CAD boundary/embedded segment missing from final mesh"}};
                    boundarySegments.insert(edge);
                    auto ij=std::array<int,2>{index(edge[0]),index(edge[1])};std::sort(ij.begin(),ij.end());cadEdges.emplace(ij,curve.entity-1);
                }
            }
            for(const auto& edge:incidence)if(edge.second>2||(edge.second==1&&!boundarySegments.count(edge.first)))throw OperationFailure{{MeshingStatus::InvalidTopology,"unexpected open or nonmanifold face edge"}};
        }
        for(const auto& edge:cadEdges)result.boundaryEdges.push_back({edge.first[0],edge.first[1],edge.second});
        if(result.mesh.triangles.empty())throw OperationFailure{{MeshingStatus::InvalidTopology,"no final surface triangles"}};
        check();result.ok=true;output=std::move(result);return {};
    } catch(const OperationFailure& f){return failure(f.status);}
      catch(const std::exception& e){return failure({MeshingStatus::GeometryQueryFailed,e.what()});}
      catch(...){return failure({MeshingStatus::GeometryQueryFailed,"non-standard CAD assembly callback failure"});}
}
}
