/** @file BdsMeshConversion.cpp
 * @brief 初始剖分到 BDS 的身份映射及顺序转换，不做几何合法化。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsMeshConversion.h"
#include "mesh/surface/algorithm6/data/HalfEdgeTopology.h"
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 检查网格转换是否被取消。
void checkRun(const RunControl& control) {
    if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"BDS conversion cancelled"}};
}
/// 检查输入是否适用于普通面 BDS 转换路径。
void checkRoute(FaceChartRoute route) {
    if(route != FaceChartRoute::General) throw OperationFailure{{MeshingStatus::InvalidInput,"BDS conversion requires General initial route"}};
}
/// 将图表节点坐标、物理身份和分类转换为 BDS 节点。
BdsPoint chartPoint(const FaceChartVertex& v) {
    BdsPoint point; point.id = v.localId; point.physicalId = v.physicalId;
    point.uv = v.uv; point.point = v.surfacePoint; point.classification = {v.entity,v.dimension};
    return point;
}
/// 校验转换后的 BDS 拓扑，成功且未取消后发布结果。
void verifyAndPublish(BdsMesh& mesh,BdsMesh& output,const RunControl& control) {
    const auto valid = validateBdsTopology(mesh,control); if(!valid.complete()) throw OperationFailure{valid};
    checkRun(control); output = std::move(mesh);
}
}
/// 将普通面初始三角剖分转换为带分类的 BDS 网格。
BdsConversionResult convertOrdinaryInitialMeshToBds(const FaceChart& chart,const WalkTriangulation& initial,
    const std::function<Point3D(Point2D)>& surfacePoint,BdsMesh& output,const RunControl& control)
{
    BdsConversionResult result;
    try {
        checkRun(control); checkRoute(chart.route);
        if(chart.entity <= 0 || !surfacePoint) throw OperationFailure{{MeshingStatus::InvalidInput,"missing ordinary BDS face geometry"}};
        const auto valid = validateHalfEdgeTopology(initial,control); if(!valid.complete()) throw OperationFailure{valid};
        if(initial.vertices.size() < 4) throw OperationFailure{{MeshingStatus::InvalidTopology,"ordinary BDS initial mesh lacks outer rectangle"}};
        if(chart.vertices.size() > control.maximumNodes || control.maximumNodes-chart.vertices.size() < 4)
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"ordinary BDS node budget exceeded"}};
        BdsMesh mesh; std::map<std::uint64_t,int> physicalToLocal;
        for(const auto& vertex : chart.vertices) {
            checkRun(control);
            if(vertex.localId < 0 || vertex.physicalId == 0) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid ordinary BDS chart identity"}};
            insertBdsPoint(mesh,chartPoint(vertex)); physicalToLocal[vertex.physicalId] = vertex.localId;
        }
        for(int i = 0; i < 4; ++i) {
            checkRun(control);
            if(result.geometryEvaluations >= control.maximumGeometryEvaluations) throw OperationFailure{{MeshingStatus::BudgetExceeded,"ordinary BDS geometry budget exceeded"}};
            BdsPoint p; p.id = -i-1; p.uv = initial.vertices[i].uv; p.classification = {chart.entity,2};
            ++result.geometryEvaluations;
            try { p.point = surfacePoint(p.uv); }
            catch(const OperationFailure&) { throw; }
            catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
            catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"unknown BDS outer surface query failure"}}; }
            if(!std::isfinite(p.point.x) || !std::isfinite(p.point.y) || !std::isfinite(p.point.z)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"non-finite BDS outer surface point"}};
            insertBdsPoint(mesh,std::move(p));
        }
        // BDS UV/XYZ来自独立reparam；PolyMesh只提供连接。域标签在BDS阶段重新建立。
        for(std::size_t i = 0; i < initial.faces.size(); ++i) {
            checkRun(control); std::array<int,3> nodes{}; bool found = true; int edge = initial.faces[i].halfEdge;
            for(int k = 0; k < 3; ++k) {
                const int origin = initial.halfEdges[edge].origin;
                if(origin < 4) nodes[k] = -origin-1;
                else {
                    const auto match = physicalToLocal.find(initial.vertices[origin].physicalId);
                    if(match == physicalToLocal.end()) found = false; else nodes[k] = match->second;
                }
                edge = initial.halfEdges[edge].next;
            }
            if(found) appendBdsTriangle(mesh,nodes); else result.skippedTriangles.push_back(int(i));
        }
        verifyAndPublish(mesh,output,control);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
/// 将周期初始剖分转换为 BDS 网格并保留分支身份。
BdsConversionResult convertPeriodicInitialMeshToBds(const InitialTriangulation& initial,BdsMesh& output,const RunControl& control)
{
    BdsConversionResult result;
    try {
        checkRun(control); checkRoute(initial.route);
        if(initial.vertices.size() > control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic BDS node budget exceeded"}};
        BdsMesh mesh; mesh.geometry.insert(initial.geometry.begin(),initial.geometry.end());
        for(const auto& vertex : initial.vertices) {
            checkRun(control); BdsPoint point = chartPoint(vertex.vertex); point.sizes = vertex.sizes;
            if(vertex.periodicCounterpart != -1) point.periodicCounterpart = vertex.periodicCounterpart;
            insertBdsPoint(mesh,std::move(point));
        }
        for(const auto& triangle : initial.triangles) { checkRun(control); appendBdsTriangle(mesh,triangle); }
        verifyAndPublish(mesh,output,control);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
