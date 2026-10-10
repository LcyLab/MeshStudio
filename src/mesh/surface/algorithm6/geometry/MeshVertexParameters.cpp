/** @file MeshVertexParameters.cpp
 * @brief 固定MVertex.cpp的参数分支和候选选择，不查询拓扑外的最近候选对。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "MeshVertexParameters.h"
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 将无效输入或拓扑条件报告为本模块的操作失败。
void invalid(const char* message) { throw OperationFailure{{MeshingStatus::InvalidInput,message}}; }
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); }
class ParameterQueries {
public:
    /// 保存节点参数查询的运行控制和共享查询计数。
    ParameterQueries(const RunControl& control,std::size_t& count) : control_(control),count_(count) {}
    /// 检查运行取消状态，取消时停止当前阶段。
    void check() const { if(control_.keepGoing && !control_.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"mesh vertex parameter query cancelled"}}; }
    /// 检查查询预算并调用参数回调，统一处理查询异常。
    template<class Function> auto request(Function function) {
        check();
        if(count_ >= control_.maximumGeometryEvaluations) throw OperationFailure{{MeshingStatus::BudgetExceeded,"mesh vertex parameter query budget exceeded"}};
        ++count_;
        try { return function(); }
        catch(const OperationFailure&) { throw; }
        catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
        catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"mesh vertex parameter geometry query failed"}}; }
    }
    /// 执行面参数查询回调，并检查返回的 UV 是否有限。
    Point2D uv(const std::function<Point2D()>& function) {
        const auto result = request(function);
        if(!finite(result)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite mesh vertex parameter"}};
        return result;
    }
private:
    const RunControl& control_; std::size_t& count_;
};
/// 按实体身份查找参数查询所需的 CAD 曲线。
const ParameterCurveGeometry& curve(const MeshVertexParameterSource& source,int entity) {
    const auto found = source.curves.find(entity);
    if(found == source.curves.end()) throw OperationFailure{{MeshingStatus::InvalidTopology,"mesh vertex incident curve metadata missing"}};
    return found->second;
}
/// 查找 CAD 顶点关联的边记录。
const std::vector<int>& incident(const MeshVertexParameterSource& source,int entity) {
    static const std::vector<int> empty;
    const auto found = source.vertexIncidentCurves.find(entity);
    return found == source.vertexIncidentCurves.end() ? empty : found->second;
}
/// 将三维点反求为面参数。
Point2D surfaceParameter(const MeshVertexParameterSource& source,Point3D p,bool onSurface,ParameterQueries& queries) {
    if(!source.surfaceCoordinates) invalid("mesh vertex surface inverse source missing");
    return queries.uv([&] { return source.surfaceCoordinates(p,onSurface); });
}
/// 查询曲线参数在指定面分支上对应的 UV 坐标。
Point2D edgeParameter(const MeshVertexParameterSource& source,int entity,double t,int direction,ParameterQueries& queries) {
    if(!source.edgeCoordinates) invalid("mesh vertex edge reparameterization source missing");
    return queries.uv([&] { return source.edgeCoordinates(entity,t,direction); });
}
/// 查询 CAD 顶点在指定面分支上的 UV 坐标。
Point2D vertexParameter(const MeshVertexParameterSource& source,int entity,int direction,ParameterQueries& queries) {
    if(!source.vertexCoordinates) invalid("mesh vertex CAD vertex reparameterization source missing");
    return queries.uv([&] { return source.vertexCoordinates(entity,direction); });
}
/// 校验节点身份、坐标及所属 CAD 实体。
void validate(const MeshVertexParameterInput& input,const MeshVertexParameterSource& source) {
    if(source.face < 0 || !input.classified || input.vertex.dimension < 0 || input.vertex.entity < 0) invalid("mesh vertex is not classified on a valid entity");
}
/// 按节点分类收集可用于反求的面参数候选。
MeshVertexParametersResult collect(const MeshVertexParameterInput& input,const MeshVertexParameterSource& source,ParameterQueries& queries) {
    queries.check(); MeshVertexParametersResult result; const auto& v = input.vertex;
    if(source.discreteSurface) { result.parameters.push_back(surfaceParameter(source,v.point,true,queries)); return result; }
    validate(input,source);
    if(v.dimension == 0) {
        for(int entity : incident(source,v.entity)) {
            queries.check(); const auto& edge = curve(source,entity);
            if(!edge.seam) continue;
            if(v.entity == edge.beginVertex) {
                result.parameters.push_back(edgeParameter(source,entity,edge.first,-1,queries));
                result.parameters.push_back(edgeParameter(source,entity,edge.first,1,queries));
            }
            if(v.entity == edge.endVertex) {
                result.parameters.push_back(edgeParameter(source,entity,edge.last,-1,queries));
                result.parameters.push_back(edgeParameter(source,entity,edge.last,1,queries));
            }
            // 原版即使输出异常提示，仍标记此边为接缝，但不为它保留参数候选。
            result.seamPresent = true;
        }
        if(!result.seamPresent) result.parameters.push_back(vertexParameter(source,v.entity,1,queries));
    }
    else if(v.dimension == 1) {
        const auto& edge = curve(source,v.entity);
        if(!edge.parametrized) return result;
        if(!input.hasParameters[0]) invalid("native edge parameter is missing; uninitialized input cannot be reproduced");
        double t = v.parameters.x;
        if(t == 0.) {
            if(!source.edgeParameterFromPoint) invalid("mesh vertex curve inverse source missing");
            t = queries.request([&] { return source.edgeParameterFromPoint(v.entity,v.point); });
        }
        if(!std::isfinite(t)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite mesh vertex edge parameter"}};
        result.parameters.push_back(edgeParameter(source,v.entity,t,1,queries));
        if(edge.seam) { result.seamPresent = true; result.parameters.push_back(edgeParameter(source,v.entity,t,-1,queries)); }
    }
    else if(v.dimension == 2 && v.entity == source.face && input.hasParameters[0] && input.hasParameters[1]) {
        if(!finite(v.parameters)) invalid("nonfinite stored mesh face parameters");
        result.parameters.push_back(v.parameters);
    }
    return result;
}
/// 计算两点之间的距离平方。
double squaredDistance(Point2D a,Point2D b) { return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y); }
}
/// 按 CAD 分类收集网格节点的可用面参数候选。
MeshVertexParametersResult collectMeshVertexParameters(const MeshVertexParameterInput& input,const MeshVertexParameterSource& source,const RunControl& control) {
    MeshVertexParametersResult result; ParameterQueries queries(control,result.geometryEvaluations);
    try {
        const auto collected = collect(input,source,queries); result.parameters = collected.parameters; result.seamPresent = collected.seamPresent;
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
/// 反求单个网格节点在目标面及接缝分支上的 UV。
MeshVertexParameterResult reparameterizeMeshVertexOnFace(const MeshVertexParameterInput& input,const MeshVertexParameterSource& source,const MeshVertexParameterOptions& options,const RunControl& control) {
    MeshVertexParameterResult result; ParameterQueries queries(control,result.geometryEvaluations);
    try {
        queries.check(); validate(input,source); const auto& v = input.vertex;
        if(input.kind == ParameterEntityKind::DiscreteCurve || input.kind == ParameterEntityKind::BoundaryLayerCurve) {
            result.parameter = surfaceParameter(source,v.point,options.onSurface,queries); result.succeeded = true; return result;
        }
        if(v.dimension == 0) {
            if(source.discreteSurface) { result.parameter = surfaceParameter(source,v.point,true,queries); result.succeeded = true; return result; }
            result.parameter = input.kind == ParameterEntityKind::GmshVertex && source.plane ? surfaceParameter(source,v.point,options.onSurface,queries) : vertexParameter(source,v.entity,options.direction,queries);
            if(options.failOnSeam) for(int entity : incident(source,v.entity)) { queries.check(); if(curve(source,entity).seam) return result; }
        }
        else if(v.dimension == 1) {
            if(source.discreteSurface) { result.parameter = surfaceParameter(source,v.point,true,queries); result.succeeded = true; return result; }
            if(!input.hasParameters[0]) invalid("native edge parameter is missing; uninitialized input cannot be reproduced");
            result.parameter = edgeParameter(source,v.entity,v.parameters.x,options.direction,queries);
            if(options.failOnSeam && curve(source,v.entity).seam) return result;
        }
        else if(v.dimension == 2 && v.entity == source.face && input.hasParameters[0] && input.hasParameters[1]) {
            if(!finite(v.parameters)) invalid("nonfinite stored mesh face parameters"); result.parameter = v.parameters;
        }
        else result.parameter = surfaceParameter(source,v.point,options.onSurface,queries);
        result.succeeded = true;
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
/// 联合选择边两端的面参数分支。
MeshEdgeParameterResult reparameterizeMeshEdgeOnFace(const MeshVertexParameterInput& first,const MeshVertexParameterInput& second,const MeshVertexParameterSource& source,const RunControl& control) {
    MeshEdgeParameterResult result; ParameterQueries queries(control,result.geometryEvaluations);
    try {
        const auto p1 = collect(first,source,queries).parameters,p2 = collect(second,source,queries).parameters;
        if(!p1.empty() && !p2.empty()) {
            std::size_t i = 0,j = 0;
            double distance = squaredDistance(p2[0],p1[0]);
            for(std::size_t k = 0; k < p2.size(); ++k) { queries.check(); const double next = squaredDistance(p2[k],p1[0]); if(next < distance) { i = k; distance = next; } }
            distance = squaredDistance(p2[0],p1[0]);
            for(std::size_t k = 0; k < p1.size(); ++k) { queries.check(); const double next = squaredDistance(p2[0],p1[k]); if(next < distance) { j = k; distance = next; } }
            result.parameters = {p1[j],p2[i]}; result.selectedCandidates = {j,i};
        }
        else {
            result.usedSurfaceFallback = true;
            result.parameters[0] = surfaceParameter(source,first.vertex.point,true,queries);
            result.parameters[1] = surfaceParameter(source,second.vertex.point,true,queries);
        }
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
