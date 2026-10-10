/** @file OccMeshVertexParameters.cpp
 * @brief 固定默认OCC原始参数查询与创建序拓扑；不实现候选选择或平滑策略。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "OccMeshVertexParameters.h"
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_Plane.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <algorithm>
#include <cmath>
#include <set>
using namespace mesher; using namespace mesher::gmshReference;
namespace {
struct OccParameterCurve {
    TopoDS_Edge edge;
    Handle(Geom_Curve) support;
    ParameterCurveGeometry metadata;
};
class OccParameterContext {
public:
    /// 为节点参数反求保存 OCC 实体索引和曲线查询环境。
    OccParameterContext(const OccCadGeometry& input,int face) : geometry(input),faceIndex(face) {
        TopTools_IndexedMapOfShape faces,vertices,edges;
        TopExp::MapShapes(input.acceptedShape(),TopAbs_FACE,faces);
        TopExp::MapShapes(input.acceptedShape(),TopAbs_VERTEX,vertices);
        TopExp::MapShapes(input.acceptedShape(),TopAbs_EDGE,edges);
        if(face < 0 || face >= faces.Extent()) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid OCC parameter face"}};
        surfaceFace = TopoDS::Face(faces(face+1));
        std::set<int> onFace;
        for(TopExp_Explorer use(surfaceFace,TopAbs_EDGE); use.More(); use.Next()) onFace.insert(edges.FindIndex(use.Current()));
        for(int i = 1; i <= edges.Extent(); ++i) {
            OccParameterCurve edge; edge.edge = TopoDS::Edge(edges(i));
            auto& m = edge.metadata;
            m.beginVertex = vertices.FindIndex(TopExp::FirstVertex(edge.edge));
            m.endVertex = vertices.FindIndex(TopExp::LastVertex(edge.edge));
            // OCC 边按实体索引顺序构造，顶点关联边列表保持该顺序。
            if(m.beginVertex > 0) incident[m.beginVertex].push_back(i);
            if(m.endVertex > 0 && m.endVertex != m.beginVertex) incident[m.endVertex].push_back(i);
            if(edge.edge.Orientation() == TopAbs_INTERNAL || edge.edge.Orientation() == TopAbs_EXTERNAL) edge.edge.Orientation(TopAbs_FORWARD);
            edge.support = BRep_Tool::Curve(edge.edge,m.first,m.last);
            m.seam = BRep_Tool::IsClosed(edge.edge,surfaceFace); m.onFace = onFace.count(i) != 0;
            curves.emplace(i,std::move(edge));
        }
    }
    /// 将三维点反求为面参数。
    Point2D surfaceParameter(Point3D p) const {
        Point2D uv; Point3D projected;
        if(!geometry.projectOnFaceSurface(faceIndex,p,uv,projected)) throw OperationFailure{{MeshingStatus::NonConverged,"OCC direct surface projection failed; native generic inverse fallback remains unimplemented"}};
        return uv;
    }
    /// 计算 CAD 边在指定曲线参数处的三维位置。
    Point3D edgePoint(int entity,double t) const {
        const auto& edge = curves.at(entity);
        if(!edge.support.IsNull()) { const auto p = edge.support->Value(t); return {p.X(),p.Y(),p.Z()}; }
        if(BRep_Tool::Degenerated(edge.edge) && edge.metadata.beginVertex > 0) return geometry.vertexPoint(edge.metadata.beginVertex-1);
        throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"OCC edge has neither support curve nor degenerate endpoint"}};
    }
    /// 查询边参数在指定面方向分支上对应的 UV 坐标。
    Point2D edgeParameter(int entity,double t,int direction) const {
        auto edge = curves.at(entity).edge;
        if(direction != 1) edge.Reverse();
        double first,last; const auto pc = BRep_Tool::CurveOnSurface(edge,surfaceFace,first,last);
        if(pc.IsNull()) return surfaceParameter(edgePoint(entity,t)); // 原版最近点查询沿用已经成功的直接投影器。
        const auto uv = pc->Value(t); return {uv.X(),uv.Y()};
    }
    /// 通过关联边端点查询 CAD 顶点在当前面上的 UV，必要时投影反求。
    Point2D vertexParameter(int entity,int direction) const {
        const auto found = incident.find(entity);
        if(found != incident.end()) for(int curve : found->second) {
            const auto& edge = curves.at(curve); if(!edge.metadata.onFace) continue;
            double first,last; const auto pc = BRep_Tool::CurveOnSurface(edge.edge,surfaceFace,first,last);
            if(pc.IsNull()) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"OCC vertex incident face edge has no endpoint pcurve range"}};
            if(edge.metadata.beginVertex == entity) return edgeParameter(curve,first,direction);
            if(edge.metadata.endVertex == entity) return edgeParameter(curve,last,direction);
        }
        return surfaceParameter(geometry.vertexPoint(entity-1));
    }
    /// 使用曲线投影反求三维点在指定 CAD 边上的参数。
    double edgeInverse(int entity,Point3D point) {
        const auto& edge = curves.at(entity);
        if(edge.support.IsNull()) throw OperationFailure{{MeshingStatus::NonConverged,"OCC curve direct projector unavailable; native generic inverse fallback remains unimplemented"}};
        auto found = projectors.find(entity);
        if(found == projectors.end()) {
            double low = edge.metadata.first,high = edge.metadata.last;
            if(edge.metadata.beginVertex != edge.metadata.endVertex) {
                const double tolerance = std::max(std::abs(high-low)*1.e-8,1.e-12); low -= tolerance; high += tolerance;
            }
            auto prepared = std::make_unique<GeomAPI_ProjectPointOnCurve>(); prepared->Init(edge.support,low,high);
            found = projectors.emplace(entity,std::move(prepared)).first;
        }
        auto& projector = *found->second; projector.Perform(gp_Pnt(point.x,point.y,point.z));
        if(!projector.NbPoints()) throw OperationFailure{{MeshingStatus::NonConverged,"OCC direct curve projection failed; native generic inverse fallback remains unimplemented"}};
        return projector.LowerDistanceParameter();
    }
    const OccCadGeometry& geometry; int faceIndex;
    TopoDS_Face surfaceFace;
    std::map<int,OccParameterCurve> curves;
    std::map<int,std::vector<int>> incident;
    std::map<int,std::unique_ptr<GeomAPI_ProjectPointOnCurve>> projectors;
};
}
/// 将 OCC 实体查询封装为网格节点参数适配接口。
MeshVertexParameterSource makeOccMeshVertexParameterSource(const OccCadGeometry& geometry,int face) {
    auto context = std::make_shared<OccParameterContext>(geometry,face);
    MeshVertexParameterSource source; source.face = face+1;
    const auto surface = BRep_Tool::Surface(context->surfaceFace); source.plane = !surface.IsNull() && surface->DynamicType() == STANDARD_TYPE(Geom_Plane);
    source.vertexIncidentCurves = context->incident;
    for(const auto& entry : context->curves) source.curves.emplace(entry.first,entry.second.metadata);
    source.vertexCoordinates = [context](int entity,int direction) { return context->vertexParameter(entity,direction); };
    source.edgeCoordinates = [context](int entity,double t,int direction) { return context->edgeParameter(entity,t,direction); };
    source.edgeParameterFromPoint = [context](int entity,Point3D p) { return context->edgeInverse(entity,p); };
    source.surfaceCoordinates = [context](Point3D p,bool) { return context->surfaceParameter(p); };
    return source;
}
/// 将 OCC 曲面求值和节点反求封装为平滑所需接口。
SurfaceSmoothingSource makeOccSurfaceSmoothingSource(const OccCadGeometry& geometry,int face) {
    const auto parameters = makeOccMeshVertexParameterSource(geometry,face);
    auto input = [](const FaceVertex& vertex) {
        MeshVertexParameterInput result; result.vertex = vertex;
        result.hasParameters = {vertex.dimension >= 1,vertex.dimension == 2}; return result;
    };
    SurfaceSmoothingSource source;
    source.metric = [&geometry,face](Point2D uv) {
        Point3D du,dv; geometry.faceSurfaceDerivatives(face,uv,du,dv);
        const auto result = evaluateSurfaceMetric(du,dv); if(!result.status.complete()) throw OperationFailure{result.status}; return result.metric;
    };
    source.surfacePoint = [&geometry,face](Point2D uv) { return SmoothingSurfacePoint{geometry.faceSurfacePoint(face,uv),true}; };
    source.reparameterizeVertex = [parameters,input](const FaceVertex& vertex,const FaceMesh&) {
        const auto result = reparameterizeMeshVertexOnFace(input(vertex),parameters);
        if(!result.status.complete()) throw OperationFailure{result.status}; return result.parameter;
    };
    source.reparameterizeEdge = [parameters,input](const FaceVertex& first,const FaceVertex& second,const FaceMesh&) {
        const auto result = reparameterizeMeshEdgeOnFace(input(first),input(second),parameters);
        if(!result.status.complete()) throw OperationFailure{result.status}; return result.parameters;
    };
    return source;
}
