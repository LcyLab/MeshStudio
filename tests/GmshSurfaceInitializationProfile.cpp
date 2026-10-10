/** @file GmshSurfaceInitializationProfile.cpp
 * @brief 普通面初始化计时、阶段捕获及原CAD查询一致性诊断；不作为正式计时或质量验收。
 * @date 2026-10-09
 */
#include "cad/occ/OccAlgorithm6Adapter.h"
#include "mesh/surface/algorithm6/initial/ConstraintRecovery.h"
#include "mesh/surface/algorithm6/initial/DomainClassification.h"
#include "mesh/surface/algorithm6/initial/InitialUvLegalization.h"
#include "mesh/surface/algorithm6/bds/BdsMeshConversion.h"
#include "mesh/surface/algorithm6/bds/BdsCurveRecovery.h"
#include "mesh/surface/algorithm6/bds/BdsDomainClassification.h"
#include "mesh/surface/algorithm6/bds/BdsNodeSizeInitialization.h"
#include "mesh/surface/algorithm6/bds/BdsInteriorPreparation.h"
#include "mesh/surface/algorithm6/postprocess/FaceMeshConversion.h"
#include "Algorithm6SurfaceArtifacts.h"
#include <BRepTools.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <Geom_Plane.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Ax3.hxx>
#include <BRep_Builder.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRepClass_Edge.hxx>
#include <BRepClass_FaceExplorer.hxx>
#include <gp_Lin2d.hxx>
#include <ShapeAnalysis.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <Standard_Failure.hxx>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <iomanip>
#include <iostream>
#include <set>
#include <tuple>
#include <utility>
using namespace mesher;
using namespace mesher::gmshReference;
namespace fs = std::filesystem;
namespace {
struct Measurement { std::size_t calls=0; double seconds=0; };
struct Timer {
    Measurement& measurement;
    std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();
    ~Timer(){++measurement.calls;measurement.seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();}
};
void require(OperationStatus status){if(!status.complete())throw OperationFailure{std::move(status)};}
template<class F> auto timed(Measurement& measurement,F source){return [&measurement,source](auto&&... args){Timer timer{measurement};return source(std::forward<decltype(args)>(args)...);};}
std::vector<BdsRecoveryCurve> curves(const BoundaryMesh& boundary,const std::vector<FaceCurveUse>& uses){
    std::vector<BdsRecoveryCurve> output;
    for(const auto& use:uses){
        const auto found=std::find_if(boundary.curves.begin(),boundary.curves.end(),[&](const auto& c){return c.entity==use.entity;});
        if(found==boundary.curves.end())throw std::runtime_error("missing diagnostic CAD curve");
        BdsRecoveryCurve c;c.entity=found->entity;c.meshDegenerated=found->meshDegenerate;c.endpoints={{found->beginNode,found->endNode}};
        for(std::size_t k=1;k<found->samples.size();++k)c.segments.push_back({found->samples[k-1].node,found->samples[k].node});
        output.push_back(std::move(c));
    }
    return output;
}
void sameInitial(const FaceMesh& a,const FaceMesh& b){
    if(a.entity!=b.entity||a.maximumVertexId!=b.maximumVertexId||a.vertices.size()!=b.vertices.size()||a.triangles!=b.triangles||a.bdsToVertex!=b.bdsToVertex||a.equivalence!=b.equivalence||a.interiorVertices!=b.interiorVertices||a.unreconstructedSeams!=b.unreconstructedSeams||a.parametricCoordinates.size()!=b.parametricCoordinates.size()||!a.triangleCornerUV.empty()||!b.triangleCornerUV.empty())throw std::runtime_error("diagnostic initialization does not match generator");
    for(const auto& entry:a.vertices){const auto& p=entry.second;const auto& q=b.vertices.at(entry.first);
        auto value=[](const FaceVertex& v){return std::make_tuple(v.id,v.physicalId,v.origin,v.dimension,v.entity,v.point.x,v.point.y,v.point.z,v.parameters.x,v.parameters.y,v.storedSize);};
        if(value(p)!=value(q))throw std::runtime_error("diagnostic vertex differs from generator");
    }
    for(const auto& entry:a.parametricCoordinates){const auto p=entry.second,q=b.parametricCoordinates.at(entry.first);if(p.x!=q.x||p.y!=q.y)throw std::runtime_error("diagnostic UV differs from generator");}
}
using WalkObserver=std::function<void(const std::string&,const WalkTriangulation&)>;
using BdsObserver=std::function<void(const std::string&,const BdsMesh&)>;
// 与旧分模块计时使用同一重放路径；观察不参与域/翻边或节点尺寸决策。
FaceMesh replayInitialization(const BoundaryMesh& boundary,Algorithm6FaceInput face,double length,
    const SurfaceMeshingOptions& options,const GeometrySizeSource& sizes,
    std::map<std::string,Measurement>& modules,std::map<std::string,Measurement>& callbacks,
    const WalkObserver& observeWalk={},const BdsObserver& observeBds={}){
    if(face.periodicChart)throw std::runtime_error("prefix replay supports ordinary general faces only");
    face.chart.nodeCoordinates=timed(callbacks["chart_parameters"],face.chart.nodeCoordinates);
    face.chart.surfacePoint=timed(callbacks["support_value"],face.chart.surfacePoint);
    face.initialMetric=timed(callbacks["initial_metric"],face.initialMetric);
    face.reparameterize=timed(callbacks["front_parameters"],face.reparameterize);
    FaceChart chart;{Timer timer{modules["chart"]};require(buildFaceChart(boundary,face.chart,chart).status);}
    if(chart.route!=FaceChartRoute::General)throw std::runtime_error("profile requires ordinary general chart");
    std::vector<FaceVertex> originals;for(const auto& n:boundary.nodes)if(n.alive){FaceVertex v;v.id=v.physicalId=n.id;v.point=n.point;v.dimension=n.dimension;v.entity=n.entity;v.parameters={n.parameter,0};v.storedSize=n.storedSize;originals.push_back(v);}
    OrdinaryInitialTriangulation initial;{Timer timer{modules["walk"]};require(buildInitialTriangulation(chart,initial));}
    if(observeWalk)observeWalk("walk",initial.mesh);
    std::vector<InitialConstraint> constraints;{Timer timer{modules["constraint_mapping"]};require(prepareInitialConstraints(chart,initial.mesh,constraints));}
    WalkTriangulation recovered;{Timer timer{modules["constraint_recovery"]};require(recoverConstraintEdges(initial.mesh,constraints,recovered).status);}
    if(observeWalk)observeWalk("constraint_recovery",recovered);
    WalkTriangulation classified;{Timer timer{modules["domain"]};require(classifyFaceDomain(recovered,chart.entity,classified).status);}
    if(observeWalk)observeWalk("domain",classified);
    WalkTriangulation legalized;{Timer timer{modules["uv_legalization"]};require(legalizeInitialUvMesh(classified,chart.entity,legalized).status);}
    if(observeWalk)observeWalk("uv_legalization",legalized);
    BdsMesh bds;{Timer timer{modules["bds_conversion"]};const auto r=convertOrdinaryInitialMeshToBds(chart,legalized,face.chart.surfacePoint,bds);require(r.status);if(!r.skippedTriangles.empty())throw std::runtime_error("diagnostic conversion skipped triangle");}
    if(observeBds)observeBds("bds_conversion",bds);
    const auto boundaryCurves=curves(boundary,face.chart.boundaryCurves),embedded=curves(boundary,face.chart.embeddedCurves);
    std::vector<BdsRecoveryConstraint> registry;{Timer timer{modules["bds_registry"]};require(buildOrdinaryBdsRecoveryRegistry(bds,boundaryCurves,embedded,registry));}
    BdsMesh boundaryRecovered;{Timer timer{modules["bds_boundary"]};const auto r=recoverOrdinaryBdsBoundary(bds,boundaryCurves,registry,boundaryRecovered);require(r.status);if(r.nativeFatal||r.needsRefinement||!r.skippedSegments.empty())throw std::runtime_error("profile requires boundary without retry");}
    if(observeBds)observeBds("bds_boundary",boundaryRecovered);
    {Timer timer{modules["bds_domain"]};require(classifyOrdinaryBdsDomain(boundaryRecovered,bds).status);}
    if(observeBds)observeBds("bds_domain",bds);
    BdsMesh embeddedRecovered;{Timer timer{modules["bds_embedded"]};const auto r=recoverOrdinaryBdsEmbedded(bds,embedded,registry,embeddedRecovered);require(r.status);if(r.hadFatalRecovery||!r.notRecovered.empty()||!r.skippedSegments.empty())throw std::runtime_error("profile embedded recovery failed");}
    if(observeBds)observeBds("bds_embedded",embeddedRecovered);
    BdsRecoveredVertexQueries queries;for(const auto& p:embeddedRecovered.points)if(p.second.physicalId){const auto& n=boundary.nodes.at(p.second.physicalId-1);queries.emplace(p.first,MeshSizeQuery{n.dimension,n.entity,n.parameter,0,n.point});}
    BdsMesh sized;{Timer timer{modules["bds_sizes"]};require(initializeOrdinaryBdsNodeSizes(embeddedRecovered,queries,length,options,sizes,sized).status);}
    if(observeBds)observeBds("bds_sizes",sized);
    BdsMesh prepared;{Timer timer{modules["bds_cleanup"]};require(prepareOrdinaryBdsInterior(sized,prepared).status);}
    if(observeBds)observeBds("bds_cleanup",prepared);
    {Timer timer{modules["metric_legalization"]};require(legalizeInitialMesh(prepared,face.initialMetric,bds).status);}
    if(observeBds)observeBds("metric_legalization",bds);
    FaceMesh mesh;mesh.entity=chart.entity;{Timer timer{modules["face_vertices"]};require(initializeFaceMeshVertices(bds,chart.entity,originals,mesh));}
    FaceMesh converted;{Timer timer{modules["face_conversion"]};require(convertBdsToFaceMesh(bds,mesh,converted).status);}mesh=std::move(converted);
    NodeSizeInitializationInput nodeInput;for(const auto& v:mesh.vertices)nodeInput.vertexOrder.push_back(v.first);
    for(const auto& c:boundaryCurves)nodeInput.boundaryCurves.push_back({c.meshDegenerated,c.segments});for(const auto& c:embedded)nodeInput.embeddedCurves.push_back({c.meshDegenerated,c.segments});
    for(auto id:face.chart.embeddedVertices)nodeInput.embeddedVertices.push_back({id,1.e22});
    NodeSizeState nodes;{Timer timer{modules["front_node_sizes"]};require(initializeNodeSizes(mesh,nodeInput,face.reparameterize,length,options,sizes,nodes).status);}
    for(const auto& n:nodes.nodes)mesh.parametricCoordinates[n.vertex]=n.parameters;
    return mesh;
}
void originalPointJson(std::ostream& out,const BoundaryMesh& boundary,std::uint64_t physical){
    if(!physical){out<<"null";return;}
    const auto& p=boundary.nodes.at(physical-1).point;out<<'['<<p.x<<','<<p.y<<','<<p.z<<']';
}
void writeWalkPrefix(const fs::path& path,const WalkTriangulation& mesh,const BoundaryMesh& boundary){
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write walk prefix");out<<std::setprecision(17)<<"{\"kind\":\"walk\",\"vertices\":[";
    for(std::size_t i=0;i<mesh.vertices.size();++i){if(i)out<<',';const auto& p=mesh.vertices[i];out<<"{\"id\":"<<i<<",\"physical\":"<<p.physicalId<<",\"uv\":["<<p.uv.x<<','<<p.uv.y<<"],\"xyz_original\":";originalPointJson(out,boundary,p.physicalId);out<<'}';}
    out<<"],\"triangles\":[";
    for(std::size_t i=0;i<mesh.faces.size();++i){if(i)out<<',';const auto& f=mesh.faces[i];int e=f.halfEdge;out<<"{\"id\":"<<i<<",\"domain\":"<<f.domain<<",\"nodes\":[";for(int k=0;k<3;++k){if(k)out<<',';out<<mesh.halfEdges.at(e).origin;e=mesh.halfEdges.at(e).next;}out<<"]}";}
    out<<"]}\n";
}
void writeBdsPrefix(const fs::path& path,const BdsMesh& mesh,const BoundaryMesh& boundary){
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write BDS prefix");out<<std::setprecision(17)<<"{\"kind\":\"bds\",\"vertices\":[";
    bool first=true;for(const auto& entry:mesh.points){if(!first)out<<',';first=false;const auto& p=entry.second;out<<"{\"id\":"<<entry.first<<",\"physical\":"<<p.physicalId<<",\"uv\":["<<p.uv.x<<','<<p.uv.y<<"],\"xyz_original\":";originalPointJson(out,boundary,p.physicalId);out<<'}';}
    out<<"],\"triangles\":[";
    for(std::size_t i=0;i<mesh.triangles.size();++i){if(i)out<<',';const auto& f=mesh.triangles[i];const auto n=getBdsTriangleNodes(mesh,int(i));out<<"{\"id\":"<<i<<",\"deleted\":"<<(f.deleted?"true":"false")<<",\"dimension\":"<<f.classification.dimension<<",\"origin\":"<<int(f.classification.origin)<<",\"nodes\":["<<n[0]<<','<<n[1]<<','<<n[2]<<"]}";}
    out<<"]}\n";
}
void writeCadBoundaryPrefix(const fs::path& path,const Algorithm6FaceInput& face,
    const Algorithm6ModelInput& model,const OccCadGeometry& geometry){
    TopTools_IndexedMapOfShape faces,edges;TopExp::MapShapes(geometry.acceptedShape(),TopAbs_FACE,faces);TopExp::MapShapes(geometry.acceptedShape(),TopAbs_EDGE,edges);
    const auto cadFace=TopoDS::Face(faces(face.chart.entity));
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write CAD boundary prefix");
    out<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"face\":"<<face.chart.entity<<",\"face_valid\":"<<(BRepCheck_Analyzer(cadFace).IsValid()?"true":"false")<<",\"face_tolerance\":"<<BRep_Tool::Tolerance(cadFace)<<",\"curves\":[";
    bool first=true;for(const auto& pc:face.trueBoundaryCurves){
        const auto found=std::find_if(model.boundary.curves.begin(),model.boundary.curves.end(),[&](const auto& c){return c.entity==pc.entity;});
        if(found==model.boundary.curves.end())throw std::runtime_error("pcurve has no physical CAD curve");
        const auto edge=TopoDS::Edge(edges(pc.entity));
        const bool degenerate=BRep_Tool::Degenerated(edge);std::optional<BRepAdaptor_Curve> directCurve;
        if(!degenerate)directCurve.emplace(edge);
        TopoDS_Vertex poleFirst,poleLast;if(degenerate){TopExp::Vertices(edge,poleFirst,poleLast);if(poleFirst.IsNull())throw std::runtime_error("degenerate CAD boundary has no vertex");}
        BRepAdaptor_Surface directSurface(cadFace,false);
        double directFirst=0,directLast=0;const auto directPC=BRep_Tool::CurveOnSurface(edge,cadFace,directFirst,directLast);
        if(directPC.IsNull())throw std::runtime_error("original CAD edge missing direct pcurve");
        ShapeAnalysis_Edge checker;double checkedDeviation=0,defaultDeviation=0;
        const bool checked=checker.CheckSameParameter(edge,cadFace,checkedDeviation,101);
        checker.CheckSameParameter(edge,cadFace,defaultDeviation,23);
        Geom2dAdaptor_Curve pcurveType(directPC);
        TopoDS_Vertex firstVertex,lastVertex;TopExp::Vertices(edge,firstVertex,lastVertex);
        if(firstVertex.IsNull()||lastVertex.IsNull())throw std::runtime_error("CAD edge diagnostic missing original endpoint");
        const auto surfaceAt=[&](double t){const auto uv=directPC->Value(t);return directSurface.Value(uv.X(),uv.Y());};
        const double vertexPcurveDeviation=std::max(BRep_Tool::Pnt(firstVertex).Distance(surfaceAt(directFirst)),
            BRep_Tool::Pnt(lastVertex).Distance(surfaceAt(directLast)));
        double vertexCurveDeviation=0,circleCoefficientDeviation=0;
        if(directCurve){
            vertexCurveDeviation=std::max(BRep_Tool::Pnt(firstVertex).Distance(directCurve->Value(directFirst)),
                BRep_Tool::Pnt(lastVertex).Distance(directCurve->Value(directLast)));
            if(directCurve->GetType()==GeomAbs_Circle&&pcurveType.GetType()==GeomAbs_Circle)
                for(const double phase:{0.,1.57079632679489661923,3.14159265358979323846}){
                    const auto uv=ElCLib::Value(phase,pcurveType.Circle());
                    circleCoefficientDeviation=std::max(circleCoefficientDeviation,
                        ElCLib::Value(phase,directCurve->Circle()).Distance(directSurface.Value(uv.X(),uv.Y())));
                }
        }
        // 周期图表回调在工厂调用后才存在；退化边没有曲线求值器，直接查原CAD顶点。
        const auto physicalPoint=[&](double t){if(found->point)return found->point(t);if(degenerate)return geometry.edgePoint(pc.entity-1,t);throw std::runtime_error("nondegenerate boundary missing curve callback");};
        const auto surfacePoint=[&](Point2D uv){return face.chart.surfacePoint?face.chart.surfacePoint(uv):geometry.faceSurfacePoint(face.chart.entity-1,uv);};
        const auto a=physicalPoint(found->first),b=physicalPoint(found->last);
        if(!first)out<<',';first=false;
        out<<"{\"edge\":"<<pc.entity<<",\"kind\":"<<int(found->kind)<<",\"edge_tolerance\":"<<BRep_Tool::Tolerance(edge)<<",\"same_parameter\":"<<(BRep_Tool::SameParameter(edge)?"true":"false")<<",\"same_range\":"<<(BRep_Tool::SameRange(edge)?"true":"false")<<",\"same_parameter_check_return\":"<<(checked?"true":"false")<<",\"same_parameter_check_maxdev\":"<<checkedDeviation<<",\"same_parameter_default23_maxdev\":"<<defaultDeviation<<",\"deviation_exceeds_edge_tolerance\":"<<(checkedDeviation>BRep_Tool::Tolerance(edge)?"true":"false")<<",\"pcurve_bounds\":["<<pc.first<<','<<pc.last<<"],\"physical_bounds\":["<<found->first<<','<<found->last<<"],\"analytic_consistency\":{";
        out<<"\"curve_type\":"<<(directCurve?int(directCurve->GetType()):-1)<<",\"pcurve_type\":"<<int(pcurveType.GetType())
           <<",\"vertex_pcurve_maxdev\":"<<vertexPcurveDeviation<<",\"vertex_curve_maxdev\":"<<vertexCurveDeviation
           <<",\"circle_coefficient_maxdev\":"<<circleCoefficientDeviation<<"},\"samples\":[";
        for(int i=0;i<=100;++i){
            if(i)out<<',';const double fraction=double(i)/100;
            const auto uv=pc.onFace(pc.first+fraction*(pc.last-pc.first),1);
            const auto support=surfacePoint(uv),physical=physicalPoint(found->first+fraction*(found->last-found->first));
            const auto directUV=directPC->Value(directFirst+fraction*(directLast-directFirst));
            const auto directSupport=directSurface.Value(directUV.X(),directUV.Y()),directPoint=directCurve?directCurve->Value(directCurve->FirstParameter()+fraction*(directCurve->LastParameter()-directCurve->FirstParameter())):BRep_Tool::Pnt(poleFirst);
            out<<"{\"fraction\":"<<fraction<<",\"pcurve_uv\":["<<uv.x<<','<<uv.y<<"],\"pcurve_xyz\":["<<support.x<<','<<support.y<<','<<support.z<<"],\"physical_xyz\":["<<physical.x<<','<<physical.y<<','<<physical.z<<"],\"paired_sample_distance\":"<<std::hypot(support.x-physical.x,support.y-physical.y,support.z-physical.z)<<",\"distance_to_physical_line_segment\":";
            if(found->kind==CurveKind::Line)out<<std::sqrt(algorithm6artifacts::segmentDistance(support,a,b));else out<<"null";
            out<<",\"direct_pcurve_pair_distance\":"<<directSupport.Distance(directPoint)
               <<",\"direct_surface_vs_callback\":"<<directSupport.Distance(gp_Pnt(support.x,support.y,support.z))
               <<",\"direct_curve_vs_callback\":"<<directPoint.Distance(gp_Pnt(physical.x,physical.y,physical.z));
            out<<'}';
        }
        out<<"]}";
    }
    out<<"]}\n";
}
const char* stateName(TopAbs_State state){
    switch(state){case TopAbs_IN:return "IN";case TopAbs_OUT:return "OUT";
        case TopAbs_ON:return "ON";default:return "UNKNOWN";}
}
// 同一原CAD、同一容差下交叉记录分类器；结果仅用于诊断，不改变生成或审计。
void writeClassifierPrefix(const fs::path& path,const FaceMesh& mesh,const TopoDS_Shape& shape){
    TopTools_IndexedMapOfShape faces,edges;TopExp::MapShapes(shape,TopAbs_FACE,faces);TopExp::MapShapes(shape,TopAbs_EDGE,edges);
    const auto face=TopoDS::Face(faces(mesh.entity));
    const auto forward=TopoDS::Face(face.Oriented(TopAbs_FORWARD));
    constexpr double tolerance=1.e-10;
    BRepTopAdaptor_FClass2d polygon(face,tolerance);
    const algorithm6audit::CadFaceQueries audited(face);
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write classifier prefix");
    out<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"face\":"<<mesh.entity
       <<",\"face_orientation\":"<<int(face.Orientation())<<",\"tolerance\":"<<tolerance<<",\"triangles\":[";
    for(std::size_t i=0;i<mesh.triangles.size();++i){
        Point2D uv{};Point3D xyz{};const auto triangle=mesh.triangles[i];std::array<Point2D,3> corners;
        for(int k=0;k<3;++k){const auto& vertex=mesh.vertices.at(triangle[k]);
            const auto found=mesh.parametricCoordinates.find(triangle[k]);
            const auto corner=mesh.triangleCornerUV.empty()?(found==mesh.parametricCoordinates.end()?vertex.parameters:found->second):mesh.triangleCornerUV.at(i)[k];
            corners[k]=corner;
            uv.x+=corner.x/3;uv.y+=corner.y/3;xyz.x+=vertex.point.x/3;xyz.y+=vertex.point.y/3;xyz.z+=vertex.point.z/3;
        }
        const gp_Pnt2d p(uv.x,uv.y);const gp_Pnt q(xyz.x,xyz.y,xyz.z);
        BRepClass_FaceClassifier standard(face,p,tolerance),physical(face,q,tolerance),
            bounded(face,p,tolerance,true),oriented(forward,p,tolerance);
        if(i)out<<',';
        out<<"{\"triangle\":"<<i<<",\"nodes\":["<<triangle[0]<<','<<triangle[1]<<','<<triangle[2]
           <<"],\"uv_centroid\":["<<uv.x<<','<<uv.y<<"],\"xyz_centroid\":["<<xyz.x<<','<<xyz.y<<','<<xyz.z
           <<"],\"standard_2d\":"<<std::quoted(stateName(standard.State()))
           <<",\"standard_3d\":"<<std::quoted(stateName(physical.State()))
           <<",\"bounded_2d\":"<<std::quoted(stateName(bounded.State()))
           <<",\"forward_2d\":"<<std::quoted(stateName(oriented.State()))
           <<",\"polygon_2d\":"<<std::quoted(stateName(polygon.Perform(p)));
        out<<",\"audit_query_exact_linear_plane\":"<<(audited.usesExactLinearPlane()?"true":"false")
           <<",\"audit_query_analytic_plane\":"<<(audited.usesAnalyticPlane()?"true":"false")
           <<",\"audit_query_state\":"<<std::quoted(stateName(audited.classify(p,tolerance)))
           <<",\"audit_query_distance\":"<<audited.distance(q);
        if(standard.State()==TopAbs_OUT){
            // 仅对失败片记录原CAD角点查询，定位前沿/平滑来源；不作网格修补。
            out<<",\"corner_queries\":[";
            for(int k=0;k<3;++k){
                if(k)out<<',';const auto point=mesh.vertices.at(triangle[k]).point;
                const gp_Pnt2d corner(corners[k].x,corners[k].y);const gp_Pnt original(point.x,point.y,point.z);
                out<<"{\"uv\":["<<corner.X()<<','<<corner.Y()<<"],\"xyz\":["<<original.X()<<','<<original.Y()<<','<<original.Z()
                   <<"],\"standard_2d\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,corner,tolerance).State()))
                   <<",\"standard_3d\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,original,tolerance).State()))
                   <<",\"polygon_2d\":"<<std::quoted(stateName(polygon.Perform(corner)))<<'}';
            }
            out<<']';
            BRepExtrema_DistShapeShape distance(BRepBuilderAPI_MakeVertex(q).Vertex(),face);
            if(!distance.IsDone())throw std::runtime_error("diagnostic point/face distance query failed");
            BRepAdaptor_Surface support(face,false);const auto projected=support.Value(uv.x,uv.y);
            out<<",\"physical_centroid_distance_to_face\":"<<distance.Value()
               <<",\"physical_centroid_support_residual\":"<<q.Distance(projected);
        }
        if(!standard.Rejected()&&!standard.NoWires()){
            const auto& edge=standard.Edge().Edge();double first=0,last=0;
            const auto curve=BRep_Tool::CurveOnSurface(edge,face,first,last);
            if(!curve.IsNull()){
                const double parameter=standard.EdgeParameter();const auto point=curve->Value(parameter);
                out<<",\"standard_edge\":"<<edges.FindIndex(edge)<<",\"standard_edge_parameter\":"<<parameter
                   <<",\"standard_edge_bounds\":["<<first<<','<<last<<"],\"parameter_outside_edge\":"<<(parameter<first||parameter>last?"true":"false")
                   <<",\"standard_edge_point_uv\":["<<point.X()<<','<<point.Y()<<']';
                BRepClass_FaceExplorer explorer(face);gp_Pnt2d checked=p;
                while(!explorer.CheckPoint(checked)){}
                gp_Lin2d ray;double length=0;
                if(explorer.Segment(checked,ray,length)){
                    out<<",\"first_ray_origin\":["<<ray.Location().X()<<','<<ray.Location().Y()
                       <<"],\"first_ray_direction\":["<<ray.Direction().X()<<','<<ray.Direction().Y()
                       <<"],\"first_ray_length\":"<<length<<",\"selected_edge_point_distance_to_first_ray\":"<<ray.Distance(point);
                }
            }
        }
        out<<'}';
    }
    out<<"]}\n";
}
void writeCoverageGrid(const fs::path& path,const TopoDS_Shape& shape,const SurfaceMeshResult& mesh){
    TopTools_IndexedMapOfShape faces;TopExp::MapShapes(shape,TopAbs_FACE,faces);
    std::vector<std::vector<std::size_t>> triangles(faces.Extent());
    for(std::size_t i=0;i<mesh.mesh.triangles.size();++i)triangles.at(mesh.triangleFaceIds.at(i)).push_back(i);
    struct Sample {int face,i,j;gp_Pnt2d uv;gp_Pnt point;double distance;TopAbs_State alternate;};
    std::vector<Sample> worst;std::size_t newlyInside=0,alternateOutside=0;
    for(int f=0;f<faces.Extent();++f){
        const auto face=TopoDS::Face(faces(f+1));const algorithm6audit::CadFaceQueries query(face);
        if(!query.usesExactLinearPlane())continue;
        double u0,u1,v0,v1;ShapeAnalysis::GetFaceUVBounds(face,u0,u1,v0,v1);
        BRepAdaptor_Surface surface(face,false);BRepTopAdaptor_FClass2d alternate(face,1.e-10);
        for(int i=0;i<=80;++i)for(int j=0;j<=80;++j){
            const gp_Pnt2d uv(u0+(u1-u0)*i/80.,v0+(v1-v0)*j/80.);
            const auto state=query.classify(uv);if(state!=TopAbs_IN&&state!=TopAbs_ON)continue;
            const auto raw=BRepClass_FaceClassifier(face,uv,1.e-10).State();if(raw==TopAbs_IN||raw==TopAbs_ON)continue;
            ++newlyInside;const auto other=alternate.Perform(uv);alternateOutside+=other!=TopAbs_IN&&other!=TopAbs_ON;
            const auto p=surface.Value(uv.X(),uv.Y());const Point3D point{p.X(),p.Y(),p.Z()};
            double best=std::numeric_limits<double>::infinity();
            for(auto k:triangles.at(f)){const auto t=mesh.mesh.triangles[k];best=std::min(best,algorithm6artifacts::triangleDistance(point,mesh.mesh.vertices.at(t[0]),mesh.mesh.vertices.at(t[1]),mesh.mesh.vertices.at(t[2])));}
            if(!std::isfinite(best))throw std::runtime_error("coverage diagnostic face has no triangles");
            worst.push_back({f,i,j,uv,p,std::sqrt(best),other});
            std::sort(worst.begin(),worst.end(),[](const auto& a,const auto& b){return a.distance>b.distance;});if(worst.size()>20)worst.resize(20);
        }
    }
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write coverage diagnostic");
    out<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"newly_inside_samples\":"<<newlyInside<<",\"alternate_outside\":"<<alternateOutside<<",\"worst\":[";
    for(std::size_t k=0;k<worst.size();++k){if(k)out<<',';const auto& s=worst[k];out<<"{\"face\":"<<s.face<<",\"grid\":["<<s.i<<','<<s.j<<"],\"uv\":["<<s.uv.X()<<','<<s.uv.Y()<<"],\"xyz\":["<<s.point.X()<<','<<s.point.Y()<<','<<s.point.Z()<<"],\"mesh_distance\":"<<s.distance<<",\"alternate_state\":"<<std::quoted(stateName(s.alternate))<<'}';}out<<"]}\n";
}
// 重放审计的七个物理样本，记录每面的最差位置；不改变查询策略或最终网格。
void writeDistancePeaks(const fs::path& path,const TopoDS_Shape& shape,const SurfaceMeshResult& mesh){
    TopTools_IndexedMapOfShape faces,edges;TopExp::MapShapes(shape,TopAbs_FACE,faces);TopExp::MapShapes(shape,TopAbs_EDGE,edges);
    struct Peak {int face=-1,sample=-1;std::size_t triangle=0;Point3D point{};Point2D uv{};double distance=-1;};
    std::vector<Peak> peaks(faces.Extent());
    std::vector<std::unique_ptr<algorithm6audit::CadFaceQueries>> queries;
    for(int f=1;f<=faces.Extent();++f)queries.emplace_back(std::make_unique<algorithm6audit::CadFaceQueries>(TopoDS::Face(faces(f))));
    std::size_t samples=0;
    for(std::size_t i=0;i<mesh.mesh.triangles.size();++i){
        const int f=mesh.triangleFaceIds.at(i);
        if(f<0||f>=faces.Extent())throw std::runtime_error("distance peak face outside CAD model");
        const auto t=mesh.mesh.triangles[i];
        const auto a=mesh.mesh.vertices.at(t[0]),b=mesh.mesh.vertices.at(t[1]),c=mesh.mesh.vertices.at(t[2]);
        const auto uv=mesh.triangleCornerUV.at(i);
        const std::array<Point3D,7> points{a,b,c,
            Point3D{(a.x+b.x)/2,(a.y+b.y)/2,(a.z+b.z)/2},
            Point3D{(b.x+c.x)/2,(b.y+c.y)/2,(b.z+c.z)/2},
            Point3D{(c.x+a.x)/2,(c.y+a.y)/2,(c.z+a.z)/2},
            Point3D{(a.x+b.x+c.x)/3,(a.y+b.y+c.y)/3,(a.z+b.z+c.z)/3}};
        const std::array<Point2D,7> parameters{uv[0],uv[1],uv[2],
            Point2D{(uv[0].x+uv[1].x)/2,(uv[0].y+uv[1].y)/2},
            Point2D{(uv[1].x+uv[2].x)/2,(uv[1].y+uv[2].y)/2},
            Point2D{(uv[2].x+uv[0].x)/2,(uv[2].y+uv[0].y)/2},
            Point2D{(uv[0].x+uv[1].x+uv[2].x)/3,(uv[0].y+uv[1].y+uv[2].y)/3}};
        for(int k=0;k<7;++k){
            const auto p=points[k];const double distance=queries.at(f)->distance(gp_Pnt(p.x,p.y,p.z));++samples;
            if(distance>peaks.at(f).distance)peaks.at(f)={f,k,i,p,parameters[k],distance};
        }
    }
    peaks.erase(std::remove_if(peaks.begin(),peaks.end(),[](const auto& p){return p.face<0;}),peaks.end());
    if(peaks.empty())throw std::runtime_error("distance peak mesh has no triangles");
    std::sort(peaks.begin(),peaks.end(),[](const auto& a,const auto& b){return a.distance>b.distance;});
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write distance peaks");
    out<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"sample_contract\":\"triangle 7 barycentric samples in audit order\",\"samples\":"<<samples
       <<",\"maximum\":"<<peaks.front().distance<<",\"sampled_faces\":"<<peaks.size()<<",\"worst_faces\":[";
    for(std::size_t i=0;i<std::min<std::size_t>(20,peaks.size());++i){
        const auto& p=peaks[i];const auto face=TopoDS::Face(faces(p.face+1));
        const gp_Pnt point(p.point.x,p.point.y,p.point.z);const gp_Pnt2d uv(p.uv.x,p.uv.y);
        BRepAdaptor_Surface surface(face,false);BRepTopAdaptor_FClass2d polygon(face,1.e-10);
        BRepExtrema_DistShapeShape raw(BRepBuilderAPI_MakeVertex(point).Vertex(),face);
        if(!raw.IsDone()||raw.NbSolution()<1)throw std::runtime_error("distance peak raw CAD query failed");
        if(i)out<<',';
        out<<"{\"face\":"<<p.face<<",\"triangle\":"<<p.triangle<<",\"sample\":"<<p.sample
           <<",\"body\":"<<mesh.triangleBodyIds.at(p.triangle)<<",\"xyz\":["<<point.X()<<','<<point.Y()<<','<<point.Z()
           <<"],\"interpolated_corner_uv\":["<<uv.X()<<','<<uv.Y()<<"],\"audit_distance\":"<<p.distance
           <<",\"raw_distance\":"<<raw.Value()<<",\"face_tolerance\":"<<BRep_Tool::Tolerance(face)<<",\"surface_type\":"<<int(surface.GetType())
           <<",\"exact_linear_plane\":"<<(queries.at(p.face)->usesExactLinearPlane()?"true":"false")
           <<",\"analytic_plane\":"<<(queries.at(p.face)->usesAnalyticPlane()?"true":"false")
           <<",\"corner_uv_support_residual\":"<<point.Distance(surface.Value(uv.X(),uv.Y()))
           <<",\"standard_2d\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,uv,1.e-10).State()))
           <<",\"standard_3d\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,point,1.e-10).State()))
           <<",\"bounded_2d\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,uv,1.e-10,true).State()))
           <<",\"polygon_2d\":"<<std::quoted(stateName(polygon.Perform(uv)));
        const auto closest=raw.PointOnShape2(1);
        out<<",\"raw_closest_xyz\":["<<closest.X()<<','<<closest.Y()<<','<<closest.Z()<<"],\"raw_support_type\":"<<int(raw.SupportTypeShape2(1));
        double edgeDistance=std::numeric_limits<double>::infinity();int closestEdge=-1;
        for(TopExp_Explorer edge(face,TopAbs_EDGE);edge.More();edge.Next()){
            BRepExtrema_DistShapeShape boundary(BRepBuilderAPI_MakeVertex(point).Vertex(),edge.Current());
            if(!boundary.IsDone())throw std::runtime_error("distance peak original edge query failed");
            if(boundary.Value()<edgeDistance){edgeDistance=boundary.Value();closestEdge=edges.FindIndex(edge.Current())-1;}
        }
        if(!std::isfinite(edgeDistance))throw std::runtime_error("distance peak face has no edge");
        out<<",\"original_face_edge_distance\":"<<edgeDistance<<",\"closest_original_edge\":"<<closestEdge;
        GeomAPI_ProjectPointOnSurf projection(point,BRep_Tool::Surface(face));
        if(projection.NbPoints()<1)throw std::runtime_error("distance peak support projection failed");
        double u=0,v=0;projection.LowerDistanceParameters(u,v);const gp_Pnt2d projected(u,v);
        out<<",\"support_projection_distance\":"<<projection.LowerDistance()<<",\"support_projection_uv\":["<<u<<','<<v
           <<"],\"projected_standard_2d\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,projected,1.e-10).State()))
           <<",\"projected_standard_at_face_tolerance\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,projected,BRep_Tool::Tolerance(face)).State()))
           <<",\"projected_bounded_2d\":"<<std::quoted(stateName(BRepClass_FaceClassifier(face,projected,1.e-10,true).State()))
           <<",\"projected_polygon_2d\":"<<std::quoted(stateName(polygon.Perform(projected)))<<'}';
    }
    out<<"]}\n";
}
}
int main(int argc,char** argv){
    if(argc==5&&std::string(argv[1])=="distance-peaks"){
        try{
            const fs::path output(argv[4]);if(fs::exists(output))throw std::runtime_error("distance peak output must be new");
            TopoDS_Shape shape;BRep_Builder builder;if(!BRepTools::Read(shape,argv[2],builder)||shape.IsNull())throw std::runtime_error("cannot read distance peak BREP");
            writeDistancePeaks(output,shape,algorithm6artifacts::readVtp(argv[3]));
            std::cout<<"actual audit distance peak locations exported; read-only diagnostic\n";return 0;
        }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
    }
    if(argc==5&&std::string(argv[1])=="cad-boundary"){
        try{
            const fs::path output(argv[4]);if(fs::exists(output))throw std::runtime_error("CAD boundary output must be new");
            TopoDS_Shape shape;BRep_Builder builder;if(!BRepTools::Read(shape,argv[2],builder)||shape.IsNull())throw std::runtime_error("cannot read CAD boundary BREP");
            OccAlgorithm6Adapter adapter(shape);const auto model=adapter.buildModelInput();const int entity=std::stoi(argv[3]);
            if(!shape.IsSame(adapter.geometry().acceptedShape())||adapter.geometry().inputWasReconciled())throw std::runtime_error("CAD consistency diagnostic requires unchanged original shape");
            const auto found=std::find_if(model.faces.begin(),model.faces.end(),[&](const auto& f){return f.chart.entity==entity;});
            if(found==model.faces.end())throw std::runtime_error("CAD boundary face outside model");
            writeCadBoundaryPrefix(output,*found,model,adapter.geometry());
            std::cout<<"original CAD boundary consistency exported; no generation or audit\n";return 0;
        }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
    }
    if(argc==5&&std::string(argv[1])=="coverage-grid"){
        try{
            const fs::path output(argv[4]);if(fs::exists(output))throw std::runtime_error("coverage output must be new");
            TopoDS_Shape shape;BRep_Builder builder;if(!BRepTools::Read(shape,argv[2],builder)||shape.IsNull())throw std::runtime_error("cannot read coverage BREP");
            writeCoverageGrid(output,shape,algorithm6artifacts::readVtp(argv[3]));
            std::cout<<"coverage grid disagreements exported; read-only diagnostic\n";return 0;
        }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
    }
    if(argc==3&&std::string(argv[1])=="repro-linear"){
        try{
            const fs::path output(argv[2]);if(fs::exists(output))throw std::runtime_error("reproduction output must be new");
            // 独立六边形，单位为模型单位；不依赖phone文件、参考网格或生成状态。
            BRepBuilderAPI_MakePolygon boundary;
            for(const auto p:{gp_Pnt(.0002,.00007,0),gp_Pnt(.0002,0,0),gp_Pnt(-.0012,0,0),
                gp_Pnt(-.00125,-.00008660254037841099,0),gp_Pnt(-.001310621778264936,-.00005160254037844890,0),
                gp_Pnt(-.0012404145188432314,.00007,0)})boundary.Add(p);
            boundary.Close();if(!boundary.IsDone())throw std::runtime_error("cannot construct reproduction boundary");
            Handle(Geom_Plane) plane=new Geom_Plane(gp_Ax3(gp_Pnt(0,0,0),gp_Dir(0,0,1),gp_Dir(1,0,0)));
            BRepBuilderAPI_MakeFace maker(plane,boundary.Wire(),true);
            if(!maker.IsDone()||!BRepCheck_Analyzer(maker.Face()).IsValid())throw std::runtime_error("invalid reproduction CAD");
            const Point2D center{-.0012503454323693857,.000006132486540509048};
            constexpr double delta=1.e-6;FaceMesh mesh;mesh.entity=1;
            const std::array<Point2D,3> uv{{{center.x-delta,center.y},{center.x+delta,center.y-delta},{center.x,center.y+delta}}};
            for(std::size_t i=0;i<uv.size();++i){FaceVertex v;v.id=i+1;v.point={uv[i].x,uv[i].y,0};mesh.vertices.emplace(v.id,v);}
            mesh.triangles.push_back({1,2,3});mesh.triangleCornerUV.push_back(uv);
            writeClassifierPrefix(output,mesh,maker.Face());
            std::cout<<"valid six-edge planar query reproduction exported; diagnostic only\n";return 0;
        }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
    }
    if(argc==6&&std::string(argv[1])=="classify-face"){
        try{
            const fs::path output(argv[5]);if(fs::exists(output))throw std::runtime_error("classifier output must be new");
            TopoDS_Shape shape;BRep_Builder builder;if(!BRepTools::Read(shape,argv[2],builder)||shape.IsNull())throw std::runtime_error("cannot read classifier BREP");
            const int entity=std::stoi(argv[3]);TopTools_IndexedMapOfShape faces;TopExp::MapShapes(shape,TopAbs_FACE,faces);
            if(entity<1||entity>faces.Extent())throw std::runtime_error("classifier face outside CAD model");
            const auto stage=algorithm6artifacts::readVtp(argv[4]);FaceMesh mesh;mesh.entity=entity;
            for(std::size_t i=0;i<stage.mesh.vertices.size();++i){FaceVertex vertex;vertex.id=i+1;vertex.point=stage.mesh.vertices[i];mesh.vertices.emplace(vertex.id,vertex);}
            for(std::size_t i=0;i<stage.mesh.triangles.size();++i){
                if(stage.triangleFaceIds.at(i)!=entity-1)throw std::runtime_error("classifier stage contains another CAD face");
                const auto t=stage.mesh.triangles[i];mesh.triangles.push_back({std::uint64_t(t[0]+1),std::uint64_t(t[1]+1),std::uint64_t(t[2]+1)});
            }
            mesh.triangleCornerUV=stage.triangleCornerUV;writeClassifierPrefix(output,mesh,shape);
            std::cout<<"classifier states exported; read-only diagnostic; no generation or audit performed\n";return 0;
        }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
    }
    if(argc>=7&&(std::string(argv[1])=="capture-faces"||std::string(argv[1])=="capture-prefixes")){
        try{
            const bool prefixes=std::string(argv[1])=="capture-prefixes";
            const fs::path directory(argv[5]);
            std::set<int> selected;
            for(int k=6;k<argc;++k)if(!selected.insert(std::stoi(argv[k])).second)throw std::runtime_error("duplicate selected face");
            SurfaceMeshingOptions options;options.sizeMax=std::stod(argv[3]);options.extendBoundarySize=std::stoi(argv[4]);options.sizeFromPoints=false;options.switchOnFailure=false;
            TopoDS_Shape shape;BRep_Builder builder;if(!BRepTools::Read(shape,argv[2],builder)||shape.IsNull())throw std::runtime_error("cannot read capture BREP");
            OccAlgorithm6Adapter adapter(shape);const auto input=adapter.buildModelInput();
            for(int face:selected)if(face<1||face>int(input.faces.size()))throw std::runtime_error("selected face outside CAD model");
            if(!fs::create_directory(directory))throw std::runtime_error("capture output must be new");
            std::map<std::pair<int,std::string>,int> attempts;
            std::map<int,FaceMesh> replayedInitial;std::map<int,int> prefixAttempts,prefixMatches;
            std::map<std::string,Measurement> modules,callbacks;
            // 运行完整模型，保留真实共享边、遍历序与模型扰动流；只导出指定面。
            const auto result=generateSurfaceMesh(input,options,{},[&](const std::string& stage,int face,const BoundaryMesh& boundary,const FaceMesh* mesh){
                if(!selected.count(face))return;
                const auto found=std::find_if(input.faces.begin(),input.faces.end(),[&](const auto& f){return f.chart.entity==face;});
                if(found==input.faces.end())throw std::runtime_error("observed face outside CAD model");
                if(prefixes&&stage=="boundary"){
                    const int attempt=++prefixAttempts[face];
                    const auto path=[&](const std::string& name){return directory/("prefix-"+name+"-face"+std::to_string(face)+"-attempt"+std::to_string(attempt)+".json");};
                    writeCadBoundaryPrefix(path("cad_boundary"),*found,input,adapter.geometry());
                    replayedInitial[face]=replayInitialization(boundary,*found,input.boundary.modelCharacteristicLength,options,input.sizes,modules,callbacks,
                        [&](const std::string& name,const WalkTriangulation& pool){writeWalkPrefix(path(name),pool,boundary);},
                        [&](const std::string& name,const BdsMesh& pool){writeBdsPrefix(path(name),pool,boundary);});
                }
                if(!mesh)return;
                if(prefixes&&stage=="initial"){
                    sameInitial(replayedInitial.at(face),*mesh);++prefixMatches[face];
                    writeClassifierPrefix(directory/("prefix-classifier-face"+std::to_string(face)+"-attempt"+std::to_string(prefixMatches[face])+".json"),*mesh,adapter.geometry().acceptedShape());
                }
                const int attempt=++attempts[{face,stage}];
                algorithm6artifacts::writeFaceVtp(*mesh,boundary,found->bodyId,directory/(stage+"-face"+std::to_string(face)+"-attempt"+std::to_string(attempt)+".vtp"));
            });
            std::ofstream out(directory/"capture.json");if(!out)throw std::runtime_error("cannot write capture report");
            out<<"{\"diagnostic_only\":true,\"generation_complete\":"<<(result.status.complete()?"true":"false")<<",\"status\":"<<int(result.status.code)<<",\"failed_face\":"<<result.failedFace<<",\"message\":"<<std::quoted(result.status.message)<<",\"captures\":[";
            bool first=true;for(const auto& entry:attempts){if(!first)out<<',';first=false;out<<"{\"face\":"<<entry.first.first<<",\"stage\":"<<std::quoted(entry.first.second)<<",\"attempts\":"<<entry.second<<'}';}out<<"],\"prefixes\":[";
            first=true;for(const auto& entry:prefixAttempts){if(!first)out<<',';first=false;out<<"{\"face\":"<<entry.first<<",\"attempts\":"<<entry.second<<",\"matched_initial\":"<<prefixMatches[entry.first]<<'}';}out<<"]}\n";out.close();
            require(result.status);
            for(int face:selected)for(const char* stage:{"initial","pre_smoothing","final"})if(!attempts.count({face,stage}))throw std::runtime_error("requested face stage was not captured");
            if(prefixes)for(int face:selected)if(!prefixAttempts[face]||prefixMatches[face]!=prefixAttempts[face])throw std::runtime_error("not every initialization prefix was verified");
            algorithm6artifacts::writeVtp(result.surface,directory/"own.vtp");
            std::cout<<"selected stages and full model exported; diagnostic only; no independent audit performed\n";return 0;
        }catch(const OperationFailure& e){std::cerr<<e.status.message<<'\n';return 1;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
    }
    if(argc!=6){std::cerr<<"usage: CAD.brep h extension new-output-directory repetitions\n";return 2;}
    try {
        const fs::path directory(argv[4]);if(!fs::create_directory(directory))throw std::runtime_error("diagnostic output must be new");
        const int repetitions=std::stoi(argv[5]);if(repetitions<1||repetitions>100)throw std::runtime_error("diagnostic repetitions must be 1..100");
        SurfaceMeshingOptions options;options.sizeMax=std::stod(argv[2]);options.extendBoundarySize=std::stoi(argv[3]);options.sizeFromPoints=false;options.switchOnFailure=false;
        std::map<std::string,Measurement> modules,callbacks;
        FaceMesh replayed;
        for(int repeat=0;repeat<repetitions;++repeat){
            // 独立适配器保持各轮CAD查询的首次初始化；不复用前一轮投影器。
            TopoDS_Shape shape;BRep_Builder builder;if(!BRepTools::Read(shape,argv[1],builder)||shape.IsNull())throw std::runtime_error("cannot read profile BREP");
            OccAlgorithm6Adapter adapter(shape);auto input=adapter.buildModelInput();
            if(input.faces.size()!=1||input.faces.front().periodicChart)throw std::runtime_error("profile supports one ordinary general face only");
            BoundaryMesh boundary;{Timer timer{modules["boundary"]};require(buildBoundaryMesh(input.boundary,options,input.sizes,boundary).status);}
            auto mesh=replayInitialization(boundary,input.faces.front(),input.boundary.modelCharacteristicLength,options,input.sizes,modules,callbacks);
            if(repeat) sameInitial(replayed,mesh);replayed=std::move(mesh);
        }
        TopoDS_Shape shape;BRep_Builder builder;if(!BRepTools::Read(shape,argv[1],builder))throw std::runtime_error("cannot read verification BREP");
        OccAlgorithm6Adapter adapter(shape);const auto input=adapter.buildModelInput();bool matched=false;
        const auto result=generateSurfaceMesh(input,options,{},[&](const std::string& stage,int,const BoundaryMesh&,const FaceMesh* mesh){if(stage=="initial"){sameInitial(replayed,*mesh);matched=true;}});
        require(result.status);if(!matched||result.statistics.boundaryRetries)throw std::runtime_error("initial comparison unavailable or generation retried");
        algorithm6artifacts::writeVtp(result.surface,directory/"own.vtp");
        std::ofstream out(directory/"initialization-profile.json");out<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"repetitions\":"<<repetitions<<",\"initial_matches_generator\":true,\"vertices\":"<<replayed.vertices.size()<<",\"triangles\":"<<replayed.triangles.size();
        for(const auto& group:{std::make_pair("modules",&modules),std::make_pair("callbacks_included_in_modules",&callbacks)}){out<<",\""<<group.first<<"\":{";bool first=true;for(const auto& item:*group.second){if(!first)out<<',';first=false;out<<std::quoted(item.first)<<":{\"calls\":"<<item.second.calls<<",\"total_seconds\":"<<item.second.seconds<<",\"seconds_per_repetition\":"<<item.second.seconds/repetitions<<'}';}out<<'}';}out<<"}\n";
        std::cout<<"initial pool matches generator; final mesh exported; diagnostic only\n";return 0;
    }catch(const OperationFailure& e){std::cerr<<e.status.message<<'\n';return 1;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}catch(const Standard_Failure& e){std::cerr<<"native OCC failure: "<<(e.GetMessageString()?e.GetMessageString():"no message")<<'\n';return 1;}
}
