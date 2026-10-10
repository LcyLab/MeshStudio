/** @file OccAlgorithm6Adapter.cpp
 * @brief 独立建立OCC indexed身份和几何回调，保持原边参数与面参数分离。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "OccAlgorithm6Adapter.h"
#include "geometry/numerics/NativeSinCos.h"
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepLProp_CLProps.hxx>
#include <BRepLProp_SLProps.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_SphericalSurface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <algorithm>
#include <cmath>
#include <set>
#include <TopoDS_Wire.hxx>
using namespace mesher; using namespace mesher::gmshReference;
namespace {
/// 将几何点转换为算法使用的三维坐标类型。
Point3D point3(const gp_Pnt& p) {return {p.X(),p.Y(),p.Z()};}
/// 将边界节点身份和参数组织为反求输入。
MeshVertexParameterInput parameterInput(const FaceVertex& v) {
    MeshVertexParameterInput p;p.vertex=v;p.hasParameters={v.dimension>=1,v.dimension==2};return p;
}
#if defined(_MSC_VER) && defined(_M_X64)
/// 根据球面参数和模型位姿计算前沿候选的三维坐标。
Point3D sphereFrontPoint(const gp_Ax3& pose,double radius,Point2D uv) {
    const auto u=evaluateNativeSinCos(uv.x),v=evaluateNativeSinCos(uv.y);
    const double r=radius*v.cosine,a3=radius*v.sine,a1=r*u.cosine,a2=r*u.sine;
    const auto& x=pose.XDirection().XYZ();const auto& y=pose.YDirection().XYZ();
    const auto& z=pose.Direction().XYZ();const auto& p=pose.Location().XYZ();
    return {a1*x.X()+a2*y.X()+a3*z.X()+p.X(),a1*x.Y()+a2*y.Y()+a3*z.Y()+p.Y(),
            a1*x.Z()+a2*y.Z()+a3*z.Z()+p.Z()};
}
/// 计算球面的参数导数和局部度量。
SurfaceMetric sphereFrontMetric(const gp_Ax3& pose,double radius,Point2D uv) {
    const auto u=evaluateNativeSinCos(uv.x),v=evaluateNativeSinCos(uv.y);
    const double r1=radius*v.cosine,r2=radius*v.sine;
    const double a1=r1*u.cosine,a2=r1*u.sine,a3=r2*u.cosine,a4=r2*u.sine;
    const auto& x=pose.XDirection().XYZ();const auto& y=pose.YDirection().XYZ();const auto& z=pose.Direction().XYZ();
    const Point3D du{-a2*x.X()+a1*y.X(),-a2*x.Y()+a1*y.Y(),-a2*x.Z()+a1*y.Z()};
    const Point3D dv{-a3*x.X()-a4*y.X()+r1*z.X(),-a3*x.Y()-a4*y.Y()+r1*z.Y(),-a3*x.Z()-a4*y.Z()+r1*z.Z()};
    const auto evaluated=evaluateSurfaceMetric(du,dv);if(!evaluated.status.complete())throw OperationFailure{evaluated.status};
    return evaluated.metric;
}
#endif
}
/// 持有本次运行的 OCC 几何对象，供算法回调使用。
OccAlgorithm6Adapter::OccAlgorithm6Adapter(const TopoDS_Shape& shape):m_geometry(shape) {}
/// 从指定 CAD 面构造算法6图表、边界和几何回调。
Algorithm6FaceInput OccAlgorithm6Adapter::buildFaceInput(int face) const {
    // 外部 OCC 查询以零基面号访问；算法实体号加一，避免与无效/外围身份混用。
    Algorithm6FaceInput result;
    result.chart.entity=face+1;result.bodyId=m_geometry.faceBodyId(face);
    TopTools_IndexedMapOfShape faces,edges,vertices;
    TopExp::MapShapes(m_geometry.acceptedShape(),TopAbs_FACE,faces);
    TopExp::MapShapes(m_geometry.acceptedShape(),TopAbs_EDGE,edges);
    TopExp::MapShapes(m_geometry.acceptedShape(),TopAbs_VERTEX,vertices);
    if(face<0||face>=faces.Extent()) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid Algorithm6 face"}};
    const auto surfaceFace=TopoDS::Face(faces(face+1));
    const BRepAdaptor_Surface support(surfaceFace);
    const Point2D periods{support.IsUPeriodic()?support.UPeriod():0.,support.IsVPeriodic()?support.VPeriod():0.};
    std::set<int> embeddedPoints;
    // 原生OCCFace只把不属于边的INTERNAL顶点登记为独立嵌入点；
    // 嵌入边端点由该边处理，不能再次生成DocRecord/尺寸约束。
    for(TopExp_Explorer explorer(surfaceFace.Oriented(TopAbs_FORWARD),TopAbs_VERTEX,TopAbs_EDGE);explorer.More();explorer.Next())
        if(explorer.Current().Orientation()==TopAbs_INTERNAL)embeddedPoints.insert(vertices.FindIndex(explorer.Current()));
    for(int id:embeddedPoints)result.chart.embeddedVertices.push_back(static_cast<std::uint64_t>(id));
    std::set<int> seen;
    // 普通面原GFace::edges是CAD遍历序的唯一边；seam信息不靠坐标猜测。
    for(TopExp_Explorer explorer(surfaceFace,TopAbs_EDGE);explorer.More();explorer.Next()) {
        const auto edge=TopoDS::Edge(explorer.Current());const int id=edges.FindIndex(edge);
        if(!seen.insert(id).second) continue;
        const bool seam=BRep_Tool::IsClosed(edge,surfaceFace);
        const FaceCurveUse use{id,seam};
        if(edge.Orientation()==TopAbs_INTERNAL) result.chart.embeddedCurves.push_back(use);
        else result.chart.boundaryCurves.push_back(use);
        if(edge.Orientation()!=TopAbs_INTERNAL) {
            double first,last;const auto pc=BRep_Tool::CurveOnSurface(edge,surfaceFace,first,last);
            if(pc.IsNull()) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"CAD boundary missing pcurve"}};
            TrueBoundaryCurveInput curve;curve.entity=id;curve.seam=seam;curve.first=first;curve.last=last;
            curve.onFace=[pc](double t,int) {const auto p=pc->Value(t);return Point2D{p.X(),p.Y()};};
            result.trueBoundaryCurves.push_back(std::move(curve));
        }
    }
    // 节点反求使用原 CAD 分类和参数；不能以展示三角片或参考输出代替几何来源。
    const auto parameters=makeOccMeshVertexParameterSource(m_geometry,face);
    result.reparameterize=[parameters](const FaceVertex& v) {
        MeshVertexParameterOptions options;options.failOnSeam=false;
        const auto r=reparameterizeMeshVertexOnFace(parameterInput(v),parameters,options);
        if(!r.status.complete()) throw OperationFailure{r.status};
        if(!r.succeeded) throw OperationFailure{{MeshingStatus::NonConverged,"CAD node requires periodic parameter path"}};
        return r.parameter;
    };
    result.chart.nodeCoordinates=[reparam=result.reparameterize](const BoundaryNode& node) {
        FaceVertex v;v.id=v.physicalId=node.id;v.point=node.point;v.dimension=node.dimension;v.entity=node.entity;
        v.parameters={node.parameter,0};v.storedSize=node.storedSize;return reparam(v);
    };
    result.chart.surfacePoint=[this,face](Point2D uv) {return m_geometry.faceSurfacePoint(face,uv);};
    result.initialMetric=[this,face](Point2D uv) {Point3D du,dv;m_geometry.faceSurfaceDerivatives(face,uv,du,dv);return evaluateSurfaceMetric(du,dv);};
    result.front.parameterBounds=[this,face] {
        SurfaceParameterBounds b;if(!m_geometry.faceTrimParameterBounds(face,b)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"trim bounds unavailable"}};
        return std::array<Point2D,2>{{{b.uMin,b.vMin},{b.uMax,b.vMax}}};
    };
    result.front.metric=[metric=result.initialMetric](Point2D uv) {auto r=metric(uv);if(!r.status.complete())throw OperationFailure{r.status};return r.metric;};
    result.front.surface=result.chart.surfacePoint;
#if defined(_MSC_VER) && defined(_M_X64)
    // 固定Windows64 SDK用fsincos求球面Value/D1。末位差会改变Newton分支；
    // 仅前沿采用同一求值路径，位置/半径来自含Location的原BREP支撑面。
    const auto sphere=Handle(Geom_SphericalSurface)::DownCast(BRep_Tool::Surface(surfaceFace));
    if(!sphere.IsNull()) {
        const auto pose=sphere->Position();const double radius=sphere->Radius();
        result.front.surface=[pose,radius](Point2D uv) {return sphereFrontPoint(pose,radius,uv);};
        result.front.metric=[pose,radius](Point2D uv) {return sphereFrontMetric(pose,radius,uv);};
    }
#endif
    result.front.surfacePoint=[surface=result.front.surface](Point2D uv) {return FrontSurfacePoint{surface(uv),true};};
    result.smoothing=makeOccSurfaceSmoothingSource(m_geometry,face);
    result.orientedNormal=[this,face](Point2D uv) {return m_geometry.faceNormal(face,uv);};
    result.finalBoundaryUV=[this,face,periods,tolerance=std::max(BRep_Tool::Tolerance(surfaceFace),1.e-10)](Point3D point,Point2D seed) {
        const auto support=m_geometry.faceSurfacePoint(face,seed);
        if(std::hypot(support.x-point.x,support.y-point.y,support.z-point.z)<=tolerance)return seed;
        Point2D uv;Point3D projected;
        if(!m_geometry.projectOnFaceSurface(face,point,uv,projected)||std::hypot(projected.x-point.x,projected.y-point.y,projected.z-point.z)>tolerance)
            throw OperationFailure{{MeshingStatus::NonConverged,"final boundary point has no consistent direct CAD projection"}};
        if(periods.x>0)uv.x+=std::round((seed.x-uv.x)/periods.x)*periods.x;
        if(periods.y>0)uv.y+=std::round((seed.y-uv.y)/periods.y)*periods.y;
        return uv;
    };
    const bool periodic=periods.x>0||periods.y>0||std::any_of(result.chart.boundaryCurves.begin(),result.chart.boundaryCurves.end(),[](const auto& c){return c.seam;});
    if(periodic) {
        const auto truth=result.trueBoundaryCurves;
        const auto embeddedCurves=result.chart.embeddedCurves;
        const auto embeddedVertices=result.chart.embeddedVertices;
        result.periodicChart=[this,face,surfaceFace,edges,truth,embeddedCurves,embeddedVertices](const BoundaryMesh& boundary,
            const SurfaceMeshingOptions& options,const RunControl& control,std::size_t& evaluations) {
            auto query=[&] {
                if(control.keepGoing&&!control.keepGoing())throw OperationFailure{{MeshingStatus::Cancelled,"periodic CAD input cancelled"}};
                if(evaluations>=control.maximumGeometryEvaluations)throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic CAD input query budget"}};
                ++evaluations;
            };
            PeriodicFaceChartInput input;input.boundary.entity=face+1;
            SurfaceParameterBounds bounds;query();
            if(!m_geometry.faceTrimParameterBounds(face,bounds))throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"periodic trim bounds unavailable"}};
            input.boundary.parameterLower={bounds.uMin,bounds.vMin};input.boundary.parameterUpper={bounds.uMax,bounds.vMax};
            input.boundary.surfacePoint=[this,face](Point2D uv){return m_geometry.faceSurfacePoint(face,uv);};
            auto pcurve=[this,face](int entity,double t,bool alternate) {
                Point2D uv;if(!m_geometry.edgeOnFaceCoordinates(face,entity-1,t,alternate,uv))
                    throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"periodic exact pcurve unavailable"}};
                return uv;
            };
            auto chain=[&](int id)->const BoundaryCurve& {
                const auto found=std::find_if(boundary.curves.begin(),boundary.curves.end(),[&](const auto& c){return c.entity==id;});
                if(found==boundary.curves.end())throw OperationFailure{{MeshingStatus::InvalidTopology,"periodic CAD edge missing from shared boundary"}};
                return *found;
            };
            // 从BREP边环获取使用顺序；保留seam重复使用和退化边，不读取参考拓扑。
            for(TopExp_Explorer wires(surfaceFace.Oriented(TopAbs_FORWARD),TopAbs_WIRE);wires.More();wires.Next()) {
                std::vector<PeriodicCurveChart> loop;
                for(TopExp_Explorer uses(wires.Current(),TopAbs_EDGE);uses.More();uses.Next()) {
                    const auto edge=TopoDS::Edge(uses.Current());if(edge.Orientation()==TopAbs_INTERNAL)continue;
                    PeriodicCurveChart c;c.entity=edges.FindIndex(edge);c.seam=BRep_Tool::IsClosed(edge,surfaceFace);
                    const auto& physical=chain(c.entity);std::pair<double,double> range;query();
                    if(!m_geometry.edgeOnFaceRange(face,c.entity-1,range))throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"periodic edge range unavailable"}};
                    std::vector<BoundarySample> samples=physical.samples;
                    if(samples.empty())samples={{physical.beginNode,range.first},{physical.endNode,range.second}};
                    for(std::size_t k=0;k<samples.size();++k) {
                        const double t=k==0?range.first:k+1==samples.size()?range.second:samples[k].parameter;
                        query();c.vertices.push_back({samples[k].node,pcurve(c.entity,t,false)});
                        if(c.seam){query();c.alternateCoordinates.push_back(pcurve(c.entity,t,true));}
                    }
                    loop.push_back(std::move(c));
                }
                if(!loop.empty())input.boundary.edgeLoops.push_back(std::move(loop));
            }
            for(auto c:truth) {
                std::pair<double,double> range;query();
                if(!m_geometry.edgeOnFaceRange(face,c.entity-1,range))throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"true boundary range unavailable"}};
                c.first=range.first;c.last=range.second;
                c.onFace=[pcurve,id=c.entity](double t,int branch){return pcurve(id,t,branch!=1);};
                input.trueBoundaryCurves.push_back(std::move(c));
            }
            input.embedded.vertices=embeddedVertices;
            input.embedded.closestSurfaceCoordinates=[this,face](Point3D p) {
                Point2D uv;Point3D projected;
                if(!m_geometry.projectOnFaceSurface(face,p,uv,projected))throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"embedded surface projection failed"}};
                return uv;
            };
            for(const auto& use:embeddedCurves) {
                PeriodicEmbeddedCurveInput c;c.entity=use.entity;const auto& physical=chain(c.entity);
                for(std::size_t k=1;k<physical.samples.size();++k)c.lines.emplace_back(physical.samples[k-1].node,physical.samples[k].node);
                std::pair<double,double> range;query();
                if(!m_geometry.edgeOnFaceRange(face,c.entity-1,range))throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"embedded edge range unavailable"}};
                c.first=range.first;c.last=range.second;
                c.point=[this,id=c.entity](double t){return m_geometry.edgePoint(id-1,t);};
                c.onFace=[pcurve,id=c.entity](double t){return pcurve(id,t,false);};
                input.embedded.curves.push_back(std::move(c));
            }
            return input;
        };
    }
    return result;
}
/// 从 CAD 模型构造共享边界及各面的算法6输入。
Algorithm6ModelInput OccAlgorithm6Adapter::buildModelInput(const std::map<int,double>& vertexSizes) const {
    if(!m_geometry.validationError().empty()) throw OperationFailure{{MeshingStatus::InvalidInput,m_geometry.validationError()}};
    Algorithm6ModelInput result;
    TopTools_IndexedMapOfShape vertices,edges,faces;
    TopExp::MapShapes(m_geometry.acceptedShape(),TopAbs_VERTEX,vertices);
    TopExp::MapShapes(m_geometry.acceptedShape(),TopAbs_EDGE,edges);
    TopExp::MapShapes(m_geometry.acceptedShape(),TopAbs_FACE,faces);
    Bnd_Box box;BRepBndLib::Add(m_geometry.acceptedShape(),box);
    if(box.IsVoid()) throw OperationFailure{{MeshingStatus::InvalidInput,"CAD has no bounding box"}};
    double a,b,c,d,e,f;box.Get(a,b,c,d,e,f);
    result.boundary.modelCharacteristicLength=std::hypot(d-a,e-b,f-c);
    for(const auto& size:vertexSizes)if(size.first<1||size.first>vertices.Extent()||!std::isfinite(size.second)||size.second<=0)
        throw OperationFailure{{MeshingStatus::InvalidInput,"invalid explicit CAD vertex size"}};
    for(int i=1;i<=vertices.Extent();++i) result.boundary.vertices.push_back({i,point3(BRep_Tool::Pnt(TopoDS::Vertex(vertices(i))))});
    for(int i=1;i<=edges.Extent();++i) {
        const auto edge=TopoDS::Edge(edges(i));BoundaryCurveInput curve;curve.entity=i;
        curve.beginVertex=vertices.FindIndex(TopExp::FirstVertex(edge));curve.endVertex=vertices.FindIndex(TopExp::LastVertex(edge));
        curve.geometryDegenerate=BRep_Tool::Degenerated(edge);
        for(int j=1;j<=faces.Extent();++j) {TopTools_IndexedMapOfShape used;TopExp::MapShapes(faces(j),TopAbs_EDGE,used);if(used.Contains(edge))curve.incidentFaces.push_back(j);}
        if(!curve.geometryDegenerate) {
            auto adaptor=std::make_shared<BRepAdaptor_Curve>(edge);
            curve.first=adaptor->FirstParameter();curve.last=adaptor->LastParameter();
            const auto type=adaptor->GetType();curve.kind=type==GeomAbs_Line?CurveKind::Line:type==GeomAbs_Circle?CurveKind::Circle:type==GeomAbs_Ellipse?CurveKind::Ellipse:CurveKind::Other;
            curve.point=[adaptor](double t) {return point3(adaptor->Value(t));};
            curve.derivative=[adaptor](double t) {gp_Pnt p;gp_Vec v;adaptor->D1(t,p,v);return Point3D{v.X(),v.Y(),v.Z()};};
            double low,high;const auto support=BRep_Tool::Curve(edge,low,high);
            if(support.IsNull()) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nondegenerate CAD edge missing curve"}};
            if(curve.beginVertex!=curve.endVertex) {const auto tol=std::max(std::abs(high-low)*1.e-8,1.e-12);low-=tol;high+=tol;}
            auto projector=std::make_shared<GeomAPI_ProjectPointOnCurve>();projector->Init(support,low,high);
            curve.inverseParameter=[projector](Point3D p) {projector->Perform(gp_Pnt(p.x,p.y,p.z));if(!projector->NbPoints())throw OperationFailure{{MeshingStatus::NonConverged,"direct CAD edge projection failed"}};return projector->LowerDistanceParameter();};
        }
        result.boundary.curves.push_back(std::move(curve));
    }
    const auto curves=std::make_shared<std::vector<BoundaryCurveInput>>(result.boundary.curves);
    const double modelLength=result.boundary.modelCharacteristicLength;
    result.sizes.pointSize=[vertexSizes,curves,modelLength](const MeshSizeQuery& q) {
        auto size=[&](int id){const auto found=vertexSizes.find(id);return found==vertexSizes.end()?1.e22:found->second;};
        if(q.dimension==0){const double value=size(q.entity);return value>=1.e22?modelLength*.1:value;}
        if(q.dimension==1){const auto& edge=curves->at(q.entity-1);const double first=size(edge.beginVertex),last=size(edge.endVertex);if(first>=1.e22&&last>=1.e22)return modelLength*.1;const double t=(q.u-edge.first)/(edge.last-edge.first);return (1-t)*first+t*last;}
        return 1.e22;
    };
    auto surfaceCurvature=[faces](int id,Point2D uv) {
        BRepAdaptor_Surface surface(TopoDS::Face(faces(id)));BRepLProp_SLProps properties(surface,2,1.e-12);properties.SetParameters(uv.x,uv.y);
        return properties.IsCurvatureDefined()?std::max(std::abs(properties.MinCurvature()),std::abs(properties.MaxCurvature())):1.e-12;
    };
    auto edgeCurvature=[this,curves,edges,surfaceCurvature](int id,double t) {
        const auto& edge=curves->at(id-1);double value=1.e-15;
        if(!edge.geometryDegenerate){BRepAdaptor_Curve curve(TopoDS::Edge(edges(id)));BRepLProp_CLProps properties(curve,2,1.e-15);properties.SetParameter(t);if(properties.IsTangentDefined())value=std::max(value,properties.Curvature());}
        for(int face:edge.incidentFaces){Point2D uv;if(!m_geometry.edgeOnFaceCoordinates(face-1,id-1,t,false,uv))throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"curvature edge pcurve unavailable"}};value=std::max(value,surfaceCurvature(face,uv));}
        return value;
    };
    result.sizes.maximumCurvature=[curves,edgeCurvature,surfaceCurvature](const MeshSizeQuery& q) {
        if(q.dimension==2)return surfaceCurvature(q.entity,{q.u,q.v});
        if(q.dimension==1)return edgeCurvature(q.entity,q.u);
        double value=0.;if(q.dimension==0)for(const auto& edge:*curves){if(edge.beginVertex==q.entity)value=std::max(value,edgeCurvature(edge.entity,edge.first));else if(edge.endVertex==q.entity)value=std::max(value,edgeCurvature(edge.entity,edge.last));}
        return value;
    };
    for(int face=0;face<m_geometry.faceCount();++face) result.faces.push_back(buildFaceInput(face));
    return result;
}
