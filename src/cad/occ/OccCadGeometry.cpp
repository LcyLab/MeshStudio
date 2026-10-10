// 文件职责：CAD 查询适配：建立实体身份和面拓扑，按需准备 OCC 曲线、曲面及投影器供一次串行运行复用。
#include "OccCadGeometry.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepCheck_ListIteratorOfListOfStatus.hxx>
#include <BRepCheck_Wire.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRep_Builder.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Extrema_ExtPC.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopAbs_Orientation.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <gp_Lin.hxx>
#include <ElSLib.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <ShapeAnalysis.hxx>

#include <limits>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace {
/// 将 OCC 点转换为网格使用的三维坐标。
mesher::Point3D point(gp_Pnt p) { return {p.X(),p.Y(),p.Z()}; }
/// 收集 BREP 校验失败涉及的面、线环和边。
std::string invalidDetails(const TopoDS_Shape& shape) {
    if(shape.IsNull())return "null B-Rep geometry";
    std::ostringstream text; text<<"invalid B-Rep geometry";
    BRepCheck_Analyzer check(shape);TopTools_IndexedMapOfShape faces;TopExp::MapShapes(shape,TopAbs_FACE,faces);
    int count=0;
    for(int f=1;f<=faces.Extent() && count<8;++f) if(!check.IsValid(faces(f))) {
        ++count;text<<"; CAD face "<<f-1;
        for(TopExp_Explorer w(faces(f),TopAbs_WIRE);w.More();w.Next()) {
            BRepCheck_Wire wire(TopoDS::Wire(w.Current()));TopoDS_Edge a,b;
            if(wire.SelfIntersect(TopoDS::Face(faces(f)),a,b)==BRepCheck_SelfIntersectingWire)text<<" SelfIntersectingWire";
        }
    }
    text<<"; unsupported defect or bounded local tolerance reconciliation failed";return text.str();
}
/// 在模型副本上修正满足限定条件的局部裁剪容差。
bool reconcileLocalTrimTolerance(const TopoDS_Shape& original,TopoDS_Shape& accepted,std::vector<std::string>& diagnostics) {
    Bnd_Box box;BRepBndLib::Add(original,box,false);if(box.IsVoid())return false;
    double x0,y0,z0,x1,y1,z1;box.Get(x0,y0,z0,x1,y1,z1);
    const double scale=std::hypot(x1-x0,y1-y0,z1-z0);
    if(!std::isfinite(scale) || !(scale>0))return false;
    BRepBuilderAPI_Copy copy(original,true,false);TopoDS_Shape candidate=copy.Shape();
    TopTools_IndexedMapOfShape vertices,edges,faces;
    TopExp::MapShapes(candidate,TopAbs_VERTEX,vertices);TopExp::MapShapes(candidate,TopAbs_EDGE,edges);TopExp::MapShapes(candidate,TopAbs_FACE,faces);
    std::vector<double> oldTolerances;
    for(int v=1;v<=vertices.Extent();++v)oldTolerances.push_back(BRep_Tool::Tolerance(TopoDS::Vertex(vertices(v))));
    BRep_Builder builder;std::vector<std::string> changes;
    for(int pass=0;pass<8;++pass) {
        BRepCheck_Analyzer check(candidate);
        if(check.IsValid()) {
            // 复制不仅要保留拓扑数量，还必须保持原实体标签顺序。
            for(auto type:{TopAbs_VERTEX,TopAbs_EDGE,TopAbs_FACE,TopAbs_WIRE,TopAbs_SHELL,TopAbs_SOLID}) {
                TopTools_IndexedMapOfShape before,after;TopExp::MapShapes(original,type,before);TopExp::MapShapes(candidate,type,after);
                if(before.Extent()!=after.Extent())return false;
                for(int i=1;i<=before.Extent();++i)if(!copy.ModifiedShape(before(i)).IsSame(after(i)))return false;
            }
            accepted=candidate;diagnostics=std::move(changes);return true;
        }
        bool changed=false;
        for(int f=1;f<=faces.Extent();++f) {
            auto face=TopoDS::Face(faces(f));if(check.IsValid(face))continue;
            BRepAdaptor_Surface surface(face);
            for(TopExp_Explorer w(face,TopAbs_WIRE);w.More();w.Next()) {
                BRepCheck_Wire wire(TopoDS::Wire(w.Current()));TopoDS_Edge a,b;
                if(wire.SelfIntersect(face,a,b)!=BRepCheck_SelfIntersectingWire || a.IsNull() || b.IsNull())continue;
                TopoDS_Vertex shared;
                for(TopExp_Explorer va(a,TopAbs_VERTEX);va.More();va.Next())for(TopExp_Explorer vb(b,TopAbs_VERTEX);vb.More();vb.Next())
                    if(va.Current().IsSame(vb.Current()))shared=TopoDS::Vertex(va.Current());
                if(shared.IsNull())continue; // 不接受线环内部的真实交叉。
                const int v=vertices.FindIndex(shared);if(v<1)continue;
                const double old=BRep_Tool::Tolerance(shared),limit=std::min(100*oldTolerances[v-1],scale*1e-6);
                double a0,a1,b0,b1;auto ca=BRep_Tool::CurveOnSurface(a,face,a0,a1),cb=BRep_Tool::CurveOnSurface(b,face,b0,b1);
                if(ca.IsNull() || cb.IsNull())continue;
                Geom2dAPI_InterCurveCurve intersection(new Geom2d_TrimmedCurve(ca,a0,a1),new Geom2d_TrimmedCurve(cb,b0,b1),1e-10);
                const auto p=BRep_Tool::Pnt(shared);double needed=0;bool bounded=true;
                const auto sample=[&](gp_Pnt2d uv) {
                    const double distance=p.Distance(surface.Value(uv.X(),uv.Y()));
                    if(!std::isfinite(distance) || distance*1.1>limit)bounded=false;
                    needed=std::max(needed,distance);
                };
                for(int k=1;k<=intersection.NbPoints();++k)sample(intersection.Point(k));
                // 延伸重叠可能掩盖线环；只允许处理靠近同一共享端点的局部数值相交。
                for(int k=1;k<=intersection.NbSegments();++k) {
                    Handle(Geom2d_Curve) x,y;intersection.Segment(k,x,y);
                    for(auto curve:{x,y}) {
                        const auto first=curve->Value(curve->FirstParameter()),last=curve->Value(curve->LastParameter());
                        if(surface.Value(first.X(),first.Y()).Distance(surface.Value(last.X(),last.Y()))>oldTolerances[v-1])bounded=false;
                        for(int j=0;j<=8;++j)sample(curve->Value(curve->FirstParameter()+(curve->LastParameter()-curve->FirstParameter())*j/8));
                    }
                }
                if(!bounded || !(needed>old) || changes.size()>=16)continue;
                const double next=needed*1.1;builder.UpdateVertex(shared,next);changed=true;
                std::ostringstream message;message<<std::setprecision(17)<<"CAD input tolerance reconciled on deep copy: face "<<f-1
                    <<", edges "<<edges.FindIndex(a)-1<<'/'<<edges.FindIndex(b)-1<<", vertex "<<v-1
                    <<", tolerance "<<old<<" -> "<<next<<" (model units); curves, surfaces, positions and topology unchanged";
                changes.push_back(message.str());
            }
        }
        if(!changed)break;
    }
    return false;
}
}

/// 建立 CAD 实体索引和面拓扑，并准备后续几何查询。
OccCadGeometry::OccCadGeometry(const TopoDS_Shape& shape,bool allowLocalToleranceRepair) : m_shape(shape) {
    if (shape.IsNull() || (!BRepCheck_Analyzer(shape).IsValid() &&
        !(allowLocalToleranceRepair && reconcileLocalTrimTolerance(shape,m_shape,m_inputDiagnostics)))) {
        m_error=invalidDetails(shape);
        return;
    }
    TopExp::MapShapes(m_shape,TopAbs_VERTEX,m_vertices);
    TopExp::MapShapes(m_shape,TopAbs_EDGE,m_edges);
    TopExp::MapShapes(m_shape,TopAbs_FACE,m_faces);
    TopExp::MapShapes(m_shape,TopAbs_SOLID,m_solids);
    m_faceBodies.assign(m_faces.Extent(),-1);
    for(int b=1;b<=m_solids.Extent();++b)for(TopExp_Explorer f(m_solids(b),TopAbs_FACE);f.More();f.Next()) {
        int face=m_faces.FindIndex(f.Current())-1;
        if(face>=0) {if(m_faceBodies[face]>=0 && m_faceBodies[face]!=b-1) {m_error="CAD face belongs to multiple solids; shared region topology requires explicit interface support";return;}
            m_faceBodies[face]=b-1;}
    }
    m_vertexPoints.reserve(m_vertices.Extent());
    for (int v=1;v<=m_vertices.Extent();++v)
        m_vertexPoints.push_back(point(BRep_Tool::Pnt(TopoDS::Vertex(m_vertices(v)))));
    m_edgeGeometry.reserve(m_edges.Extent());
    for (int e=1;e<=m_edges.Extent();++e) {
        const auto edge=TopoDS::Edge(m_edges(e)); TopoDS_Vertex a,b;
        TopExp::Vertices(edge,a,b); double first=0,last=0; BRep_Tool::Range(edge,first,last);
        EdgeGeometry prepared; prepared.vertices={m_vertices.FindIndex(a)-1,m_vertices.FindIndex(b)-1};
        prepared.range={first,last}; prepared.degenerate=BRep_Tool::Degenerated(edge);
        if (!prepared.degenerate) prepared.curve.emplace(edge);
        m_edgeGeometry.push_back(std::move(prepared));
    }
    m_surfaces.reserve(m_faces.Extent());
    m_closed=TopExp_Explorer(m_shape,TopAbs_SOLID).More();
    m_topology.resize(m_faces.Extent());
    for (int f=0;f<m_faces.Extent();++f) {
        const TopoDS_Face face=TopoDS::Face(m_faces(f+1));
        // 保留原 Restriction=true 的裁剪范围及 Location，只准备一次。
        m_surfaces.emplace_back(face);
        for (TopExp_Explorer wires(face,TopAbs_WIRE);wires.More();wires.Next()) {
            const TopoDS_Wire wire=TopoDS::Wire(wires.Current());
            std::vector<mesher::EdgeUse> loop;
            for (BRepTools_WireExplorer e(wire,face);e.More();e.Next()) {
                const TopoDS_Edge current=e.Current();
                const int id=m_edges.FindIndex(current)-1;
                const bool reversed=e.Orientation()==TopAbs_REVERSED;
                if (id>=0) loop.push_back({id,reversed,0});
                if (id>=0) for (int branch:{0,1}) {
                    const auto key=std::make_tuple(f,id,branch);
                    if (m_trimCurves.count(key)) continue;
                    auto edge=TopoDS::Edge(m_edges(id+1));
                    edge.Orientation(branch?TopAbs_REVERSED:TopAbs_FORWARD);
                    TrimCurve trim; trim.curve=BRep_Tool::CurveOnSurface(edge,face,trim.first,trim.last);
                    m_trimCurves.emplace(key,std::move(trim));
                }
            }
            // 线环中的边方向可能与全局边索引不同，周期接缝还具有两条参数曲线；联合选择方向和分支，使连续边在三维及 UV 中都连接。
            if (!loop.empty()) {
                std::vector<std::vector<mesher::EdgeUse>> options(loop.size());
                for (size_t i=0;i<loop.size();++i) {
                    const bool seam=BRep_Tool::IsClosed(
                        TopoDS::Edge(m_edges(loop[i].edge+1)),face);
                    for (int rev=0;rev<2;++rev)
                        for (int branch=0;branch<(seam?2:1);++branch)
                            options[i].push_back({loop[i].edge,rev!=0,branch});
                }
                auto uvEnd=[&](const mesher::EdgeUse& use,bool start) {
                    const auto& pc=m_trimCurves.at({f,use.edge,use.branch?1:0});
                    if (pc.curve.IsNull()) return gp_Pnt2d(1e100,1e100);
                    return pc.curve->Value((start==use.reversed)?pc.last:pc.first);
                };
                auto vertexEnd=[&](const mesher::EdgeUse& use,bool start) {
                    const auto ends=m_edgeGeometry[use.edge].vertices;
                    return ((start==use.reversed)?ends.second:ends.first)+1;
                };
                auto transition=[&](const mesher::EdgeUse& left,
                                    const mesher::EdgeUse& right) {
                    if (vertexEnd(left,false)!=vertexEnd(right,true)) return 1e30;
                    return uvEnd(left,false).SquareDistance(uvEnd(right,true));
                };
                double best=std::numeric_limits<double>::infinity();
                std::vector<int> bestStates(loop.size(),0);
                for (int first=0;first<static_cast<int>(options[0].size());++first) {
                    std::vector<double> dp(options[0].size(),std::numeric_limits<double>::infinity());
                    dp[first]=0;
                    std::vector<std::vector<int>> previous(loop.size());
                    for (size_t i=1;i<loop.size();++i) {
                        std::vector<double> next(options[i].size(),std::numeric_limits<double>::infinity());
                        previous[i].resize(options[i].size(),-1);
                        for (int s=0;s<static_cast<int>(options[i].size());++s)
                            for (int p=0;p<static_cast<int>(dp.size());++p) {
                                const double score=dp[p]+transition(options[i-1][p],options[i][s]);
                                if (score<next[s]) { next[s]=score; previous[i][s]=p; }
                            }
                        dp.swap(next);
                    }
                    for (int last=0;last<static_cast<int>(dp.size());++last) {
                        const double score=dp[last]+transition(options.back()[last],options[0][first]);
                        if (score<best) {
                            best=score;
                            int s=last;
                            for (int i=static_cast<int>(loop.size())-1;i>=1;--i) {
                                bestStates[i]=s; s=previous[i][s];
                            }
                            bestStates[0]=first;
                        }
                    }
                }
                if (!std::isfinite(best) || best>=1e30) {
                    m_error="trim wire has no connected UV/vertex ordering on face "+std::to_string(f);
                    return;
                }
                for (size_t i=0;i<loop.size();++i) loop[i]=options[i][bestStates[i]];
            }
            if (!loop.empty()) m_topology[f].loops.push_back(std::move(loop));
        }
    }
}

/// 查询指定 CAD 面所属的部件编号。
int OccCadGeometry::faceBodyId(int face) const {return m_faceBodies.at(face);}
/// 查询不同部件的 CAD 接触或相交关系。
mesher::CadBodyContact OccCadGeometry::bodyContact(int firstFace,int secondFace) const {
    using Contact=mesher::CadBodyContact;
    const int a=faceBodyId(firstFace),b=faceBodyId(secondFace);
    if(a<0 || b<0 || a==b)return Contact::None;
    auto facePair=std::make_pair(std::min(firstFace,secondFace),std::max(firstFace,secondFace));
    auto cached=m_contactPairs.find(facePair);if(cached!=m_contactPairs.end())return cached->second;
    auto bodyPair=std::make_pair(std::min(a,b),std::max(a,b));
    Contact bodyState=Contact::BoundaryContact;
    auto body=m_bodyPairs.find(bodyPair);
    if(body==m_bodyPairs.end()) {
        BRepAlgoAPI_Common common(m_solids(a+1),m_solids(b+1));common.Build();
        if(!common.IsDone())bodyState=Contact::QueryFailed;
        else {
            TopTools_IndexedMapOfShape volume;TopExp::MapShapes(common.Shape(),TopAbs_SOLID,volume);
            GProp_GProps mass;BRepGProp::VolumeProperties(common.Shape(),mass);
            if(volume.Extent()>0 && std::abs(mass.Mass())>0)bodyState=Contact::VolumeOverlap;
        }
        m_bodyPairs[bodyPair]=bodyState;
    } else bodyState=body->second;
    if(bodyState!=Contact::BoundaryContact) {m_contactPairs[facePair]=bodyState;return bodyState;}
    BRepExtrema_DistShapeShape distance(m_faces(firstFace+1),m_faces(secondFace+1));distance.Perform();
    Contact state=Contact::QueryFailed;
    if(distance.IsDone()) {
        const double tol=std::max(BRep_Tool::Tolerance(TopoDS::Face(m_faces(firstFace+1))),BRep_Tool::Tolerance(TopoDS::Face(m_faces(secondFace+1))));
        state=distance.Value()<=tol?Contact::BoundaryContact:Contact::None;
    }
    m_contactPairs[facePair]=state;return state;
}
/// 返回 CAD 顶点数量。
int OccCadGeometry::vertexCount() const { return m_vertices.Extent(); }
/// 查询指定 CAD 顶点的三维坐标。
mesher::Point3D OccCadGeometry::vertexPoint(int v) const {
    return m_vertexPoints.at(v);
}
/// 返回 CAD 边数量。
int OccCadGeometry::edgeCount() const { return m_edges.Extent(); }
/// 查询 CAD 边关联的端点编号。
std::pair<int,int> OccCadGeometry::edgeVertices(int e) const {
    return m_edgeGeometry.at(e).vertices;
}
/// 查询 CAD 边的曲线参数范围。
std::pair<double,double> OccCadGeometry::edgeRange(int e) const {
    return m_edgeGeometry.at(e).range;
}
/// 计算 CAD 边在指定曲线参数处的三维位置。
mesher::Point3D OccCadGeometry::edgePoint(int e,double t) const {
    const auto& edge=m_edgeGeometry.at(e);
    if (edge.degenerate) return vertexPoint(edge.vertices.first);
    return point(edge.curve->Value(t));
}
/// 计算 CAD 边在指定参数处的一阶导数。
mesher::Point3D OccCadGeometry::edgeDerivative(int e,double t) const {
    const auto& edge=m_edgeGeometry.at(e);if(edge.degenerate)return {};
    gp_Pnt p;gp_Vec derivative;edge.curve->D1(t,p,derivative);
    return {derivative.X(),derivative.Y(),derivative.Z()};
}
/// 返回 CAD 面数量。
int OccCadGeometry::faceCount() const { return m_faces.Extent(); }
/// 判断 CAD 边是否为退化边。
bool OccCadGeometry::edgeDegenerate(int e) const {
    return m_edgeGeometry.at(e).degenerate;
}
/// 返回面的线环、边使用关系和参数信息。
mesher::FaceTopology OccCadGeometry::faceTopology(int f) const { return m_topology[f]; }
/// 计算边上节点在指定面分支中的 UV 坐标。
mesher::Point2D OccCadGeometry::edgeUV(int f,const mesher::EdgeUse& use,double t) const {
    const auto found=m_trimCurves.find({f,use.edge,use.branch?1:0});
    if (found==m_trimCurves.end() || found->second.curve.IsNull()) {
        const double n=std::numeric_limits<double>::quiet_NaN(); return {n,n};
    }
    const gp_Pnt2d p=found->second.curve->Value(t);
    return {p.X(),p.Y()};
}
/// 计算指定面在 UV 参数处的三维位置。
mesher::Point3D OccCadGeometry::facePoint(int f,mesher::Point2D uv) const {
    return point(m_surfaces.at(f).Value(uv.x,uv.y));
}
/// 查询边在指定面上的参数曲线范围。
bool OccCadGeometry::edgeOnFaceRange(int f,int e,std::pair<double,double>& range) const {
    if(f<0 || f>=m_faces.Extent() || e<0 || e>=m_edges.Extent()) return false;
    auto edge=TopoDS::Edge(m_edges(e+1));
    if(edge.Orientation()==TopAbs_INTERNAL || edge.Orientation()==TopAbs_EXTERNAL) edge.Orientation(TopAbs_FORWARD);
    double first,last;
    if(BRep_Tool::Degenerated(edge)) {
        const auto pc=BRep_Tool::CurveOnSurface(edge,TopoDS::Face(m_faces(f+1)),first,last);
        if(pc.IsNull()) return false;
    } else {
        const auto curve=BRep_Tool::Curve(edge,first,last);
        if(curve.IsNull()) return false;
    }
    if(!std::isfinite(first) || !std::isfinite(last)) return false;
    range={first,last}; return true;
}
/// 计算指定面分支中边参数对应的 UV 坐标。
bool OccCadGeometry::edgeOnFaceCoordinates(int f,int e,double t,bool reversed,mesher::Point2D& uv) const {
    if(f<0 || f>=m_faces.Extent() || e<0 || e>=m_edges.Extent() || !std::isfinite(t)) return false;
    // 缓存 branch 是强制 FORWARD/REVERSED；原生 dir 的基准是全局边本来的朝向。
    const bool originalReversed=m_edges(e+1).Orientation()==TopAbs_REVERSED;
    const int branch=originalReversed!=reversed ? 1 : 0;
    const auto found=m_trimCurves.find({f,e,branch});
    Handle(Geom2d_Curve) pc;
    if(found!=m_trimCurves.end()) pc=found->second.curve;
    else {
        auto edge=TopoDS::Edge(m_edges(e+1).Oriented(branch?TopAbs_REVERSED:TopAbs_FORWARD));
        double first,last; pc=BRep_Tool::CurveOnSurface(edge,TopoDS::Face(m_faces(f+1)),first,last);
    }
    if(pc.IsNull()) return false;
    const auto p=pc->Value(t);
    if(!std::isfinite(p.X()) || !std::isfinite(p.Y())) return false;
    uv={p.X(),p.Y()}; return true;
}
/// 判断指定边是否为该面的周期接缝。
bool OccCadGeometry::edgeSeamOnFace(int f,int e,bool& seam) const {
    if(f<0 || f>=m_faces.Extent() || e<0 || e>=m_edges.Extent()) return false;
    seam=BRep_Tool::IsClosed(TopoDS::Edge(m_edges(e+1)),TopoDS::Face(m_faces(f+1))); return true;
}
/// 计算指定面参数处的单位法向。
mesher::Point3D OccCadGeometry::faceNormal(int f,mesher::Point2D uv) const {
    const TopoDS_Face face=TopoDS::Face(m_faces(f+1));
    gp_Pnt p; gp_Vec du,dv;
    m_surfaces.at(f).D1(uv.x,uv.y,p,du,dv);
    gp_Vec n=du.Crossed(dv);
    if (face.Orientation()==TopAbs_REVERSED) n.Reverse();
    if (n.Magnitude()==0) return {0,0,0};
    return {n.X(),n.Y(),n.Z()};
}
/// 计算面的参数导数，供局部度量和法向计算使用。
void OccCadGeometry::faceDerivatives(int f,mesher::Point2D uv,
                                     mesher::Point3D& du,mesher::Point3D& dv) const {
    gp_Pnt p; gp_Vec u,v;
    m_surfaces.at(f).D1(uv.x,uv.y,p,u,v);
    du={u.X(),u.Y(),u.Z()}; dv={v.X(),v.Y(),v.Z()};
}
/// 判断输入拓扑是否要求生成闭合表面。
bool OccCadGeometry::expectsClosedSurface() const { return m_closed; }
/// 直接查询底层曲面的参数导数。
void OccCadGeometry::faceSurfaceDerivatives(int f,mesher::Point2D uv,
                                           mesher::Point3D& du,mesher::Point3D& dv) const {
    if(f < 0 || f >= m_faces.Extent()) throw std::out_of_range("invalid support surface face");
    auto found = m_supportSurfaces.find(f);
    if(found == m_supportSurfaces.end()) {
        auto surface = BRep_Tool::Surface(TopoDS::Face(m_faces(f+1)));
        if(surface.IsNull()) throw std::runtime_error("missing BREP support surface");
        found = m_supportSurfaces.emplace(f,surface).first;
    }
    // Gmsh的OCCFace直接调用Geom_Surface；BRepAdaptor的B样条缓存求值末位不同。
    gp_Pnt p; gp_Vec u,v; found->second->D1(uv.x,uv.y,p,u,v);
    du = {u.X(),u.Y(),u.Z()}; dv = {v.X(),v.Y(),v.Z()};
}
/// 直接计算底层曲面在指定 UV 处的三维位置。
mesher::Point3D OccCadGeometry::faceSurfacePoint(int f,mesher::Point2D uv) const {
    if(f < 0 || f >= m_faces.Extent()) throw std::out_of_range("invalid support surface face");
    auto found = m_supportSurfaces.find(f);
    if(found == m_supportSurfaces.end()) {
        auto surface = BRep_Tool::Surface(TopoDS::Face(m_faces(f+1)));
        if(surface.IsNull()) throw std::runtime_error("missing BREP support surface");
        found = m_supportSurfaces.emplace(f,surface).first;
    }
    const auto p = found->second->Value(uv.x,uv.y);
    return {p.X(),p.Y(),p.Z()};
}
/// 查询底层曲面的参数范围。
bool OccCadGeometry::faceParameterBounds(int f,mesher::SurfaceParameterBounds& bounds) const {
    const auto& surface=m_surfaces.at(f);
    bounds={surface.FirstUParameter(),surface.LastUParameter(),surface.FirstVParameter(),surface.LastVParameter()};
    return true;
}
/// 查询裁剪面实际使用的 UV 包围范围。
bool OccCadGeometry::faceTrimParameterBounds(int f,mesher::SurfaceParameterBounds& bounds) const {
    if(f<0 || f>=m_faces.Extent()) return false;
    mesher::SurfaceParameterBounds trimmed;
    ShapeAnalysis::GetFaceUVBounds(TopoDS::Face(m_faces(f+1)),trimmed.uMin,trimmed.uMax,trimmed.vMin,trimmed.vMax);
    if(!std::isfinite(trimmed.uMin) || !std::isfinite(trimmed.uMax) ||
       !std::isfinite(trimmed.vMin) || !std::isfinite(trimmed.vMax)) return false;
    bounds=trimmed; return true;
}
/// 将三维点投影到底层曲面，返回曲面参数和位置。
bool OccCadGeometry::projectOnFaceSurface(int f,mesher::Point3D query,mesher::Point2D& uv,mesher::Point3D& projected) const {
    if(f<0 || f>=m_faces.Extent() || !std::isfinite(query.x) || !std::isfinite(query.y) || !std::isfinite(query.z)) return false;
    auto found=m_surfaceProjectors.find(f);
    if(found==m_surfaceProjectors.end()) {
        mesher::SurfaceParameterBounds bounds;
        if(!faceTrimParameterBounds(f,bounds)) return false;
        const auto& surface=m_surfaces.at(f);
        if(!surface.IsUPeriodic()) {
            const double tolerance=std::max(std::abs(bounds.uMax-bounds.uMin)*1.e-8,1.e-12);
            bounds.uMin-=tolerance; bounds.uMax+=tolerance;
        }
        if(!surface.IsVPeriodic()) {
            const double tolerance=std::max(std::abs(bounds.vMax-bounds.vMin)*1.e-8,1.e-12);
            bounds.vMin-=tolerance; bounds.vMax+=tolerance;
        }
        const auto support=BRep_Tool::Surface(TopoDS::Face(m_faces(f+1)));
        if(support.IsNull()) return false;
        auto prepared=std::make_unique<GeomAPI_ProjectPointOnSurf>();
        prepared->Init(support,bounds.uMin,bounds.uMax,bounds.vMin,bounds.vMax);
        found=m_surfaceProjectors.emplace(f,std::move(prepared)).first;
    }
    auto& projector=*found->second;
    projector.Perform(gp_Pnt(query.x,query.y,query.z));
    if(!projector.NbPoints()) return false;
    mesher::Point2D parameter; projector.LowerDistanceParameters(parameter.x,parameter.y);
    const auto closest=projector.NearestPoint();
    if(!std::isfinite(parameter.x) || !std::isfinite(parameter.y) || !std::isfinite(closest.X()) ||
       !std::isfinite(closest.Y()) || !std::isfinite(closest.Z())) return false;
    uv=parameter; projected=point(closest); return true;
}
/// 返回 CAD 查询适配器的准备工作统计。
OccCadGeometry::PreparationStatistics OccCadGeometry::preparationStatistics() const {
    size_t curves=0; for (const auto& e:m_edgeGeometry) if (e.curve) ++curves;
    return {m_surfaces.size(),curves,m_trimCurves.size()};
}
/// 判断 CAD 面是否相对底层曲面反向。
bool OccCadGeometry::faceReversed(int f) const {
    return m_faces(f+1).Orientation()==TopAbs_REVERSED;
}
/// 反求三维点在指定 CAD 面上的 UV 坐标。
bool OccCadGeometry::faceCoordinates(int f,mesher::Point3D p,mesher::Point2D seed,mesher::Point2D& uv) const {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
       !std::isfinite(seed.x)||!std::isfinite(seed.y))return false;
    const auto& surface=m_surfaces.at(f);const gp_Pnt physical(p.x,p.y,p.z);
    if(surface.GetType()==GeomAbs_Plane)ElSLib::Parameters(surface.Plane(),physical,uv.x,uv.y);
    else if(surface.GetType()==GeomAbs_Cylinder) {
        ElSLib::Parameters(surface.Cylinder(),physical,uv.x,uv.y);
        const double period=2*std::acos(-1.0);
        uv.x+=period*std::round((seed.x-uv.x)/period);
    } else return false;
    return std::isfinite(uv.x)&&std::isfinite(uv.y);
}
/// 查询面的 U、V 周期长度。
mesher::Point2D OccCadGeometry::facePeriods(int f) const {
    return m_surfaces.at(f).GetType()==GeomAbs_Cylinder?mesher::Point2D{2*std::acos(-1.0),0}:mesher::Point2D{0,0};
}
/// 判断两个面是否共享可直接使用的解析参数图表。
bool OccCadGeometry::commonSurfaceChart(int a,int b) const {
    if(a==b)return true;
    const auto& first=m_surfaces.at(a);const auto& second=m_surfaces.at(b);
    if(first.GetType()!=second.GetType())return false;
    // 按浮点尺度判定解析支撑是否相同；缝合容差不能把真实的小台阶或折角误判为可移除分割。
    constexpr double angular=1e-12;
    if(first.GetType()==GeomAbs_Plane) {
        const auto x=first.Plane(),y=second.Plane();
        const double scale=std::max({1.0,std::hypot(first.LastUParameter()-first.FirstUParameter(),first.LastVParameter()-first.FirstVParameter()),
            std::hypot(second.LastUParameter()-second.FirstUParameter(),second.LastVParameter()-second.FirstVParameter())});
        const double tolerance=128*std::numeric_limits<double>::epsilon()*scale;
        if(!x.Axis().Direction().IsParallel(y.Axis().Direction(),angular)||x.Distance(y.Location())>tolerance)return false;
    } else if(first.GetType()==GeomAbs_Cylinder) {
        const auto x=first.Cylinder(),y=second.Cylinder();
        const double scale=std::max({1.0,x.Radius(),y.Radius(),first.LastVParameter()-first.FirstVParameter(),second.LastVParameter()-second.FirstVParameter()});
        const double tolerance=128*std::numeric_limits<double>::epsilon()*scale;
        if(!x.Axis().Direction().IsParallel(y.Axis().Direction(),angular)||
           std::abs(x.Radius()-y.Radius())>tolerance||gp_Lin(x.Axis()).Distance(y.Location())>tolerance)return false;
    } else return false;
    mesher::SurfaceParameterBounds bounds;faceParameterBounds(a,bounds);
    const mesher::Point2D uv{(bounds.uMin+bounds.uMax)*.5,(bounds.vMin+bounds.vMax)*.5};
    mesher::Point2D other;
    if(!faceCoordinates(b,facePoint(a,uv),uv,other))return false;
    const auto n=faceNormal(a,uv),m=faceNormal(b,other);
    const double norm=std::hypot(n.x,n.y,n.z)*std::hypot(m.x,m.y,m.z);
    return std::isfinite(norm)&&norm>0&&(n.x*m.x+n.y*m.y+n.z*m.z)/norm>1-1e-12;
}
/// 判断 UV 点位于裁剪面的内部、边界还是外部。
mesher::CadTrimLocation OccCadGeometry::faceTrimLocation(int f,mesher::Point2D uv) const {
    if(!std::isfinite(uv.x)||!std::isfinite(uv.y))return mesher::CadTrimLocation::Unknown;
    const BRepClass_FaceClassifier classifier(TopoDS::Face(m_faces(f+1)),gp_Pnt2d(uv.x,uv.y),1e-9);
    switch(classifier.State()) {
        case TopAbs_IN:return mesher::CadTrimLocation::Inside;
        case TopAbs_ON:return mesher::CadTrimLocation::Boundary;
        case TopAbs_OUT:return mesher::CadTrimLocation::Outside;
        default:return mesher::CadTrimLocation::Unknown;
    }
}
/// 在实际裁剪面上求最近点及对应 UV 参数。
bool OccCadGeometry::closestTrimCoordinates(int f,mesher::Point3D input,mesher::Point2D seed,mesher::Point2D& uv,double& residual) const {
    if(f<0||f>=faceCount()||!std::isfinite(input.x)||!std::isfinite(input.y)||!std::isfinite(input.z)||
       !std::isfinite(seed.x)||!std::isfinite(seed.y))return false;
    const auto face=TopoDS::Face(m_faces(f+1));const gp_Pnt physical(input.x,input.y,input.z);
    BRepExtrema_DistShapeShape closest(BRepBuilderAPI_MakeVertex(physical).Vertex(),face);
    if(!closest.IsDone()||closest.NbSolution()<1||!std::isfinite(closest.Value()))return false;
    const auto& surface=m_surfaces.at(f);
    const double uPeriod=surface.IsUPeriodic()?surface.UPeriod():0,vPeriod=surface.IsVPeriodic()?surface.VPeriod():0;
    double best=std::numeric_limits<double>::infinity();mesher::Point2D bestUV{};
    // 从实际裁剪面的最近点反求 UV；只投影到底层曲面会把圆环孔洞中的采样点错误接受。
    for(int i=1;i<=closest.NbSolution();++i) {
        const auto nearest=closest.PointOnShape2(i);gp_Pnt supportReference=nearest;
        const double tolerance=std::max(BRep_Tool::Tolerance(face),128*std::numeric_limits<double>::epsilon()*
            std::max({1.,std::abs(nearest.X()),std::abs(nearest.Y()),std::abs(nearest.Z())}));
        const auto accept=[&](mesher::Point2D candidate) {
            if(!std::isfinite(candidate.x)||!std::isfinite(candidate.y))return;
            if(uPeriod>0)candidate.x+=uPeriod*std::round((seed.x-candidate.x)/uPeriod);
            if(vPeriod>0)candidate.y+=vPeriod*std::round((seed.y-candidate.y)/vPeriod);
            for(int u=uPeriod>0?-1:0;u<=(uPeriod>0?1:0);++u)for(int v=vPeriod>0?-1:0;v<=(vPeriod>0?1:0);++v) {
                const mesher::Point2D branch{candidate.x+u*uPeriod,candidate.y+v*vPeriod};
                const auto location=faceTrimLocation(f,branch);
                if(location!=mesher::CadTrimLocation::Inside&&location!=mesher::CadTrimLocation::Boundary)continue;
                const auto p=facePoint(f,branch);const gp_Pnt support(p.x,p.y,p.z);
                if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||support.Distance(supportReference)>tolerance)continue;
                const double distance=physical.Distance(support);
                if(std::isfinite(distance)&&distance<best){best=distance;bestUV=branch;}
            }
        };
        if(closest.SupportTypeShape2(i)==BRepExtrema_IsInFace) {
            mesher::Point2D candidate;closest.ParOnFaceS2(i,candidate.x,candidate.y);accept(candidate);
        } else if(closest.SupportTypeShape2(i)==BRepExtrema_IsOnEdge) {
            const int edge=m_edges.FindIndex(closest.SupportOnShape2(i))-1;double t=0;closest.ParOnEdgeS2(i,t);
            for(const auto& loop:m_topology.at(f).loops)for(auto use:loop)if(use.edge==edge)accept(edgeUV(f,use,t));
        } else {
            const int vertex=m_vertices.FindIndex(closest.SupportOnShape2(i))-1;
            for(const auto& loop:m_topology.at(f).loops)for(auto use:loop) {
                const auto vertices=edgeVertices(use.edge);const auto range=edgeRange(use.edge);
                if(vertices.first==vertex)accept(edgeUV(f,use,range.first));
                if(vertices.second==vertex)accept(edgeUV(f,use,range.second));
            }
        }
        if(closest.SupportTypeShape2(i)!=BRepExtrema_IsInFace||!std::isfinite(best)) {
            // 导入的三维边曲线与面参数曲线可能参数不完全一致；在实际曲面边线上求最小值，不通过扩大容差掩盖差异。
            for(const auto& loop:m_topology.at(f).loops)for(auto use:loop) {
                const auto& trim=m_trimCurves.at({f,use.edge,use.branch?1:0});if(trim.curve.IsNull())continue;
                auto edge=TopoDS::Edge(m_edges(use.edge+1));edge.Orientation(use.branch?TopAbs_REVERSED:TopAbs_FORWARD);
                const BRepAdaptor_Curve curve(edge,face);
                const auto candidateAt=[&](double t){const auto candidate=trim.curve->Value(t);
                    supportReference=curve.Value(t);accept({candidate.X(),candidate.Y()});};
                candidateAt(trim.first);candidateAt(trim.last);
                if(!BRep_Tool::Degenerated(edge)) {
                    Extrema_ExtPC extrema(physical,curve,trim.first,trim.last);
                    if(!extrema.IsDone())continue;
                    for(int e=1;e<=extrema.NbExt();++e)if(extrema.IsMin(e))candidateAt(extrema.Point(e).Parameter());
                }
            }
            supportReference=nearest;
        }
        // 世界坐标曲面也覆盖无法获得边参数的特殊支撑和极点；仍须核对裁剪归属及重建点与 OCC 最近点结果的一致性。
        if(!std::isfinite(best)) {
            GeomAPI_ProjectPointOnSurf projection(nearest,BRep_Tool::Surface(face));
            for(int p=1;p<=projection.NbPoints();++p) {
                mesher::Point2D candidate;projection.Parameters(p,candidate.x,candidate.y);accept(candidate);
            }
        }
    }
    if(!std::isfinite(best))return false;
    uv=bestUV;residual=best;return true;
}
/// 返回几何输入的校验错误说明。
std::string OccCadGeometry::validationError() const { return m_error; }
/// 查询几何的三维包围范围。
bool OccCadGeometry::bounds(mesher::Point3D& lo,mesher::Point3D& hi) const {
    if (!m_error.empty()) return false;
    Bnd_Box box;
    BRepBndLib::Add(m_shape,box,false);
    if (box.IsVoid() || box.IsOpen()) return false;
    box.Get(lo.x,lo.y,lo.z,hi.x,hi.y,hi.z);
    return true;
}
