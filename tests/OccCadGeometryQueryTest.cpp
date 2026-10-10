// 对照直接 OCC 查询，验证求值对象复用不改变 Location、方向及缝分支。
#include "cad/occ/OccCadGeometry.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <gp_Circ.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <STEPControl_Reader.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <Geom2d_Curve.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax1.hxx>
#include <type_traits>
#include <iostream>
#include <iomanip>

namespace {
bool same(mesher::Point3D a,const gp_XYZ& b) { return a.x==b.X() && a.y==b.Y() && a.z==b.Z(); }
template<class T,class=void> struct Preparation {
    static bool check(const T&,int,int) { return false; }
};
template<class T> struct Preparation<T,std::void_t<decltype(std::declval<T>().preparationStatistics())>> {
    static bool check(const T& g,int faces,int curves) {
        const auto s=g.preparationStatistics();
        return s.surfaceAdaptors==size_t(faces) && s.curveAdaptors==size_t(curves) && s.pcurves>0;
    }
};
bool verify(TopoDS_Shape shape,bool& seam,bool& pole) {
    OccCadGeometry g(shape);
    if (!g.validationError().empty()) return false;
    TopTools_IndexedMapOfShape faces,edges,vertices;
    TopExp::MapShapes(shape,TopAbs_FACE,faces); TopExp::MapShapes(shape,TopAbs_EDGE,edges);
    TopExp::MapShapes(shape,TopAbs_VERTEX,vertices);
    int regularCurves=0;
    for (int e=0;e<g.edgeCount();++e) {
        const auto edge=TopoDS::Edge(edges(e+1));
        const bool degenerate=BRep_Tool::Degenerated(edge); pole|=degenerate;
        if (g.edgeDegenerate(e)!=degenerate) return false;
        if (!degenerate) ++regularCurves;
        const auto range=g.edgeRange(e);
        for (double w:{0.0,0.13,0.73,1.0}) {
            const double t=range.first+(range.second-range.first)*w;
            const auto p=degenerate?BRep_Tool::Pnt(TopoDS::Vertex(vertices(g.edgeVertices(e).first+1))):BRepAdaptor_Curve(edge).Value(t);
            if (!same(g.edgePoint(e,t),p.XYZ())) return false;
            gp_Pnt directPoint;gp_Vec derivative;
            if(!degenerate)BRepAdaptor_Curve(edge).D1(t,directPoint,derivative);
            if(!same(g.edgeDerivative(e,t),derivative.XYZ()))return false;
        }
    }
    for (int f=0;f<g.faceCount();++f) {
        const auto face=TopoDS::Face(faces(f+1)); BRepAdaptor_Surface direct(face);
        mesher::SurfaceParameterBounds b; if (!g.faceParameterBounds(f,b)) return false;
        if (b.uMin!=direct.FirstUParameter() || b.uMax!=direct.LastUParameter() ||
            b.vMin!=direct.FirstVParameter() || b.vMax!=direct.LastVParameter()) return false;
        // 反向顺序再次求值，使缓存不能依赖“参数只递增”。
        for (double w:{0.83,0.21,0.61,0.21}) {
            const mesher::Point2D uv{b.uMin+(b.uMax-b.uMin)*w,b.vMin+(b.vMax-b.vMin)*(1-w)};
            gp_Pnt p; gp_Vec du,dv; direct.D1(uv.x,uv.y,p,du,dv);
            mesher::Point3D u,v; g.faceDerivatives(f,uv,u,v);
            gp_Vec n=du.Crossed(dv); if (face.Orientation()==TopAbs_REVERSED) n.Reverse();
            if (!same(g.facePoint(f,uv),p.XYZ()) || !same(u,du.XYZ()) || !same(v,dv.XYZ()) || !same(g.faceNormal(f,uv),n.XYZ())) return false;
        }
        for (const auto& loop:g.faceTopology(f).loops) for (auto use:loop) {
            const auto original=TopoDS::Edge(edges(use.edge+1)); seam|=BRep_Tool::IsClosed(original,face);
            const auto range=g.edgeRange(use.edge);
            for (int branch:{0,1}) {
                auto edge=original; edge.Orientation(branch?TopAbs_REVERSED:TopAbs_FORWARD); use.branch=branch;
                double a,b; const auto pc=BRep_Tool::CurveOnSurface(edge,face,a,b);
                if (pc.IsNull()) return false;
                for (double w:{0.0,0.17,0.91,1.0}) {
                    const double t=range.first+(range.second-range.first)*w; const auto p=pc->Value(t); const auto q=g.edgeUV(f,use,t);
                    if (p.X()!=q.x || p.Y()!=q.y) return false;
                }
            }
        }
    }
    return Preparation<OccCadGeometry>::check(g,g.faceCount(),regularCurves);
}
}
int main(int argc,char** argv) {
    if(argc==2) {
        STEPControl_Reader reader;if(reader.ReadFile(argv[1])!=IFSelect_RetDone||!reader.TransferRoots())return 2;
        const auto shape=reader.OneShape();OccCadGeometry source(shape);TopTools_IndexedMapOfShape faces;TopExp::MapShapes(shape,TopAbs_FACE,faces);
        const mesher::Point3D sample{50,485.25948575741745,-108.96866149419685};const mesher::Point2D seed{1.468118673680076,-.21976710721158882};
        mesher::Point2D uv;double distance=0;const bool supported=source.closestTrimCoordinates(0,sample,seed,uv,distance);
        std::cout<<std::setprecision(17)<<"real trim support="<<supported<<" uv="<<uv.x<<','<<uv.y<<" distance="<<distance<<'\n';
        const auto face=TopoDS::Face(faces(1));BRepExtrema_DistShapeShape nearest(BRepBuilderAPI_MakeVertex(gp_Pnt(sample.x,sample.y,sample.z)).Vertex(),face);
        std::cout<<"nearest done="<<nearest.IsDone()<<" distance="<<nearest.Value()<<" faceTolerance="<<BRep_Tool::Tolerance(face)<<'\n';
        TopTools_IndexedMapOfShape edges;TopExp::MapShapes(shape,TopAbs_EDGE,edges);
        for(int i=1;i<=nearest.NbSolution();++i) {
            std::cout<<"supportType="<<nearest.SupportTypeShape2(i)<<'\n';
            if(nearest.SupportTypeShape2(i)==BRepExtrema_IsOnEdge) {
                const int edge=edges.FindIndex(nearest.SupportOnShape2(i))-1;double t;nearest.ParOnEdgeS2(i,t);
                for(const auto& loop:source.faceTopology(0).loops)for(auto use:loop)if(use.edge==edge) {
                    const auto q=source.edgeUV(0,use,t);const auto p=source.facePoint(0,q);const auto closest=nearest.PointOnShape2(i);
                    std::cout<<"edge="<<edge<<" edgeTolerance="<<BRep_Tool::Tolerance(TopoDS::Edge(edges(edge+1)))<<" uv="<<q.x<<','<<q.y<<" classification="<<int(source.faceTrimLocation(0,q))
                        <<" reconstruction="<<gp_Pnt(p.x,p.y,p.z).Distance(closest)<<'\n';
                }
            }
        }
        return supported?0:1;
    }
    {
        const gp_Ax2 axes(gp_Pnt(0,0,0),gp_Dir(0,0,1));
        const TopoDS_Wire outer=BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(axes,5)));
        TopoDS_Wire inner=BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(axes,2)));inner.Reverse();
        BRepBuilderAPI_MakeFace face(outer);face.Add(inner);OccCadGeometry ring(face.Shape());
        mesher::Point2D seed{},uv;double residual=0;
        const mesher::Point3D outside{1.9,0,0};ring.faceCoordinates(0,outside,{},seed);
        if(!ring.validationError().empty()||ring.faceTrimLocation(0,seed)!=mesher::CadTrimLocation::Outside||
           !ring.closestTrimCoordinates(0,outside,seed,uv,residual)||std::abs(residual-.1)>1e-8||
           ring.faceTrimLocation(0,uv)!=mesher::CadTrimLocation::Boundary) {
            std::cerr<<"FAIL nearest support must lie on actual trimmed annulus, not its underlying plane\n";return 1;
        }
        const auto p=ring.facePoint(0,uv);
        if(std::hypot(p.x-2,p.y,p.z)>1e-8)return 1;
    }
    {
        auto box=BRepPrimAPI_MakeBox(2,3,4).Shape();OccCadGeometry valid(box,true);
        if(!valid.validationError().empty() || valid.inputWasReconciled() || !valid.acceptedShape().IsSame(box))return 1;
        BRepBuilderAPI_MakePolygon bowtie;
        for(auto p:{gp_Pnt(0,0,0),gp_Pnt(2,2,0),gp_Pnt(0,2,0),gp_Pnt(2,0,0)})bowtie.Add(p);
        bowtie.Close();auto invalid=BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,0,0),gp_Dir(0,0,1)),bowtie.Wire()).Shape();
        if(BRepCheck_Analyzer(invalid).IsValid()) {std::cerr<<"invalid fixture unexpectedly valid\n";return 1;}
        OccCadGeometry refused(invalid,true);
        if(refused.validationError().empty() || refused.inputWasReconciled() || BRepCheck_Analyzer(invalid).IsValid()) {
            std::cerr<<"real interior trim crossing accepted or source mutated\n";return 1;
        }
    }
    bool seam=false,pole=false;
    for (TopoDS_Shape shape:{BRepPrimAPI_MakeBox(2,3,4).Shape(),BRepPrimAPI_MakeCylinder(3,7).Shape(),BRepPrimAPI_MakeSphere(3).Shape()}) {
        gp_Trsf pose; pose.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(1,2,3)),0.47); pose.SetTranslationPart(gp_Vec(12,-7,3));
        shape.Move(TopLoc_Location(pose));
        if (!verify(shape,seam,pole)) { std::cerr<<"FAIL prepared geometry/direct OCC query equivalence\n"; return 1; }
        shape.Reverse(); if (!verify(shape,seam,pole)) return 1;
    }
    if (!seam || !pole) return 1;
    std::cout<<"PASS prepared OCC queries, transformed/reversed surfaces, seam branches and poles\n";
}
