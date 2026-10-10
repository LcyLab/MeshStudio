#pragma once
/** @file Algorithm6CadFixtures.h
 * @brief 同一BREP保存周期裁剪孔洞及真实INTERNAL约束，供两侧独立生成。
 * @date 2026-10-09
 */
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepCheck_ListIteratorOfListOfStatus.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom2d_Line.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Pln.hxx>
#include <cmath>
#include <stdexcept>
#include <sstream>

namespace algorithm6fixtures {
inline void requireValid(const TopoDS_Shape& shape,const char* name) {
    if(!BRepLib::BuildCurves3d(shape))throw std::runtime_error(std::string(name)+": 3D curve construction failed");
    BRepCheck_Analyzer checker(shape);if(checker.IsValid())return;
    std::ostringstream message;message<<name<<": invalid CAD";
    for(auto kind:{TopAbs_FACE,TopAbs_WIRE,TopAbs_EDGE,TopAbs_VERTEX})for(TopExp_Explorer part(shape,kind);part.More();part.Next()) {
        const auto result=checker.Result(part.Current());if(result.IsNull())continue;
        auto record=[&](const BRepCheck_ListOfStatus& values){for(BRepCheck_ListIteratorOfListOfStatus i(values);i.More();i.Next())if(i.Value()!=BRepCheck_NoError)message<<" shape="<<int(kind)<<" orientation="<<int(part.Current().Orientation())<<" status="<<int(i.Value());};
        record(result->Status());
        for(result->InitContextIterator();result->MoreShapeInContext();result->NextShapeInContext())record(result->StatusOnShape());
    }
    throw std::runtime_error(message.str());
}
inline TopoDS_Edge parametricEdge(const Handle(Geom_Surface)& surface,gp_Pnt2d a,gp_Pnt2d b) {
    const double length=a.Distance(b);
    const Handle(Geom2d_Curve) line=new Geom2d_Line(a,gp_Dir2d(b.X()-a.X(),b.Y()-a.Y()));
    return BRepBuilderAPI_MakeEdge(line,surface,0,length).Edge();
}
inline TopoDS_Shape periodicHole() {
    const Handle(Geom_Surface) support=new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0,0,0),gp_Dir(0,0,1)),2);
    BRepBuilderAPI_MakeFace face(support,.2,2.4,0,3,1.e-7);
    BRepBuilderAPI_MakeWire hole;
    const gp_Pnt2d p[4]={{.8,.8},{.8,2.2},{1.5,2.2},{1.5,.8}};
    for(int k=0;k<4;++k)hole.Add(parametricEdge(support,p[k],p[(k+1)%4]));
    face.Add(hole.Wire());auto result=face.Face();
    requireValid(result,"periodic hole fixture");
    return result;
}
inline TopoDS_Shape embeddedPlane() {
    const Handle(Geom_Surface) support=new Geom_Plane(gp_Pln(gp_Pnt(0,0,0),gp_Dir(0,0,1)));
    auto face=BRepBuilderAPI_MakeFace(support,0,4,0,3,1.e-7).Face();
    // INTERNAL属于边，而不是整条wire；整wire设INTERNAL会违反OCC面环方向检查。
    auto edge=parametricEdge(support,{1,1},{3,1});edge.Orientation(TopAbs_INTERNAL);
    TopoDS_Wire wire;BRep_Builder builder;builder.MakeWire(wire);builder.Add(wire,edge);builder.Add(face,wire);
    requireValid(face,"embedded plane fixture");
    return face;
}
inline TopoDS_Shape embeddedCylinder() {
    const auto shape=BRepPrimAPI_MakeCylinder(2,3).Shape();
    for(TopExp_Explorer f(shape,TopAbs_FACE);f.More();f.Next()) {
        auto face=TopoDS::Face(f.Current());if(BRepAdaptor_Surface(face).GetType()!=GeomAbs_Cylinder)continue;
        // 正在构造自己的新测试实体；嵌入成员写入前解锁已挂入shell的侧面。
        const bool wasFree=face.Free();face.Free(true);
        const auto support=BRep_Tool::Surface(face);
        auto edge=parametricEdge(support,{.8,1.5},{1.8,1.5});edge.Orientation(TopAbs_INTERNAL);
        TopoDS_Wire wire;BRep_Builder builder;builder.MakeWire(wire);builder.Add(wire,edge);builder.Add(face,wire);
        auto vertex=BRepBuilderAPI_MakeVertex(gp_Pnt(2*std::cos(2.8),2*std::sin(2.8),2.1)).Vertex();
        vertex.Orientation(TopAbs_INTERNAL);builder.Add(face,vertex);
        face.Free(wasFree);
        requireValid(shape,"embedded cylinder fixture");
        return shape;
    }
    throw std::runtime_error("embedded cylinder side missing");
}
}
