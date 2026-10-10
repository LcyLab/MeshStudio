#pragma once

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepFeat_SplitShape.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <stdexcept>

namespace occsplit {
// Split a face inside one solid. A boolean slice into two solids would test
// body separation rather than the removable CAD segmentation needed here.
inline TopoDS_Shape plane(bool split) {
    const auto shape=BRepPrimAPI_MakeBox(16,12,4).Shape();
    if(!split)return shape;
    for(TopExp_Explorer f(shape,TopAbs_FACE);f.More();f.Next()) {
        const auto face=TopoDS::Face(f.Current());BRepAdaptor_Surface s(face);
        if(s.GetType()!=GeomAbs_Plane || std::abs(s.Plane().Location().Z()-4)>1e-9)continue;
        BRepFeat_SplitShape operation(shape);
        operation.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(8,0,4),gp_Pnt(8,12,4)).Edge(),face);
        operation.Build();
        if(operation.IsDone() && BRepCheck_Analyzer(operation.Shape()).IsValid())return operation.Shape();
        throw std::runtime_error("failed same-solid plane split fixture");
    }
    throw std::runtime_error("top plane missing in fixture");
}
inline TopoDS_Shape cylinder(bool split) {
    const auto shape=BRepPrimAPI_MakeCylinder(2,8).Shape();
    if(!split)return shape;
    for(TopExp_Explorer f(shape,TopAbs_FACE);f.More();f.Next()) {
        const auto face=TopoDS::Face(f.Current());BRepAdaptor_Surface s(face);
        if(s.GetType()!=GeomAbs_Cylinder)continue;
        const gp_Circ circle(gp_Ax2(gp_Pnt(0,0,4),gp_Dir(0,0,1)),2);
        BRepFeat_SplitShape operation(shape);
        operation.Add(BRepBuilderAPI_MakeEdge(circle).Edge(),face);operation.Build();
        if(operation.IsDone() && BRepCheck_Analyzer(operation.Shape()).IsValid())return operation.Shape();
        throw std::runtime_error("failed same-solid cylinder split fixture");
    }
    throw std::runtime_error("cylinder missing in fixture");
}
} // namespace occsplit
