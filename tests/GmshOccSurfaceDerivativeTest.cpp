/** @file GmshOccSurfaceDerivativeTest.cpp
 * @brief 支撑面直接求导/求值的B样条舍入、位置变换、反向与重复查询验证。
 * @date 2026-10-08
 */
#include "cad/occ/OccCadGeometry.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Tool.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TopoDS.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Ax1.hxx>
#include <iostream>
#include <stdexcept>
int main() {
    TColgp_Array2OfPnt poles(1,4,1,4);
    for(int i = 1; i <= 4; ++i) for(int j = 1; j <= 4; ++j) {
        const double u = (i-1)/3.,v = (j-1)/3.; poles(i,j) = gp_Pnt(4.*u+1.5*v,3.*v,1.2*u*v+.4*u*u);
    }
    TColStd_Array1OfReal knots(1,2); knots(1) = 0.; knots(2) = 1.;
    TColStd_Array1OfInteger mult(1,2); mult(1) = 4; mult(2) = 4;
    Handle(Geom_BSplineSurface) surface = new Geom_BSplineSurface(poles,knots,knots,mult,mult,3,3);
    const TopoDS_Face base = BRepBuilderAPI_MakeFace(surface,1.e-7);
    gp_Trsf transform; transform.SetRotation(gp_Ax1(gp_Pnt(0.,0.,0.),gp_Dir(1.,2.,3.)),.43);
    transform.SetTranslationPart(gp_Vec(12.,-7.,3.));
    for(bool located : {false,true}) for(bool reversed : {false,true}) {
        TopoDS_Shape shape = located ? base.Located(TopLoc_Location(transform)) : base;
        if(reversed) shape.Reverse(); const auto face = TopoDS::Face(shape);
        OccCadGeometry geometry(shape); if(!geometry.validationError().empty()) { std::cerr << geometry.validationError() << '\n'; return 1; }
        const auto direct = BRep_Tool::Surface(face);
        for(mesher::Point2D uv : {mesher::Point2D{.8214285714285707,.8333333333333341},{.17,.91},{.71,.21},{.17,.91}}) {
            gp_Pnt p; gp_Vec u,v; direct->D1(uv.x,uv.y,p,u,v);
            mesher::Point3D du,dv; geometry.faceSurfaceDerivatives(0,uv,du,dv);
            if(du.x != u.X() || du.y != u.Y() || du.z != u.Z() || dv.x != v.X() || dv.y != v.Y() || dv.z != v.Z()) { std::cerr << "support surface derivative differs\n"; return 1; }
            const auto expected = direct->Value(uv.x,uv.y);
            const auto actual = geometry.faceSurfacePoint(0,uv);
            if(actual.x != expected.X() || actual.y != expected.Y() || actual.z != expected.Z()) { std::cerr << "support surface point differs\n"; return 1; }
        }
        mesher::Point3D du{8.,9.,10.},dv{11.,12.,13.}; bool rejected = false;
        try { geometry.faceSurfaceDerivatives(-1,{0.,0.},du,dv); } catch(const std::out_of_range&) { rejected = true; }
        if(!rejected || du.x != 8. || dv.x != 11.) { std::cerr << "invalid face mutated derivatives\n"; return 1; }
    }
    return 0;
}
