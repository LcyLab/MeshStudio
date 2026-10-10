/** @file GmshOccProjectionTest.cpp
 * @brief 验证与原生 OCCFace 相同的受限 UV 支撑面投影，不按孔夹紧。
 * @date 2026-10-07
 */
#include "cad/occ/OccCadGeometry.h"
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <ShapeAnalysis.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

int main()
{
    int failures = 0, queries = 0;
    auto check = [&](bool ok, const char* name) {
        if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    const gp_Ax2 axes(gp_Pnt(0,0,0),gp_Dir(0,0,1));
    const TopoDS_Wire outer = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(axes,2.)));
    TopoDS_Wire inner = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(axes,.5)));
    inner.Reverse(); BRepBuilderAPI_MakeFace annulus(outer); annulus.Add(inner);
    gp_Trsf transform; transform.SetTranslation(gp_Vec(10.,-4.,7.));
    const auto shifted = BRepBuilderAPI_Transform(BRepPrimAPI_MakeCylinder(1.,2.).Shape(),transform).Shape();
    const std::vector<TopoDS_Shape> shapes{annulus.Shape(),BRepPrimAPI_MakeSphere(1.).Shape(),
        BRepPrimAPI_MakeCylinder(1.,2.).Shape(),BRepPrimAPI_MakeTorus(2.,.6).Shape(),shifted};
    for(const auto& shape : shapes) {
        OccCadGeometry geometry(shape);
        check(geometry.validationError().empty(),"valid projection fixture");
        TopTools_IndexedMapOfShape faces; TopExp::MapShapes(shape,TopAbs_FACE,faces);
        for(int f = 0; f < geometry.faceCount(); ++f) {
            const auto face = TopoDS::Face(faces(f+1));
            BRepAdaptor_Surface surface(face); const auto support = BRep_Tool::Surface(face);
            double umin,umax,vmin,vmax; ShapeAnalysis::GetFaceUVBounds(face,umin,umax,vmin,vmax);
            const double uc = .5*(umin+umax), vc = .5*(vmin+vmax);
            if(!surface.IsUPeriodic()) { const double d = std::max(std::abs(umax-umin)*1.e-8,1.e-12); umin -= d; umax += d; }
            if(!surface.IsVPeriodic()) { const double d = std::max(std::abs(vmax-vmin)*1.e-8,1.e-12); vmin -= d; vmax += d; }
            GeomAPI_ProjectPointOnSurf reference; reference.Init(support,umin,umax,vmin,vmax);
            for(const auto& parameter : std::vector<mesher::Point2D>{{uc,vc},{umin+.17*(umax-umin),vmin+.27*(vmax-vmin)},
                                                                   {umin+.83*(umax-umin),vmin+.73*(vmax-vmin)}}) {
                const auto on = geometry.facePoint(f,parameter);
                for(const double offset : {0.,.1}) {
                    const mesher::Point3D query{on.x+.03*offset,on.y-.02*offset,on.z+offset};
                    reference.Perform(gp_Pnt(query.x,query.y,query.z));
                    check(reference.NbPoints() > 0,"native projection fixture has a solution");
                    if(!reference.NbPoints()) continue;
                    double u,v; reference.LowerDistanceParameters(u,v); const auto expected = reference.NearestPoint();
                    mesher::Point2D uv{123.,456.}; mesher::Point3D projected{789.,987.,654.};
                    const bool ok = geometry.projectOnFaceSurface(f,query,uv,projected);
                    check(ok,"exact native projection capability is available");
                    if(ok) {
                        check(uv.x == u && uv.y == v && projected.x == expected.X() && projected.y == expected.Y() && projected.z == expected.Z(),
                              "restricted UV projection and closest point match native OCC exactly");
                        ++queries;
                    }
                }
            }
        }
        mesher::Point2D uv{123.,456.}; mesher::Point3D projected{789.,987.,654.};
        check(!geometry.projectOnFaceSurface(-1,{},uv,projected) && uv.x == 123. && projected.z == 654.,
              "invalid face preserves projection outputs");
        check(!geometry.projectOnFaceSurface(0,{std::numeric_limits<double>::infinity(),0.,0.},uv,projected) &&
              uv.y == 456. && projected.x == 789.,"nonfinite query preserves projection outputs");
    }
    OccCadGeometry ring(annulus.Shape()); mesher::Point2D uv; mesher::Point3D projected;
    const bool hole = ring.projectOnFaceSurface(0,{0.,0.,.1},uv,projected);
    check(hole && projected.x == 0. && projected.y == 0. && projected.z == 0. &&
          ring.faceTrimLocation(0,uv) == mesher::CadTrimLocation::Outside,"support projection does not move a point inside a CAD hole onto its edge");
    std::cout << "OCC native projections=" << queries << " failures=" << failures << '\n';
    return failures ? 1 : 0;
}
