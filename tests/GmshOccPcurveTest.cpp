#include "cad/occ/OccCadGeometry.h"
#include <BRep_Tool.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <ShapeAnalysis.hxx>
#include <iostream>
#include <cmath>
#include <set>
int main()
{
    int failures = 0, queried = 0;
    auto check = [&](bool ok, const char* name) {
        if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    for(const auto& shape : {BRepPrimAPI_MakeSphere(1.).Shape(),BRepPrimAPI_MakeCylinder(1.,2.).Shape(),
                             BRepPrimAPI_MakeTorus(2.,.6).Shape()}) {
        OccCadGeometry geometry(shape);
        check(geometry.validationError().empty(),"valid exact CAD fixture");
        TopTools_IndexedMapOfShape edges, faces;
        TopExp::MapShapes(shape,TopAbs_EDGE,edges); TopExp::MapShapes(shape,TopAbs_FACE,faces);
        for(int faceId = 0; faceId < geometry.faceCount(); ++faceId) {
            const auto face = TopoDS::Face(faces(faceId+1));
            mesher::SurfaceParameterBounds expectedBounds, trimBounds{123.,456.,789.,987.};
            ShapeAnalysis::GetFaceUVBounds(face,expectedBounds.uMin,expectedBounds.uMax,expectedBounds.vMin,expectedBounds.vMax);
            check(geometry.faceTrimParameterBounds(faceId,trimBounds) && trimBounds.uMin == expectedBounds.uMin &&
                  trimBounds.uMax == expectedBounds.uMax && trimBounds.vMin == expectedBounds.vMin && trimBounds.vMax == expectedBounds.vMax,
                  "native chart bounds follow trimmed pcurves, not derivative adaptor bounds");
            std::set<int> unique;
            for(const auto& loop : geometry.faceTopology(faceId).loops)
                for(const auto& use : loop) unique.insert(use.edge);
            for(int edgeId : unique) {
                auto edge = TopoDS::Edge(edges(edgeId+1));
                if(edge.Orientation() == TopAbs_INTERNAL || edge.Orientation() == TopAbs_EXTERNAL) edge.Orientation(TopAbs_FORWARD);
                double first, last;
                if(BRep_Tool::Degenerated(edge)) BRep_Tool::CurveOnSurface(edge,face,first,last);
                else BRep_Tool::Curve(edge,first,last);
                std::pair<double,double> range{123.,456.};
                const bool ranged = geometry.edgeOnFaceRange(faceId,edgeId,range);
                check(ranged && range.first == first && range.second == last,"exact native edge-on-face parameter bounds");
                bool seam = false;
                check(geometry.edgeSeamOnFace(faceId,edgeId,seam) && seam == bool(BRep_Tool::IsClosed(edge,face)),
                      "CAD seam query is explicit and exact");
                for(bool reversed : {false,true}) {
                    const auto oriented = reversed ? TopoDS::Edge(edge.Reversed()) : edge;
                    double low, high; const auto pc = BRep_Tool::CurveOnSurface(oriented,face,low,high);
                    for(double fraction : {0.,.5,1.}) {
                        const double parameter = first+(last-first)*fraction;
                        mesher::Point2D uv{123.,456.};
                        const bool evaluated = geometry.edgeOnFaceCoordinates(faceId,edgeId,parameter,reversed,uv);
                        check(evaluated,"exact pcurve capability is available");
                        if(evaluated && !pc.IsNull()) {
                            const auto expected = pc->Value(parameter);
                            check(uv.x == expected.X() && uv.y == expected.Y(),"original/reversed native curve orientation is preserved");
                            const auto point = geometry.facePoint(faceId,uv), onEdge = geometry.edgePoint(edgeId,parameter);
                            const double dx = point.x-onEdge.x, dy = point.y-onEdge.y, dz = point.z-onEdge.z;
                            check(std::sqrt(dx*dx+dy*dy+dz*dz) < 1.e-10,"pcurve maps back to exact physical edge including poles");
                            ++queried;
                        }
                    }
                }
            }
        }
        std::pair<double,double> range{123.,456.}; mesher::Point2D uv{123.,456.}; bool seam = true;
        check(!geometry.edgeOnFaceRange(-1,0,range) && range == std::make_pair(123.,456.),"bad face retains range output");
        check(!geometry.edgeOnFaceCoordinates(0,geometry.edgeCount(),0.,false,uv) && uv.x == 123. && uv.y == 456.,
              "bad edge retains UV output");
        check(!geometry.edgeSeamOnFace(geometry.faceCount(),0,seam) && seam,"bad face retains seam output");
        mesher::SurfaceParameterBounds trimBounds{123.,456.,789.,987.};
        check(!geometry.faceTrimParameterBounds(-1,trimBounds) && trimBounds.uMin == 123. && trimBounds.uMax == 456. &&
              trimBounds.vMin == 789. && trimBounds.vMax == 987.,"invalid face retains trimmed parameter bounds");
    }
    std::cout << "OCC pcurve queries=" << queried << " failures=" << failures << '\n';
    return failures ? 1 : 0;
}
