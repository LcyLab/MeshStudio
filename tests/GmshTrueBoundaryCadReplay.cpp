/** @file GmshTrueBoundaryCadReplay.cpp
 * @brief 同一 BREP 的精确 OCC pcurve 重放；输入只冻结拓扑身份/朝向/遍历序，不冻结 UV。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/chart/PeriodicFaceChart.h"
#include "cad/occ/OccCadGeometry.h"
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <Standard_Failure.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main(int argc, char** argv)
{
    if(argc != 4) { std::cerr << "usage: GmshTrueBoundaryCadReplay CAD.brep topology-input.tsv output.json\n"; return 2; }
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read exact BREP");
        OccCadGeometry geometry(shape);
        if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        TopTools_IndexedMapOfShape edges, faces;
        TopExp::MapShapes(shape,TopAbs_EDGE,edges); TopExp::MapShapes(shape,TopAbs_FACE,faces);
        std::ifstream in(argv[2]); in.imbue(std::locale::classic());
        int faceTag; std::size_t count;
        if(!(in >> faceTag >> count) || faceTag < 1 || faceTag > faces.Extent() || count > 1000000)
            throw std::runtime_error("invalid topology face identity/count");
        std::vector<TrueBoundaryCurveInput> curves;
        for(std::size_t i = 0; i < count; ++i) {
            int tag, orientation, seam, faceOrientation;
            if(!(in >> tag >> orientation >> seam >> faceOrientation) || tag < 1 || tag > edges.Extent() ||
               orientation < 0 || orientation > 3 || faceOrientation < 0 || faceOrientation > 3 || (seam != 0 && seam != 1))
                throw std::runtime_error("invalid topology edge identity/orientation");
            // 输入仅固定观察到的拓扑遍历序；身份/朝向在自研 BREP 适配器中复核。
            const auto face = TopoDS::Face(faces(faceTag));
            auto edge = TopoDS::Edge(edges(tag));
            if(edge.Orientation()==TopAbs_INTERNAL || edge.Orientation()==TopAbs_EXTERNAL) edge.Orientation(TopAbs_FORWARD);
            if(int(edge.Orientation()) != orientation || int(face.Orientation()) != faceOrientation)
                throw std::runtime_error("CAD orientation differs from native topology metadata");
            TopTools_IndexedMapOfShape members; TopExp::MapShapes(face,TopAbs_EDGE,members);
            bool queriedSeam;
            if(!members.Contains(edge) || !geometry.edgeSeamOnFace(faceTag-1,tag-1,queriedSeam) || bool(seam) != queriedSeam)
                throw std::runtime_error("CAD identity or seam classification differs from topology input");
            TrueBoundaryCurveInput curve; curve.entity = tag; curve.seam = bool(seam);
            std::pair<double,double> range;
            if(!geometry.edgeOnFaceRange(faceTag-1,tag-1,range)) throw std::runtime_error("exact curve-on-face range unavailable");
            curve.first = range.first; curve.last = range.second;
            curve.onFace = [&geometry,faceTag,tag](double t, int branch) {
                Point2D uv;
                if(!geometry.edgeOnFaceCoordinates(faceTag-1,tag-1,t,branch!=1,uv))
                    throw std::runtime_error("exact pcurve missing; generic/robust fallback not reproduced");
                return uv;
            };
            curves.push_back(std::move(curve));
        }
        if(!in) throw std::runtime_error("truncated topology input");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected topology input tail");
        std::vector<TrueBoundarySegment> segments;
        const auto result = sampleTrueBoundary(curves,segments);
        if(!result.status.complete()) throw std::runtime_error(result.status.message);
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        out << "{\"face\":" << faceTag << ",\"segments\":[";
        for(std::size_t i = 0; i < segments.size(); ++i) {
            if(i) out << ','; const auto& s = segments[i];
            out << '[' << s.curveEntity << ',' << s.branch << ',' << s.from.x << ',' << s.from.y << ',' << s.to.x << ',' << s.to.y << ']';
        }
        out << "]}\n";
        if(!out) throw std::runtime_error("true boundary output write failed");
        return 0;
    }
    catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
