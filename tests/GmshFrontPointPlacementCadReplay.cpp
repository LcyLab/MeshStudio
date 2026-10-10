/** @file GmshFrontPointPlacementCadReplay.cpp
 * @brief 同BREP上的候选点叶重放；只冻结三角形UV/XYZ/双尺寸，不冻结度量或修正。
 * @date 2026-10-08
 */
#include "cad/occ/OccCadGeometry.h"
#include "mesh/surface/algorithm6/front/FrontPointPlacement.h"
#include "GmshFrontPointJson.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
using namespace mesher;
using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    if(argc != 4) return 2;
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read frozen BREP");
        OccCadGeometry geometry(shape); if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream in(argv[2]); in.imbue(std::locale::classic()); std::size_t count;
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid front point case count");
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17) << "{\"cases\":[";
        for(std::size_t i = 0; i < count; ++i) {
            int face,edge,extension; if(!(in >> face >> edge >> extension) || face < 1 || face > geometry.faceCount() || (extension != 0 && extension != 1)) throw std::runtime_error("invalid front point case header");
            FrontTriangle triangle; triangle.vertices = {1,2,3}; FaceMesh mesh; NodeSizeState nodes;
            for(std::uint64_t id = 1; id <= 3; ++id) {
                InitializedNodeSize node; node.vertex = id; FaceVertex vertex; vertex.id = vertex.physicalId = id;
                if(!(in >> node.parameters.x >> node.parameters.y >> vertex.point.x >> vertex.point.y >> vertex.point.z >> node.sizes.meshSize >> node.sizes.backgroundSize)) throw std::runtime_error("truncated front point triangle");
                nodes.indices[id] = nodes.nodes.size(); nodes.nodes.push_back(node); mesh.vertices.emplace(id,vertex);
            }
            if(i) out << ',';
            writeFrontPointJson(out,triangle,edge,mesh,nodes,extension != 0,[&](Point2D uv) {
                Point3D du,dv; geometry.faceSurfaceDerivatives(face-1,uv,du,dv);
                const auto evaluated = evaluateSurfaceMetric(du,dv); if(!evaluated.status.complete()) throw OperationFailure{evaluated.status}; return evaluated.metric;
            },[&](Point2D uv) { return geometry.faceSurfacePoint(face-1,uv); });
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected front point input tail");
        out << "]}\n"; if(!out) throw std::runtime_error("front point output failed"); return 0;
    }
    catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
    catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
