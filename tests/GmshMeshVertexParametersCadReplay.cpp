/** @file GmshMeshVertexParametersCadReplay.cpp
 * @brief 从同BREP独立生成拓扑/原始pcurve/投影参数源；不冻结结果UV或候选。
 * @date 2026-10-08
 */
#include "cad/occ/OccMeshVertexParameters.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
using namespace mesher; using namespace mesher::gmshReference;
void require(OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; }
int main(int argc,char** argv) {
    if(argc != 4) return 2;
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read raw parameter BREP");
        OccCadGeometry geometry(shape); if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream in(argv[2]); in.imbue(std::locale::classic()); std::size_t count;
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid parameter case count");
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17) << "{\"cases\":[";
        for(std::size_t item = 0; item < count; ++item) {
            int face,direction,seam,surface; std::size_t n,p;
            if(!(in >> face >> n >> p >> direction >> seam >> surface) || face < 1 || face > geometry.faceCount() || n > 1000000 || p > 1000000) throw std::runtime_error("invalid parameter header");
            const auto source = makeOccMeshVertexParameterSource(geometry,face-1);
            std::vector<MeshVertexParameterInput> nodes(n); std::vector<std::array<int,2>> pairs(p);
            for(auto& node : nodes) {
                auto& v = node.vertex; int hasU,hasV;
                if(!(in >> v.id >> v.dimension >> v.entity >> v.point.x >> v.point.y >> v.point.z >> v.parameters.x >> v.parameters.y >> hasU >> hasV)) throw std::runtime_error("truncated raw parameter node");
                node.hasParameters = {hasU != 0,hasV != 0};
            }
            for(auto& pair : pairs) if(!(in >> pair[0] >> pair[1]) || pair[0] < 0 || pair[1] < 0 || std::size_t(pair[0]) >= n || std::size_t(pair[1]) >= n) throw std::runtime_error("invalid parameter pair");
            if(item) out << ','; out << "{\"nodes\":[";
            for(std::size_t i = 0; i < n; ++i) {
                const auto all = collectMeshVertexParameters(nodes[i],source); require(all.status);
                const auto single = reparameterizeMeshVertexOnFace(nodes[i],source,{surface != 0,seam != 0,direction}); require(single.status);
                if(i) out << ','; out << "{\"all\":[";
                for(std::size_t j = 0; j < all.parameters.size(); ++j) { if(j) out << ','; const auto uv = all.parameters[j]; out << '[' << uv.x << ',' << uv.y << ']'; }
                out << "],\"single\":[" << (single.succeeded ? "true" : "false") << ',' << single.parameter.x << ',' << single.parameter.y << "]}";
            }
            out << "],\"pairs\":[";
            for(std::size_t i = 0; i < p; ++i) {
                const auto result = reparameterizeMeshEdgeOnFace(nodes[pairs[i][0]],nodes[pairs[i][1]],source); require(result.status);
                if(i) out << ','; const auto a = result.parameters[0],b = result.parameters[1]; out << '[' << a.x << ',' << a.y << ',' << b.x << ',' << b.y << ']';
            }
            out << "]}";
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected parameter input tail");
        out << "]}\n"; if(!out) throw std::runtime_error("parameter output failed"); return 0;
    } catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
