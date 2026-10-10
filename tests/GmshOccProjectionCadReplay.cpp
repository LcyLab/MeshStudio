/** @file GmshOccProjectionCadReplay.cpp
 * @brief 同一 BREP 的原生支撑面投影；冻结查询点，不冻结投影结果。
 * @date 2026-10-07
 */
#include "cad/occ/OccCadGeometry.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
int main(int argc, char** argv)
{
    if(argc != 4) { std::cerr << "usage: GmshOccProjectionCadReplay CAD.brep queries.tsv output.json\n"; return 2; }
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read frozen BREP");
        OccCadGeometry geometry(shape);
        if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream in(argv[2]); in.imbue(std::locale::classic());
        std::size_t count;
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid projection query count");
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17) << "{\"queries\":[";
        for(std::size_t i = 0; i < count; ++i) {
            int face; mesher::Point3D point; mesher::Point2D seed,uv; mesher::Point3D projected;
            if(!(in >> face >> point.x >> point.y >> point.z >> seed.x >> seed.y) || face < 1 || face > geometry.faceCount())
                throw std::runtime_error("invalid projection query or face identity");
            // Native OCC projection ignores the seed on its successful direct path.
            if(!geometry.projectOnFaceSurface(face-1,point,uv,projected))
                throw std::runtime_error("native direct projection unavailable; generic inverse fallback not reproduced");
            const auto surface = geometry.facePoint(face-1,uv);
            if(i) out << ',';
            out << '[' << face << ',' << uv.x << ',' << uv.y << ',' << projected.x << ',' << projected.y << ',' << projected.z << ','
                << surface.x << ',' << surface.y << ',' << surface.z << ']';
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected projection query tail");
        out << "]}\n"; if(!out) throw std::runtime_error("projection output write failed");
        return 0;
    }
    catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
