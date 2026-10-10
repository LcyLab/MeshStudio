/** @file GmshPeriodicBoundaryLoopReplay.cpp
 * @brief 冻结原生边 pcurve 后重放周期环拼接；不链接 CAD/Gmsh。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/chart/FaceChart.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main(int argc, char** argv)
{
    if(argc != 3) { std::cerr << "usage: GmshPeriodicBoundaryLoopReplay input.tsv output.json\n"; return 2; }
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic());
        double tolerance; bool seamFirst; std::size_t count;
        if(!(in >> tolerance >> seamFirst >> count) || count > 1000000)
            throw std::runtime_error("invalid frozen loop header");
        std::vector<PeriodicCurveChart> curves;
        for(std::size_t i = 0; i < count; ++i) {
            PeriodicCurveChart curve; std::size_t points;
            if(!(in >> curve.entity >> curve.seam >> points) || points > 1000000)
                throw std::runtime_error("invalid frozen curve header");
            for(std::size_t j = 0; j < points; ++j) {
                PeriodicLoopVertex vertex; Point2D alternate;
                in >> vertex.physicalId >> vertex.uv.x >> vertex.uv.y;
                if(curve.seam) { in >> alternate.x >> alternate.y; curve.alternateCoordinates.push_back(alternate); }
                curve.vertices.push_back(vertex);
            }
            curves.push_back(std::move(curve));
        }
        if(!in) throw std::runtime_error("truncated frozen loop input");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected frozen input tail");
        PeriodicBoundaryLoop loop;
        const auto status = assemblePeriodicBoundaryLoop(curves,tolerance,seamFirst,loop);
        if(!status.complete() && status.code != MeshingStatus::NonConverged)
            throw std::runtime_error(status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        out << "{\"closed\":" << (status.complete() ? "true" : "false") << ",\"vertices\":[";
        for(std::size_t i = 0; i < loop.vertices.size(); ++i) {
            if(i) out << ','; const auto& v = loop.vertices[i];
            out << '[' << v.physicalId << ',' << v.uv.x << ',' << v.uv.y << ']';
        }
        out << "]}\n";
        if(!out) throw std::runtime_error("loop replay write failed");
        return 0;
    }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
