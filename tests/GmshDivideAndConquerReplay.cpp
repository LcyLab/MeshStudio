/** @file GmshDivideAndConquerReplay.cpp
 * @brief 纯自研分治剖分重放；输入为固定原生 DocRecord 的已扰动点。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/initial/DivideAndConquerTriangulation.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher::gmshReference;
int main(int argc,char** argv)
{
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic()); std::size_t count;
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid point count");
        std::vector<InitialPoint> points(count);
        for(auto& p : points) if(!(in >> p.localId >> p.coordinates.x >> p.coordinates.y)) throw std::runtime_error("truncated point record");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected point tail");
        DivideAndConquerTriangulation mesh;
        const auto status = buildDivideAndConquerTriangulation(points,mesh);
        if(!status.complete()) throw std::runtime_error(status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        out << "{\"hull_count\":" << mesh.hullPointCount << ",\"sorted_points\":[";
        for(std::size_t i = 0; i < mesh.sortedPoints.size(); ++i) {
            if(i) out << ','; const auto& p = mesh.sortedPoints[i]; out << '[' << p.localId << ',' << p.coordinates.x << ',' << p.coordinates.y << ']';
        }
        out << "],\"triangles\":[";
        for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
            if(i) out << ','; const auto& t = mesh.triangles[i]; out << '[' << t[0] << ',' << t[1] << ',' << t[2] << ']';
        }
        out << "]}\n"; if(!out) throw std::runtime_error("point replay output failed");
        return 0;
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
