/** @file GmshWalkTriangulationReplay.cpp
 * @brief 普通初始入口/纯 Walk 池重放，不读取原生连通性。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
#include "GmshInitialPoolJson.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main(int argc,char** argv)
{
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic()); std::size_t count; std::string mode;
        if(!(in >> mode >> count) || count > 1000000) throw std::runtime_error("invalid initial point count");
        Point2D lower,upper;
        if(mode == "walk" && !(in >> lower.x >> lower.y >> upper.x >> upper.y)) throw std::runtime_error("invalid rectangle");
        std::vector<InitialChartVertex> points(count);
        for(auto& p : points) if(!(in >> p.physicalId >> p.uv.x >> p.uv.y)) throw std::runtime_error("truncated initial points");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected initial point tail");
        WalkTriangulation mesh; OperationStatus status;
        if(mode == "walk") status = buildWalkTriangulation(lower,upper,points,mesh);
        else if(mode == "initial") {
            FaceChart chart; chart.initialVertices = points;
            for(std::size_t i = 0; i < points.size(); ++i) {
                FaceChartVertex v; v.localId = int(i); v.physicalId = points[i].physicalId; chart.vertices.push_back(v);
            }
            OrdinaryInitialTriangulation initial; status = buildInitialTriangulation(chart,initial); mesh = std::move(initial.mesh);
        }
        else throw std::runtime_error("unknown initial mode");
        if(!status.complete()) throw std::runtime_error(status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17) << '{';
        writeInitialPoolFields(out,mesh); out << "}\n"; if(!out) throw std::runtime_error("initial pool output failed");
        return 0;
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
