/** @file GmshInitialMeshLegalizationReplay.cpp
 * @brief 初始BDS面积/度量圆/Quality(false)叶函数重放；不链接CAD/Gmsh。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/InitialMeshLegalization.h"
#include "GmshBdsPoolJson.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
void number(std::ostream& out,double v) {
    if(std::isnan(v)) out << "\"nan\"";
    else if(std::isinf(v)) out << (v < 0. ? "\"-inf\"" : "\"inf\"");
    else if(v == 0. && std::signbit(v)) out << "\"-0\"";
    else out << v;
}
}
int main(int argc,char** argv) {
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic());
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        std::string mode; std::size_t count;
        if(!(in >> mode >> count) || count > 1000000) throw std::runtime_error("invalid metric leaf input");
        if(mode == "area" || mode == "circle") {
            out << '[';
            for(std::size_t i = 0; i < count; ++i) {
                if(i) out << ',';
                if(mode == "area") {
                    std::array<BdsPoint,3> points;
                    for(auto& p : points) if(!(in >> p.uv.x >> p.uv.y >> p.degenerated)) throw std::runtime_error("truncated area");
                    number(out,computeBdsParameterTriangleArea(points[0],points[1],points[2]));
                } else {
                    std::array<Point2D,4> points; SurfaceMetric metric;
                    for(auto& p : points) if(!(in >> p.x >> p.y)) throw std::runtime_error("truncated circle UV");
                    if(!(in >> metric.gUU >> metric.gUV >> metric.gVV)) throw std::runtime_error("truncated metric");
                    const auto r = shouldFlipInitialEdge(points,metric);
                    if(!r.status.complete()) throw OperationFailure{r.status};
                    out << '[' << r.circle.center.x << ',' << r.circle.center.y << ',' << r.circle.radiusSquared << ','
                        << r.queryDistanceSquared << ',' << r.tolerance << ',' << (r.inside ? 1 : 0) << ']';
                }
            }
            out << ']';
        } else if(mode == "swap") {
            BdsMesh mesh;
            for(std::size_t i = 0; i < count; ++i) {
                BdsPoint p; int dim,tag,counterpart,modified;
                if(!(in >> p.id >> p.physicalId >> p.uv.x >> p.uv.y >> p.point.x >> p.point.y >> p.point.z
                    >> dim >> tag >> p.sizes.meshSize >> p.sizes.backgroundSize >> counterpart >> modified >> p.degenerated)) throw std::runtime_error("truncated swap points");
                if(dim >= 0) { p.classification = {tag,dim}; mesh.geometry.insert(p.classification); }
                if(counterpart != -1) p.periodicCounterpart = counterpart; p.configurationModified = modified != 0;
                insertBdsPoint(mesh,p);
            }
            std::size_t triangles; if(!(in >> triangles) || triangles > 1000000) throw std::runtime_error("invalid triangle count");
            for(std::size_t i = 0; i < triangles; ++i) { std::array<int,3> t; if(!(in >> t[0] >> t[1] >> t[2])) throw std::runtime_error("truncated triangles"); appendBdsTriangle(mesh,t); }
            std::size_t labels; if(!(in >> labels) || labels > 1000000) throw std::runtime_error("invalid label count");
            for(std::size_t i = 0; i < labels; ++i) {
                int kind,first,second,tag,dim; if(!(in >> kind >> first >> second >> tag >> dim)) throw std::runtime_error("truncated labels");
                const BdsClassification g{tag,dim}; mesh.geometry.insert(g);
                if(kind == 0) { const int e = findBdsEdge(mesh,first,second); if(e < 0) throw std::runtime_error("missing labelled edge"); mesh.edges[e].classification = g; }
                else if(kind == 1 && first >= 0 && std::size_t(first) < mesh.triangles.size()) mesh.triangles[first].classification = g;
                else throw std::runtime_error("invalid label");
            }
            int first,second; if(!(in >> first >> second)) throw std::runtime_error("missing edge");
            const int edge = findBdsEdge(mesh,first,second); if(edge < 0) throw std::runtime_error("missing swap edge");
            const auto n = getBdsEdgeNeighborhood(mesh,edge); const bool swapped = n && canSwapInitialBdsEdge(mesh,edge,*n);
            if(swapped) swapBdsEdgeTopology(mesh,edge,*n);
            out << "{\"mesh\":"; writeBdsPoolJson(out,mesh); out << ",\"swapped\":" << (swapped ? "true" : "false") << '}';
        } else throw std::runtime_error("unknown metric leaf mode");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected input tail");
        out << '\n'; if(!out) throw std::runtime_error("metric leaf write failed"); return 0;
    } catch(const OperationFailure& e) { std::cerr << e.status.message << '\n'; return 1; }
      catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
