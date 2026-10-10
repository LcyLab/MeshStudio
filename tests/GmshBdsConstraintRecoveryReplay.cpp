/** @file GmshBdsConstraintRecoveryReplay.cpp
 * @brief BDS数值求交和单次恢复的全池/穿越扫描重放。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsConstraintRecovery.h"
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
void number(std::ostream& out,double value) {
    if(std::isnan(value)) out << "\"nan\"";
    else if(std::isinf(value)) out << (value < 0. ? "\"-inf\"" : "\"inf\"");
    else if(value == 0. && std::signbit(value)) out << "\"-0\"";
    else out << value;
}
}
int main(int argc,char** argv)
{
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic()); std::string mode; std::size_t count;
        if(!(in >> mode >> count) || count > 1000000) throw std::runtime_error("invalid BDS recovery input");
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        if(mode == "intersection") {
            out << '[';
            for(std::size_t i = 0; i < count; ++i) {
                Point2D a,b,c,d; if(!(in >> a.x >> a.y >> b.x >> b.y >> c.x >> c.y >> d.x >> d.y)) throw std::runtime_error("truncated BDS intersection");
                const auto result = intersectBdsRecoverySegments(a,b,c,d);
                if(i) out << ','; out << '[' << int(result.systemSolved) << ',' << int(result.intersects) << ',';
                number(out,result.parameters.x); out << ','; number(out,result.parameters.y); out << ']';
            }
            out << "]\n";
        }
        else if(mode == "recover") {
            BdsMesh mesh;
            for(std::size_t i = 0; i < count; ++i) {
                BdsPoint p; int dimension,tag,counterpart,modified;
                if(!(in >> p.id >> p.physicalId >> p.uv.x >> p.uv.y >> p.point.x >> p.point.y >> p.point.z >> dimension >> tag >> p.sizes.meshSize >> p.sizes.backgroundSize >> counterpart >> modified)) throw std::runtime_error("truncated BDS recovery points");
                p.classification = {tag,dimension}; p.configurationModified = modified != 0;
                if(counterpart != -1) p.periodicCounterpart = counterpart;
                insertBdsPoint(mesh,std::move(p));
            }
            std::size_t triangles; if(!(in >> triangles) || triangles > 1000000) throw std::runtime_error("invalid BDS triangle count");
            for(std::size_t i = 0; i < triangles; ++i) {
                std::array<int,3> nodes; if(!(in >> nodes[0] >> nodes[1] >> nodes[2])) throw std::runtime_error("truncated BDS triangles");
                appendBdsTriangle(mesh,nodes);
            }
            std::size_t labels; if(!(in >> labels) || labels > 1000000) throw std::runtime_error("invalid BDS labels");
            for(std::size_t i = 0; i < labels; ++i) {
                int kind,first,second,tag,dimension;
                if(!(in >> kind >> first >> second >> tag >> dimension)) throw std::runtime_error("truncated BDS labels");
                const BdsClassification g{tag,dimension}; mesh.geometry.insert(g);
                if(kind == 0) { const int edge = findBdsEdge(mesh,first,second); if(edge < 0) throw std::runtime_error("label edge missing"); mesh.edges[edge].classification = g; }
                else if(kind == 1 && first >= 0 && std::size_t(first) < mesh.triangles.size()) mesh.triangles[first].classification = g;
                else throw std::runtime_error("invalid BDS label kind or face");
            }
            std::size_t requests; if(!(in >> requests) || requests > 1000000) throw std::runtime_error("invalid BDS requests");
            std::vector<BdsRecoveryConstraint> registered(requests);
            for(auto& r : registered) if(!(in >> r.first >> r.second >> r.curveEntity)) throw std::runtime_error("truncated BDS requests");
            int first,second; if(!(in >> first >> second)) throw std::runtime_error("missing BDS recovery endpoints");
            BdsMesh recovered; const auto result = recoverBdsConstraintEdge(mesh,first,second,recovered,registered);
            if(!result.status.complete()) throw OperationFailure{result.status};
            out << "{\"mesh\":"; writeBdsPoolJson(out,recovered);
            out << ",\"edge\":" << result.recoveredEdge << ",\"fatal\":" << (result.nativeFatal ? "true" : "false") << ",\"not_recovered\":[";
            for(std::size_t i = 0; i < result.notRecovered.size(); ++i) { if(i) out << ','; const auto& r = result.notRecovered[i]; out << '[' << r.first << ',' << r.second << ',' << r.curveEntity << ']'; }
            out << "],\"scans\":[";
            for(std::size_t i = 0; i < result.scans.size(); ++i) {
                if(i) out << ','; const auto& scan = result.scans[i]; out << "{\"intersected\":[";
                for(std::size_t j = 0; j < scan.intersectedEdges.size(); ++j) { if(j) out << ','; out << scan.intersectedEdges[j]; }
                out << "],\"attempts\":[";
                for(std::size_t j = 0; j < scan.attempts.size(); ++j) { if(j) out << ','; out << '[' << scan.attempts[j].first << ',' << (scan.attempts[j].second ? "true" : "false") << ']'; }
                out << "]}";
            }
            out << "]}\n";
        }
        else throw std::runtime_error("unknown BDS recovery mode");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected BDS input tail");
        if(!out) throw std::runtime_error("BDS recovery output failed"); return 0;
    } catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
