/** @file GmshParametricDomainReplay.cpp
 * @brief 参数域严格穿越叶重放；纯算法不依赖Gmsh或CAD。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/chart/ParametricDomain.h"
#include <fstream>
#include <iostream>
using namespace mesher; using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); std::ofstream out(argv[2]); std::size_t count;
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid domain case count"); out << '[';
        for(std::size_t i = 0; i < count; ++i) {
            std::size_t n; Point2D point,far;
            if(!(in >> n >> point.x >> point.y >> far.x >> far.y) || n > 1000000) throw std::runtime_error("invalid domain query");
            std::vector<TrueBoundarySegment> segments(n); for(auto& s : segments) if(!(in >> s.from.x >> s.from.y >> s.to.x >> s.to.y)) throw std::runtime_error("truncated segment");
            const auto result = testParametricDomain(segments,point,far); if(!result.status.complete()) throw OperationFailure{result.status};
            if(i) out << ','; out << '[' << (result.inside ? "true" : "false") << ',' << result.crossingCount << ']';
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected domain tail"); out << "]\n"; if(!out) throw std::runtime_error("domain output failed"); return 0;
    } catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
