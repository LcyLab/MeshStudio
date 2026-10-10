/** @file GmshFrontQueueReplay.cpp
 * @brief 冻结MTri3池/AllTris身份序及显式操作的纯自研重放，无CAD/Gmsh。
 * @date 2026-10-08
 */
#include "GmshFrontQueueJson.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
int main(int argc,char** argv) {
    using namespace mesher::gmshReference;
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic()); double limit; std::size_t count,initial,operationCount;
        if(!(in >> limit >> count >> initial >> operationCount) || count > 1000000 || initial > count || operationCount > 1000000) throw std::runtime_error("invalid front queue leaf header");
        FrontMesh mesh; mesh.triangles.resize(count); mesh.orderedTriangles.resize(initial);
        for(auto& t : mesh.triangles) {
            int deleted;
            if(!(in >> t.vertices[0] >> t.vertices[1] >> t.vertices[2] >> t.radius >> deleted >> t.neighbors[0] >> t.neighbors[1] >> t.neighbors[2]) || (deleted != 0 && deleted != 1)) throw std::runtime_error("invalid frozen front triangle");
            t.deleted = deleted != 0;
        }
        for(auto& index : mesh.orderedTriangles) if(!(in >> index)) throw std::runtime_error("truncated initial set");
        std::vector<std::array<double,3>> operations(operationCount);
        for(auto& operation : operations) if(!(in >> operation[0] >> operation[1] >> operation[2])) throw std::runtime_error("truncated queue operations");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected queue leaf input tail");
        FrontQueueState queue; const auto status = initializeFrontQueue(mesh,queue,limit); if(!status.complete()) throw std::runtime_error(status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17); writeFrontQueueJson(out,mesh,queue,operations); out << '\n';
        if(!out) throw std::runtime_error("front queue leaf output failed"); return 0;
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
