/** @file GmshFaceMeshConversionReplay.cpp
 * @brief 冻结BDS/原节点输入的BDS2GMSH三角叶阶段重放；不调用CAD/Gmsh。
 * @date 2026-10-08
 */
#include "GmshFaceMeshJson.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic()); int entity; std::size_t points,originals,triangles;
        if(!(in >> entity >> points >> originals >> triangles) || points > 1000000 || originals > 1000000 || triangles > 1000000) throw std::runtime_error("invalid face mesh leaf header");
        BdsMesh bds;
        for(std::size_t i = 0; i < points; ++i) {
            BdsPoint p; if(!(in >> p.id >> p.physicalId >> p.point.x >> p.point.y >> p.point.z >> p.uv.x >> p.uv.y)) throw std::runtime_error("truncated leaf points");
            insertBdsPoint(bds,p);
        }
        std::vector<FaceVertex> originalVertices(originals);
        for(auto& p : originalVertices) {
            if(!(in >> p.id >> p.dimension >> p.entity >> p.point.x >> p.point.y >> p.point.z >> p.parameters.x >> p.parameters.y >> p.storedSize)) throw std::runtime_error("truncated leaf originals");
            p.physicalId = p.id;
        }
        std::vector<bool> deleted;
        for(std::size_t i = 0; i < triangles; ++i) {
            std::array<int,3> nodes; int flag;
            if(!(in >> nodes[0] >> nodes[1] >> nodes[2] >> flag) || (flag != 0 && flag != 1)) throw std::runtime_error("invalid leaf triangle");
            appendBdsTriangle(bds,nodes); deleted.push_back(flag != 0);
        }
        for(std::size_t i = 0; i < triangles; ++i) if(deleted[i]) {
            auto& t = bds.triangles[i]; t.deleted = true;
            for(int e : t.edges) { auto& faces = bds.edges[e].faces; faces.erase(std::remove(faces.begin(),faces.end(),int(i)),faces.end()); }
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected leaf input tail");
        FaceMesh original,converted; auto status = initializeFaceMeshVertices(bds,entity,originalVertices,original);
        if(!status.complete()) throw std::runtime_error(status.message);
        const auto result = convertBdsToFaceMesh(bds,original,converted);
        if(!result.status.complete()) throw std::runtime_error(result.status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17); writeFaceMeshJson(out,converted,result); out << '\n';
        if(!out) throw std::runtime_error("face mesh leaf write failed"); return 0;
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
