/** @file GmshFrontInsertionReplay.cpp
 * @brief 冻结XYZ/UV/分类/尺寸/池与空腔的纯insertVertexB叶重放；不查询CAD或BGM。
 * @date 2026-10-08
 */
#include "GmshFrontInsertionJson.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
int main(int argc,char** argv) {
    using namespace mesher::gmshReference;
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic());
        std::size_t n,t,a,active,c,s,newNode; int extension,force,star;
        if(!(in >> n >> t >> a >> active >> c >> s >> newNode >> extension >> force >> star) || !n || n > 1000000 || t > 1000000 || a > t || active > t || c > t || s > 1000000 || newNode >= n) throw std::runtime_error("invalid insertion leaf header");
        FaceMesh mesh; NodeSizeState nodes; FaceVertex vertex; InitializedNodeSize added;
        for(std::size_t i = 0; i < n; ++i) {
            FaceVertex v; InitializedNodeSize node;
            if(!(in >> v.id >> node.parameters.x >> node.parameters.y >> v.point.x >> v.point.y >> v.point.z >> v.dimension >> node.sizes.meshSize >> node.sizes.backgroundSize)) throw std::runtime_error("truncated insertion node");
            node.vertex = v.id;
            if(i == newNode) { vertex = v; added = node; }
            else { if(!mesh.vertices.emplace(v.id,v).second || !nodes.indices.emplace(v.id,nodes.nodes.size()).second) throw std::runtime_error("duplicate insertion node"); nodes.nodes.push_back(node); }
        }
        FrontMesh front; front.triangles.resize(t);
        for(auto& triangle : front.triangles) {
            int deleted;
            if(!(in >> triangle.vertices[0] >> triangle.vertices[1] >> triangle.vertices[2] >> triangle.radius >> deleted >> triangle.neighbors[0] >> triangle.neighbors[1] >> triangle.neighbors[2])) throw std::runtime_error("truncated insertion triangle");
            triangle.deleted = deleted != 0;
        }
        FrontQueueState queue; queue.limit = frontalRadiusLimit();
        auto readSet = [&](std::size_t count,FrontTriangleSet& set) { for(std::size_t i = 0; i < count; ++i) { int index; if(!(in >> index)) throw std::runtime_error("truncated insertion set"); set.insert(makeFrontTriangleKey(front.triangles.at(index),index)); } };
        readSet(a,queue.allTriangles); readSet(active,queue.activeTriangles);
        FrontCavityResult cavity;
        for(std::size_t i = 0; i < c; ++i) { int index; if(!(in >> index)) throw std::runtime_error("truncated insertion cavity"); cavity.triangles.push_back(index); }
        for(std::size_t i = 0; i < s; ++i) { FrontShellEdge edge; if(!(in >> edge.triangle >> edge.edge)) throw std::runtime_error("truncated insertion shell"); cavity.shell.push_back(edge); }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected insertion input tail");
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        writeFrontInsertionJson(out,front,queue,mesh,nodes,cavity,vertex,added,{extension != 0,star != 0,force != 0}); out << '\n'; if(!out) throw std::runtime_error("insertion leaf output failed"); return 0;
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
