/** @file GmshFrontCavityReplay.cpp
 * @brief 冻结UV/尺寸/三角形运行池的纯空腔叶重放，无CAD/Gmsh。
 * @date 2026-10-08
 */
#include "GmshFrontCavityJson.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
int main(int argc,char** argv) {
    using namespace mesher::gmshReference;
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic()); std::size_t n,t,a,e; int worst,force; mesher::Point2D point; SurfaceMetric metric;
        if(!(in >> n >> t >> a >> e >> worst >> force >> point.x >> point.y >> metric.gUU >> metric.gUV >> metric.gVV) || n > 1000000 || t > 1000000 || a > t || e > 1000000) throw std::runtime_error("invalid front cavity leaf header");
        NodeSizeState nodes; FrontMesh mesh;
        for(std::size_t i = 0; i < n; ++i) {
            InitializedNodeSize node;
            if(!(in >> node.vertex >> node.parameters.x >> node.parameters.y >> node.sizes.meshSize >> node.sizes.backgroundSize) || !nodes.indices.emplace(node.vertex,nodes.nodes.size()).second) throw std::runtime_error("invalid frozen front node");
            nodes.nodes.push_back(node);
        }
        mesh.triangles.resize(t);
        for(auto& triangle : mesh.triangles) {
            int deleted;
            if(!(in >> triangle.vertices[0] >> triangle.vertices[1] >> triangle.vertices[2] >> triangle.radius >> deleted >> triangle.neighbors[0] >> triangle.neighbors[1] >> triangle.neighbors[2])) throw std::runtime_error("truncated frozen triangle");
            triangle.deleted = deleted != 0;
        }
        FrontTriangleSet all;
        for(std::size_t i = 0; i < a; ++i) { int index; if(!(in >> index)) throw std::runtime_error("truncated initial all set"); all.insert(makeFrontTriangleKey(mesh.triangles.at(index),index)); }
        for(std::size_t i = 0; i < e; ++i) { std::array<std::uint64_t,2> edge; if(!(in >> edge[0] >> edge[1])) throw std::runtime_error("truncated internal edges"); if(edge[1] < edge[0]) std::swap(edge[0],edge[1]); nodes.internalEdges.insert(edge); }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected cavity leaf input tail");
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        writeFrontCavityJson(out,mesh,nodes,all,worst,point,metric,force != 0); out << '\n'; if(!out) throw std::runtime_error("cavity leaf output failed"); return 0;
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
