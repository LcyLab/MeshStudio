/** @file GmshFrontMeshTransferReplay.cpp
 * @brief 前沿结束池的独立转移重放；不链接Gmsh、不重新生成候选。
 * @date 2026-10-08
 */
#include "GmshFrontMeshTransferJson.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
int main(int argc,char** argv) {
    using namespace mesher::gmshReference;
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic());
        std::size_t n,t,a,e; int hasEquivalence,physical;
        if(!(in >> n >> t >> a >> e >> hasEquivalence >> physical) || n > 1000000 || t > 1000000 || a > t || e > n)
            throw std::runtime_error("invalid transfer header");
        FaceMesh mesh; mesh.entity = 1; NodeSizeState nodes;
        for(std::size_t i = 0; i < n; ++i) {
            FaceVertex v; InitializedNodeSize node;
            if(!(in >> v.id >> node.parameters.x >> node.parameters.y >> v.point.x >> v.point.y >> v.point.z)) throw std::runtime_error("truncated transfer node");
            node.vertex = v.id; v.dimension = 2; v.entity = 1;
            if(!mesh.vertices.emplace(v.id,v).second || !nodes.indices.emplace(v.id,nodes.nodes.size()).second) throw std::runtime_error("duplicate transfer node");
            nodes.nodes.push_back(node); mesh.maximumVertexId = std::max(mesh.maximumVertexId,v.id);
        }
        FrontMesh front; front.triangles.resize(t);
        for(auto& triangle : front.triangles) {
            int deleted;
            if(!(in >> triangle.vertices[0] >> triangle.vertices[1] >> triangle.vertices[2] >> triangle.radius >> deleted)) throw std::runtime_error("truncated transfer triangle");
            triangle.deleted = deleted != 0;
        }
        FrontQueueState queue;
        for(std::size_t i = 0; i < a; ++i) { int id; if(!(in >> id)) throw std::runtime_error("truncated transfer AllTris"); queue.allTriangles.insert(makeFrontTriangleKey(front.triangles.at(id),id)); }
        for(std::size_t i = 0; i < e; ++i) { std::uint64_t from,to; if(!(in >> from >> to) || !mesh.equivalence.emplace(from,to).second) throw std::runtime_error("invalid transfer equivalence"); }
        FrontMeshTransferOptions options; options.usePhysicalNormals = physical != 0; if(hasEquivalence) options.equivalence = &mesh.equivalence;
        FaceMesh output; const auto result = transferFrontMesh(front,queue,mesh,nodes,options,output);
        if(!result.status.complete()) throw std::runtime_error(result.status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        writeFrontMeshTransferJson(out,result,output);
        if(!out) throw std::runtime_error("transfer output failed");
        return 0;
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
