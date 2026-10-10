/** @file GmshFrontPointInsertionCadReplay.cpp
 * @brief 同BREP单次insertAPoint；冻结原运行池/查询位置，独立CAD Value/BGM/空腔与提交。
 * @date 2026-10-08
 */
#include "GmshFrontPointInsertionJson.h"
#include "cad/occ/OccCadGeometry.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
using namespace mesher; using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    if(argc != 4) return 2;
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read frozen BREP");
        OccCadGeometry geometry(shape); if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream in(argv[2]); in.imbue(std::locale::classic()); std::size_t count;
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid point insertion case count");
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17) << "{\"cases\":[";
        for(std::size_t item = 0; item < count; ++item) {
            int face,worst,extension,star,failed,mode; std::size_t n,t,a,active,e; Point2D point; SurfaceMetric metric;
            double maximumSize,modelLength; FaceMesh mesh;
            if(!(in >> face >> n >> t >> a >> active >> e >> worst >> mesh.maximumVertexId >> extension >> star >> failed >> mode >> maximumSize >> modelLength >> point.x >> point.y >> metric.gUU >> metric.gUV >> metric.gVV) || face < 1 || face > geometry.faceCount() || !n || n > 1000000 || !t || t > 1000000 || a > t || active > t || e > 1000000 || mode < 0 || mode > 1) throw std::runtime_error("invalid point insertion case header");
            mesh.entity = face; NodeSizeState nodes; nodes.nodes.reserve(n+1);
            for(std::size_t i = 0; i < n; ++i) {
                FaceVertex vertex; InitializedNodeSize node; int alive;
                if(!(in >> vertex.id >> node.parameters.x >> node.parameters.y >> vertex.point.x >> vertex.point.y >> vertex.point.z >> vertex.dimension >> node.sizes.meshSize >> node.sizes.backgroundSize >> alive)) throw std::runtime_error("truncated point insertion node");
                node.vertex = vertex.id;
                if(alive && (!mesh.vertices.emplace(vertex.id,vertex).second || !nodes.indices.emplace(vertex.id,nodes.nodes.size()).second)) throw std::runtime_error("duplicate live insertion node");
                nodes.nodes.push_back(node);
            }
            FrontMesh front; front.triangles.resize(t);
            for(auto& triangle : front.triangles) {
                int deleted;
                if(!(in >> triangle.vertices[0] >> triangle.vertices[1] >> triangle.vertices[2] >> triangle.radius >> deleted >> triangle.neighbors[0] >> triangle.neighbors[1] >> triangle.neighbors[2])) throw std::runtime_error("truncated point insertion triangle");
                triangle.deleted = deleted != 0;
            }
            FrontQueueState queue; queue.limit = frontalRadiusLimit();
            auto readSet = [&](std::size_t count,FrontTriangleSet& set) { for(std::size_t i = 0; i < count; ++i) { int index; if(!(in >> index)) throw std::runtime_error("truncated insertion set"); set.insert(makeFrontTriangleKey(front.triangles.at(index),index)); } };
            readSet(a,queue.allTriangles); readSet(active,queue.activeTriangles);
            for(std::size_t i = 0; i < e; ++i) { std::array<std::uint64_t,2> edge; if(!(in >> edge[0] >> edge[1])) throw std::runtime_error("truncated insertion internal edge"); if(edge[1] < edge[0]) std::swap(edge[0],edge[1]); nodes.internalEdges.insert(edge); }
            SurfaceMeshingOptions options; options.sizeFromPoints = false; options.sizeMax = maximumSize;
            GeometrySizeSource sizes;
            sizes.sizeCallback = [&](const MeshSizeQuery& q,double) { const auto p = q.point; return mode ? maximumSize*(.25+.6/(1.+p.x*p.x+p.y*p.y+p.z*p.z)) : maximumSize; };
            FrontInsertionSource source;
            source.surfacePoint = [&](Point2D uv) { return FrontSurfacePoint{geometry.faceSurfacePoint(face-1,uv),failed == 0}; };
            source.backgroundSize = [&](const MeshSizeQuery& q) { return evaluateMeshSize(q,modelLength,options,sizes); };
            const auto result = insertFrontPoint(front,queue,mesh,nodes,worst,point,metric,source,{extension != 0,star != 0,false});
            if(item) out << ','; writeFrontPointInsertionJson(out,result,front,queue,mesh,nodes,modelLength);
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected point insertion input tail");
        out << "]}\n"; if(!out) throw std::runtime_error("point insertion output failed"); return 0;
    } catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
