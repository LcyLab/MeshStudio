/** @file GmshFrontGenerationCadReplay.cpp
 * @brief 冻结初始运行池；独立CAD候选/域门控/插点与动态队列全程对照。
 * @date 2026-10-08
 */
#include "GmshFrontGenerationJson.h"
#include "cad/occ/OccCadGeometry.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
using namespace mesher; using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    if(argc != 4) return 2;
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read frozen BREP");
        OccCadGeometry geometry(shape); if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream in(argv[2]); in.imbue(std::locale::classic()); std::size_t count;
        if(!(in >> count) || count > 10000) throw std::runtime_error("invalid generation case count");
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17) << "{\"cases\":[";
        for(std::size_t item = 0; item < count; ++item) {
            int face,gate,extension,mode; std::size_t n,t,a,e,b,maximumPops; double maximumSize,modelLength; FaceMesh mesh;
            if(!(in >> face >> n >> t >> a >> e >> b >> gate >> mesh.maximumVertexId >> maximumPops >> extension >> mode >> maximumSize >> modelLength) || face < 1 || face > geometry.faceCount() || !n || n > 1000000 || !t || t > 1000000 || a > t || e > 1000000 || b > 1000000 || mode < 0 || mode > 1) throw std::runtime_error("invalid generation header");
            mesh.entity = face; NodeSizeState nodes;
            for(std::size_t i = 0; i < n; ++i) {
                FaceVertex vertex; InitializedNodeSize node; int alive;
                if(!(in >> vertex.id >> node.parameters.x >> node.parameters.y >> vertex.point.x >> vertex.point.y >> vertex.point.z >> vertex.dimension >> node.sizes.meshSize >> node.sizes.backgroundSize >> alive)) throw std::runtime_error("truncated generation node");
                node.vertex = vertex.id; vertex.entity = face;
                if(alive && (!mesh.vertices.emplace(vertex.id,vertex).second || !nodes.indices.emplace(vertex.id,nodes.nodes.size()).second)) throw std::runtime_error("duplicate live generation node");
                nodes.nodes.push_back(node);
            }
            FrontMesh front; front.triangles.resize(t);
            for(auto& triangle : front.triangles) {
                int deleted;
                if(!(in >> triangle.vertices[0] >> triangle.vertices[1] >> triangle.vertices[2] >> triangle.radius >> deleted >> triangle.neighbors[0] >> triangle.neighbors[1] >> triangle.neighbors[2])) throw std::runtime_error("truncated generation triangle");
                triangle.deleted = deleted != 0;
            }
            FrontQueueState queue; queue.limit = frontalRadiusLimit();
            for(std::size_t i = 0; i < a; ++i) { int index; if(!(in >> index)) throw std::runtime_error("truncated generation AllTris"); queue.allTriangles.insert(makeFrontTriangleKey(front.triangles.at(index),index)); }
            for(const auto& key : queue.allTriangles) front.orderedTriangles.push_back(key.triangle);
            const auto queueStatus = initializeFrontQueue(front,queue); if(!queueStatus.complete()) throw OperationFailure{queueStatus};
            for(std::size_t i = 0; i < e; ++i) { std::array<std::uint64_t,2> edge; if(!(in >> edge[0] >> edge[1])) throw std::runtime_error("truncated internal edge"); if(edge[1] < edge[0]) std::swap(edge[0],edge[1]); nodes.internalEdges.insert(edge); }
            std::vector<TrueBoundarySegment> boundary(b); for(auto& s : boundary) if(!(in >> s.from.x >> s.from.y >> s.to.x >> s.to.y)) throw std::runtime_error("truncated true boundary");
            SurfaceMeshingOptions sizeOptions; sizeOptions.sizeFromPoints = false; sizeOptions.sizeMax = maximumSize;
            GeometrySizeSource sizes; sizes.sizeCallback = [&](const MeshSizeQuery& q,double) { const auto p = q.point; return mode ? maximumSize*(.25+.6/(1.+p.x*p.x+p.y*p.y+p.z*p.z)) : maximumSize; };
            FrontGenerationSource source;
            source.parameterBounds = [&] { SurfaceParameterBounds range; if(!geometry.faceTrimParameterBounds(face-1,range)) throw std::runtime_error("trimmed face range unavailable"); return std::array<Point2D,2>{{{range.uMin,range.vMin},{range.uMax,range.vMax}}}; };
            source.metric = [&](Point2D uv) { Point3D du,dv; geometry.faceSurfaceDerivatives(face-1,uv,du,dv); const auto m = evaluateSurfaceMetric(du,dv); if(!m.status.complete()) throw OperationFailure{m.status}; return m.metric; };
            source.surface = [&](Point2D uv) { return geometry.faceSurfacePoint(face-1,uv); };
            source.surfacePoint = [&](Point2D uv) { return FrontSurfacePoint{geometry.faceSurfacePoint(face-1,uv),true}; };
            source.backgroundSize = [&](const MeshSizeQuery& q) { return evaluateMeshSize(q,modelLength,sizeOptions,sizes); };
            FrontGenerationOptions options; options.extendBoundarySize = extension != 0; options.trueBoundary = gate ? &boundary : nullptr; options.maximumPops = maximumPops;
            if(item) out << ','; writeFrontGenerationJson(out,front,queue,mesh,nodes,source,options,modelLength);
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected generation tail"); out << "]}\n"; if(!out) throw std::runtime_error("generation output failed"); return 0;
    } catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
