/** @file GmshSurfaceSmoothingCadReplay.cpp
 * @brief 同BREP独立D1/Value的平滑核对照；全部节点为同一面的MFaceVertex语义。
 * @date 2026-10-08
 */
#include "GmshSurfaceSmoothingJson.h"
#include "cad/occ/OccCadGeometry.h"
#include "cad/occ/OccMeshVertexParameters.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>
using namespace mesher; using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    const bool classified = argc == 5 && std::string(argv[4]) == "--classified";
    if(argc != 4 && !classified) return 2;
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read smoothing BREP");
        OccCadGeometry geometry(shape); if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream in(argv[2]); in.imbue(std::locale::classic()); std::size_t count;
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid smoothing case count");
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17) << "{\"cases\":[";
        for(std::size_t item = 0; item < count; ++item) {
            int face,failed; std::size_t n,t; FaceMesh mesh; SurfaceSmoothingOptions options;
            if(!(in >> face >> n >> t >> options.iterations >> failed) || face < 1 || face > geometry.faceCount() || !n || n > 1000000 || !t || t > 1000000) throw std::runtime_error("invalid smoothing header");
            mesh.entity = face; std::map<std::uint64_t,std::array<bool,2>> availability;
            for(std::size_t i = 0; i < n; ++i) {
                FaceVertex v; int protectedVertex; v.dimension = 2; v.entity = face;
                if(classified) {
                    int hasU,hasV;
                    if(!(in >> v.id >> v.dimension >> v.entity >> v.point.x >> v.point.y >> v.point.z >> v.parameters.x >> v.parameters.y >> hasU >> hasV >> protectedVertex)) throw std::runtime_error("invalid classified smoothing node");
                    availability[v.id] = {hasU != 0,hasV != 0};
                }
                else if(!(in >> v.id >> v.parameters.x >> v.parameters.y >> v.point.x >> v.point.y >> v.point.z >> protectedVertex)) throw std::runtime_error("invalid smoothing node");
                if(!v.id || !mesh.vertices.emplace(v.id,v).second) throw std::runtime_error("duplicate or invalid smoothing node identity");
                if(protectedVertex) options.faceVerticesWithBoundaryLayerData.insert(v.id);
                mesh.maximumVertexId = std::max(mesh.maximumVertexId,v.id);
            }
            mesh.triangles.resize(t);
            for(auto& triangle : mesh.triangles) if(!(in >> triangle[0] >> triangle[1] >> triangle[2])) throw std::runtime_error("truncated smoothing triangle");
            std::size_t pointQueries = 0;
            SurfaceSmoothingSource source;
            source.metric = [&](Point2D uv) {
                Point3D du,dv; geometry.faceSurfaceDerivatives(face-1,uv,du,dv);
                const auto result = evaluateSurfaceMetric(du,dv); if(!result.status.complete()) throw OperationFailure{result.status}; return result.metric;
            };
            source.surfacePoint = [&](Point2D uv) { ++pointQueries; return SmoothingSurfacePoint{geometry.faceSurfacePoint(face-1,uv),failed <= 0 || pointQueries != std::size_t(failed)}; };
            if(classified) {
                const auto parameters = makeOccMeshVertexParameterSource(geometry,face-1);
                auto input = [availability](const FaceVertex& v) { MeshVertexParameterInput result; result.vertex = v; result.hasParameters = availability.at(v.id); return result; };
                source.reparameterizeVertex = [parameters,input](const FaceVertex& v,const FaceMesh&) {
                    const auto result = reparameterizeMeshVertexOnFace(input(v),parameters);
                    if(!result.status.complete()) throw OperationFailure{result.status};
                    // Native parametricCoordinates ignores the false seam flag and uses the computed UV.
                    return result.parameter;
                };
                source.reparameterizeEdge = [parameters,input](const FaceVertex& a,const FaceVertex& b,const FaceMesh&) {
                    const auto result = reparameterizeMeshEdgeOnFace(input(a),input(b),parameters);
                    if(!result.status.complete()) throw OperationFailure{result.status}; return result.parameters;
                };
            }
            else {
                source.reparameterizeVertex = [](const FaceVertex& v,const FaceMesh&) { return v.parameters; };
                source.reparameterizeEdge = [](const FaceVertex& a,const FaceVertex& b,const FaceMesh&) { return std::array<Point2D,2>{a.parameters,b.parameters}; };
            }
            std::vector<SmoothingVertexEvent> events;
            const auto result = smoothSurfaceMesh(mesh,source,options,{},[&](const auto& event) { events.push_back(event); });
            if(item) out << ','; writeSurfaceSmoothingJson(out,result,events,mesh);
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected smoothing input tail");
        out << "]}\n"; if(!out) throw std::runtime_error("smoothing output failed"); return 0;
    } catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
