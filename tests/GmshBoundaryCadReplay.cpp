/** @file GmshBoundaryCadReplay.cpp
 * @brief 测试端同一 BREP 的 OCC 精确查询，重放最终 0D/1D 物理链。
 * @date 2026-10-07
 * 核心 BoundaryRegistry 不链接 OCC/Gmsh；此工具不改变 CAD 或 production 默认。
 */
#include "mesh/surface/algorithm6/boundary/BoundaryRegistry.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
Point3D point3(const gp_Pnt& p) { return {p.X(),p.Y(),p.Z()}; }
}
int main(int argc, char** argv)
{
    if(argc < 5) { std::cerr << "usage: GmshBoundaryCadReplay CAD.brep size modelLC output.json [entity:segment ...]\n"; return 2; }
    try {
        const double size = std::stod(argv[2]);
        if(!std::isfinite(size) || size <= 0.) throw std::runtime_error("invalid size");
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read BREP");
        TopTools_IndexedMapOfShape vertices, edges, faces;
        TopExp::MapShapes(shape,TopAbs_VERTEX,vertices);
        TopExp::MapShapes(shape,TopAbs_EDGE,edges);
        TopExp::MapShapes(shape,TopAbs_FACE,faces);
        BoundaryInput input; input.modelCharacteristicLength = std::stod(argv[3]);
        for(int i = 1; i <= vertices.Extent(); ++i)
            input.vertices.push_back({i,point3(BRep_Tool::Pnt(TopoDS::Vertex(vertices(i))))});
        for(int i = 1; i <= edges.Extent(); ++i) {
            const auto edge = TopoDS::Edge(edges(i));
            BoundaryCurveInput curve; curve.entity = i;
            curve.beginVertex = vertices.FindIndex(TopExp::FirstVertex(edge));
            curve.endVertex = vertices.FindIndex(TopExp::LastVertex(edge));
            curve.geometryDegenerate = BRep_Tool::Degenerated(edge);
            for(int face = 1; face <= faces.Extent(); ++face) {
                TopTools_IndexedMapOfShape faceEdges; TopExp::MapShapes(faces(face),TopAbs_EDGE,faceEdges);
                if(faceEdges.Contains(edge)) curve.incidentFaces.push_back(face);
            }
            if(!curve.geometryDegenerate) {
                auto adaptor = std::make_shared<BRepAdaptor_Curve>(edge);
                curve.first = adaptor->FirstParameter(); curve.last = adaptor->LastParameter();
                const auto type = adaptor->GetType();
                curve.kind = type == GeomAbs_Line ? CurveKind::Line : type == GeomAbs_Circle ? CurveKind::Circle :
                             type == GeomAbs_Ellipse ? CurveKind::Ellipse : CurveKind::Other;
                curve.point = [adaptor](double t) { return point3(adaptor->Value(t)); };
                curve.derivative = [adaptor](double t) {
                    gp_Pnt p; gp_Vec d; adaptor->D1(t,p,d); return Point3D{d.X(),d.Y(),d.Z()};
                };
                // OCCEdge::_project：端点不同才扩展投影区间，不强制截回 bounds。
                double first, last; const auto nativeCurve = BRep_Tool::Curve(edge,first,last);
                if(curve.beginVertex != curve.endVertex) {
                    const double tolerance = std::max(std::abs(last-first)*1.e-8,1.e-12);
                    first -= tolerance; last += tolerance;
                }
                auto projector = std::make_shared<GeomAPI_ProjectPointOnCurve>();
                projector->Init(nativeCurve,first,last);
                curve.inverseParameter = [projector](Point3D p) {
                    projector->Perform(gp_Pnt(p.x,p.y,p.z));
                    if(!projector->NbPoints()) throw std::runtime_error("OCC projection failed; generic inverse fallback not implemented");
                    return projector->LowerDistanceParameter();
                };
            }
            input.curves.push_back(std::move(curve));
        }
        SurfaceMeshingOptions options; options.sizeMax = size; options.sizeFromPoints = false;
        std::string field;
        for(int arg = 5; arg < argc; ++arg) {
            const std::string value = argv[arg];
            if(value.rfind("--field=",0) == 0) field = value.substr(8);
        }
        GeometrySizeSource sizes;
        if(!field.empty()) {
            if(field != "step" && field != "graded" && field != "endpoint")
                throw std::runtime_error("unknown boundary field");
            sizes.sizeCallback = [field](const MeshSizeQuery& query, double) {
                if(field == "step") return query.point.x < 5. ? .1 : 10.;
                if(field == "graded") return .1 + .8*std::abs(query.point.x);
                return query.dimension == 0 ? .05 : .3;
            };
        }
        BoundaryMesh mesh;
        const auto result = buildBoundaryMesh(input,options,sizes,mesh);
        if(!result.status.complete()) throw std::runtime_error(result.status.message);
        // 先冻结各阶段，最终输出才能在恢复查询失败时整体失败。
        using Row = std::array<double,8>;
        using Chain = std::pair<int,std::vector<Row>>;
        std::vector<std::vector<Chain>> recoveryStages;
        const auto snapshot = [&] {
            std::vector<Chain> stage;
            for(const auto& curve : mesh.curves) {
                std::vector<Row> rows;
                for(const auto& sample : curve.samples) {
                    const auto& node = mesh.nodes.at(sample.node-1);
                    rows.push_back({double(node.id),sample.parameter,node.point.x,node.point.y,node.point.z,
                                    double(node.dimension),double(node.entity),node.storedSize});
                }
                stage.push_back({curve.entity,std::move(rows)});
            }
            recoveryStages.push_back(std::move(stage));
        };
        snapshot();
        for(int arg = 5; arg < argc; ++arg) {
            const std::string request = argv[arg]; const auto colon = request.find(':');
            if(request.rfind("--field=",0) == 0) continue;
            if(colon == std::string::npos) throw std::runtime_error("recovery request requires entity:segment");
            const int entity = std::stoi(request.substr(0,colon));
            const int segment = std::stoi(request.substr(colon+1));
            const auto chain = std::find_if(mesh.curves.begin(),mesh.curves.end(),
                [&](const BoundaryCurve& c) { return c.entity == entity; });
            if(chain == mesh.curves.end() || segment < 0 || std::size_t(segment+1) >= chain->samples.size())
                throw std::runtime_error("recovery request outside current chain");
            const auto recovery = refineUnrecoveredEdges(input,options,sizes,
                {{entity,chain->samples[segment].node,chain->samples[segment+1].node}},mesh);
            if(!recovery.status.complete()) throw std::runtime_error(recovery.status.message);
            snapshot();
        }
        std::ofstream out(argv[4]); if(!out) throw std::runtime_error("cannot open boundary output");
        out.imbue(std::locale::classic()); out << std::setprecision(17);
        out << "{\"schema\":1,\"nodes\":[";
        bool comma = false;
        for(const auto& node : mesh.nodes) if(node.alive) {
            if(comma) out << ','; comma = true;
            out << '[' << node.id << ',' << node.point.x << ',' << node.point.y << ',' << node.point.z << ','
                << node.dimension << ',' << node.entity << ",[";
            if(node.dimension) out << node.parameter;
            out << "]]";
        }
        out << "],\"elements\":["; comma = false; std::uint64_t element = 0;
        for(const auto& node : mesh.nodes) if(node.dimension == 0 && node.alive) {
            if(comma) out << ','; comma = true;
            out << '[' << ++element << ",0," << node.entity << ",15,[" << node.id << "]]";
        }
        for(const auto& curve : mesh.curves) for(std::size_t i = 1; i < curve.samples.size(); ++i) {
            if(comma) out << ','; comma = true;
            out << '[' << ++element << ",1," << curve.entity << ",1,[" << curve.samples[i-1].node
                << ',' << curve.samples[i].node << "]]";
        }
        out << "],\"actual_algorithms\":[],\"geometry_evaluations\":" << result.geometryEvaluations
            << ",\"allocated_node_count\":" << mesh.nodes.size() << ",\"erased_node_count\":"
            << std::count_if(mesh.nodes.begin(),mesh.nodes.end(),[](const BoundaryNode& n) { return !n.alive; });
        out << ",\"boundary_chains\":[";
        bool chainComma = false;
        for(const auto& chain : recoveryStages.front()) {
            if(chain.second.empty()) continue;
            if(chainComma) out << ','; chainComma = true;
            out << "{\"stage\":\"boundary_chain\",\"entity\":" << chain.first << ",\"rows\":[";
            for(std::size_t row = 0; row < chain.second.size(); ++row) {
                if(row) out << ','; out << '[';
                for(int k = 0; k < 8; ++k) {
                    if(k) out << ',';
                    const double value = chain.second[row][k];
                    if(k == 0 || k == 5 || k == 6) out << static_cast<std::int64_t>(value);
                    else if(std::isfinite(value)) out << value;
                    else out << '"' << (std::isnan(value) ? "nan" : value < 0. ? "-inf" : "inf") << '"';
                }
                out << ']';
            }
            out << "]}";
        }
        out << ']';
        if(recoveryStages.size() > 1) {
            out << ",\"recovery_stages\":[";
            for(std::size_t stage = 0; stage < recoveryStages.size(); ++stage) {
                if(stage) out << ','; out << '[';
                for(std::size_t c = 0; c < recoveryStages[stage].size(); ++c) {
                    if(c) out << ',';
                    const auto& chain = recoveryStages[stage][c];
                    out << "{\"stage\":\"boundary_recovery\",\"entity\":" << chain.first << ",\"rows\":[";
                    for(std::size_t row = 0; row < chain.second.size(); ++row) {
                        if(row) out << ','; out << '[';
                        for(int k = 0; k < 8; ++k) {
                            if(k) out << ',';
                            const double value = chain.second[row][k];
                            if(k == 0 || k == 5 || k == 6) out << static_cast<std::int64_t>(value);
                            else if(std::isfinite(value)) out << value;
                            else out << '"' << (std::isnan(value) ? "nan" : value < 0. ? "-inf" : "inf") << '"';
                        }
                        out << ']';
                    }
                    out << "]}";
                }
                out << ']';
            }
            out << ']';
        }
        out << "}\n";
        if(!out) throw std::runtime_error("boundary output write failed");
        std::cout << "boundary replay vertices=" << input.vertices.size() << " curves=" << input.curves.size()
                  << " allocated_nodes=" << mesh.nodes.size() << '\n';
        return 0;
    }
    catch(const Standard_Failure& e) { std::cerr << "OCC: " << e.GetMessageString() << '\n'; return 1; }
    catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
