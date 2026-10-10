/** @file GmshPeriodicBoundaryChartCadReplay.cpp
 * @brief 用生产几何适配器求周期边界 UV/面坐标；输入仅冻结 1D 网格和拓扑顺序。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/chart/PeriodicFaceChart.h"
#include "cad/occ/OccCadGeometry.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <Standard_Failure.hxx>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;

int main(int argc, char** argv)
{
    if(argc != 4) {
        std::cerr << "usage: GmshPeriodicBoundaryChartCadReplay CAD.brep boundary-topology.tsv output.json\n";
        return 2;
    }
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read exact BREP");
        OccCadGeometry geometry(shape);
        if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        TopTools_IndexedMapOfShape edges, faces;
        TopExp::MapShapes(shape,TopAbs_EDGE,edges); TopExp::MapShapes(shape,TopAbs_FACE,faces);
        std::ifstream in(argv[2]); in.imbue(std::locale::classic());
        PeriodicFaceBoundaryInput input; SurfaceMeshingOptions options;
        options.sizeFromPoints = false;
        std::size_t nodeCount, loopCount; bool graded;
        if(!(in >> input.entity >> nodeCount >> loopCount >> input.modelCharacteristicLength
                >> options.sizeMax >> options.sizeFactor >> graded) || input.entity < 1 || input.entity > faces.Extent() ||
           nodeCount > 1000000 || loopCount > 10000) throw std::runtime_error("invalid boundary topology header");
        SurfaceParameterBounds bounds;
        if(!geometry.faceTrimParameterBounds(input.entity-1,bounds)) throw std::runtime_error("exact trimmed face bounds unavailable");
        input.parameterLower = {bounds.uMin,bounds.vMin}; input.parameterUpper = {bounds.uMax,bounds.vMax};
        BoundaryMesh boundary;
        for(std::size_t i = 0; i < nodeCount; ++i) {
            BoundaryNode node;
            if(!(in >> node.id >> node.point.x >> node.point.y >> node.point.z >> node.dimension >> node.entity >> node.parameter) ||
               !node.id || node.id > 1000000) throw std::runtime_error("invalid frozen 1D physical node");
            if(boundary.nodes.size() < node.id) {
                const auto begin = boundary.nodes.size(); boundary.nodes.resize(node.id);
                for(std::size_t k = begin; k < boundary.nodes.size(); ++k) boundary.nodes[k].alive = false;
            }
            if(boundary.nodes[node.id-1].alive) throw std::runtime_error("duplicate frozen physical node");
            boundary.nodes[node.id-1] = node;
        }
        const auto face = TopoDS::Face(faces(input.entity));
        TopTools_IndexedMapOfShape members; TopExp::MapShapes(face,TopAbs_EDGE,members);
        for(std::size_t i = 0; i < loopCount; ++i) {
            std::size_t count;
            if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid topology edge loop");
            std::vector<PeriodicCurveChart> loop;
            for(std::size_t j = 0; j < count; ++j) {
                PeriodicCurveChart curve; std::size_t points; int orientation, faceOrientation;
                if(!(in >> curve.entity >> curve.seam >> orientation >> faceOrientation >> points) ||
                   curve.entity < 1 || curve.entity > edges.Extent() || points < 2 || points > 1000000)
                    throw std::runtime_error("invalid curve use identity/count");
                auto edge = TopoDS::Edge(edges(curve.entity));
                if(edge.Orientation()==TopAbs_INTERNAL || edge.Orientation()==TopAbs_EXTERNAL) edge.Orientation(TopAbs_FORWARD);
                bool seam;
                if(!members.Contains(edge) || int(edge.Orientation()) != orientation || int(face.Orientation()) != faceOrientation ||
                   !geometry.edgeSeamOnFace(input.entity-1,curve.entity-1,seam) || seam != curve.seam)
                    throw std::runtime_error("CAD identity/orientation/seam differs from native topology");
                std::pair<double,double> range;
                if(!geometry.edgeOnFaceRange(input.entity-1,curve.entity-1,range)) throw std::runtime_error("exact pcurve range unavailable");
                // 原版先起点、边内点、终点；闭环两端物理 ID 相同，参数仍分别取上下限。
                for(std::size_t k = 0; k < points; ++k) {
                    std::uint64_t id;
                    if(!(in >> id) || !id || id > boundary.nodes.size() || !boundary.nodes[id-1].alive)
                        throw std::runtime_error("unknown physical node in curve use");
                    const auto& node = boundary.nodes[id-1];
                    const bool endpoint = k == 0 || k+1 == points;
                    if(endpoint ? node.dimension != 0 : node.dimension != 1 || node.entity != curve.entity)
                        throw std::runtime_error("wrong endpoint/interior classification in curve use");
                    const double t = k == 0 ? range.first : k+1 == points ? range.second : node.parameter;
                    Point2D uv;
                    if(!geometry.edgeOnFaceCoordinates(input.entity-1,curve.entity-1,t,false,uv))
                        throw std::runtime_error("exact pcurve missing; native generic fallback not reproduced");
                    curve.vertices.push_back({id,uv});
                    if(curve.seam) {
                        if(!geometry.edgeOnFaceCoordinates(input.entity-1,curve.entity-1,t,true,uv))
                            throw std::runtime_error("exact alternate seam pcurve unavailable");
                        curve.alternateCoordinates.push_back(uv);
                    }
                }
                loop.push_back(std::move(curve));
            }
            input.edgeLoops.push_back(std::move(loop));
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected boundary topology input tail");
        input.surfacePoint = [&](Point2D uv) { return geometry.facePoint(input.entity-1,uv); };
        GeometrySizeSource sizes;
        if(graded) sizes.sizeCallback = [](const MeshSizeQuery& q, double lc) {
            return std::min(lc,.12+.015*std::abs(q.point.x)+.005*std::abs(q.point.y));
        };
        PeriodicBoundaryChart chart;
        const auto result = buildPeriodicBoundaryChart(boundary,input,options,sizes,chart);
        if(!result.status.complete()) throw std::runtime_error(result.status.message);
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        out << "{\"face\":" << chart.entity << ",\"position_count\":" << chart.loopPositionCount
            << ",\"maximum_id\":" << chart.maximumLocalId << ",\"bounds\":[" << chart.lower.x << ',' << chart.lower.y << ','
            << chart.upper.x << ',' << chart.upper.y << "],\"vertices\":[";
        for(std::size_t i = 0; i < chart.vertices.size(); ++i) {
            if(i) out << ','; const auto& p = chart.vertices[i]; const auto& v = p.vertex;
            out << '[' << v.localId << ',' << v.physicalId << ',' << v.uv.x << ',' << v.uv.y << ',' << v.surfacePoint.x << ','
                << v.surfacePoint.y << ',' << v.surfacePoint.z << ',' << v.dimension << ',' << v.entity << ','
                << p.sizes.backgroundSize << ',' << p.sizes.meshSize << ',' << p.periodicCounterpart << ']';
        }
        out << "],\"loops\":[";
        for(std::size_t i = 0; i < chart.loops.size(); ++i) {
            if(i) out << ','; out << '[';
            for(std::size_t j = 0; j < chart.loops[i].size(); ++j) { if(j) out << ','; out << chart.loops[i][j]; }
            out << ']';
        }
        out << "],\"selections\":[";
        for(std::size_t i = 0; i < chart.selections.size(); ++i) {
            if(i) out << ','; const auto& s = chart.selections[i]; out << '[' << s.tolerance << ',' << (s.seamFirst ? 1 : 0) << ']';
        }
        out << "]}\n"; if(!out) throw std::runtime_error("CAD chart replay output write failed");
        return 0;
    }
    catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
