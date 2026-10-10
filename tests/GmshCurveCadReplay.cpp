/** @file GmshCurveCadReplay.cpp
 * @brief 对同一 BREP 的原生 OCC D1 运行独立曲线模块，导出可对照阶段。
 * 这是测试端几何接入，核心仍只读取回调，不链接 Gmsh。
 */
#include "mesh/surface/algorithm6/boundary/CurveDiscretization.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>

using namespace mesher;
using namespace mesher::gmshReference;
namespace {
void recordTable(std::ostream& stream, const char* stage, int entity, const CurveDensityTable& table)
{
    stream << "{\"stage\":\"" << stage << "\",\"entity\":" << entity << ",\"samples\":[";
    for(std::size_t i = 0; i < table.samples.size(); ++i) {
        if(i) stream << ',';
        const auto& sample = table.samples[i];
        stream << '[' << sample.parameter << ',' << sample.density << ','
               << sample.cumulativeDensity << ',' << sample.speed << ']';
    }
    stream << "]}\n";
}

int minimumSegments(const TopoDS_Edge& edge, const BRepAdaptor_Curve& curve)
{
    const auto type = curve.GetType();
    const auto kind = type == GeomAbs_Line ? CurveKind::Line : type == GeomAbs_Circle ? CurveKind::Circle :
                      type == GeomAbs_Ellipse ? CurveKind::Ellipse : CurveKind::Other;
    TopoDS_Vertex first, last;
    TopExp::Vertices(edge, first, last);
    return computeMinimumCurveSegments(kind, curve.LastParameter() - curve.FirstParameter(),
        !first.IsNull() && !last.IsNull() && first.IsSame(last), 1, {});
}
}

int main(int argc, char** argv)
{
    if(argc != 4) { std::cerr << "usage: GmshCurveCadReplay CAD.brep size output.jsonl\n"; return 2; }
    try {
        const double size = std::stod(argv[2]);
        if(!std::isfinite(size) || size <= 0.) throw std::runtime_error("invalid mesh size");
        BRep_Builder builder;
        TopoDS_Shape shape;
        if(!BRepTools::Read(shape, argv[1], builder) || shape.IsNull()) throw std::runtime_error("cannot read CAD BREP");
        TopTools_IndexedMapOfShape edges;
        TopExp::MapShapes(shape, TopAbs_EDGE, edges);
        std::ofstream output(argv[3]);
        if(!output) throw std::runtime_error("cannot open curve replay output");
        output.imbue(std::locale::classic()); output << std::setprecision(17);
        for(int entity = 1; entity <= edges.Extent(); ++entity) {
            const auto edge = TopoDS::Edge(edges(entity));
            if(BRep_Tool::Degenerated(edge)) continue;
            BRepAdaptor_Curve curve(edge);
            const auto derivative = [&](double parameter) {
                gp_Pnt point; gp_Vec tangent;
                curve.D1(parameter, point, tangent);
                return Point3D{tangent.X(), tangent.Y(), tangent.Z()};
            };
            CurveDensityTable table;
            auto status = integrateCurveDensity(curve.FirstParameter(), curve.LastParameter(), derivative,
                [=](double) { return size; }, table);
            if(!status.complete()) throw std::runtime_error(status.message);
            recordTable(output, "curve_density", entity, table);
            status = smoothCurveSizes(table, std::sqrt(1.8));
            if(!status.complete()) throw std::runtime_error(status.message);
            recordTable(output, "curve_smoothed", entity, table);
            const int minimum = minimumSegments(edge, curve);
            const int count = computeCurvePointCount(table.samples.back().cumulativeDensity, minimum);
            output << "{\"stage\":\"curve_count\",\"entity\":" << entity << ",\"integral\":"
                   << table.samples.back().cumulativeDensity << ",\"minimum_segments\":" << minimum
                   << ",\"point_count\":" << count << "}\n";
            std::vector<double> parameters;
            status = distributeCurveParameters(table, count, parameters);
            if(!status.complete()) throw std::runtime_error(status.message);
            std::size_t bracket = 1;
            for(int i = 1; i < count - 1; ++i) {
                const double cumulative = double(i) * (table.samples.back().cumulativeDensity / double(count - 1));
                output << "{\"stage\":\"curve_parameter\",\"entity\":" << entity
                       << ",\"index\":" << i << ",\"parameter\":" << parameters[i]
                       << ",\"cumulative\":" << cumulative << "}\n";
                while(bracket < table.samples.size() && !(table.samples[bracket].cumulativeDensity >= cumulative &&
                      table.samples[bracket - 1].cumulativeDensity < cumulative)) ++bracket;
                if(bracket >= table.samples.size()) throw std::runtime_error("curve stored-size bracket missing");
                const auto tangent = derivative(parameters[i]);
                const double speed = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z);
                const auto stored = computeStoredCurveSize(table.samples[bracket - 1], table.samples[bracket], speed);
                if(!stored.status.complete() || !stored.finiteResult)
                    throw std::runtime_error("nonfinite stored curve size in finite CAD replay fixture");
                const auto point = curve.Value(parameters[i]);
                output << "{\"stage\":\"curve_node_creation\",\"entity\":" << entity << ",\"index\":" << i
                       << ",\"parameter\":" << parameters[i] << ",\"speed\":" << speed << ",\"stored_size\":" << stored.size
                       << ",\"x\":" << point.X() << ",\"y\":" << point.Y() << ",\"z\":" << point.Z() << "}\n";
            }
        }
        if(!output) throw std::runtime_error("curve replay write failed");
        std::cout << "replayed " << edges.Extent() << " CAD edges\n";
        return 0;
    }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
