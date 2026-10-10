/** @file GmshPeriodicBoundaryChartReplay.cpp
 * @brief 冻结 pcurve/面坐标后重放周期 BDS 边界初始化，不链接 Gmsh/OCC。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/chart/PeriodicFaceChart.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main(int argc, char** argv)
{
    if(argc != 3) { std::cerr << "usage: GmshPeriodicBoundaryChartReplay input.tsv output.json\n"; return 2; }
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic());
        PeriodicFaceBoundaryInput input; SurfaceMeshingOptions options;
        options.sizeFromPoints = false;
        std::size_t nodeCount, loopCount, surfaceCount; bool graded;
        if(!(in >> input.entity >> nodeCount >> loopCount >> surfaceCount >> input.parameterLower.x >> input.parameterLower.y
                >> input.parameterUpper.x >> input.parameterUpper.y >> input.modelCharacteristicLength
                >> options.sizeMax >> options.sizeFactor >> graded) || nodeCount > 1000000 || loopCount > 10000 || surfaceCount > 1000000)
            throw std::runtime_error("invalid frozen periodic chart header");
        BoundaryMesh boundary;
        for(std::size_t i = 0; i < nodeCount; ++i) {
            BoundaryNode node;
            in >> node.id >> node.point.x >> node.point.y >> node.point.z >> node.dimension >> node.entity >> node.parameter;
            if(!in || !node.id || node.id > 1000000) throw std::runtime_error("invalid frozen physical node");
            if(boundary.nodes.size() < node.id) {
                const auto begin = boundary.nodes.size(); boundary.nodes.resize(node.id);
                for(std::size_t k = begin; k < boundary.nodes.size(); ++k) boundary.nodes[k].alive = false;
            }
            if(boundary.nodes[node.id-1].alive) throw std::runtime_error("duplicate frozen physical node");
            boundary.nodes[node.id-1] = node;
        }
        for(std::size_t i = 0; i < loopCount; ++i) {
            std::size_t count; if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid frozen edge loop");
            std::vector<PeriodicCurveChart> loop;
            for(std::size_t j = 0; j < count; ++j) {
                PeriodicCurveChart curve; std::size_t points;
                if(!(in >> curve.entity >> curve.seam >> points) || points > 1000000)
                    throw std::runtime_error("invalid frozen curve header");
                for(std::size_t k = 0; k < points; ++k) {
                    PeriodicLoopVertex vertex; Point2D alternate;
                    in >> vertex.physicalId >> vertex.uv.x >> vertex.uv.y;
                    if(curve.seam) { in >> alternate.x >> alternate.y; curve.alternateCoordinates.push_back(alternate); }
                    curve.vertices.push_back(vertex);
                }
                loop.push_back(std::move(curve));
            }
            input.edgeLoops.push_back(std::move(loop));
        }
        std::map<std::pair<double,double>,Point3D> surface;
        for(std::size_t i = 0; i < surfaceCount; ++i) {
            Point2D uv; Point3D point; in >> uv.x >> uv.y >> point.x >> point.y >> point.z;
            auto inserted = surface.emplace(std::make_pair(uv.x,uv.y),point);
            if(!inserted.second && (inserted.first->second.x != point.x || inserted.first->second.y != point.y || inserted.first->second.z != point.z))
                throw std::runtime_error("inconsistent frozen surface query");
        }
        if(!in) throw std::runtime_error("truncated frozen chart input");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected frozen input tail");
        input.surfacePoint = [&](Point2D uv) { return surface.at({uv.x,uv.y}); };
        GeometrySizeSource sizes;
        if(graded) sizes.sizeCallback = [](const MeshSizeQuery& q, double lc) {
            return std::min(lc,.12+.015*std::abs(q.point.x)+.005*std::abs(q.point.y));
        };
        PeriodicBoundaryChart chart;
        const auto result = buildPeriodicBoundaryChart(boundary,input,options,sizes,chart);
        if(!result.status.complete()) throw std::runtime_error(result.status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
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
        out << "]}\n"; if(!out) throw std::runtime_error("chart replay output write failed");
        return 0;
    }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
