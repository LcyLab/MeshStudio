/** @file GmshFaceChartReplay.cpp
 * @brief 冻结原生 UV 输入后重放图表组装；不链接 Gmsh/OCC，不将它当几何适配器验收。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/chart/FaceChart.h"
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
    if(argc != 3) { std::cerr << "usage: GmshFaceChartReplay frozen-input.tsv output.json\n"; return 2; }
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic());
        FaceChartInput input; BoundaryMesh boundary;
        std::size_t nodeCount, curveCount, boundaryCount, embeddedCount, embeddedVertexCount;
        if(!(in >> input.entity >> nodeCount >> curveCount >> boundaryCount >> embeddedCount >> embeddedVertexCount))
            throw std::runtime_error("invalid frozen face header");
        std::map<std::uint64_t,Point2D> uv;
        std::map<std::pair<double,double>,Point3D> points;
        for(std::size_t i = 0; i < nodeCount; ++i) {
            BoundaryNode node; bool available; Point2D param; Point3D point;
            in >> node.id >> node.point.x >> node.point.y >> node.point.z >> node.dimension >> node.entity
               >> node.parameter >> node.storedSize >> available >> param.x >> param.y >> point.x >> point.y >> point.z;
            if(!in || !node.id || node.id > 1000000) throw std::runtime_error("invalid frozen boundary node");
            if(boundary.nodes.size() < node.id) {
                const auto first = boundary.nodes.size(); boundary.nodes.resize(node.id);
                for(std::size_t k = first; k < boundary.nodes.size(); ++k) boundary.nodes[k].alive = false;
            }
            if(boundary.nodes[node.id-1].alive) throw std::runtime_error("duplicate frozen boundary node");
            boundary.nodes[node.id-1] = node;
            if(available) { uv.emplace(node.id,param); points.emplace(std::make_pair(param.x,param.y),point); }
        }
        for(std::size_t i = 0; i < curveCount; ++i) {
            BoundaryCurve curve; std::size_t samples;
            in >> curve.entity >> curve.beginNode >> curve.endNode >> curve.meshDegenerate >> curve.geometryDegenerate >> samples;
            for(std::size_t j = 0; j < samples; ++j) {
                BoundarySample sample; in >> sample.node >> sample.parameter; curve.samples.push_back(sample);
            }
            boundary.curves.push_back(std::move(curve));
        }
        for(std::size_t i = 0; i < boundaryCount; ++i) {
            FaceCurveUse use; in >> use.entity >> use.seam; input.boundaryCurves.push_back(use);
        }
        for(std::size_t i = 0; i < embeddedCount; ++i) {
            FaceCurveUse use; in >> use.entity >> use.seam; input.embeddedCurves.push_back(use);
        }
        for(std::size_t i = 0; i < embeddedVertexCount; ++i) {
            std::uint64_t id; in >> id; input.embeddedVertices.push_back(id);
        }
        if(!in) throw std::runtime_error("truncated frozen chart input");
        std::string trailing; if(in >> trailing) throw std::runtime_error("unexpected frozen input tail");
        input.nodeCoordinates = [&](const BoundaryNode& node) { return uv.at(node.id); };
        input.surfacePoint = [&](Point2D p) { return points.at({p.x,p.y}); };
        FaceChart chart; const auto result = buildFaceChart(boundary,input,chart);
        if(!result.status.complete()) throw std::runtime_error(result.status.message);
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        out << "{\"face\":" << chart.entity << ",\"vertices\":[";
        for(std::size_t i = 0; i < chart.vertices.size(); ++i) {
            if(i) out << ','; const auto& v = chart.vertices[i];
            out << '[' << v.localId << ',' << v.physicalId << ',' << v.uv.x << ',' << v.uv.y << ','
                << v.surfacePoint.x << ',' << v.surfacePoint.y << ',' << v.surfacePoint.z << ',' << v.dimension << ',' << v.entity << ']';
        }
        out << "],\"initial_vertices\":[";
        for(std::size_t i = 0; i < chart.initialVertices.size(); ++i) {
            if(i) out << ','; const auto& v = chart.initialVertices[i];
            out << '[' << v.physicalId << ',' << v.uv.x << ',' << v.uv.y << ']';
        }
        out << "],\"constraints\":[";
        for(std::size_t i = 0; i < chart.constraints.size(); ++i) {
            if(i) out << ','; const auto& c = chart.constraints[i];
            out << '[' << c.curveEntity << ',' << c.from << ',' << c.to << ',' << (c.embedded ? 1 : 0) << ']';
        }
        out << "]}\n";
        if(!out) throw std::runtime_error("chart replay output write failed");
        std::cout << "face chart " << chart.entity << " nodes=" << chart.vertices.size() << '\n';
        return 0;
    }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
