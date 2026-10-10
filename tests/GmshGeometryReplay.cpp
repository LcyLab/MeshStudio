/** @file GmshGeometryReplay.cpp
 * @brief 读取固定数值案例重放本项目底层模块；参考 DLL 由外部 Python 调用。
 */
#include "geometry/predicates/GeomPredicates.h"
#include "mesh/surface/algorithm6/geometry/SurfaceMetric.h"
#include "geometry/numerics/LinearSystem3.h"
#include "mesh/surface/algorithm6/boundary/CurveDiscretization.h"
#include "mesh/surface/algorithm6/geometry/CurveSurfaceIntersection.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
const char* intersectionStopName(IntersectionStop stop)
{
    switch(stop) {
    case IntersectionStop::InitialResidual: return "initialResidual";
    case IntersectionStop::NewtonStep: return "newtonStep";
    case IntersectionStop::ExactZeroResidualExit: return "exactZeroResidualExit";
    case IntersectionStop::ParameterNormLimit: return "parameterNormLimit";
    case IntersectionStop::IterationLimit: return "iterationLimit";
    case IntersectionStop::NonfiniteStep: return "nonfiniteStep";
    case IntersectionStop::InvalidQuery: return "invalidQuery";
    case IntersectionStop::Cancelled: return "cancelled";
    case IntersectionStop::BudgetExceeded: return "budgetExceeded";
    }
    throw std::runtime_error("unknown intersection stop reason");
}
}
int main(int argc, char** argv)
{
    if(argc != 3) { std::cerr << "usage: GmshGeometryReplay inputs.tsv output.jsonl\n"; return 2; }
    try {
        std::ifstream input(argv[1]); std::ofstream output(argv[2]);
        if(!input || !output) throw std::runtime_error("cannot open geometry replay input/output");
        input.imbue(std::locale::classic()); output.imbue(std::locale::classic()); output << std::setprecision(17);
        std::string line;
        int entity = 0;
        while(std::getline(input, line)) {
            if(line.empty()) continue;
            std::istringstream row(line); row.imbue(std::locale::classic());
            std::string operation; row >> operation;
            std::vector<double> data;
            double value;
            while(row >> value) data.push_back(value);
            if(!row.eof()) throw std::runtime_error("malformed geometry replay number");
            std::vector<double> values;
            std::string diagnostics;
            auto uv = [&](int index) { return Point2D{data[index], data[index + 1]}; };
            auto xyz = [&](int index) { return Point3D{data[index], data[index + 1], data[index + 2]}; };
            if(operation == "metric_circle" && data.size() == 9) {
                const auto circle = computeMetricCircumcircle({uv(0), uv(2), uv(4)}, {data[6], data[7], data[8]});
                if(!circle.status.complete()) throw std::runtime_error(circle.status.message);
                values = {circle.center.x, circle.center.y, circle.radiusSquared};
            }
            else if(operation == "physical_circle" && data.size() == 9) {
                const auto circle = computePhysicalCircumradius({xyz(0), xyz(3), xyz(6)});
                if(!circle.status.complete()) throw std::runtime_error(circle.status.message);
                values = {circle.center.x, circle.center.y, circle.center.z, circle.radius};
            }
            else if(operation == "linear_system" && data.size() == 12) {
                Matrix3 matrix{};
                for(int i = 0; i < 3; ++i) for(int j = 0; j < 3; ++j) matrix[i][j] = data[3 * i + j];
                const auto solved = solveColumnPivotedQr(matrix, {data[9], data[10], data[11]});
                if(!solved.finiteResult) throw std::runtime_error("linear replay solve returned nonfinite values");
                values.assign(solved.solution.begin(), solved.solution.end());
            }
            else if(operation == "curve_smoothing" && data.size() >= 10 && (data.size() - 2) % 4 == 0) {
                const int count = static_cast<int>(data[1]);
                if(count < 2 || data.size() != static_cast<std::size_t>(2 + 4 * count))
                    throw std::runtime_error("invalid curve smoothing replay width");
                CurveDensityTable table;
                for(int i = 0; i < count; ++i) {
                    const int begin = 2 + 4 * i;
                    table.samples.push_back({data[begin], data[begin + 1], data[begin + 2], data[begin + 3]});
                }
                const auto status = smoothCurveSizes(table, data[0]);
                if(!status.complete()) throw std::runtime_error(status.message);
                for(const auto& sample : table.samples) {
                    values.push_back(sample.parameter); values.push_back(sample.density);
                    values.push_back(sample.cumulativeDensity); values.push_back(sample.speed);
                }
            }
            else if(operation == "intersection" && data.size() == 16) {
                const SurfaceFunction surface = [&](Point2D point) {
                    return Point3D{point.x, point.y, data[1] + data[2] * (point.x * point.x + point.y * point.y)};
                };
                const Vector3 initial{data[3], data[4], data[5]};
                const auto intersection = data[0] == 1.
                    ? intersectCircleWithSurface({xyz(6), xyz(9), xyz(12), data[15]}, surface, initial)
                    : solveCurveSurfaceIntersection([](double t) { return Point3D{t, 0., t * t}; }, surface, initial);
                if(!intersection.status.complete() && intersection.status.code != MeshingStatus::NonConverged)
                    throw std::runtime_error(intersection.status.message);
                values = {double(intersection.intersectionFound), intersection.parameters[0],
                          intersection.parameters[1], intersection.parameters[2], double(intersection.residualEvaluations)};
                diagnostics = std::string(",\"intersection_stop\":\"") + intersectionStopName(intersection.stop) + '"';
            }
            else if(operation == "curve_filter" || operation == "curve_filter_field") {
                const bool field = operation == "curve_filter_field";
                const int offset = field ? 4 : 3;
                if(data.size() < static_cast<std::size_t>(offset)) throw std::runtime_error("missing curve filtering replay header");
                const int count = static_cast<int>(data[offset - 1]);
                if(count < 0 || data.size() != static_cast<std::size_t>(offset + 4 * count))
                    throw std::runtime_error("invalid curve filter replay width");
                std::vector<CurveNode> nodes;
                for(int i = 0; i < count; ++i)
                    nodes.push_back({static_cast<std::uint64_t>(i), data[offset + 4 * i], xyz(offset + 1 + 4 * i), -1.});
                const auto filtered = filterCurveNodes(nodes, {}, 0., static_cast<int>(data[offset - 2]),
                    [&](double t) { return field ? data[0] * (1. + data[1] * t) : data[0]; });
                if(!filtered.status.complete()) throw std::runtime_error(filtered.status.message);
                values.push_back(double(nodes.size()));
                for(const auto& node : nodes) values.push_back(double(node.id));
            }
            else if(operation == "orient2d" && data.size() == 6) values = {double(orient2d(uv(0), uv(2), uv(4)))};
            else if(operation == "orient2d_value" && data.size() == 6) {
                const double area = orient2dValue(uv(0), uv(2), uv(4));
                values = {area, double((area > 0.) - (area < 0.))};
            }
            else if(operation == "incircle" && data.size() == 8) values = {double(incircle(uv(0), uv(2), uv(4), uv(6)))};
            else if(operation == "incircle_oriented" && data.size() == 8)
                values = {double(incircleOriented(uv(0), uv(2), uv(4), uv(6)))};
            else throw std::runtime_error("unsupported geometry replay operation/width");
            output << "{\"stage\":\"" << operation << "\",\"entity\":" << entity++ << ",\"values\":[";
            for(std::size_t i = 0; i < values.size(); ++i) { if(i) output << ','; output << values[i]; }
            output << ']' << diagnostics << "}\n";
        }
        if(!input.eof() || !output) throw std::runtime_error("geometry replay read/write failed");
        std::cout << "replayed " << entity << " geometry cases\n";
        return 0;
    }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
