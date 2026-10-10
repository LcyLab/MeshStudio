// 文件职责：单面图表准备：把共享物理边界映射为面内 UV，区分普通、直接三角形和周期入口。
#include "FaceChart.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>

namespace mesher::gmshReference {
/// 按 CAD 物理身份建立普通面 UV 图表、约束及初始插入顺序。
FaceChartBuildResult buildFaceChart(const BoundaryMesh& boundary, const FaceChartInput& input,
    FaceChart& output, const RunControl& control)
{
    FaceChartBuildResult result;
    try {
        auto check = [&] {
            if(control.keepGoing && !control.keepGoing())
                throw OperationFailure{{MeshingStatus::Cancelled,"face chart cancelled"}};
        };
        auto query = [&](auto&& callback) {
            check();
            if(result.geometryEvaluations >= control.maximumGeometryEvaluations)
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"face chart geometry-query budget exceeded"}};
            ++result.geometryEvaluations;
            try { return callback(); }
            catch(const OperationFailure&) { throw; }
            catch(const std::exception& error) {
                throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}};
            }
            catch(...) {
                throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"unknown face chart query exception"}};
            }
        };
        check();
        if(input.entity < 0)
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid CAD face identity"}};
        std::map<int,const BoundaryCurve*> curves;
        for(const auto& curve : boundary.curves)
            if(!curves.emplace(curve.entity,&curve).second)
                throw OperationFailure{{MeshingStatus::InvalidTopology,"duplicate boundary curve identity"}};
        auto curveAt = [&](int entity) -> const BoundaryCurve& {
            const auto found = curves.find(entity);
            if(found == curves.end())
                throw OperationFailure{{MeshingStatus::InvalidTopology,"face uses unknown CAD edge"}};
            return *found->second;
        };
        auto nodeAt = [&](std::uint64_t id) -> const BoundaryNode& {
            if(!id || id > boundary.nodes.size())
                throw OperationFailure{{MeshingStatus::InvalidTopology,"face uses unknown physical node"}};
            const auto& node = boundary.nodes[id-1];
            if(node.id != id || !node.alive || (node.dimension != 0 && node.dimension != 1) ||
               !std::isfinite(node.point.x) || !std::isfinite(node.point.y) || !std::isfinite(node.point.z))
                throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid face physical node"}};
            return node;
        };
        std::set<std::uint64_t> physicalNodes, parity;
        const auto toggle = [&](std::uint64_t id) {
            if(!parity.insert(id).second) parity.erase(id);
        };
        FaceChart chart; chart.entity = input.entity;
        for(const auto& use : input.boundaryCurves) {
            check();
            const auto& curve = curveAt(use.entity);
            if(use.seam) {
                result.route = FaceChartRoute::PeriodicRequired;
                result.status = {MeshingStatus::NonConverged,"seam requires the periodic face chart path"};
                return result;
            }
            chart.boundaryVersions.push_back({curve.entity,curve.version});
            if(curve.meshDegenerate) continue;
            for(std::size_t segment = 1; segment < curve.samples.size(); ++segment) {
                const auto from = curve.samples[segment-1].node, to = curve.samples[segment].node;
                nodeAt(from); nodeAt(to);
                physicalNodes.insert(from); physicalNodes.insert(to);
                toggle(from); toggle(to);
            }
        }
        if(!parity.empty())
            throw OperationFailure{{MeshingStatus::InvalidTopology,"1D mesh does not form a closed face boundary"}};
        for(const auto& use : input.embeddedCurves) {
            check();
            const auto& curve = curveAt(use.entity);
            if(use.seam) {
                result.route = FaceChartRoute::PeriodicRequired;
                result.status = {MeshingStatus::NonConverged,"embedded seam requires the multiple-UV initial chart path"};
                return result;
            }
            chart.boundaryVersions.push_back({curve.entity,curve.version});
            if(curve.meshDegenerate) continue;
            nodeAt(curve.beginNode); nodeAt(curve.endNode);
            physicalNodes.insert(curve.beginNode); physicalNodes.insert(curve.endNode);
            for(const auto& sample : curve.samples) { nodeAt(sample.node); physicalNodes.insert(sample.node); }
        }
        for(auto id : input.embeddedVertices) {
            check();
            if(nodeAt(id).dimension != 0)
                throw OperationFailure{{MeshingStatus::InvalidTopology,"embedded vertex must have CAD dimension zero"}};
            physicalNodes.insert(id);
        }
        if(physicalNodes.size() > control.maximumNodes ||
           physicalNodes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"face chart node budget exceeded"}};
        std::map<std::uint64_t,int> localIds;
        for(auto id : physicalNodes) {
            const auto& node = nodeAt(id);
            const int local = static_cast<int>(chart.vertices.size());
            localIds.emplace(id,local);
            chart.vertices.push_back({local,id,node.dimension,node.entity,{},node.point});
        }
        if(physicalNodes.size() <= 3) {
            chart.route = physicalNodes.size() == 3 ? FaceChartRoute::DirectTriangle : FaceChartRoute::SkipInsufficientNodes;
            result.route = chart.route;
            check(); output = std::move(chart); return result;
        }
        if(!input.nodeCoordinates || !input.surfacePoint)
            throw OperationFailure{{MeshingStatus::InvalidInput,"face chart requires exact UV and surface evaluators"}};
        auto coordinates = [&](std::uint64_t id) {
            const auto uv = query([&] { return input.nodeCoordinates(nodeAt(id)); });
            if(!std::isfinite(uv.x) || !std::isfinite(uv.y))
                throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite face UV query"}};
            return uv;
        };
        for(auto& vertex : chart.vertices) {
            vertex.uv = coordinates(vertex.physicalId);
            const auto point = query([&] { return input.surfacePoint(vertex.uv); });
            if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite face surface point"}};
            vertex.surfacePoint = point;
        }
        const auto appendConstraints = [&](int entity, bool embedded) {
            const auto& curve = curveAt(entity);
            if(curve.meshDegenerate) return;
            for(std::size_t segment = 1; segment < curve.samples.size(); ++segment)
                chart.constraints.push_back({entity,localIds.at(curve.samples[segment-1].node),
                    localIds.at(curve.samples[segment].node),embedded});
        };
        for(const auto& use : input.boundaryCurves) appendConstraints(use.entity,false);
        for(const auto& use : input.embeddedCurves) appendConstraints(use.entity,true);
        // getNodeCopies 的插入顺序和 STL unordered_map 枚举顺序是默认入口合同。
        // 此处 ordinary 路径没有 seam 分支；同一物理节点的不可变 UV 查询相同。
        std::unordered_map<std::uint64_t,Point2D> copies;
        auto collectCopies = [&](int entity) {
            check();
            const auto& curve = curveAt(entity);
            if(curve.meshDegenerate) return;
            std::set<std::uint64_t> nodes;
            for(std::size_t segment = 1; segment < curve.samples.size(); ++segment) {
                nodes.insert(curve.samples[segment-1].node); nodes.insert(curve.samples[segment].node);
            }
            for(auto id : nodes) copies.emplace(id,coordinates(id));
        };
        for(const auto& use : input.boundaryCurves) collectCopies(use.entity);
        for(const auto& use : input.embeddedCurves) collectCopies(use.entity);
        if(input.boundaryCurves.empty() && input.embeddedCurves.empty())
            for(const auto& curve : boundary.curves) collectCopies(curve.entity);
        for(auto id : input.embeddedVertices) copies.emplace(id,coordinates(id));
        if(copies.size() > control.maximumNodes)
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"initial chart copy budget exceeded"}};
        for(const auto& copy : copies) chart.initialVertices.push_back({copy.first,copy.second});
        check(); output = std::move(chart);
    }
    catch(const OperationFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    catch(...) { result.status = {MeshingStatus::GeometryQueryFailed,"unknown face chart exception"}; }
    return result;
}

/// 在 UV 中选择边方向和接缝分支，拼接闭合边界环。
OperationStatus assemblePeriodicBoundaryLoop(const std::vector<PeriodicCurveChart>& curves,
    double tolerance, bool seamFirst, PeriodicBoundaryLoop& output, const RunControl& control)
{
    try {
        auto check = [&] {
            if(control.keepGoing && !control.keepGoing())
                throw OperationFailure{{MeshingStatus::Cancelled,"periodic boundary loop cancelled"}};
        };
        check();
        if(!std::isfinite(tolerance) || tolerance <= 0.)
            return {MeshingStatus::InvalidInput,"periodic loop tolerance must be finite and positive"};
        auto finite = [](Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); };
        for(const auto& curve : curves) {
            check();
            if(curve.entity < 0 || curve.vertices.size() < 2 ||
               (curve.seam && curve.alternateCoordinates.size() != curve.vertices.size()) ||
               (!curve.seam && !curve.alternateCoordinates.empty()))
                return {MeshingStatus::InvalidInput,"invalid periodic curve chart or seam copy count"};
            for(const auto& vertex : curve.vertices)
                if(!vertex.physicalId || !finite(vertex.uv))
                    return {MeshingStatus::InvalidInput,"invalid periodic physical identity or UV"};
            for(auto uv : curve.alternateCoordinates)
                if(!finite(uv)) return {MeshingStatus::InvalidInput,"nonfinite periodic seam UV"};
        }
        auto distance = [](Point2D a, Point2D b) {
            const double x = a.x-b.x, y = a.y-b.y;
            return std::sqrt(x*x+y*y);
        };
        PeriodicBoundaryLoop loop;
        for(int initial = 0; initial < 2; ++initial) {
            if(!loop.vertices.empty()) break;
            loop.initialDirection = initial;
            for(std::size_t i = 0; i < curves.size(); ++i) {
                check();
                const auto& curve = curves[i];
                const auto size = curve.vertices.size();
                auto variant = [&](int index) {
                    const bool reverse = index % 2 != 0, alternate = index >= 2;
                    std::vector<PeriodicLoopVertex> points;
                    points.reserve(size);
                    for(std::size_t j = 0; j < size; ++j) {
                        const auto k = reverse ? size-1-j : j;
                        points.push_back({curve.vertices[k].physicalId,
                            alternate ? curve.alternateCoordinates[k] : curve.vertices[k].uv});
                    }
                    return points;
                };
                if(i == 0) {
                    loop.vertices = variant(initial + (curve.seam && seamFirst ? 2 : 0));
                    continue;
                }
                const auto last = loop.vertices.back().uv;
                const int count = curve.seam ? 4 : 2;
                double distances[4] = {distance(last,curve.vertices.front().uv),
                                      distance(last,curve.vertices.back().uv),0.,0.};
                if(curve.seam) {
                    distances[2] = distance(last,curve.alternateCoordinates.front());
                    distances[3] = distance(last,curve.alternateCoordinates.back());
                }
                int selected = -1;
                // 原版要求严格唯一最小；相等时不得用首项或实体方向决胜。
                for(int candidate = 0; candidate < count; ++candidate) {
                    bool closest = distances[candidate] < tolerance;
                    for(int other = 0; other < count; ++other)
                        if(other != candidate && !(distances[candidate] < distances[other])) closest = false;
                    if(closest) { selected = candidate; break; }
                }
                if(selected < 0) {
                    if(initial == 0) { loop.vertices.clear(); break; }
                    return {MeshingStatus::NonConverged,"no unique periodic curve continuation within tolerance"};
                }
                auto points = variant(selected);
                loop.vertices.pop_back();
                loop.vertices.insert(loop.vertices.end(),points.begin(),points.end());
            }
        }
        if(!loop.vertices.empty()) {
            if(!(distance(loop.vertices.back().uv,loop.vertices.front().uv) < tolerance))
                return {MeshingStatus::NonConverged,"periodic boundary loop does not close within tolerance"};
            loop.vertices.pop_back();
        }
        if(loop.vertices.size() > control.maximumNodes)
            return {MeshingStatus::BudgetExceeded,"periodic boundary loop node budget exceeded"};
        check(); output = std::move(loop);
        return {};
    }
    catch(const OperationFailure& failure) { return failure.status; }
    catch(const std::exception& error) { return {MeshingStatus::InvalidInput,error.what()}; }
    catch(...) { return {MeshingStatus::InvalidInput,"unknown periodic loop input failure"}; }
}
}
