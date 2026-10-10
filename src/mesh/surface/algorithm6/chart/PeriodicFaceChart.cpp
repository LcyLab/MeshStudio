/** @file PeriodicFaceChart.cpp
 * @brief 周期边界、嵌入点和初始图表；保留无节点引用的嵌入边分类登记。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "PeriodicFaceChart.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <set>

namespace mesher::gmshReference {
/// 推进本次运行的随机状态并对周期图表 UV 作微小扰动。
Point2D PeriodicInitialPerturbation::perturb(Point2D uv,double span,double factor) {
    auto next=[&]{state=state*214013u+2531011u;return (state>>16)&32767u;};
    const double x=factor*span*double(next())/32767.;
    const double y=factor*span*double(next())/32767.;
    return {uv.x+x,uv.y+y};
}
/// 为周期面建立保留接缝分支的节点图表。
FaceChartBuildResult buildPeriodicFaceChart(const BoundaryMesh& physical,
    const PeriodicFaceChartInput& input, const SurfaceMeshingOptions& options,
    const GeometrySizeSource& sizes, PeriodicFaceChart& output, const RunControl& control)
{
    FaceChartBuildResult result;
    try {
        auto check = [&] {
            if(control.keepGoing && !control.keepGoing())
                throw OperationFailure{{MeshingStatus::Cancelled,"periodic face chart cancelled"}};
        };
        auto remaining = [&] {
            RunControl phase = control;
            phase.maximumGeometryEvaluations -= result.geometryEvaluations;
            return phase;
        };
        PeriodicFaceChart chart;
        const auto truth = sampleTrueBoundary(input.trueBoundaryCurves,chart.trueBoundary,remaining());
        result.geometryEvaluations += truth.geometryEvaluations;
        if(!truth.status.complete()) throw OperationFailure{truth.status};
        const auto boundary = buildPeriodicBoundaryChart(physical,input.boundary,options,sizes,chart.boundary,remaining());
        result.geometryEvaluations += boundary.geometryEvaluations; result.route = boundary.route;
        if(!boundary.status.complete()) throw OperationFailure{boundary.status};
        if(boundary.route != FaceChartRoute::General) { check(); output = std::move(chart); return result; }
        if(!input.embedded.perturbInitialPoint)
            throw OperationFailure{{MeshingStatus::InvalidInput,"periodic native chart requires an explicit initial perturbation stream"}};
        auto embeddedInput = input.embedded;
        embeddedInput.modelCharacteristicLength = input.boundary.modelCharacteristicLength;
        embeddedInput.surfacePoint = input.boundary.surfacePoint;
        const auto embedded = appendPeriodicEmbeddedChart(physical,chart.boundary,embeddedInput,options,sizes,chart.embedded,remaining());
        result.geometryEvaluations += embedded.geometryEvaluations;
        if(!embedded.status.complete()) throw OperationFailure{embedded.status};
        if(chart.embedded.initialPointCountMismatch)
            throw OperationFailure{{MeshingStatus::InvalidTopology,"native embedded seam selection caused a DocRecord point-count mismatch"}};
        chart.expectedInitialPointCount = chart.boundary.loopPositionCount+chart.embedded.expectedInitialPointCount;
        if(chart.expectedInitialPointCount > control.maximumNodes)
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic initial record budget exceeded"}};
        chart.initialPoints = chart.embedded.initialPoints;
        std::map<int,const PeriodicBoundaryVertex*> byId;
        for(const auto& point : chart.boundary.vertices) byId.emplace(point.vertex.localId,&point);
        for(const auto& loop : chart.boundary.loops) for(int id : loop) {
            check();
            const auto found = byId.find(id);
            if(found == byId.end()) throw OperationFailure{{MeshingStatus::InvalidTopology,"unknown boundary DocRecord point"}};
            Point2D initial = found->second->vertex.uv;
            if(embeddedInput.perturbInitialPoint) {
                try { initial = embeddedInput.perturbInitialPoint(initial); }
                catch(const OperationFailure&) { throw; }
                catch(...) { throw OperationFailure{{MeshingStatus::InvalidInput,"boundary initial point perturbation failed"}}; }
            }
            if(!std::isfinite(initial.x) || !std::isfinite(initial.y))
                throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite boundary initial point perturbation"}};
            chart.initialPoints.push_back({id,initial});
        }
        if(chart.initialPoints.size() != chart.expectedInitialPointCount)
            throw OperationFailure{{MeshingStatus::InvalidTopology,"periodic initial record count differs from native allocation"}};
        check(); output = std::move(chart);
    }
    catch(const OperationFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    catch(...) { result.status = {MeshingStatus::InvalidTopology,"unknown periodic face chart failure"}; }
    return result;
}

/// 将内嵌节点和曲线追加到周期图表。
BoundaryOperationResult appendPeriodicEmbeddedChart(const BoundaryMesh& physical,
    const PeriodicBoundaryChart& boundary, const PeriodicEmbeddedInput& input,
    const SurfaceMeshingOptions& options, const GeometrySizeSource& sizes,
    PeriodicEmbeddedChart& output, const RunControl& control)
{
    BoundaryOperationResult result;
    try {
        auto check = [&] {
            if(control.keepGoing && !control.keepGoing())
                throw OperationFailure{{MeshingStatus::Cancelled,"periodic embedded chart cancelled"}};
        };
        auto query = [&](auto&& callback) {
            check();
            if(result.geometryEvaluations >= control.maximumGeometryEvaluations)
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic embedded query budget exceeded"}};
            ++result.geometryEvaluations;
            try { return callback(); }
            catch(const OperationFailure&) { throw; }
            catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
            catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"unknown embedded CAD query exception"}}; }
        };
        auto finiteUv = [](Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); };
        auto finitePoint = [](Point3D p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); };
        auto nodeAt = [&](std::uint64_t id) -> const BoundaryNode& {
            if(!id || id > physical.nodes.size()) throw OperationFailure{{MeshingStatus::InvalidTopology,"unknown embedded physical node"}};
            const auto& node = physical.nodes[id-1];
            if(!node.alive || node.id != id || (node.dimension != 0 && node.dimension != 1) || node.entity < 0 ||
               !finitePoint(node.point) || (node.dimension == 1 && !std::isfinite(node.parameter)))
                throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid embedded physical classification/coordinates"}};
            return node;
        };
        check();
        PeriodicEmbeddedChart chart; chart.maximumLocalId = boundary.maximumLocalId;
        if(boundary.route == FaceChartRoute::SkipInsufficientNodes || boundary.route == FaceChartRoute::DirectTriangle) {
            output = std::move(chart); return result;
        }
        if(boundary.route != FaceChartRoute::General || boundary.maximumLocalId < 0 ||
           !std::isfinite(input.modelCharacteristicLength) || input.modelCharacteristicLength <= 0.)
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid periodic embedded route/model length"}};
        std::map<int,const PeriodicBoundaryVertex*> byId;
        for(const auto& vertex : boundary.vertices) {
            nodeAt(vertex.vertex.physicalId);
            if(vertex.vertex.localId < 0 || vertex.vertex.localId > boundary.maximumLocalId || !finiteUv(vertex.vertex.uv) ||
               !byId.emplace(vertex.vertex.localId,&vertex).second)
                throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid boundary BDS identity/UV"}};
        }
        std::vector<const PeriodicBoundaryVertex*> pointerOrder;
        if(input.boundaryCopyOrder.empty()) {
            for(const auto& entry : byId) pointerOrder.push_back(entry.second);
            std::sort(pointerOrder.begin(),pointerOrder.end(),std::less<const PeriodicBoundaryVertex*>{});
        } else {
            if(input.boundaryCopyOrder.size() != byId.size())
                throw OperationFailure{{MeshingStatus::InvalidInput,"boundary copy order must cover all BDS points"}};
            std::set<int> seen;
            for(int id : input.boundaryCopyOrder) {
                const auto found = byId.find(id);
                if(found == byId.end() || !seen.insert(id).second)
                    throw OperationFailure{{MeshingStatus::InvalidInput,"invalid boundary copy pointer order"}};
                pointerOrder.push_back(found->second);
            }
        }
        // 与原版一样，仅边界参与此 inverse map；后面的嵌入点不写回。
        std::map<std::uint64_t,std::vector<const PeriodicBoundaryVertex*>> inverse;
        for(const auto* vertex : pointerOrder) inverse[vertex->vertex.physicalId].push_back(vertex);
        std::set<std::uint64_t> absent;
        for(const auto& curve : input.curves) {
            if(curve.entity < 0) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid embedded curve identity"}};
            for(const auto& line : curve.lines) for(auto id : {line.first,line.second}) {
                nodeAt(id); if(inverse.find(id) == inverse.end()) absent.insert(id);
            }
        }
        chart.expectedInitialPointCount = input.vertices.size()+absent.size();
        auto create = [&](const BoundaryNode& node, int dimension, int entity, int sizeEntity) {
            check();
            if(boundary.vertices.size() >= control.maximumNodes ||
               chart.vertices.size() >= control.maximumNodes-boundary.vertices.size() || chart.maximumLocalId == std::numeric_limits<int>::max())
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic embedded node/ID budget exceeded"}};
            if(!input.closestSurfaceCoordinates || !input.surfacePoint)
                throw OperationFailure{{MeshingStatus::InvalidInput,"embedded chart requires native support projection and surface point"}};
            const auto uv = query([&] { return input.closestSurfaceCoordinates(node.point); });
            if(!finiteUv(uv)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite embedded projection UV"}};
            const auto point = query([&] { return input.surfacePoint(uv); });
            if(!finitePoint(point)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite embedded surface point"}};
            const MeshSizeQuery sizeQuery{node.dimension,node.dimension == 0 ? node.entity : sizeEntity,
                node.dimension == 0 ? 0. : node.parameter,0.,node.point};
            const auto size = query([&] { return evaluateMeshSize(sizeQuery,input.modelCharacteristicLength,options,sizes); });
            if(!size.status.complete()) throw OperationFailure{size.status};
            if(size.usedInvalidSizeFallback) ++chart.invalidSizeFallbacks;
            const int id = ++chart.maximumLocalId;
            chart.vertices.push_back({{id,node.id,dimension,entity,uv,point},{size.size,size.size},-1});
            Point2D initial = uv;
            if(input.perturbInitialPoint) {
                try { initial = input.perturbInitialPoint(uv); }
                catch(const OperationFailure&) { throw; }
                catch(...) { throw OperationFailure{{MeshingStatus::InvalidInput,"initial point perturbation failed"}}; }
            }
            if(!finiteUv(initial)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite initial point perturbation"}};
            chart.initialPoints.push_back({id,initial}); return id;
        };
        for(auto id : input.vertices) {
            const auto& node = nodeAt(id);
            if(node.dimension != 0) throw OperationFailure{{MeshingStatus::InvalidTopology,"embedded CAD vertex must be zero-dimensional"}};
            create(node,0,-node.entity,node.entity);
        }
        std::set<std::uint64_t> created;
        std::map<std::uint64_t,int> facile;
        for(const auto& curve : input.curves) {
            check();
            // 原版在读取线段前登记，端点全部复用或空边也不能丢掉分类。
            chart.geometry.push_back({-curve.entity,1});
            for(const auto& line : curve.lines) for(auto id : {line.first,line.second}) {
                check(); const auto& node = nodeAt(id); int selected = -1;
                const auto found = inverse.find(id);
                if(found != inverse.end()) {
                    if(found->second.size() == 1) selected = found->second.front()->vertex.localId;
                    else {
                        if(!curve.point || !curve.onFace || !std::isfinite(curve.first) || !std::isfinite(curve.last))
                            throw OperationFailure{{MeshingStatus::InvalidInput,"multi-copy endpoint requires native curve point/pcurve range"}};
                        const auto first = query([&] { return curve.point(curve.first); });
                        const auto last = query([&] { return curve.point(curve.last); });
                        if(!finitePoint(first) || !finitePoint(last)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite embedded curve endpoint"}};
                        auto distance = [&](Point3D p) {
                            const double x = node.point.x-p.x,y = node.point.y-p.y,z = node.point.z-p.z;
                            return std::sqrt(x*x+y*y+z*z);
                        };
                        const double parameter = distance(first) < distance(last) ? curve.first : curve.last;
                        const auto uv = query([&] { return curve.onFace(parameter); });
                        if(!finiteUv(uv)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite embedded endpoint pcurve"}};
                        for(const auto* copy : found->second) {
                            const double x = uv.x-copy->vertex.uv.x,y = uv.y-copy->vertex.uv.y;
                            if(std::sqrt(x*x+y*y) < 1.e-10) { selected = copy->vertex.localId; break; }
                        }
                        if(selected < 0) chart.seamSelectionFailures.push_back({curve.entity,id});
                    }
                    // 原版每次覆盖，包括找不到 seam 副本时的 nullptr。
                    facile[id] = selected;
                }
                if(selected < 0 && created.find(id) == created.end()) {
                    created.insert(id); facile[id] = create(node,1,-curve.entity,curve.entity);
                }
            }
            for(const auto& line : curve.lines) {
                const auto a = facile.find(line.first),b = facile.find(line.second);
                if(a != facile.end() && b != facile.end() && a->second >= 0 && b->second >= 0)
                    chart.constraints.push_back({curve.entity,a->second,b->second,true});
            }
        }
        chart.initialPointCountMismatch = chart.vertices.size() != chart.expectedInitialPointCount;
        check(); output = std::move(chart);
    }
    catch(const OperationFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    catch(...) { result.status = {MeshingStatus::InvalidTopology,"unknown periodic embedded chart failure"}; }
    return result;
}

/// 拼接周期面各边界环并建立图表约束。
FaceChartBuildResult buildPeriodicBoundaryChart(const BoundaryMesh& boundary,
    const PeriodicFaceBoundaryInput& input, const SurfaceMeshingOptions& options,
    const GeometrySizeSource& sizes, PeriodicBoundaryChart& output, const RunControl& control)
{
    FaceChartBuildResult result;
    try {
        auto check = [&] {
            if(control.keepGoing && !control.keepGoing())
                throw OperationFailure{{MeshingStatus::Cancelled,"periodic boundary chart cancelled"}};
        };
        auto query = [&](auto&& callback) {
            check();
            if(result.geometryEvaluations >= control.maximumGeometryEvaluations)
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic chart geometry-query budget exceeded"}};
            ++result.geometryEvaluations;
            try { return callback(); }
            catch(const OperationFailure&) { throw; }
            catch(const std::exception& error) {
                throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}};
            }
            catch(...) {
                throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"unknown periodic chart query exception"}};
            }
        };
        auto finiteUv = [](Point2D uv) { return std::isfinite(uv.x) && std::isfinite(uv.y); };
        check();
        if(input.entity < 0 || !finiteUv(input.parameterLower) || !finiteUv(input.parameterUpper) ||
           !std::isfinite(input.modelCharacteristicLength) || input.modelCharacteristicLength <= 0.)
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid periodic face identity or bounds/model length"}};
        const double du = input.parameterUpper.x-input.parameterLower.x;
        const double dv = input.parameterUpper.y-input.parameterLower.y;
        const double scale = std::sqrt(du*du+dv*dv);
        if(!std::isfinite(scale) || scale <= 0.)
            throw OperationFailure{{MeshingStatus::InvalidInput,"periodic face UV scale must be finite and positive"}};
        auto nodeAt = [&](std::uint64_t id) -> const BoundaryNode& {
            if(!id || id > boundary.nodes.size())
                throw OperationFailure{{MeshingStatus::InvalidTopology,"unknown periodic physical node"}};
            const auto& node = boundary.nodes[id-1];
            if(node.id != id || !node.alive || (node.dimension != 0 && node.dimension != 1) || node.entity < 0 ||
               !std::isfinite(node.point.x) || !std::isfinite(node.point.y) || !std::isfinite(node.point.z) ||
               (node.dimension == 1 && !std::isfinite(node.parameter)))
                throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid periodic boundary node/classification"}};
            return node;
        };
        auto distance = [](Point2D a, Point2D b) {
            const double x = a.x-b.x, y = a.y-b.y;
            return std::sqrt(x*x+y*y);
        };
        PeriodicBoundaryChart chart; chart.entity = input.entity;
        const double factors[] = {1.e-12,1.e-9,1.e-6,1.e-3,1.e-2,1.e-1};
        for(const auto& curves : input.edgeLoops) {
            check();
            PeriodicBoundaryLoop loop; PeriodicLoopSelection selection;
            bool selected = false;
            for(double factor : factors) {
                const double tolerance = factor*scale;
                for(bool seamFirst : {false,true}) {
                    const auto status = assemblePeriodicBoundaryLoop(curves,tolerance,seamFirst,loop,control);
                    if(status.complete()) {
                        selection = {tolerance,seamFirst,loop.initialDirection}; selected = true; break;
                    }
                    if(status.code != MeshingStatus::NonConverged) throw OperationFailure{status};
                }
                if(selected) break;
            }
            if(!selected)
                throw OperationFailure{{MeshingStatus::NonConverged,"periodic 1D mesh does not form a closed loop"}};
            if(loop.vertices.size() > control.maximumNodes-chart.loopPositionCount ||
               loop.vertices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())-chart.loopPositionCount)
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic chart loop-position/ID budget exceeded"}};
            const auto priorCount = chart.vertices.size();
            std::vector<int> ids;
            for(std::size_t position = 0; position < loop.vertices.size(); ++position) {
                check();
                const auto& vertex = loop.vertices[position];
                const auto& node = nodeAt(vertex.physicalId);
                int localId = -1;
                // 只搜索此前完成的环，不能提前复用 recoverMapLocal 的当前环点。
                if(node.dimension == 0) {
                    for(std::size_t previous = 0; previous < priorCount; ++previous) {
                        const auto& existing = chart.vertices[previous].vertex;
                        if(existing.physicalId == vertex.physicalId && distance(existing.uv,vertex.uv) < selection.tolerance) {
                            localId = existing.localId; break;
                        }
                    }
                }
                if(localId < 0) {
                    if(!input.surfacePoint)
                        throw OperationFailure{{MeshingStatus::InvalidInput,"periodic chart requires exact surface point evaluator"}};
                    const auto point = query([&] { return input.surfacePoint(vertex.uv); });
                    if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                        throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite periodic surface point"}};
                    MeshSizeQuery sizeQuery{node.dimension,node.entity,node.dimension == 1 ? node.parameter : 0.,0.,node.point};
                    const auto size = query([&] {
                        return evaluateMeshSize(sizeQuery,input.modelCharacteristicLength,options,sizes);
                    });
                    if(!size.status.complete()) throw OperationFailure{size.status};
                    if(size.usedInvalidSizeFallback) ++chart.invalidSizeFallbacks;
                    localId = static_cast<int>(chart.loopPositionCount+position);
                    chart.vertices.push_back({{localId,node.id,node.dimension,node.entity,vertex.uv,point},
                        {size.size,size.size},-1});
                    chart.maximumLocalId = std::max(chart.maximumLocalId,localId);
                    if(!chart.hasBounds) { chart.lower = chart.upper = vertex.uv; chart.hasBounds = true; }
                    else {
                        chart.lower.x = std::min(chart.lower.x,vertex.uv.x); chart.lower.y = std::min(chart.lower.y,vertex.uv.y);
                        chart.upper.x = std::max(chart.upper.x,vertex.uv.x); chart.upper.y = std::max(chart.upper.y,vertex.uv.y);
                    }
                }
                ids.push_back(localId);
            }
            chart.loopPositionCount += loop.vertices.size();
            chart.loops.push_back(std::move(ids)); chart.selections.push_back(selection);
        }
        std::map<std::uint64_t,std::size_t> previousCopy;
        for(std::size_t index = 0; index < chart.vertices.size(); ++index) {
            auto& current = chart.vertices[index];
            const auto found = previousCopy.find(current.vertex.physicalId);
            if(found != previousCopy.end()) {
                auto& previous = chart.vertices[found->second];
                current.periodicCounterpart = previous.vertex.localId;
                previous.periodicCounterpart = current.vertex.localId;
            }
            previousCopy[current.vertex.physicalId] = index;
        }
        chart.route = chart.loopPositionCount < 3 ? FaceChartRoute::SkipInsufficientNodes :
                      chart.loopPositionCount == 3 ? FaceChartRoute::DirectTriangle : FaceChartRoute::General;
        result.route = chart.route;
        check(); output = std::move(chart);
    }
    catch(const OperationFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    catch(...) { result.status = {MeshingStatus::GeometryQueryFailed,"unknown periodic boundary chart failure"}; }
    return result;
}
/// 采样周期面的真实参数边界，供域检查使用。
BoundaryOperationResult sampleTrueBoundary(const std::vector<TrueBoundaryCurveInput>& orderedCurves,
    std::vector<TrueBoundarySegment>& output, const RunControl& control)
{
    BoundaryOperationResult result;
    try {
        auto check = [&] {
            if(control.keepGoing && !control.keepGoing())
                throw OperationFailure{{MeshingStatus::Cancelled,"true boundary sampling cancelled"}};
        };
        check();
        std::set<int> entities;
        std::size_t samples = 0;
        for(const auto& curve : orderedCurves) {
            if(curve.entity < 0 || !entities.insert(curve.entity).second || !curve.onFace ||
               !std::isfinite(curve.first) || !std::isfinite(curve.last))
                throw OperationFailure{{MeshingStatus::InvalidInput,"invalid or duplicate true boundary curve/capability"}};
            const std::size_t count = curve.seam ? 600 : 300;
            if(count > control.maximumNodes-samples)
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"true boundary sampled-node budget exceeded"}};
            samples += count;
        }
        std::vector<TrueBoundarySegment> segments;
        for(const auto& curve : orderedCurves) {
            for(int branch = 0; branch < (curve.seam ? 2 : 1); ++branch) {
                Point2D previous;
                for(int k = 0; k < 300; ++k) {
                    check();
                    if(result.geometryEvaluations >= control.maximumGeometryEvaluations)
                        throw OperationFailure{{MeshingStatus::BudgetExceeded,"true boundary pcurve-query budget exceeded"}};
                    ++result.geometryEvaluations;
                    const double t = double(k)/299.;
                    const double parameter = curve.first+(curve.last-curve.first)*t;
                    Point2D point;
                    try { point = curve.onFace(parameter,branch); }
                    catch(const OperationFailure&) { throw; }
                    catch(const std::exception& error) {
                        throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}};
                    }
                    catch(...) {
                        throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"unknown true boundary pcurve exception"}};
                    }
                    if(!std::isfinite(point.x) || !std::isfinite(point.y))
                        throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"nonfinite true boundary UV query"}};
                    if(k) segments.push_back({curve.entity,branch,previous,point});
                    previous = point;
                }
            }
        }
        check(); output = std::move(segments);
    }
    catch(const OperationFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::InvalidInput,error.what()}; }
    catch(...) { result.status = {MeshingStatus::GeometryQueryFailed,"unknown true boundary sampling failure"}; }
    return result;
}
} // namespace mesher::gmshReference
