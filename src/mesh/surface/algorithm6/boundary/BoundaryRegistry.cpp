// 文件职责：Algorithm 6 边界准备：登记 CAD 顶点与共享曲线节点，按恢复请求细分，并报告受影响的面。
#include "BoundaryRegistry.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace mesher::gmshReference {
namespace {
using BoundaryFailure = OperationFailure;
/// 要求子操作完整成功，否则向上返回失败。
void requireComplete(const OperationStatus& status)
{
    if(!status.complete()) throw BoundaryFailure{status};
}
/// 检查节点的坐标和相关数值是否有限。
bool finitePoint(const Point3D& p)
{ return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
/// 计算向量的欧氏长度。
double norm(Point3D p) { return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z); }

struct BoundaryWork {
    const BoundaryInput& input;
    const SurfaceMeshingOptions& options;
    const GeometrySizeSource& sizes;
    const RunControl& control;
    BoundaryMesh mesh;
    std::size_t evaluations = 0;

    /// 记录几何查询工作量并将回调异常转换为几何查询失败。
    template<class Query> auto evaluateCad(Query&& callback) {
        query();
        try { return callback(); }
        catch(const OperationFailure&) { throw; }
        catch(const std::exception& error) {
            throw BoundaryFailure{{MeshingStatus::GeometryQueryFailed,error.what()}};
        }
        catch(...) {
            throw BoundaryFailure{{MeshingStatus::GeometryQueryFailed,"unknown boundary CAD query exception"}};
        }
    }

    /// 检查运行取消状态，取消时停止当前阶段。
    void check() const {
        if(control.keepGoing && !control.keepGoing())
            throw BoundaryFailure{{MeshingStatus::Cancelled, "boundary operation cancelled"}};
    }
    /// 检查取消和几何查询预算，然后登记一次查询。
    void query() {
        check();
        if(evaluations >= control.maximumGeometryEvaluations)
            throw BoundaryFailure{{MeshingStatus::BudgetExceeded, "boundary geometry-query budget exceeded"}};
        ++evaluations;
    }
    /// 查询边曲线上的三维点并检查坐标是否有限。
    Point3D point(const BoundaryCurveInput& curve, double t) {
        const auto p = evaluateCad([&] { return curve.point(t); });
        if(!finitePoint(p))
            throw BoundaryFailure{{MeshingStatus::GeometryQueryFailed, "nonfinite boundary CAD point"}};
        return p;
    }
    /// 查询边曲线的一阶导数并检查数值是否合法。
    Point3D derivative(const BoundaryCurveInput& curve, double t) {
        const auto p = evaluateCad([&] { return curve.derivative(t); });
        if(!finitePoint(p) || !std::isfinite(norm(p)))
            throw BoundaryFailure{{MeshingStatus::GeometryQueryFailed, "nonfinite boundary CAD derivative"}};
        return p;
    }
    /// 计算边界节点尺寸并记录无效尺寸的回退统计。
    double size(int dimension, int entity, double t, Point3D p) {
        query();
        const auto value = evaluateMeshSize({dimension,entity,t,0.,p},
            input.modelCharacteristicLength, options, sizes);
        requireComplete(value.status);
        mesh.invalidSizeFallbacks += value.usedInvalidSizeFallback;
        return value.size;
    }
    /// 检查追加节点是否会超出本次运行的节点预算。
    void checkNodeBudget(std::size_t count) const {
        if(mesh.nodes.size() > control.maximumNodes || count > control.maximumNodes - mesh.nodes.size())
            throw BoundaryFailure{{MeshingStatus::BudgetExceeded, "boundary allocated-node budget exceeded"}};
    }
    /// 分配具有物理身份的边界节点并记录所属 CAD 实体。
    std::uint64_t allocate(Point3D p, int dimension, int entity, double t = 0., double lc = -1.) {
        check(); checkNodeBudget(1);
        const auto id = static_cast<std::uint64_t>(mesh.nodes.size()) + 1;
        mesh.nodes.push_back({id,p,dimension,entity,t,lc,true});
        return id;
    }
};
} // namespace

/// 独立采样 CAD 曲线，建立共享物理节点与边界链。
BoundaryOperationResult buildBoundaryMesh(const BoundaryInput& input, const SurfaceMeshingOptions& options,
    const GeometrySizeSource& sizes, BoundaryMesh& output, const RunControl& control)
{
    BoundaryWork work{input,options,sizes,control};
    try {
        work.check();
        if(!std::isfinite(input.modelCharacteristicLength) || input.modelCharacteristicLength <= 0. ||
           !std::isfinite(input.geometryTolerance) || input.geometryTolerance < 0. ||
           !std::isfinite(input.edgeLengthTolerance) || input.edgeLengthTolerance < 0. ||
           !std::isfinite(options.curveIntegrationPrecision) || options.curveIntegrationPrecision <= 0. ||
           !std::isfinite(options.curveSmoothRatio) || options.curveSmoothRatio < 1.)
            throw BoundaryFailure{{MeshingStatus::InvalidInput, "invalid boundary integration options"}};
        std::map<int,std::uint64_t> vertexNodes;
        std::map<int,std::set<int>> incidentCurves;
        std::set<int> curveEntities;
        // 原生先建全部 0D 节点（含孤立点），再按显式实体顺序生成边。
        for(const auto& vertex : input.vertices) {
            if(vertex.entity < 0 || !finitePoint(vertex.point))
                throw BoundaryFailure{{MeshingStatus::InvalidInput, "invalid boundary CAD vertex"}};
            if(vertexNodes.count(vertex.entity))
                throw BoundaryFailure{{MeshingStatus::InvalidTopology, "duplicate boundary CAD vertex identity"}};
            vertexNodes.emplace(vertex.entity, work.allocate(vertex.point,0,vertex.entity));
        }
        for(const auto& curve : input.curves) {
            if(curve.entity < 0 || !curveEntities.insert(curve.entity).second ||
               !vertexNodes.count(curve.beginVertex) || !vertexNodes.count(curve.endVertex))
                throw BoundaryFailure{{MeshingStatus::InvalidTopology, "duplicate curve or unknown CAD endpoint"}};
            if(!std::isfinite(curve.first) || !std::isfinite(curve.last) ||
               (!curve.geometryDegenerate && curve.first >= curve.last) ||
               (!curve.geometryDegenerate && (!curve.point || !curve.derivative || !curve.inverseParameter)))
                throw BoundaryFailure{{MeshingStatus::InvalidInput, "boundary requires exact CAD point, D1 and inverse parameter"}};
            for(int face : curve.incidentFaces) if(face < 0)
                throw BoundaryFailure{{MeshingStatus::InvalidTopology, "invalid incident CAD face identity"}};
            incidentCurves[curve.beginVertex].insert(curve.entity);
            incidentCurves[curve.endVertex].insert(curve.entity);
        }
        RunControl localControl; localControl.keepGoing = control.keepGoing;
        for(const auto& curve : input.curves) {
            work.check();
            BoundaryCurve chain;
            chain.entity = curve.entity;
            chain.beginNode = vertexNodes.at(curve.beginVertex);
            chain.endNode = vertexNodes.at(curve.endVertex);
            chain.incidentFaces = curve.incidentFaces;
            const bool closed = chain.beginNode == chain.endNode;
            chain.geometryDegenerate = curve.geometryDegenerate;
            if(curve.geometryDegenerate) {
                chain.meshDegenerate = closed;
                work.mesh.curves.push_back(std::move(chain));
                continue; // OCC degenerate(1)：不调用边积分/几何、不建 MLine。
            }
            const auto originalBegin = work.mesh.nodes[chain.beginNode-1].point;
            bool zeroClosedLine = false;
            if(closed && curve.kind == CurveKind::Line) {
                const auto middle = work.point(curve,.5); // 原生字面参数 .5，非 bounds 中点。
                zeroClosedLine = norm({middle.x-originalBegin.x,middle.y-originalBegin.y,
                                       middle.z-originalBegin.z}) < input.geometryTolerance;
            }
            CurveDensityTable lengthTable;
            if(!zeroClosedLine) {
                requireComplete(integrateCurveDensity(curve.first,curve.last,
                    [&](double t) { return work.derivative(curve,t); },
                    [](double) { return 1.; }, lengthTable,
                    options.curveIntegrationPrecision * input.modelCharacteristicLength, localControl));
                chain.length = lengthTable.samples.back().cumulativeDensity;
                chain.integrationDepthLimitReached = lengthTable.nativeIntegrationDepthLimitReached;
            }
            chain.tooSmall = chain.length < input.edgeLengthTolerance;
            CurveDensityTable table;
            int count = 1, minimumSegments = 0;
            if(!(chain.length == 0. && input.edgeLengthTolerance == 0.)) {
                requireComplete(integrateCurveDensity(curve.first,curve.last,
                    [&](double t) { return work.derivative(curve,t); },
                    [&](double t) {
                        const auto p = work.point(curve,t);
                        double endpointSize = 1.e22;
                        if(t == curve.first) endpointSize = work.size(0,curve.beginVertex,t,p);
                        else if(t == curve.last) endpointSize = work.size(0,curve.endVertex,t,p);
                        return std::min(endpointSize,work.size(1,curve.entity,t,p));
                    }, table,options.curveIntegrationPrecision,localControl));
                chain.integrationDepthLimitReached |= table.nativeIntegrationDepthLimitReached;
                // meshGEdgeProcessing 在积分后重新查询全部样本速度，再平滑。
                for(auto& sample : table.samples) sample.speed = norm(work.derivative(curve,sample.parameter));
                requireComplete(smoothCurveSizes(table,std::sqrt(options.curveSmoothRatio),localControl));
                chain.smoothingLimitReached = table.nativeSmoothingLimitReached;
                minimumSegments = computeMinimumCurveSegments(curve.kind,curve.last-curve.first,
                    closed,curve.prescribedMinimumSegments,options);
                count = computeCurvePointCount(table.samples.back().cumulativeDensity,minimumSegments);
            }
            const bool relocateBegin = closed && incidentCurves[curve.beginVertex].size() == 1;
            const auto relocatedBegin = relocateBegin ? work.point(curve,curve.first) : originalBegin;
            std::vector<CurveNode> nodes;
            if(count > 2) {
                work.checkNodeBudget(static_cast<std::size_t>(count-2));
                std::vector<double> parameters;
                requireComplete(distributeCurveParameters(table,count,parameters,localControl));
                std::size_t bracket = 1;
                const double spacing = table.samples.back().cumulativeDensity / double(count-1);
                for(int i = 1; i < count-1; ++i) {
                    const double cumulative = double(i)*spacing;
                    while(bracket < table.samples.size() &&
                          !(table.samples[bracket].cumulativeDensity >= cumulative &&
                            table.samples[bracket-1].cumulativeDensity < cumulative)) ++bracket;
                    if(bracket == table.samples.size())
                        throw BoundaryFailure{{MeshingStatus::GeometryQueryFailed, "boundary cumulative bracket missing"}};
                    const auto stored = computeStoredCurveSize(table.samples[bracket-1],table.samples[bracket],
                        norm(work.derivative(curve,parameters[i])));
                    requireComplete(stored.status); // 非有限 lc 仍是原生元数据，不替换为 BGM。
                    const auto p = work.point(curve,parameters[i]);
                    nodes.push_back({work.allocate(p,1,curve.entity,parameters[i],stored.size),parameters[i],p,stored.size});
                }
            }
            if(!nodes.empty()) {
                const auto filtered = filterCurveNodes(nodes,originalBegin,[&] {
                    return work.evaluateCad([&] { return curve.inverseParameter(originalBegin); });
                },minimumSegments-1,
                    [&](double t) { return work.size(1,curve.entity,t,work.point(curve,t)); },localControl);
                requireComplete(filtered.status);
                chain.minimumNodeCountPreventedRemoval = filtered.minimumNodeCountPreventedRemoval;
                for(auto id : filtered.removedNodeIds) work.mesh.nodes[id-1].alive = false;
            }
            chain.samples.push_back({chain.beginNode,curve.first});
            for(const auto& node : nodes) chain.samples.push_back({node.id,node.parameter});
            chain.samples.push_back({chain.endNode,curve.last});
            // 线段已经引用物理节点身份，闭环移动必须晚于过滤。
            if(relocateBegin) work.mesh.nodes[chain.beginNode-1].point = relocatedBegin;
            chain.meshDegenerate = chain.tooSmall || (closed && nodes.size() < 2);
            work.mesh.curves.push_back(std::move(chain));
        }
        work.check();
        output = std::move(work.mesh);
        return {{},work.evaluations};
    }
    catch(const BoundaryFailure& failure) { return {failure.status,work.evaluations}; }
    catch(const std::invalid_argument& error) { return {{MeshingStatus::InvalidInput,error.what()},work.evaluations}; }
    catch(const std::overflow_error& error) { return {{MeshingStatus::InvalidInput,error.what()},work.evaluations}; }
    catch(const std::exception& error) { return {{MeshingStatus::GeometryQueryFailed,error.what()},work.evaluations}; }
    catch(...) { return {{MeshingStatus::GeometryQueryFailed,"unknown boundary CAD query exception"},work.evaluations}; }
}

/// 查找指定边界曲线关联的面实体。
OperationStatus findIncidentFaces(const BoundaryMesh& mesh, int entity, std::vector<int>& output)
{
    const auto curve = std::find_if(mesh.curves.begin(),mesh.curves.end(),
        [&](const BoundaryCurve& value) { return value.entity == entity; });
    if(curve == mesh.curves.end()) return {MeshingStatus::InvalidInput,"unknown boundary CAD edge"};
    std::vector<int> faces;
    for(int face : curve->incidentFaces)
        if(std::find(faces.begin(),faces.end(),face) == faces.end()) faces.push_back(face);
    output = std::move(faces);
    return {};
}
/// 细分无法恢复的 CAD 边界子段并使关联面重建。
BoundaryRefinementResult refineUnrecoveredEdges(const BoundaryInput& input, const SurfaceMeshingOptions& options,
    const GeometrySizeSource& sizes, const std::vector<UnrecoveredEdge>& requests,
    BoundaryMesh& mesh, const RunControl& control)
{
    BoundaryRefinementResult result;
    BoundaryWork work{input,options,sizes,control};
    try {
        work.check();
        work.mesh = mesh;
        work.checkNodeBudget(0);
        auto nodeAt = [&](std::uint64_t id) -> const BoundaryNode& {
            if(!id || id > work.mesh.nodes.size())
                throw BoundaryFailure{{MeshingStatus::InvalidTopology,"boundary node identity outside pool"}};
            const auto& node = work.mesh.nodes[id-1];
            if(node.id != id || !node.alive || !finitePoint(node.point) ||
               (node.dimension != 0 && node.dimension != 1) || !std::isfinite(node.parameter))
                throw BoundaryFailure{{MeshingStatus::InvalidTopology,"invalid or erased boundary node identity"}};
            return node;
        };
        std::map<int,std::size_t> curveSlots;
        std::map<int,const BoundaryCurveInput*> geometryCurves;
        for(std::size_t i = 0; i < work.mesh.curves.size(); ++i)
            if(!curveSlots.emplace(work.mesh.curves[i].entity,i).second)
                throw BoundaryFailure{{MeshingStatus::InvalidTopology,"duplicate boundary curve identity"}};
        for(const auto& curve : input.curves)
            if(!geometryCurves.emplace(curve.entity,&curve).second)
                throw BoundaryFailure{{MeshingStatus::InvalidInput,"duplicate recovery CAD curve identity"}};
        for(std::size_t index = 0; index < requests.size(); ++index) {
            work.check();
            const auto& request = requests[index];
            const auto slot = curveSlots.find(request.curveEntity);
            const auto source = geometryCurves.find(request.curveEntity);
            if(slot == curveSlots.end() || source == geometryCurves.end() ||
               !request.from || request.from > work.mesh.nodes.size() ||
               !request.to || request.to > work.mesh.nodes.size())
                throw BoundaryFailure{{MeshingStatus::InvalidInput,"unknown recovery curve or physical node"}};
            auto& chain = work.mesh.curves[slot->second];
            const auto& curve = *source->second;
            if(!std::isfinite(curve.first) || !std::isfinite(curve.last) || curve.first > curve.last)
                throw BoundaryFailure{{MeshingStatus::InvalidInput,"invalid recovery CAD parameter bounds"}};
            const auto& begin = nodeAt(chain.beginNode); const auto& end = nodeAt(chain.endNode);
            if(begin.dimension != 0 || end.dimension != 0 || begin.entity != curve.beginVertex ||
               end.entity != curve.endVertex || (!chain.samples.empty() &&
               (chain.samples.size() < 2 || chain.samples.front().node != chain.beginNode ||
                chain.samples.back().node != chain.endNode)))
                throw BoundaryFailure{{MeshingStatus::InvalidTopology,"boundary endpoint classification inconsistent with CAD"}};
            std::vector<int> faces;
            requireComplete(findIncidentFaces(work.mesh,request.curveEntity,faces));
            for(int face : faces) if(std::find(result.incidentFaces.begin(),result.incidentFaces.end(),face) == result.incidentFaces.end())
                result.incidentFaces.push_back(face);
            std::vector<BoundarySample> updated;
            if(!chain.samples.empty()) updated.push_back(chain.samples.front());
            std::size_t splitCount = 0;
            for(std::size_t segment = 1; segment < chain.samples.size(); ++segment) {
                work.check();
                const auto firstId = chain.samples[segment-1].node;
                const auto lastId = chain.samples[segment].node;
                // 复制属性：allocate 会移动节点池，不能保留节点引用跨分配。
                const auto firstNode = nodeAt(firstId), lastNode = nodeAt(lastId);
                if((firstId == request.from && lastId == request.to) ||
                   (firstId == request.to && lastId == request.from)) {
                    if(!curve.point)
                        throw BoundaryFailure{{MeshingStatus::InvalidInput,"missing recovery CAD point evaluator"}};
                    auto endpointParameter = [&](const BoundaryNode& node) {
                        if(node.dimension == 0 && node.entity == curve.beginVertex) return curve.first;
                        if(node.dimension == 0 && node.entity == curve.endVertex) return curve.last;
                        if(node.dimension != 1 || node.entity != curve.entity)
                            throw BoundaryFailure{{MeshingStatus::InvalidTopology,"recovery node not classified on requested CAD curve"}};
                        return node.parameter;
                    };
                    double first = endpointParameter(firstNode), last = endpointParameter(lastNode);
                    const bool closed = curve.beginVertex == curve.endVertex;
                    if(closed && firstNode.dimension == 0)
                        first = std::abs(last-curve.first) < std::abs(last-curve.last) ? curve.first : curve.last;
                    if(closed && lastNode.dimension == 0)
                        last = std::abs(first-curve.first) < std::abs(first-curve.last) ? curve.first : curve.last;
                    double firstSize = firstNode.dimension ? firstNode.storedSize : -1.;
                    double lastSize = lastNode.dimension ? lastNode.storedSize : -1.;
                    if(firstSize == -1.) firstSize = work.size(firstNode.dimension,firstNode.entity,0.,firstNode.point);
                    if(lastSize == -1.) lastSize = work.size(lastNode.dimension,lastNode.entity,0.,lastNode.point);
                    const double parameter = .5*(last+first);
                    const double stored = .5*(firstSize+lastSize);
                    const auto point = work.point(curve,parameter);
                    const auto id = work.allocate(point,1,curve.entity,parameter,stored);
                    updated.push_back({id,parameter});
                    ++splitCount;
                }
                updated.push_back(chain.samples[segment]);
            }
            if(splitCount) {
                if(chain.version == std::numeric_limits<std::uint64_t>::max())
                    throw BoundaryFailure{{MeshingStatus::InvalidTopology,"boundary version overflow"}};
                chain.samples = std::move(updated);
                ++chain.version;
                chain.meshDegenerate = chain.tooSmall ||
                    (chain.beginNode == chain.endNode && chain.samples.size() < 4);
                result.splitSegments += splitCount;
            }
            else result.unmatchedRequests.push_back(index);
        }
        work.check();
        mesh = std::move(work.mesh);
    }
    catch(const BoundaryFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::GeometryQueryFailed,error.what()}; }
    catch(...) { result.status = {MeshingStatus::GeometryQueryFailed,"unknown boundary recovery query exception"}; }
    result.geometryEvaluations = work.evaluations;
    if(!result.status.complete()) {
        result.incidentFaces.clear(); result.unmatchedRequests.clear(); result.splitSegments = 0;
    }
    return result;
}
}
