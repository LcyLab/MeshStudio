/** @file SurfaceGenerator.cpp
 * @brief 普通/周期CAD输入到最终网格编排；重用已逐值验证的算法模块。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "SurfaceGenerator.h"
#include "MeshAssembly.h"
#include "mesh/surface/algorithm6/initial/ConstraintRecovery.h"
#include "mesh/surface/algorithm6/initial/DomainClassification.h"
#include "mesh/surface/algorithm6/initial/InitialUvLegalization.h"
#include "mesh/surface/algorithm6/bds/BdsMeshConversion.h"
#include "mesh/surface/algorithm6/bds/BdsCurveRecovery.h"
#include "mesh/surface/algorithm6/bds/BdsDomainClassification.h"
#include "mesh/surface/algorithm6/bds/BdsNodeSizeInitialization.h"
#include "mesh/surface/algorithm6/bds/BdsInteriorPreparation.h"
#include "mesh/surface/algorithm6/postprocess/FaceMeshConversion.h"
#include "mesh/surface/algorithm6/front/FrontTriangleInitialization.h"
#include "mesh/surface/algorithm6/postprocess/FrontMeshTransfer.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>

namespace mesher::gmshReference {
/// 编排周期面的图表、约束恢复、前沿生成和网格转移。
Algorithm6FaceResult generatePeriodicFaceMesh(const BoundaryMesh&, const Algorithm6FaceInput&,
                                              double, const GeometrySizeSource&,
                                              const SurfaceMeshingOptions&,
                                              PeriodicInitialPerturbation&, const RunControl&,
                                              const SurfaceStageObserver&);

/// 按节点身份和原 UV 分支生成最终逐角 UV 坐标。
std::size_t finalizeFaceMeshCoordinates(FaceMesh& mesh, const Algorithm6FaceInput& input,
                                        const RunControl& control) {
    // 最终逐角 UV 单独生成；缓存包含原 UV 分支，不能把周期缝两侧压成同一参数。
    std::map<std::tuple<std::uint64_t, double, double>, Point2D> cache;
    std::vector<std::array<Point2D, 3>> corners;
    corners.reserve(mesh.triangles.size());
    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        std::array<Point2D, 3> uv;
        for (int k = 0; k < 3; ++k) {
            if (control.keepGoing && !control.keepGoing())
                throw OperationFailure{{MeshingStatus::Cancelled, "final corner UV cancelled"}};
            const auto id = mesh.triangles[i][k];
            const auto& v = mesh.vertices.at(id);
            uv[k] = v.dimension == 2                ? v.parameters
                    : mesh.triangleCornerUV.empty() ? mesh.parametricCoordinates.at(id)
                                                    : mesh.triangleCornerUV.at(i)[k];
            if (v.dimension <= 1 && input.finalBoundaryUV) {
                const auto key = std::make_tuple(id, uv[k].x, uv[k].y);
                const auto found = cache.find(key);
                if (found != cache.end())
                    uv[k] = found->second;
                else {
                    if (cache.size() >= control.maximumGeometryEvaluations)
                        throw OperationFailure{
                            {MeshingStatus::BudgetExceeded, "final corner UV query budget"}};
                    uv[k] = input.finalBoundaryUV(v.point, uv[k]);
                    cache.emplace(key, uv[k]);
                }
            }
        }
        corners.push_back(uv);
    }
    mesh.triangleCornerUV = std::move(corners);
    return cache.size();
}

namespace {
/// 检查操作状态或前置条件，失败时抛出本模块的明确错误。
void require(OperationStatus status) {
    if (!status.complete())
        throw OperationFailure{std::move(status)};
}

/// 检查运行取消状态，取消时停止当前阶段。
void check(const RunControl& control) {
    if (control.keepGoing && !control.keepGoing())
        throw OperationFailure{{MeshingStatus::Cancelled, "surface generation cancelled"}};
}

/// 从运行控制中扣除已用几何查询预算。
RunControl remaining(const RunControl& control, std::size_t used) {
    auto result = control;
    result.maximumGeometryEvaluations =
        used >= control.maximumGeometryEvaluations ? 0 : control.maximumGeometryEvaluations - used;
    return result;
}

/// 由面的边使用关系组织 BDS 约束恢复曲线。
std::vector<BdsRecoveryCurve> recoveryCurves(const BoundaryMesh& boundary,
                                             const std::vector<FaceCurveUse>& uses) {
    std::vector<BdsRecoveryCurve> result;
    for (const auto& use : uses) {
        auto i = std::find_if(boundary.curves.begin(), boundary.curves.end(),
                              [&](const auto& c) { return c.entity == use.entity; });
        if (i == boundary.curves.end())
            throw OperationFailure{{MeshingStatus::InvalidTopology, "unknown face CAD curve"}};
        BdsRecoveryCurve c;
        c.entity = i->entity;
        c.meshDegenerated = i->meshDegenerate;
        c.endpoints = std::array<std::uint64_t, 2>{i->beginNode, i->endNode};
        for (std::size_t k = 1; k < i->samples.size(); ++k)
            c.segments.push_back({i->samples[k - 1].node, i->samples[k].node});
        result.push_back(std::move(c));
    }
    return result;
}

/// 将有效边界节点转换为保留物理身份的面节点。
std::vector<FaceVertex> originalVertices(const BoundaryMesh& boundary) {
    std::vector<FaceVertex> result;
    for (const auto& n : boundary.nodes)
        if (n.alive) {
            FaceVertex v;
            v.id = v.physicalId = n.id;
            v.point = n.point;
            v.dimension = n.dimension;
            v.entity = n.entity;
            v.parameters = {n.parameter, 0};
            v.storedSize = n.storedSize;
            result.push_back(v);
        }
    return result;
}
}

/// 根据图表类型编排普通或周期面的完整生成流程。
static Algorithm6FaceResult
generateFaceMeshWithRandom(const BoundaryMesh& boundary, const Algorithm6FaceInput& input,
                           double modelLength, const GeometrySizeSource& sizes,
                           const SurfaceMeshingOptions& options,
                           PeriodicInitialPerturbation& random, const RunControl& control,
                           const SurfaceStageObserver& observer) {
    if (input.periodicChart)
        return generatePeriodicFaceMesh(boundary, input, modelLength, sizes, options, random,
                                        control, observer);
    Algorithm6FaceResult result;
    auto stage = [&](const char* name) {
        result.failedStage = name;
        check(control);
    };
    auto limits = [&] { return remaining(control, result.statistics.geometryEvaluations); };
    auto spend = [&](std::size_t n) {
        result.statistics.geometryEvaluations += n;
        if (result.statistics.geometryEvaluations > control.maximumGeometryEvaluations)
            throw OperationFailure{{MeshingStatus::BudgetExceeded, "cumulative face query budget"}};
    };
    auto observe = [&](const char* name, const FaceMesh& mesh) {
        if (observer)
            observer(name, input.chart.entity, boundary, &mesh);
    };
    try {
        // 1. 在共享边界上建立面内图表，按拓扑选择普通面或交由周期入口处理。
        stage("chart");
        FaceChart chart;
        const auto chartResult = buildFaceChart(boundary, input.chart, chart, limits());
        spend(chartResult.geometryEvaluations);
        require(chartResult.status);
        if (chart.route == FaceChartRoute::PeriodicRequired)
            throw OperationFailure{
                {MeshingStatus::NonConverged, "periodic CAD face requires a periodic chart input"}};
        const auto originals = originalVertices(boundary);
        FaceMesh mesh;
        mesh.entity = input.chart.entity;
        if (chart.route == FaceChartRoute::SkipInsufficientNodes)
            throw OperationFailure{
                {MeshingStatus::InvalidTopology, "face has fewer than three boundary nodes"}};
        if (chart.route == FaceChartRoute::DirectTriangle) {
            // 只有三个边界节点时直接保留它们；仍须反求参数并生成最终逐角 UV。
            for (const auto& p : chart.vertices) {
                const auto& n = boundary.nodes.at(p.physicalId - 1);
                FaceVertex v;
                v.id = v.physicalId = n.id;
                v.point = n.point;
                v.dimension = n.dimension;
                v.entity = n.entity;
                v.parameters = {n.parameter, 0};
                v.storedSize = n.storedSize;
                mesh.vertices.emplace(v.id, v);
                mesh.maximumVertexId = std::max(mesh.maximumVertexId, v.id);
                check(control);
                if (!limits().maximumGeometryEvaluations)
                    throw OperationFailure{
                        {MeshingStatus::BudgetExceeded, "direct face UV budget"}};
                mesh.parametricCoordinates[v.id] = input.reparameterize(v);
                spend(1);
            }
            mesh.triangles.push_back({chart.vertices[0].physicalId, chart.vertices[1].physicalId,
                                      chart.vertices[2].physicalId});
            observe("initial", mesh);
            observe("pre_smoothing", mesh);
            stage("final_coordinates");
            spend(finalizeFaceMeshCoordinates(mesh, input, limits()));
            observe("final", mesh);
            result.mesh = std::move(mesh);
            result.failedStage.clear();
            return result;
        }
        // 2. 初始剖分先恢复约束，再分类域和合法化；孔洞/域外片不能直接进入前沿。
        stage("initial");
        OrdinaryInitialTriangulation initial;
        require(buildInitialTriangulation(chart, initial, limits()));
        std::vector<InitialConstraint> constraints;
        require(prepareInitialConstraints(chart, initial.mesh, constraints, limits()));
        WalkTriangulation recovered;
        const auto recoveredResult =
            recoverConstraintEdges(initial.mesh, constraints, recovered, limits());
        require(recoveredResult.status);
        WalkTriangulation classified;
        require(classifyFaceDomain(recovered, chart.entity, classified, limits()).status);
        WalkTriangulation legalized;
        require(legalizeInitialUvMesh(classified, chart.entity, legalized, limits()).status);
        // 3. 转为 BDS 运行拓扑，按物理边界身份再次恢复约束，随后处理域和嵌入边。
        stage("bds_conversion");
        BdsMesh bds;
        const auto conversion = convertOrdinaryInitialMeshToBds(
            chart, legalized, input.chart.surfacePoint, bds, limits());
        spend(conversion.geometryEvaluations);
        require(conversion.status);
        if (!conversion.skippedTriangles.empty())
            throw OperationFailure{
                {MeshingStatus::InvalidTopology, "initial conversion skipped triangles"}};
        const auto curves = recoveryCurves(boundary, input.chart.boundaryCurves),
                   embedded = recoveryCurves(boundary, input.chart.embeddedCurves);
        std::vector<BdsRecoveryConstraint> registry;
        require(buildOrdinaryBdsRecoveryRegistry(bds, curves, embedded, registry, limits()));
        stage("boundary_recovery");
        BdsMesh boundaryRecovered;
        const auto recovery =
            recoverOrdinaryBdsBoundary(bds, curves, registry, boundaryRecovered, false, limits());
        require(recovery.status);
        if (!recovery.skippedSegments.empty())
            throw OperationFailure{
                {MeshingStatus::InvalidTopology, "boundary recovery skipped physical segment"}};
        if (recovery.nativeFatal || recovery.needsRefinement) {
            // 单面不能私自更改共享边；把未恢复段交回模型入口细分并重建关联面。
            for (const auto& edge : recovery.notRecovered)
                result.unrecovered.push_back({edge.curveEntity,
                                              bds.points.at(edge.first).physicalId,
                                              bds.points.at(edge.second).physicalId});
            throw OperationFailure{
                {MeshingStatus::NonConverged, "boundary recovery requires CAD edge refinement"}};
        }
        stage("bds_domain");
        require(classifyOrdinaryBdsDomain(boundaryRecovered, bds, limits()).status);
        BdsMesh embeddedRecovered;
        const auto embeddedResult =
            recoverOrdinaryBdsEmbedded(bds, embedded, registry, embeddedRecovered, limits());
        require(embeddedResult.status);
        if (embeddedResult.hadFatalRecovery || !embeddedResult.notRecovered.empty() ||
            !embeddedResult.skippedSegments.empty())
            throw OperationFailure{
                {MeshingStatus::NonConverged, "embedded constraint recovery incomplete"}};
        // 4. 初始化 BDS 尺寸和度量合法化，再转换为后续前沿使用的 FaceMesh。
        stage("bds_sizes");
        BdsRecoveredVertexQueries queries;
        for (const auto& p : embeddedRecovered.points)
            if (p.second.physicalId) {
                const auto& n = boundary.nodes.at(p.second.physicalId - 1);
                queries.emplace(p.first,
                                MeshSizeQuery{n.dimension, n.entity, n.parameter, 0, n.point});
            }
        BdsMesh sized;
        const auto sizedResult = initializeOrdinaryBdsNodeSizes(
            embeddedRecovered, queries, modelLength, options, sizes, sized, false, limits());
        spend(sizedResult.queries.size());
        require(sizedResult.status);
        stage("initial_legalization");
        BdsMesh prepared;
        require(prepareOrdinaryBdsInterior(sized, prepared, limits()).status);
        const auto metricResult = legalizeInitialMesh(prepared, input.initialMetric, bds, limits());
        spend(metricResult.metricQueries);
        require(metricResult.status);
        require(initializeFaceMeshVertices(bds, chart.entity, originals, mesh, limits()));
        FaceMesh converted;
        require(convertBdsToFaceMesh(bds, mesh, converted, limits()).status);
        mesh = std::move(converted);
        // 5. 为原节点准备尺寸及面参数；边界尺寸和背景尺寸分别保留。
        stage("front_sizes");
        NodeSizeInitializationInput nodeInput;
        for (const auto& v : mesh.vertices)
            nodeInput.vertexOrder.push_back(v.first);
        for (const auto& c : curves)
            nodeInput.boundaryCurves.push_back({c.meshDegenerated, c.segments});
        for (const auto& c : embedded)
            nodeInput.embeddedCurves.push_back({c.meshDegenerated, c.segments});
        // BREP没有独立点尺寸属性；未指定点尺寸与原生MAX_LC一致，不引入负尺寸。
        for (auto id : input.chart.embeddedVertices)
            nodeInput.embeddedVertices.push_back({id, 1.e22});
        NodeSizeState nodes;
        const auto nodeResult = initializeNodeSizes(mesh, nodeInput, input.reparameterize,
                                                    modelLength, options, sizes, nodes, limits());
        spend(nodeResult.reparameterizations.size() + nodeResult.embeddedQueries.size());
        require(nodeResult.status);
        for (const auto& n : nodes.nodes)
            mesh.parametricCoordinates[n.vertex] = n.parameters;
        observe("initial", mesh);
        // 6. 建立活动前沿并动态插点；队列、候选、空腔和插入模块各自承担一个步骤。
        stage("front");
        FrontMesh front;
        require(
            initializeFrontTriangles(mesh, nodes, options.extendBoundarySize != 0, front, limits())
                .status);
        FrontQueueState queue;
        require(initializeFrontQueue(front, queue, frontalRadiusLimit(), limits()));
        auto source = input.front;
        source.backgroundSize = [&](const MeshSizeQuery& q) {
            return evaluateMeshSize(q, modelLength, options, sizes);
        };
        // meshGenerator普通入口调用bowyerWatsonFrontal(gf)，只有周期入口传true_boundary。
        FrontGenerationOptions frontOptions;
        frontOptions.extendBoundarySize = options.extendBoundarySize != 0;
        const auto generated =
            generateFrontMesh(front, queue, mesh, nodes, source, frontOptions, limits());
        spend(generated.geometryEvaluations);
        require(generated.status);
        result.statistics.inserted = generated.inserted;
        // 7. 将前沿运行池转回面网格，再执行原平滑；阶段输出使用同一批节点身份。
        stage("transfer");
        require(transferFrontMesh(front, queue, mesh, nodes, {}, mesh, limits()).status);
        observe("pre_smoothing", mesh);
        stage("smoothing");
        SurfaceSmoothingOptions smoothOptions;
        smoothOptions.iterations = options.smoothingIterations;
        const auto smoothed = smoothSurfaceMesh(mesh, input.smoothing, smoothOptions, limits());
        spend(smoothed.geometryEvaluations);
        require(smoothed.status);
        // 8. 平滑完成后生成最终逐角参数；这一步不移动边界点的物理位置。
        stage("final_coordinates");
        spend(finalizeFaceMeshCoordinates(mesh, input, limits()));
        observe("final", mesh);
        result.mesh = std::move(mesh);
        result.failedStage.clear();
    } catch (const OperationFailure& f) {
        result.status = f.status;
    } catch (const std::exception& e) {
        result.status = {MeshingStatus::GeometryQueryFailed, e.what()};
    } catch (...) {
        result.status = {MeshingStatus::GeometryQueryFailed,
                         "non-standard geometry exception during " + result.failedStage};
    }
    return result;
}

/// 编排单个 CAD 面从边界图表到最终面网格的生成流程。
Algorithm6FaceResult
generateFaceMesh(const BoundaryMesh& boundary, const Algorithm6FaceInput& input, double modelLength,
                 const GeometrySizeSource& sizes, const SurfaceMeshingOptions& options,
                 const RunControl& control, const SurfaceStageObserver& observer) {
    PeriodicInitialPerturbation random;
    return generateFaceMeshWithRandom(boundary, input, modelLength, sizes, options, random, control,
                                      observer);
}

/// 维护共享边界及必要重试，生成全部面后装配完整表面。
Algorithm6SurfaceResult generateSurfaceMesh(const Algorithm6ModelInput& input,
                                            const SurfaceMeshingOptions& options,
                                            const RunControl& control,
                                            const SurfaceStageObserver& observer) {
    Algorithm6SurfaceResult result;
    // 为全部面预留状态：未运行、失败或共享边版本失效的面，都不能算完整成功。
    for (const auto& face : input.faces) {
        Algorithm6FaceSummary summary;
        summary.face = face.chart.entity;
        summary.bodyId = face.bodyId;
        result.faces.push_back(std::move(summary));
    }
    try {
        check(control);
        if (input.faces.empty() || options.switchOnFailure || options.maximumRecoveryRetries < 0 ||
            options.smoothingIterations < 0 || options.oldInitialDelaunay ||
            (options.extendBoundarySize != 0 && options.extendBoundarySize != 1) ||
            !std::isfinite(options.sizeMax) || options.sizeMax <= 0 ||
            !std::isfinite(options.initialRandomFactor) || options.initialRandomFactor < 0)
            throw OperationFailure{
                {MeshingStatus::InvalidInput,
                 "Algorithm 6 requires faces, positive size, valid perturbation, boundary extension 0/1 and disabled algorithm switch"}};
        // 固定Generator在整个串行运行开始srand一次，不能在逐面/重试中重置。
        PeriodicInitialPerturbation random;
        result.failedStage = "boundary";
        BoundaryMesh boundary;
        auto limits = [&] { return remaining(control, result.statistics.geometryEvaluations); };
        const auto built =
            buildBoundaryMesh(input.boundary, options, input.sizes, boundary, limits());
        result.statistics.geometryEvaluations += built.geometryEvaluations;
        require(built.status);
        std::vector<FaceMesh> meshes(input.faces.size());
        std::vector<bool> pending(input.faces.size(), true);
        // meshGFace局部递归与Generator的PENDING遍历是两层调度。失败面不阻止邻面
        // 细分共享边；只失效关联面，保留其它已完成面及其生成顺序。
        for (int pass = 0;; ++pass) {
            bool attempted = false;
            for (std::size_t i = 0; i < input.faces.size(); ++i) {
                if (!pending[i])
                    continue;
                attempted = true;
                const auto& face = input.faces[i];
                auto& summary = result.faces[i];
                for (int retry = 0; retry <= options.maximumRecoveryRetries; ++retry) {
                    check(control);
                    result.failedFace = face.chart.entity;
                    if (observer)
                        observer("boundary", face.chart.entity, boundary, nullptr);
                    auto generated = generateFaceMeshWithRandom(
                        boundary, face, input.boundary.modelCharacteristicLength, input.sizes,
                        options, random, limits(), observer);
                    summary.attempted = true;
                    summary.current = generated.status.complete();
                    summary.status = generated.status;
                    summary.failedStage = generated.failedStage;
                    summary.statistics = generated.statistics;
                    result.statistics.geometryEvaluations +=
                        generated.statistics.geometryEvaluations;
                    result.statistics.inserted += generated.statistics.inserted;
                    if (generated.status.complete()) {
                        meshes[i] = std::move(generated.mesh);
                        pending[i] = false;
                        break;
                    }
                    result.failedStage = generated.failedStage;
                    if (generated.unrecovered.empty())
                        throw OperationFailure{generated.status};
                    result.failedStage = "boundary_refinement";
                    // 原生到达局部重试上限的最后一次仍细分边，供后续邻面使用，
                    // 但不再递归重试当前面；模型遍历随后回访PENDING面。
                    const auto refined =
                        refineUnrecoveredEdges(input.boundary, options, input.sizes,
                                               generated.unrecovered, boundary, limits());
                    result.statistics.geometryEvaluations += refined.geometryEvaluations;
                    require(refined.status);
                    if (!refined.splitSegments || !refined.unmatchedRequests.empty())
                        throw OperationFailure{{MeshingStatus::NonConverged,
                                                "boundary retry made no complete progress"}};
                    ++result.statistics.boundaryRetries;
                    for (std::size_t j = 0; j < input.faces.size(); ++j)
                        if (std::find(refined.incidentFaces.begin(), refined.incidentFaces.end(),
                                      input.faces[j].chart.entity) != refined.incidentFaces.end()) {
                            pending[j] = true;
                            result.faces[j].current = false;
                            meshes[j] = FaceMesh{};
                        }
                }
            }
            if (std::none_of(pending.begin(), pending.end(), [](bool p) { return p; }))
                break;
            // 对应固定Generator的串行模型重访上限，禁止无限边界加密。
            if (!attempted || pass > options.maximumRecoveryRetries) {
                const auto i = std::find(pending.begin(), pending.end(), true) - pending.begin();
                result.failedFace = input.faces[i].chart.entity;
                result.failedStage = result.faces[i].failedStage;
                throw OperationFailure{{MeshingStatus::NonConverged,
                                        "model boundary recovery exhausted pending-face passes"}};
            }
        }
        result.failedFace = -1;
        // 全部面已对应最新共享边后才装配；任何阶段失败都不发布部分模型为成功网格。
        result.failedStage = "assembly";
        std::vector<AssemblyFace> assembly;
        for (std::size_t k = 0; k < meshes.size(); ++k)
            assembly.push_back({&meshes[k], input.faces[k].bodyId, input.faces[k].orientedNormal});
        SurfaceMeshResult surface;
        require(assembleSurfaceMesh(boundary, assembly, surface, limits(), &result.failedFace));
        result.statistics.geometryEvaluations += surface.mesh.triangles.size();
        check(control);
        result.surface = std::move(surface);
        result.failedStage.clear();
    } catch (const OperationFailure& f) {
        result.status = f.status;
    } catch (const std::exception& e) {
        result.status = {MeshingStatus::GeometryQueryFailed, e.what()};
    } catch (...) {
        result.status = {MeshingStatus::GeometryQueryFailed,
                         "non-standard geometry exception during " + result.failedStage};
    }
    if (!result.status.complete() && result.failedStage == "assembly" && result.failedFace >= 0) {
        const auto failed =
            std::find_if(result.faces.begin(), result.faces.end(),
                         [&](const auto& face) { return face.face == result.failedFace; });
        if (failed != result.faces.end()) {
            failed->current = false;
            failed->status = result.status;
            failed->failedStage = "assembly";
        }
    }
    return result;
}
}
