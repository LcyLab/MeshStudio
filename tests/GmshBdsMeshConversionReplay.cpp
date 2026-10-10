/** @file GmshBdsMeshConversionReplay.cpp
 * @brief BDS原位及普通前缀至完整前沿/转移重放；CAD变体独立求导/求值，前沿反求冻结。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsMeshConversion.h"
#include "mesh/surface/algorithm6/bds/BdsCurveRecovery.h"
#include "mesh/surface/algorithm6/bds/BdsDomainClassification.h"
#include "mesh/surface/algorithm6/bds/BdsNodeSizeInitialization.h"
#include "mesh/surface/algorithm6/bds/BdsInteriorPreparation.h"
#include "mesh/surface/algorithm6/bds/InitialMeshLegalization.h"
#include "mesh/surface/algorithm6/initial/ConstraintRecovery.h"
#include "mesh/surface/algorithm6/initial/DomainClassification.h"
#include "mesh/surface/algorithm6/initial/InitialUvLegalization.h"
#include "GmshBdsPoolJson.h"
#include "GmshBdsMetricJson.h"
#include "GmshFaceMeshJson.h"
#include "GmshNodeSizeJson.h"
#include "GmshFrontMeshJson.h"
#include "GmshFrontQueueJson.h"
#include "GmshFrontPointJson.h"
#include "GmshFrontGenerationJson.h"
#include "GmshFrontMeshTransferJson.h"
#include "GmshSurfaceSmoothingJson.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
#ifdef MESHSTUDIO_REFERENCE_CAD_METRIC
#include "cad/occ/OccCadGeometry.h"
#include "cad/occ/OccMeshVertexParameters.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <memory>
#endif
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
void require(OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; }
void writeDomainJson(std::ostream& out,const BdsMesh& mesh,const BdsDomainClassificationResult& result) {
    out << "{\"bds\":"; writeBdsPoolJson(out,mesh,true); out << ",\"fills\":[";
    for(std::size_t i = 0; i < result.fills.size(); ++i) {
        if(i) out << ','; const auto& f = result.fills[i];
        out << "{\"seed\":" << f.seed << ",\"classification\":[" << f.classification.tag << ',' << f.classification.dimension << ',' << int(f.classification.origin) << "],\"visits\":[";
        for(std::size_t j = 0; j < f.visits.size(); ++j) { if(j) out << ','; out << '[' << f.visits[j].first << ',' << (f.visits[j].second ? "true" : "false") << ']'; } out << "]}";
    }
    out << "],\"cleared_exterior\":[";
    for(std::size_t i = 0; i < result.clearedExteriorTriangles.size(); ++i) { if(i) out << ','; out << result.clearedExteriorTriangles[i]; }
    out << "],\"interior_triangles\":" << result.interiorTriangles << '}';
}
void writeCurveRecoveryJson(std::ostream& out,const BdsMesh& mesh,const std::vector<BdsRecoveryConstraint>& registered,const BdsBoundaryRecoveryResult& result,const BdsEmbeddedRecoveryResult* embedded = nullptr) {
    const auto& failed = embedded ? embedded->notRecovered : result.notRecovered;
    const auto& attempts = embedded ? embedded->attempts : result.attempts;
    const auto& skipped = embedded ? embedded->skippedSegments : result.skippedSegments;
    out << "{\"bds\":"; writeBdsPoolJson(out,mesh,embedded != nullptr);
    auto requests = [&](const char* name,const std::vector<BdsRecoveryConstraint>& values) {
        out << ",\"" << name << "\":[";
        for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; const auto& r = values[i]; out << '[' << r.first << ',' << r.second << ',' << r.curveEntity << ']'; } out << ']';
    };
    requests("registered",registered); requests("not_recovered",failed);
    if(embedded) out << ",\"had_fatal_recovery\":" << (embedded->hadFatalRecovery ? "true" : "false");
    else out << ",\"fatal\":" << (result.nativeFatal ? "true" : "false") << ",\"needs_refinement\":" << (result.needsRefinement ? "true" : "false") << ",\"failed_curve\":" << result.failedCurve;
    out << ",\"attempts\":[";
    for(std::size_t i = 0; i < attempts.size(); ++i) {
        if(i) out << ','; const auto& a = attempts[i];
        out << "{\"curve\":" << a.curveEntity << ",\"index\":" << a.segmentIndex << ",\"first\":" << a.first << ",\"second\":" << a.second << ",\"edge\":" << a.recovery.recoveredEdge << ",\"fatal\":" << (a.recovery.nativeFatal ? "true" : "false") << ",\"scans\":[";
        for(std::size_t j = 0; j < a.recovery.scans.size(); ++j) {
            if(j) out << ','; const auto& s = a.recovery.scans[j]; out << "{\"intersected\":[";
            for(std::size_t k = 0; k < s.intersectedEdges.size(); ++k) { if(k) out << ','; out << s.intersectedEdges[k]; }
            out << "],\"attempts\":[";
            for(std::size_t k = 0; k < s.attempts.size(); ++k) { if(k) out << ','; out << '[' << s.attempts[k].first << ',' << (s.attempts[k].second ? "true" : "false") << ']'; } out << "]}";
        }
        out << "]}";
    }
    out << "],\"skipped\":[";
    for(std::size_t i = 0; i < skipped.size(); ++i) { if(i) out << ','; out << '[' << skipped[i].first << ',' << skipped[i].second << ']'; } out << "]}";
}
}
int main(int argc,char** argv)
{
#ifdef MESHSTUDIO_REFERENCE_CAD_METRIC
    if(argc != 3 && argc != 4) return 2;
#else
    if(argc != 3) return 2;
#endif
    try {
        const int inputArgument = argc-2;
#ifdef MESHSTUDIO_REFERENCE_CAD_METRIC
        std::unique_ptr<OccCadGeometry> geometry;
        if(argc == 4) {
            BRep_Builder builder; TopoDS_Shape shape;
            if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read frozen BREP");
            geometry = std::make_unique<OccCadGeometry>(shape);
            if(!geometry->validationError().empty()) throw std::runtime_error(geometry->validationError());
        }
#endif
        std::ifstream in(argv[inputArgument]); in.imbue(std::locale::classic()); std::string mode; std::size_t count;
        if(!(in >> mode >> count) || count > 1000000) throw std::runtime_error("invalid BDS input");
        BdsMesh mesh;
        std::optional<BdsBoundaryRecoveryResult> boundaryResult; std::vector<BdsRecoveryConstraint> registry;
        std::optional<BdsDomainClassificationResult> domainResult;
        std::optional<BdsEmbeddedRecoveryResult> embeddedResult;
        std::optional<BdsNodeSizeInitializationResult> sizeResult;
        std::optional<BdsInteriorPreparationResult> interiorResult;
        std::optional<InitialMeshLegalizationResult> metricResult;
        std::optional<FaceMeshConversionResult> faceMeshResult;
        FaceMesh faceMesh;
        std::optional<NodeSizeInitializationResult> frontSizeResult;
        NodeSizeState frontSizes;
        FrontMesh frontMesh;
        int frontBoundaryExtension = 1;
        std::optional<FrontTriangleInitializationResult> frontTriangleResult;
        FrontInsertionSource frontInsertionSource; double frontInsertionLength = 0.;
        const bool frontInsertionMode = mode == "ordinary-front-insertion";
        const bool smoothingMode = mode == "ordinary-smoothing";
        const bool frontGenerationMode = smoothingMode || mode == "ordinary-front-generation";
        const bool frontCavityMode = mode == "ordinary-front-cavity";
        const bool frontPointMode = mode == "ordinary-front-points" || frontCavityMode || frontInsertionMode;
        const bool frontQueueMode = mode == "ordinary-front-queue" || frontPointMode || frontGenerationMode;
        const bool frontTriangleMode = mode == "ordinary-front-triangles" || frontQueueMode;
        const bool frontSizeMode = mode == "ordinary-front-node-sizes" || frontTriangleMode;
        const bool faceMeshMode = mode == "ordinary-face-mesh" || frontSizeMode;
        const bool metricMode = mode == "ordinary-metric" || faceMeshMode;
        if(mode == "topology") {
            InitialTriangulation initial;
            for(std::size_t i = 0; i < count; ++i) {
                PeriodicBoundaryVertex p;
                if(!(in >> p.vertex.localId >> p.vertex.physicalId >> p.vertex.uv.x >> p.vertex.uv.y >> p.vertex.surfacePoint.x >> p.vertex.surfacePoint.y >> p.vertex.surfacePoint.z
                    >> p.vertex.dimension >> p.vertex.entity >> p.sizes.meshSize >> p.sizes.backgroundSize >> p.periodicCounterpart)) throw std::runtime_error("truncated BDS points");
                initial.vertices.push_back(p);
            }
            std::size_t triangles; if(!(in >> triangles) || triangles > 1000000) throw std::runtime_error("invalid BDS triangle count");
            initial.triangles.resize(triangles);
            for(auto& t : initial.triangles) if(!(in >> t[0] >> t[1] >> t[2])) throw std::runtime_error("truncated BDS triangles");
            require(convertPeriodicInitialMeshToBds(initial,mesh).status);
        }
        else if(mode == "ordinary" || mode == "ordinary-boundary" || mode == "ordinary-domain" || mode == "ordinary-embedded" || mode == "ordinary-node-sizes" || mode == "ordinary-interior" || metricMode) {
            FaceChart chart; if(!(in >> chart.entity)) throw std::runtime_error("missing BDS face tag");
            chart.initialVertices.resize(count);
            for(auto& p : chart.initialVertices) if(!(in >> p.physicalId >> p.uv.x >> p.uv.y)) throw std::runtime_error("truncated PolyMesh inputs");
            std::size_t vertices; if(!(in >> vertices) || vertices > 1000000) throw std::runtime_error("invalid BDS chart size");
            chart.vertices.resize(vertices);
            for(auto& p : chart.vertices) if(!(in >> p.localId >> p.physicalId >> p.uv.x >> p.uv.y >> p.surfacePoint.x >> p.surfacePoint.y >> p.surfacePoint.z >> p.dimension >> p.entity)) throw std::runtime_error("truncated BDS chart");
            std::size_t constraints; if(!(in >> constraints) || constraints > 1000000) throw std::runtime_error("invalid BDS constraints");
            std::map<std::uint64_t,int> ids; for(const auto& p : chart.vertices) ids[p.physicalId] = p.localId;
            for(std::size_t i = 0; i < constraints; ++i) {
                std::uint64_t from,to; int tag;
                if(!(in >> from >> to >> tag)) throw std::runtime_error("truncated physical BDS constraints");
                chart.constraints.push_back({tag,ids.at(from),ids.at(to),false});
            }
            // 测试冻结四次 CAD 求值，不冻结外围UV；查找必须命中自研扩盒位置。
            std::map<std::pair<double,double>,Point3D> surface;
            for(int i = 0; i < 4; ++i) {
                Point2D uv; Point3D p;
                if(!(in >> uv.x >> uv.y >> p.x >> p.y >> p.z)) throw std::runtime_error("truncated outer CAD values");
                surface.emplace(std::make_pair(uv.x,uv.y),p);
            }
            OrdinaryInitialTriangulation initial; require(buildInitialTriangulation(chart,initial));
            std::vector<InitialConstraint> edges; require(prepareInitialConstraints(chart,initial.mesh,edges));
            WalkTriangulation recovered; require(recoverConstraintEdges(initial.mesh,edges,recovered).status);
            WalkTriangulation colored; require(classifyFaceDomain(recovered,chart.entity,colored).status);
            WalkTriangulation legalized; require(legalizeInitialUvMesh(colored,chart.entity,legalized).status);
            const auto result = convertOrdinaryInitialMeshToBds(chart,legalized,[&](Point2D uv) { return surface.at({uv.x,uv.y}); },mesh);
            require(result.status); if(!result.skippedTriangles.empty()) throw std::runtime_error("ordinary conversion skipped triangles");
            if(mode != "ordinary") {
                int refineAll; if(!(in >> refineAll) || (refineAll != 0 && refineAll != 1)) throw std::runtime_error("invalid refineAll flag");
                auto readCurves = [&] {
                    std::size_t size; if(!(in >> size) || size > 1000000) throw std::runtime_error("invalid curve count");
                    std::vector<BdsRecoveryCurve> curves(size);
                    for(auto& c : curves) {
                        int degenerated,hasEndpoints; std::array<std::uint64_t,2> endpoints; std::size_t segments;
                        if(!(in >> c.entity >> degenerated >> hasEndpoints >> endpoints[0] >> endpoints[1] >> segments) || segments > 1000000 ||
                           (degenerated != 0 && degenerated != 1) || (hasEndpoints != 0 && hasEndpoints != 1)) throw std::runtime_error("invalid curve metadata");
                        c.meshDegenerated = degenerated != 0; if(hasEndpoints) c.endpoints = endpoints; c.segments.resize(segments);
                        for(auto& segment : c.segments) if(!(in >> segment[0] >> segment[1])) throw std::runtime_error("truncated curve segments");
                    }
                    return curves;
                };
                const auto boundary = readCurves(), embedded = readCurves();
                require(buildOrdinaryBdsRecoveryRegistry(mesh,boundary,embedded,registry));
                BdsMesh recovered; boundaryResult = recoverOrdinaryBdsBoundary(mesh,boundary,registry,recovered,refineAll != 0);
                require(boundaryResult->status); mesh = std::move(recovered);
                if(mode == "ordinary-domain" || mode == "ordinary-embedded" || mode == "ordinary-node-sizes" || mode == "ordinary-interior" || metricMode) {
                    if(boundaryResult->nativeFatal || boundaryResult->needsRefinement) throw OperationFailure{{MeshingStatus::NonConverged,"native boundary failure/retry prevents domain classification"}};
                    BdsMesh colored; domainResult = classifyOrdinaryBdsDomain(mesh,colored); require(domainResult->status); mesh = std::move(colored);
                    if(mode == "ordinary-embedded" || mode == "ordinary-node-sizes" || mode == "ordinary-interior" || metricMode) {
                        BdsMesh embeddedMesh; embeddedResult = recoverOrdinaryBdsEmbedded(mesh,embedded,registry,embeddedMesh);
                        require(embeddedResult->status); mesh = std::move(embeddedMesh);
                    }
                    if(mode == "ordinary-node-sizes" || mode == "ordinary-interior" || metricMode) {
                        std::size_t count; double modelLength,maximum; int callbackMode,slitEdge;
                        if(!(in >> count >> modelLength >> maximum >> callbackMode >> slitEdge) || count > 1000000 || callbackMode < 0 || callbackMode > 3)
                            throw std::runtime_error("invalid node size configuration");
                        BdsRecoveredVertexQueries queries;
                        for(std::size_t i = 0; i < count; ++i) {
                            int id; MeshSizeQuery q;
                            if(!(in >> id >> q.dimension >> q.entity >> q.u >> q.v >> q.point.x >> q.point.y >> q.point.z) || !queries.emplace(id,q).second)
                                throw std::runtime_error("invalid recovered vertex size query");
                        }
                        SurfaceMeshingOptions options; options.sizeMax = maximum; options.sizeFromPoints = false;
                        GeometrySizeSource source;
                        if(callbackMode != 0) source.sizeCallback = [=](const MeshSizeQuery& q,double) {
                            if(callbackMode >= 2 && q.dimension == 1 && q.entity == slitEdge) return std::min(maximum*.18,.08);
                            return callbackMode == 1 || callbackMode == 3 ? maximum*(.25+.6/(1.+q.point.x*q.point.x+q.point.y*q.point.y+q.point.z*q.point.z)) : maximum;
                        };
                        frontInsertionLength = modelLength;
                        frontInsertionSource.backgroundSize = [=](const MeshSizeQuery& q) { return evaluateMeshSize(q,modelLength,options,source); };
                        BdsMesh sized; sizeResult = initializeOrdinaryBdsNodeSizes(mesh,queries,modelLength,options,source,sized);
                        require(sizeResult->status); mesh = std::move(sized);
                        if(mode == "ordinary-interior" || metricMode) {
                            BdsMesh prepared; interiorResult = prepareOrdinaryBdsInterior(mesh,prepared);
                            require(interiorResult->status); mesh = std::move(prepared);
                        }
                        if(metricMode) {
                            std::size_t queries;
                            if(!(in >> queries) || queries > 1000000) throw std::runtime_error("invalid metric query count");
                            std::vector<std::array<double,8>> derivatives(queries);
                            for(auto& row : derivatives) for(auto& value : row) if(!(in >> value)) throw std::runtime_error("truncated metric derivatives");
                            std::size_t next = 0;
                            InitialSurfaceMetricSource metricSource = [&](Point2D uv) {
#ifdef MESHSTUDIO_REFERENCE_CAD_METRIC
                                if(!geometry) throw std::runtime_error("metric CAD replay requires BREP input");
                                Point3D du,dv; geometry->faceSurfaceDerivatives(chart.entity-1,uv,du,dv);
                                return evaluateSurfaceMetric(du,dv);
#else
                                if(next >= derivatives.size()) throw std::runtime_error("unexpected metric query");
                                const auto& row = derivatives[next++];
                                if(uv.x != row[0] || uv.y != row[1]) throw std::runtime_error("metric query order/UV differs");
                                return evaluateSurfaceMetric({row[2],row[3],row[4]},{row[5],row[6],row[7]});
#endif
                            };
                            BdsMesh legalized; metricResult = legalizeInitialMesh(mesh,metricSource,legalized);
                            require(metricResult->status); mesh = std::move(legalized);
#ifndef MESHSTUDIO_REFERENCE_CAD_METRIC
                            if(next != derivatives.size()) throw std::runtime_error("unconsumed metric queries");
#endif
                        }
                        if(faceMeshMode) {
                            std::size_t originalCount;
                            if(!(in >> originalCount) || originalCount > 1000000) throw std::runtime_error("invalid original face vertex count");
                            std::vector<FaceVertex> originals(originalCount);
                            for(auto& v : originals) {
                                if(!(in >> v.id >> v.dimension >> v.entity >> v.point.x >> v.point.y >> v.point.z >> v.parameters.x >> v.parameters.y >> v.storedSize))
                                    throw std::runtime_error("truncated original MVertex metadata");
                                v.physicalId = v.id;
                            }
                            FaceMesh originalMesh; require(initializeFaceMeshVertices(mesh,chart.entity,originals,originalMesh));
                            // 普通入口直接转换；仅周期入口在此前重建临时缝副本。
                            faceMeshResult = convertBdsToFaceMesh(mesh,originalMesh,faceMesh);
                            require(faceMeshResult->status);
                            if(frontSizeMode) {
                                NodeSizeInitializationInput input; std::size_t count;
                                if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid front vertex order size");
                                input.vertexOrder.resize(count);
                                for(auto& id : input.vertexOrder) if(!(in >> id)) throw std::runtime_error("truncated front vertex order");
                                if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid embedded vertex size");
                                input.embeddedVertices.resize(count);
                                for(auto& v : input.embeddedVertices) if(!(in >> v.first >> v.second)) throw std::runtime_error("truncated embedded vertex metadata");
                                if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid front reparameterization count");
                                std::map<std::uint64_t,Point2D> parameters;
                                for(std::size_t i = 0; i < count; ++i) {
                                    std::uint64_t id; Point2D uv;
                                    if(!(in >> id >> uv.x >> uv.y) || !parameters.emplace(id,uv).second) throw std::runtime_error("invalid frozen front CAD query");
                                }
                                for(const auto& c : boundary) input.boundaryCurves.push_back({c.meshDegenerated,c.segments});
                                for(const auto& c : embedded) input.embeddedCurves.push_back({c.meshDegenerated,c.segments});
                                frontSizeResult = initializeNodeSizes(faceMesh,input,[&](const FaceVertex& v) { return parameters.at(v.id); },modelLength,options,source,frontSizes);
                                require(frontSizeResult->status);
                                if(frontTriangleMode) {
                                    int extension;
                                    if(!(in >> extension) || (extension != 0 && extension != 1)) throw std::runtime_error("invalid resolved front boundary extension flag");
                                    frontBoundaryExtension = extension;
                                    frontTriangleResult = initializeFrontTriangles(faceMesh,frontSizes,extension != 0,frontMesh);
                                    require(frontTriangleResult->status);
                                }
                            }
                        }
                    }
                }
            }
        }
        else throw std::runtime_error("unknown BDS replay mode");
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected BDS input tail");
        std::ofstream out(argv[inputArgument+1]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        if(frontTriangleResult) {
            out << "{\"node_sizes\":"; writeNodeSizeJson(out,frontSizes,*frontSizeResult);
            out << ",\"front_triangles\":"; writeFrontMeshJson(out,frontMesh,*frontTriangleResult);
            if(frontQueueMode) {
                FrontQueueState queue; require(initializeFrontQueue(frontMesh,queue));
                out << ",\"front_queue\":"; writeFrontQueueJson(out,frontMesh,queue);
                if(frontGenerationMode) {
#ifdef MESHSTUDIO_REFERENCE_CAD_METRIC
                    if(!geometry) throw std::runtime_error("front generation requires actual CAD");
                    FrontGenerationSource source;
                    source.parameterBounds = [&] { SurfaceParameterBounds range; if(!geometry->faceTrimParameterBounds(faceMesh.entity-1,range)) throw std::runtime_error("trimmed face range unavailable"); return std::array<Point2D,2>{{{range.uMin,range.vMin},{range.uMax,range.vMax}}}; };
                    source.metric = [&](Point2D uv) { Point3D du,dv; geometry->faceSurfaceDerivatives(faceMesh.entity-1,uv,du,dv); const auto m = evaluateSurfaceMetric(du,dv); require(m.status); return m.metric; };
                    source.surface = [&](Point2D uv) { return geometry->faceSurfacePoint(faceMesh.entity-1,uv); };
                    source.surfacePoint = [&](Point2D uv) { return FrontSurfacePoint{geometry->faceSurfacePoint(faceMesh.entity-1,uv),true}; };
                    source.backgroundSize = frontInsertionSource.backgroundSize;
                    FrontGenerationOptions options; options.extendBoundarySize = frontBoundaryExtension != 0; options.maximumPops = 20000;
                    out << ",\"front_generation\":"; writeFrontGenerationJson(out,frontMesh,queue,faceMesh,frontSizes,source,options,frontInsertionLength);
                    const auto transferred = transferFrontMesh(frontMesh,queue,faceMesh,frontSizes,{},faceMesh);
                    out << ",\"front_transfer\":"; writeFrontMeshTransferJson(out,transferred,faceMesh);
                    if(smoothingMode) {
                        const auto smoothingSource = makeOccSurfaceSmoothingSource(*geometry,faceMesh.entity-1);
                        std::vector<SmoothingVertexEvent> events;
                        const auto smoothed = smoothSurfaceMesh(faceMesh,smoothingSource,{}, {},[&](const auto& event) { events.push_back(event); });
                        out << ",\"smoothing\":"; writeSurfaceSmoothingJson(out,smoothed,events,faceMesh);
                    }
#else
                    throw std::runtime_error("front generation requires CAD-enabled target");
#endif
                }
                if(frontPointMode) {
#ifdef MESHSTUDIO_REFERENCE_CAD_METRIC
                    if(!geometry) throw std::runtime_error("front point replay requires actual CAD");
                    frontInsertionSource.surfacePoint = [&](Point2D uv) { return FrontSurfacePoint{geometry->faceSurfacePoint(faceMesh.entity-1,uv),true}; };
                    out << ",\"front_points\":";
                    writeInitialFrontPointsJson(out,faceMesh,frontSizes,frontMesh,frontBoundaryExtension != 0,[&](Point2D uv) {
                        Point3D du,dv; geometry->faceSurfaceDerivatives(faceMesh.entity-1,uv,du,dv);
                        const auto evaluated = evaluateSurfaceMetric(du,dv); require(evaluated.status); return evaluated.metric;
                    },[&](Point2D uv) { return geometry->faceSurfacePoint(faceMesh.entity-1,uv); },frontCavityMode,frontInsertionMode ? &frontInsertionSource : nullptr,frontInsertionLength);
#else
                    throw std::runtime_error("front point replay requires CAD-enabled target");
#endif
                }
            }
            out << '}';
        }
        else if(frontSizeResult) writeNodeSizeJson(out,frontSizes,*frontSizeResult);
        else if(faceMeshResult) writeFaceMeshJson(out,faceMesh,*faceMeshResult);
        else if(metricResult) writeBdsMetricJson(out,mesh,*metricResult);
        else if(interiorResult) {
            out << "{\"bds\":"; writeBdsPoolJson(out,mesh,true);
            auto list = [&](const std::vector<int>& values) { out << '['; for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; out << values[i]; } out << ']'; };
            out << ",\"unclassified_triangles\":"; list(interiorResult->unclassifiedTriangles);
            out << ",\"orphan_edges\":"; list(interiorResult->orphanEdges);
            out << ",\"removed_outer_points\":"; list(interiorResult->removedOuterPoints);
            auto mapping = [&](const char* name,const BdsPoolRemapping& m) { out << ",\"" << name << "\":{\"triangles\":"; list(m.triangles); out << ",\"edges\":"; list(m.edges); out << '}'; };
            mapping("first_cleanup",interiorResult->firstCleanup); mapping("second_cleanup",interiorResult->secondCleanup); out << '}';
        }
        else if(sizeResult) {
            out << "{\"bds\":"; writeBdsPoolJson(out,mesh,true); out << ",\"queries\":[";
            for(std::size_t i = 0; i < sizeResult->queries.size(); ++i) {
                if(i) out << ','; const auto& r = sizeResult->queries[i]; const auto& q = r.query;
                out << '[' << r.pointId << ',' << q.dimension << ',' << q.entity << ',' << q.u << ',' << q.v << ',' << q.point.x << ',' << q.point.y << ',' << q.point.z << ',' << r.evaluation.size << ']';
            }
            out << "],\"unconstrained\":[";
            for(std::size_t i = 0; i < sizeResult->unconstrainedPoints.size(); ++i) { if(i) out << ','; out << sizeResult->unconstrainedPoints[i]; } out << "]}";
        }
        else if(embeddedResult) writeCurveRecoveryJson(out,mesh,registry,*boundaryResult,&*embeddedResult);
        else if(domainResult) writeDomainJson(out,mesh,*domainResult);
        else if(boundaryResult) writeCurveRecoveryJson(out,mesh,registry,*boundaryResult); else writeBdsPoolJson(out,mesh);
        out << '\n'; if(!out) throw std::runtime_error("BDS output failed");
        return 0;
    }
#ifdef MESHSTUDIO_REFERENCE_CAD_METRIC
      catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
#endif
      catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
