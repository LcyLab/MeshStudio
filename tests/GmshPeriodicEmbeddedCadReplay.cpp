/** @file GmshPeriodicEmbeddedCadReplay.cpp
 * @brief 同一BREP周期图表至完整前沿/转移重放；冻结原1D元数据与前沿反求。
 * @date 2026-10-07
 * @modified 2026-10-08
 */
#include "mesh/surface/algorithm6/chart/PeriodicFaceChart.h"
#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
#include "mesh/surface/algorithm6/bds/BdsMeshConversion.h"
#include "GmshBdsPoolJson.h"
#include "GmshPeriodicBdsRecoveryJson.h"
#include "GmshBdsDomainJson.h"
#include "GmshBdsMetricJson.h"
#include "GmshFaceMeshJson.h"
#include "GmshNodeSizeJson.h"
#include "GmshFrontMeshJson.h"
#include "GmshFrontQueueJson.h"
#include "GmshFrontPointJson.h"
#include "GmshFrontGenerationJson.h"
#include "GmshFrontMeshTransferJson.h"
#include "GmshSurfaceSmoothingJson.h"
#include "cad/occ/OccMeshVertexParameters.h"
#include "mesh/surface/algorithm6/postprocess/PeriodicSeamReconstruction.h"
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
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <string>
using namespace mesher;
using namespace mesher::gmshReference;

static void readPeriodicFaceTopology(std::istream& in, OccCadGeometry& geometry,
    const TopoDS_Shape& shape, int faceTag, const BoundaryMesh& physical, PeriodicFaceChartInput& input)
{
    TopTools_IndexedMapOfShape edges, faces;
    TopExp::MapShapes(shape,TopAbs_EDGE,edges); TopExp::MapShapes(shape,TopAbs_FACE,faces);
    const auto face = TopoDS::Face(faces(faceTag));
    TopTools_IndexedMapOfShape members; TopExp::MapShapes(face,TopAbs_EDGE,members);
    auto verify = [&](int tag, int orientation, int seam, int faceOrientation) {
        if(tag < 1 || tag > edges.Extent() || orientation < 0 || orientation > 3 ||
           faceOrientation < 0 || faceOrientation > 3 || (seam != 0 && seam != 1))
            throw std::runtime_error("invalid full-chart CAD metadata");
        auto edge = TopoDS::Edge(edges(tag));
        if(edge.Orientation()==TopAbs_INTERNAL || edge.Orientation()==TopAbs_EXTERNAL) edge.Orientation(TopAbs_FORWARD);
        bool queriedSeam;
        if(!members.Contains(edge) || int(edge.Orientation()) != orientation || int(face.Orientation()) != faceOrientation ||
           !geometry.edgeSeamOnFace(faceTag-1,tag-1,queriedSeam) || queriedSeam != bool(seam))
            throw std::runtime_error("full-chart CAD identity/orientation/seam differs from native metadata");
    };
    std::size_t count;
    if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid true-boundary topology count");
    for(std::size_t i = 0; i < count; ++i) {
        int tag, orientation, seam, faceOrientation;
        if(!(in >> tag >> orientation >> seam >> faceOrientation)) throw std::runtime_error("truncated true-boundary topology");
        verify(tag,orientation,seam,faceOrientation);
        TrueBoundaryCurveInput curve; curve.entity = tag; curve.seam = bool(seam);
        std::pair<double,double> range;
        if(!geometry.edgeOnFaceRange(faceTag-1,tag-1,range)) throw std::runtime_error("true-boundary pcurve range unavailable");
        curve.first = range.first; curve.last = range.second;
        curve.onFace = [&geometry,faceTag,tag](double t, int branch) {
            Point2D uv;
            if(!geometry.edgeOnFaceCoordinates(faceTag-1,tag-1,t,branch!=1,uv))
                throw std::runtime_error("true-boundary exact pcurve unavailable; generic fallback pending");
            return uv;
        };
        input.trueBoundaryCurves.push_back(std::move(curve));
    }
    std::size_t loops;
    if(!(in >> loops) || loops > 10000) throw std::runtime_error("invalid full-chart loop count");
    for(std::size_t i = 0; i < loops; ++i) {
        if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid full-chart curve count");
        std::vector<PeriodicCurveChart> loop;
        for(std::size_t j = 0; j < count; ++j) {
            PeriodicCurveChart curve; int seam, orientation, faceOrientation; std::size_t points;
            if(!(in >> curve.entity >> seam >> orientation >> faceOrientation >> points) || points < 2 || points > 1000000)
                throw std::runtime_error("invalid full-chart curve use");
            verify(curve.entity,orientation,seam,faceOrientation); curve.seam = bool(seam);
            std::pair<double,double> range;
            if(!geometry.edgeOnFaceRange(faceTag-1,curve.entity-1,range)) throw std::runtime_error("full-chart pcurve range unavailable");
            for(std::size_t k = 0; k < points; ++k) {
                std::uint64_t id;
                if(!(in >> id) || !id || id > physical.nodes.size() || !physical.nodes[id-1].alive)
                    throw std::runtime_error("unknown full-chart physical node");
                const auto& node = physical.nodes[id-1];
                const bool endpoint = k == 0 || k+1 == points;
                if(endpoint ? node.dimension != 0 : node.dimension != 1 || node.entity != curve.entity)
                    throw std::runtime_error("wrong full-chart physical classification");
                const double t = k == 0 ? range.first : k+1 == points ? range.second : node.parameter;
                Point2D uv;
                if(!geometry.edgeOnFaceCoordinates(faceTag-1,curve.entity-1,t,false,uv))
                    throw std::runtime_error("full-chart exact pcurve unavailable; generic fallback pending");
                curve.vertices.push_back({id,uv});
                if(curve.seam) {
                    if(!geometry.edgeOnFaceCoordinates(faceTag-1,curve.entity-1,t,true,uv))
                        throw std::runtime_error("full-chart alternate seam pcurve unavailable");
                    curve.alternateCoordinates.push_back(uv);
                }
            }
            loop.push_back(std::move(curve));
        }
        input.boundary.edgeLoops.push_back(std::move(loop));
    }
}

int main(int argc, char** argv)
{
    const bool smoothing = argc == 5 && std::string(argv[4]) == "--smoothing";
    const bool frontGeneration = smoothing || (argc == 5 && std::string(argv[4]) == "--front-generation");
    const bool frontInsertion = argc == 5 && std::string(argv[4]) == "--front-insertion";
    const bool frontCavity = argc == 5 && std::string(argv[4]) == "--front-cavity";
    const bool frontPoints = frontInsertion || frontCavity || (argc == 5 && std::string(argv[4]) == "--front-points");
    const bool frontQueue = frontGeneration || frontPoints || (argc == 5 && std::string(argv[4]) == "--front-queue");
    const bool frontTriangles = frontQueue || (argc == 5 && std::string(argv[4]) == "--front-triangles");
    const bool frontNodes = frontTriangles || (argc == 5 && std::string(argv[4]) == "--front-node-sizes");
    const bool faceMesh = frontNodes || (argc == 5 && std::string(argv[4]) == "--face-mesh");
    const bool bdsMetric = faceMesh || (argc == 5 && std::string(argv[4]) == "--bds-metric");
    const bool bdsInterior = bdsMetric || (argc == 5 && std::string(argv[4]) == "--bds-interior");
    const bool bdsDomain = bdsInterior || (argc == 5 && std::string(argv[4]) == "--bds-domain");
    const bool bdsRecovery = bdsDomain || (argc == 5 && std::string(argv[4]) == "--bds-recovery");
    const bool bdsConversion = bdsRecovery || (argc == 5 && std::string(argv[4]) == "--bds-conversion");
    const bool initialTriangulation = bdsConversion || (argc == 5 && std::string(argv[4]) == "--initial-triangulation");
    const bool fullChart = initialTriangulation || (argc == 5 && std::string(argv[4]) == "--full-chart");
    if(argc != 4 && !fullChart) { std::cerr << "usage: GmshPeriodicEmbeddedCadReplay CAD.brep stage-input.tsv output.json [--full-chart|--initial-triangulation|--bds-conversion|--bds-recovery|--bds-domain|--bds-interior|--bds-metric|--face-mesh|--front-node-sizes|--front-triangles|--front-queue|--front-points|--front-cavity]\n"; return 2; }
    try {
        BRep_Builder builder; TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read exact BREP");
        OccCadGeometry geometry(shape);
        if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream in(argv[2]); in.imbue(std::locale::classic());
        PeriodicBoundaryChart boundary; PeriodicEmbeddedInput input; SurfaceMeshingOptions options;
        options.sizeFromPoints = false;
        std::size_t nodes, bds, copies, metadata; bool graded; double randomFactor; unsigned int seed;
        if(!(in >> boundary.entity >> nodes >> bds >> copies >> metadata >> boundary.maximumLocalId >> input.modelCharacteristicLength
                >> options.sizeMax >> options.sizeFactor >> graded >> randomFactor >> seed) ||
           boundary.entity < 1 || boundary.entity > geometry.faceCount() || nodes > 1000000 || bds > 1000000 || copies > 1000000 || metadata > 1000000)
            throw std::runtime_error("invalid embedded stage header");
        if(fullChart && (bds || boundary.maximumLocalId)) throw std::runtime_error("full chart must not consume frozen boundary BDS");
        BoundaryMesh physical;
        for(std::size_t i = 0; i < nodes; ++i) {
            BoundaryNode node;
            if(!(in >> node.id >> node.point.x >> node.point.y >> node.point.z >> node.dimension >> node.entity >> node.parameter) ||
               !node.id || node.id > 1000000) throw std::runtime_error("invalid frozen physical node");
            if(physical.nodes.size() < node.id) {
                const auto begin = physical.nodes.size(); physical.nodes.resize(node.id);
                for(std::size_t k = begin; k < physical.nodes.size(); ++k) physical.nodes[k].alive = false;
            }
            if(physical.nodes[node.id-1].alive) throw std::runtime_error("duplicate frozen physical node");
            physical.nodes[node.id-1] = node;
        }
        for(std::size_t i = 0; i < bds; ++i) {
            PeriodicBoundaryVertex p; auto& v = p.vertex;
            if(!(in >> v.localId >> v.physicalId >> v.uv.x >> v.uv.y >> v.surfacePoint.x >> v.surfacePoint.y >> v.surfacePoint.z
                    >> v.dimension >> v.entity >> p.sizes.backgroundSize >> p.sizes.meshSize >> p.periodicCounterpart))
                throw std::runtime_error("truncated frozen boundary BDS");
            boundary.vertices.push_back(p);
        }
        for(std::size_t i = 0; i < copies; ++i) { int id; if(!(in >> id)) throw std::runtime_error("truncated copy pointer order"); input.boundaryCopyOrder.push_back(id); }
        for(std::size_t i = 0; i < metadata; ++i) {
            int kind, entity; std::uint64_t a,b;
            if(!(in >> kind >> entity >> a >> b)) throw std::runtime_error("truncated embedded topology metadata");
            if(kind == 0) {
                if(!a || a > physical.nodes.size() || physical.nodes[a-1].entity != entity) throw std::runtime_error("wrong embedded CAD vertex identity");
                input.vertices.push_back(a);
            } else if(kind == 2) {
                if(entity < 1 || entity > geometry.edgeCount()) throw std::runtime_error("wrong embedded edge identity");
                PeriodicEmbeddedCurveInput curve; curve.entity = entity;
                std::pair<double,double> range;
                if(!geometry.edgeOnFaceRange(boundary.entity-1,entity-1,range)) throw std::runtime_error("native embedded curve range unavailable");
                curve.first = range.first; curve.last = range.second;
                curve.point = [&geometry,entity](double t) { return geometry.edgePoint(entity-1,t); };
                curve.onFace = [&geometry,entity,&boundary](double t) {
                    Point2D uv; Point3D projected;
                    if(!geometry.edgeOnFaceCoordinates(boundary.entity-1,entity-1,t,false,uv) &&
                       !geometry.projectOnFaceSurface(boundary.entity-1,geometry.edgePoint(entity-1,t),uv,projected))
                        throw std::runtime_error("native pcurve/projection unavailable; generic inverse fallback not reproduced");
                    return uv;
                };
                input.curves.push_back(std::move(curve));
            } else if(kind == 1) {
                if(input.curves.empty() || input.curves.back().entity != entity) throw std::runtime_error("wrong native embedded edge order");
                input.curves.back().lines.push_back({a,b});
            } else throw std::runtime_error("invalid embedded metadata kind");
        }
        PeriodicFaceChartInput fullInput;
        if(fullChart) readPeriodicFaceTopology(in,geometry,shape,boundary.entity,physical,fullInput);
        PeriodicBdsRecoveryInput recoveryInput;
        if(bdsRecovery) {
            int refine; std::size_t curves;
            if(!(in >> refine >> curves) || (refine != 0 && refine != 1) || curves > 1000000) throw std::runtime_error("invalid periodic recovery metadata");
            recoveryInput.refineAllEdges = refine != 0; recoveryInput.boundaryCurves.resize(curves);
            for(auto& curve : recoveryInput.boundaryCurves) {
                std::size_t segments; if(!(in >> curve.entity >> segments) || segments > 1000000) throw std::runtime_error("invalid periodic recovery curve");
                curve.segments.resize(segments); for(auto& pair : curve.segments) if(!(in >> pair[0] >> pair[1])) throw std::runtime_error("truncated periodic recovery segments");
            }
        }
        std::vector<FaceVertex> originalVertices;
        if(faceMesh) {
            std::size_t count; if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid original MVertex metadata count");
            originalVertices.resize(count);
            for(auto& v : originalVertices) {
                if(!(in >> v.id >> v.dimension >> v.entity >> v.point.x >> v.point.y >> v.point.z >> v.parameters.x >> v.parameters.y >> v.storedSize))
                    throw std::runtime_error("truncated original MVertex metadata");
                v.physicalId = v.id;
                if(!v.id || v.id > physical.nodes.size()) throw std::runtime_error("unknown original physical vertex");
                const auto& original = physical.nodes[v.id-1];
                if(v.dimension != original.dimension || v.entity != original.entity || v.point.x != original.point.x || v.point.y != original.point.y || v.point.z != original.point.z)
                    throw std::runtime_error("original MVertex metadata differs from frozen 1D physical mesh");
            }
        }
        NodeSizeInitializationInput frontInput; std::map<std::uint64_t,Point2D> frontParameters;
        bool extendBoundarySize = true;
        if(frontNodes) {
            std::size_t count;
            if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid periodic front vertex order count");
            frontInput.vertexOrder.resize(count);
            for(auto& id : frontInput.vertexOrder) if(!(in >> id)) throw std::runtime_error("truncated periodic front vertex order");
            if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid periodic embedded vertex count");
            frontInput.embeddedVertices.resize(count);
            for(auto& v : frontInput.embeddedVertices) if(!(in >> v.first >> v.second)) throw std::runtime_error("truncated periodic embedded vertex metadata");
            if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid periodic front CAD query count");
            for(std::size_t i = 0; i < count; ++i) {
                std::uint64_t id; Point2D uv;
                if(!(in >> id >> uv.x >> uv.y) || !frontParameters.emplace(id,uv).second) throw std::runtime_error("invalid periodic front CAD parameters");
            }
            auto readCurves = [&] {
                if(!(in >> count) || count > 1000000) throw std::runtime_error("invalid periodic front curve count");
                std::vector<NodeSizeCurve> curves(count);
                for(auto& curve : curves) {
                    int degenerated; std::size_t segments;
                    if(!(in >> degenerated >> segments) || (degenerated != 0 && degenerated != 1) || segments > 1000000) throw std::runtime_error("invalid periodic front curve metadata");
                    curve.meshDegenerated = degenerated != 0; curve.segments.resize(segments);
                    for(auto& edge : curve.segments) if(!(in >> edge[0] >> edge[1])) throw std::runtime_error("truncated periodic front curve segments");
                }
                return curves;
            };
            frontInput.boundaryCurves = readCurves(); frontInput.embeddedCurves = readCurves();
        }
        if(frontTriangles) {
            int extension; if(!(in >> extension) || (extension != 0 && extension != 1)) throw std::runtime_error("invalid resolved front boundary extension flag");
            extendBoundarySize = extension != 0;
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected embedded input tail");
        input.closestSurfaceCoordinates = [&](Point3D point) {
            Point2D uv; Point3D projected;
            if(!geometry.projectOnFaceSurface(boundary.entity-1,point,uv,projected))
                throw std::runtime_error("native direct projection unavailable; generic inverse fallback not reproduced");
            return uv;
        };
        input.surfacePoint = [&](Point2D uv) { return geometry.facePoint(boundary.entity-1,uv); };
        SurfaceParameterBounds bounds;
        if(!geometry.faceTrimParameterBounds(boundary.entity-1,bounds)) throw std::runtime_error("native UV bounds unavailable");
        const double du = bounds.uMax-bounds.uMin, dv = bounds.vMax-bounds.vMin;
        const double scale = std::sqrt(du*du+dv*dv);
        std::srand(seed);
        input.perturbInitialPoint = [&](Point2D uv) {
            const double x = randomFactor*scale*double(std::rand())/double(RAND_MAX);
            const double y = randomFactor*scale*double(std::rand())/double(RAND_MAX);
            return Point2D{uv.x+x,uv.y+y};
        };
        GeometrySizeSource sizes;
        if(graded) sizes.sizeCallback = [](const MeshSizeQuery& q,double lc) { return std::min(lc,.12+.015*std::abs(q.point.x)+.005*std::abs(q.point.y)); };
        PeriodicEmbeddedChart chart;
        std::vector<TrueBoundarySegment> truth;
        InitialTriangulation initialMesh;
        InitialTriangulationResult initialResult;
        if(fullChart) {
            fullInput.boundary.entity = boundary.entity;
            fullInput.boundary.parameterLower = {bounds.uMin,bounds.vMin}; fullInput.boundary.parameterUpper = {bounds.uMax,bounds.vMax};
            fullInput.boundary.modelCharacteristicLength = input.modelCharacteristicLength;
            fullInput.boundary.surfacePoint = input.surfacePoint; fullInput.embedded = input;
            PeriodicFaceChart complete;
            const auto result = buildPeriodicFaceChart(physical,fullInput,options,sizes,complete);
            if(!result.status.complete()) throw std::runtime_error(result.status.message);
            if(initialTriangulation) initialResult = buildPeriodicInitialTriangulation(complete,fullInput.boundary,initialMesh);
            if(bdsRecovery) {
                recoveryInput.boundaryLoops = complete.boundary.loops;
                for(const auto& constraint : complete.embedded.constraints) recoveryInput.embedded.push_back({constraint.from,constraint.to});
            }
            chart = std::move(complete.embedded);
            chart.vertices.insert(chart.vertices.begin(),complete.boundary.vertices.begin(),complete.boundary.vertices.end());
            chart.initialPoints = std::move(complete.initialPoints);
            chart.expectedInitialPointCount = complete.expectedInitialPointCount;
            truth = std::move(complete.trueBoundary);
        } else {
            const auto result = appendPeriodicEmbeddedChart(physical,boundary,input,options,sizes,chart);
            if(!result.status.complete()) throw std::runtime_error(result.status.message);
        }
        std::ofstream out(argv[3]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        out << "{\"maximum_id\":" << chart.maximumLocalId << ",\"expected_count\":" << chart.expectedInitialPointCount
            << ",\"mismatch\":" << (chart.initialPointCountMismatch ? "true" : "false") << ",\"vertices\":[";
        for(std::size_t i = 0; i < chart.vertices.size(); ++i) {
            if(i) out << ','; const auto& p = chart.vertices[i]; const auto& v = p.vertex;
            out << '[' << v.localId << ',' << v.physicalId << ',' << v.uv.x << ',' << v.uv.y << ',' << v.surfacePoint.x << ','
                << v.surfacePoint.y << ',' << v.surfacePoint.z << ',' << v.dimension << ',' << v.entity << ','
                << p.sizes.backgroundSize << ',' << p.sizes.meshSize << ',' << p.periodicCounterpart << ']';
        }
        out << "],\"initial_points\":[";
        for(std::size_t i = 0; i < chart.initialPoints.size(); ++i) {
            if(i) out << ','; const auto& p = chart.initialPoints[i]; out << '[' << p.localId << ',' << p.coordinates.x << ',' << p.coordinates.y << ']';
        }
        out << "],\"constraints\":[";
        for(std::size_t i = 0; i < chart.constraints.size(); ++i) { if(i) out << ','; const auto& e = chart.constraints[i]; out << '[' << e.from << ',' << e.to << ']'; }
        out << "],\"seam_selection_failures\":[";
        for(std::size_t i = 0; i < chart.seamSelectionFailures.size(); ++i) { if(i) out << ','; const auto& e = chart.seamSelectionFailures[i]; out << '[' << e.curveEntity << ',' << e.physicalId << ']'; }
        out << ']';
        if(fullChart) {
            out << ",\"true_boundary\":[";
            for(std::size_t i = 0; i < truth.size(); ++i) {
                if(i) out << ','; const auto& s = truth[i];
                out << '[' << s.curveEntity << ',' << s.branch << ',' << s.from.x << ',' << s.from.y << ',' << s.to.x << ',' << s.to.y << ']';
            }
            out << ']';
        }
        if(initialTriangulation) {
            out << ",\"periodic_initial\":{";
            if(!initialResult.status.complete()) {
                out << "\"status\":" << int(initialResult.status.code);
                std::cerr << "periodic initial status=" << int(initialResult.status.code) << ' ' << initialResult.status.message << '\n';
            } else {
                out << "\"bounds\":[" << initialMesh.outerLower.x << ',' << initialMesh.outerLower.y << ','
                    << initialMesh.outerUpper.x << ',' << initialMesh.outerUpper.y << "],\"vertices\":[";
                for(std::size_t i = 0; i < initialMesh.vertices.size(); ++i) {
                    if(i) out << ','; const auto& p = initialMesh.vertices[i]; const auto& v = p.vertex;
                    out << '[' << v.localId << ',' << v.physicalId << ',' << v.uv.x << ',' << v.uv.y << ','
                        << v.surfacePoint.x << ',' << v.surfacePoint.y << ',' << v.surfacePoint.z << ',' << v.dimension << ','
                        << v.entity << ',' << p.sizes.backgroundSize << ',' << p.sizes.meshSize << ',' << p.periodicCounterpart << ']';
                }
                out << "],\"sorted_points\":[";
                for(std::size_t i = 0; i < initialMesh.triangulation.sortedPoints.size(); ++i) {
                    if(i) out << ','; const auto& p = initialMesh.triangulation.sortedPoints[i];
                    out << '[' << p.localId << ',' << p.coordinates.x << ',' << p.coordinates.y << ']';
                }
                out << "],\"triangles\":[";
                for(std::size_t i = 0; i < initialMesh.triangles.size(); ++i) {
                    if(i) out << ','; const auto& t = initialMesh.triangles[i]; out << '[' << t[0] << ',' << t[1] << ',' << t[2] << ']';
                }
                out << ']';
            }
            out << '}';
        }
        if(bdsConversion && initialResult.status.complete()) {
            BdsMesh bds; const auto result = convertPeriodicInitialMeshToBds(initialMesh,bds);
            if(!result.status.complete()) throw std::runtime_error(result.status.message);
            out << ",\"bds\":"; writeBdsPoolJson(out,bds);
            if(bdsRecovery) {
                BdsMesh recovered; const auto recovery = recoverPeriodicBdsConstraints(bds,recoveryInput,recovered);
                if(!recovery.status.complete()) throw std::runtime_error(recovery.status.message);
                out << ",\"periodic_recovery\":"; writePeriodicBdsRecoveryJson(out,recovered,recovery);
                if(bdsDomain) {
                    if(recovery.embeddedFailed || recovery.needsRetry) throw std::runtime_error("periodic domain unavailable: constraint recovery failed or requires 1D retry");
                    BdsMesh classified; const auto domain = classifyPeriodicBdsDomain(recovered,classified);
                    if(!domain.status.complete()) throw std::runtime_error(domain.status.message);
                    out << ",\"periodic_domain\":"; writeBdsDomainJson(out,classified,domain);
                    if(bdsInterior) {
                        BdsMesh prepared; const auto interior = prepareBdsInterior(classified,prepared);
                        if(!interior.status.complete()) throw std::runtime_error(interior.status.message);
                        out << ",\"periodic_interior\":"; writeBdsInteriorJson(out,prepared,interior);
                        if(bdsMetric) {
                            InitialSurfaceMetricSource metricSource = [&](Point2D uv) {
                                Point3D du,dv; geometry.faceSurfaceDerivatives(boundary.entity-1,uv,du,dv);
                                return evaluateSurfaceMetric(du,dv);
                            };
                            BdsMesh legalized; const auto metric = legalizeInitialMesh(prepared,metricSource,legalized);
                            if(!metric.status.complete()) throw std::runtime_error(metric.status.message);
                            out << ",\"periodic_metric\":"; writeBdsMetricJson(out,legalized,metric);
                            if(faceMesh) {
                                FaceMesh original, reconstructed, converted;
                                auto status = initializeFaceMeshVertices(legalized,boundary.entity,originalVertices,original);
                                if(!status.complete()) throw std::runtime_error(status.message);
                                const auto seam = reconstructPeriodicSeamVertices(legalized,original,reconstructed);
                                if(!seam.status.complete()) throw std::runtime_error(seam.status.message);
                                const auto conversion = convertBdsToFaceMesh(legalized,reconstructed,converted);
                                if(!conversion.status.complete()) throw std::runtime_error(conversion.status.message);
                                out << ",\"periodic_face_mesh\":"; writeFaceMeshJson(out,converted,conversion);
                                if(frontNodes) {
                                    NodeSizeState nodes;
                                    const auto initialized = initializeNodeSizes(converted,frontInput,[&](const FaceVertex& v) { return frontParameters.at(v.id); },
                                        input.modelCharacteristicLength,options,sizes,nodes);
                                    if(!initialized.status.complete()) throw std::runtime_error(initialized.status.message);
                                    out << ",\"periodic_front_node_sizes\":"; writeNodeSizeJson(out,nodes,initialized);
                                    if(frontTriangles) {
                                        FrontMesh front;
                                        const auto initial = initializeFrontTriangles(converted,nodes,extendBoundarySize,front);
                                        if(!initial.status.complete()) throw std::runtime_error(initial.status.message);
                                        out << ",\"periodic_front_triangles\":"; writeFrontMeshJson(out,front,initial);
                                        if(frontQueue) {
                                            FrontQueueState queue; const auto status = initializeFrontQueue(front,queue);
                                            if(!status.complete()) throw std::runtime_error(status.message);
                                            out << ",\"periodic_front_queue\":"; writeFrontQueueJson(out,front,queue);
                                            if(frontGeneration) {
                                                FrontGenerationSource source;
                                                source.parameterBounds = [&] { SurfaceParameterBounds range; if(!geometry.faceTrimParameterBounds(boundary.entity-1,range)) throw std::runtime_error("trimmed face range unavailable"); return std::array<Point2D,2>{{{range.uMin,range.vMin},{range.uMax,range.vMax}}}; };
                                                source.metric = [&](Point2D uv) { Point3D du,dv; geometry.faceSurfaceDerivatives(boundary.entity-1,uv,du,dv); const auto m = evaluateSurfaceMetric(du,dv); if(!m.status.complete()) throw OperationFailure{m.status}; return m.metric; };
                                                source.surface = [&](Point2D uv) { return geometry.faceSurfacePoint(boundary.entity-1,uv); };
                                                source.surfacePoint = [&](Point2D uv) { return FrontSurfacePoint{geometry.faceSurfacePoint(boundary.entity-1,uv),true}; };
                                                source.backgroundSize = [&](const MeshSizeQuery& q) { return evaluateMeshSize(q,input.modelCharacteristicLength,options,sizes); };
                                                FrontGenerationOptions generation; generation.extendBoundarySize = extendBoundarySize; generation.trueBoundary = &truth; generation.maximumPops = 20000;
                                                out << ",\"periodic_front_generation\":"; writeFrontGenerationJson(out,front,queue,converted,nodes,source,generation,input.modelCharacteristicLength);
                                                FrontMeshTransferOptions transferOptions; transferOptions.equivalence = &converted.equivalence;
                                                const auto transferred = transferFrontMesh(front,queue,converted,nodes,transferOptions,converted);
                                                out << ",\"periodic_front_transfer\":"; writeFrontMeshTransferJson(out,transferred,converted);
                                                if(smoothing) {
                                                    const auto smoothingSource = makeOccSurfaceSmoothingSource(geometry,boundary.entity-1);
                                                    std::vector<SmoothingVertexEvent> events;
                                                    const auto smoothed = smoothSurfaceMesh(converted,smoothingSource,{}, {},[&](const auto& event) { events.push_back(event); });
                                                    out << ",\"periodic_smoothing\":"; writeSurfaceSmoothingJson(out,smoothed,events,converted);
                                                }
                                            }
                                            if(frontPoints) {
                                                FrontInsertionSource insertionSource;
                                                insertionSource.surfacePoint = [&](Point2D uv) { return FrontSurfacePoint{geometry.faceSurfacePoint(boundary.entity-1,uv),true}; };
                                                insertionSource.backgroundSize = [&](const MeshSizeQuery& q) { return evaluateMeshSize(q,input.modelCharacteristicLength,options,sizes); };
                                                out << ",\"periodic_front_points\":";
                                                writeInitialFrontPointsJson(out,converted,nodes,front,extendBoundarySize,[&](Point2D uv) {
                                                    Point3D du,dv; geometry.faceSurfaceDerivatives(boundary.entity-1,uv,du,dv);
                                                    const auto evaluated = evaluateSurfaceMetric(du,dv);
                                                    if(!evaluated.status.complete()) throw OperationFailure{evaluated.status}; return evaluated.metric;
                                                },[&](Point2D uv) { return geometry.faceSurfacePoint(boundary.entity-1,uv); },frontCavity,frontInsertion ? &insertionSource : nullptr,input.modelCharacteristicLength);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        out << "}\n"; if(!out) throw std::runtime_error("embedded chart output write failed");
        return 0;
    }
    catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n'; return 1; }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
