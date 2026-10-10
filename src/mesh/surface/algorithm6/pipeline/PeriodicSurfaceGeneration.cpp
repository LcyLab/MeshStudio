/** @file PeriodicSurfaceGeneration.cpp
 * @brief CAD周期面完整编排与缝副本生命周期；复用既有周期算法。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "SurfaceGenerator.h"
#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
#include "mesh/surface/algorithm6/bds/BdsMeshConversion.h"
#include "mesh/surface/algorithm6/bds/PeriodicBdsRecovery.h"
#include "mesh/surface/algorithm6/bds/BdsDomainClassification.h"
#include "mesh/surface/algorithm6/bds/BdsInteriorPreparation.h"
#include "mesh/surface/algorithm6/postprocess/PeriodicSeamReconstruction.h"
#include "mesh/surface/algorithm6/front/FrontTriangleInitialization.h"
#include "mesh/surface/algorithm6/postprocess/FrontMeshTransfer.h"
#include <algorithm>
#include <cmath>
#include <set>
namespace mesher::gmshReference {
/// 按节点身份和原 UV 分支生成最终逐角 UV 坐标。
std::size_t finalizeFaceMeshCoordinates(FaceMesh&,const Algorithm6FaceInput&,const RunControl&);
/// 编排周期面的图表、约束恢复、前沿生成和网格转移。
Algorithm6FaceResult generatePeriodicFaceMesh(const BoundaryMesh& boundary,const Algorithm6FaceInput& input,double modelLength,
    const GeometrySizeSource& sizes,const SurfaceMeshingOptions& options,PeriodicInitialPerturbation& random,
    const RunControl& control,const SurfaceStageObserver& observer) {
    Algorithm6FaceResult result;
    auto require=[](OperationStatus s){if(!s.complete())throw OperationFailure{std::move(s)};};
    auto check=[&]{if(control.keepGoing&&!control.keepGoing())throw OperationFailure{{MeshingStatus::Cancelled,"periodic surface generation cancelled"}};};
    auto stage=[&](const char* name){result.failedStage=name;check();};
    auto limits=[&]{auto c=control;c.maximumGeometryEvaluations=result.statistics.geometryEvaluations>=control.maximumGeometryEvaluations?0:control.maximumGeometryEvaluations-result.statistics.geometryEvaluations;return c;};
    auto spend=[&](std::size_t n){result.statistics.geometryEvaluations+=n;if(result.statistics.geometryEvaluations>control.maximumGeometryEvaluations)throw OperationFailure{{MeshingStatus::BudgetExceeded,"cumulative periodic query budget"}};};
    auto observe=[&](const char* name,const FaceMesh& mesh){if(observer)observer(name,input.chart.entity,boundary,&mesh);};
    try {
        stage("periodic_chart");std::size_t queries=0;
        auto chartInput=input.periodicChart(boundary,options,limits(),queries);spend(queries);
        const double span=std::hypot(chartInput.boundary.parameterUpper.x-chartInput.boundary.parameterLower.x,
            chartInput.boundary.parameterUpper.y-chartInput.boundary.parameterLower.y);
        chartInput.embedded.perturbInitialPoint=[&random,span,factor=options.initialRandomFactor](Point2D uv){return random.perturb(uv,span,factor);};
        chartInput.boundary.modelCharacteristicLength=modelLength;
        PeriodicFaceChart chart;const auto built=buildPeriodicFaceChart(boundary,chartInput,options,sizes,chart,limits());spend(built.geometryEvaluations);require(built.status);
        stage("periodic_initial");InitialTriangulation initial;
        const auto initialResult=buildPeriodicInitialTriangulation(chart,chartInput.boundary,initial,limits());spend(initialResult.geometryEvaluations);require(initialResult.status);
        if(initial.route==FaceChartRoute::SkipInsufficientNodes)throw OperationFailure{{MeshingStatus::InvalidTopology,"periodic face has fewer than three boundary positions"}};
        stage("periodic_bds_conversion");BdsMesh bds;require(convertPeriodicInitialMeshToBds(initial,bds,limits()).status);
        auto curves=[&](const std::vector<FaceCurveUse>& uses) {
            std::vector<BdsRecoveryCurve> values;
            for(const auto& use:uses) {
                const auto found=std::find_if(boundary.curves.begin(),boundary.curves.end(),[&](const auto& c){return c.entity==use.entity;});
                if(found==boundary.curves.end())throw OperationFailure{{MeshingStatus::InvalidTopology,"periodic shared curve missing"}};
                BdsRecoveryCurve c;c.entity=found->entity;c.meshDegenerated=found->meshDegenerate;c.endpoints=std::array<std::uint64_t,2>{found->beginNode,found->endNode};
                for(std::size_t k=1;k<found->samples.size();++k)c.segments.push_back({found->samples[k-1].node,found->samples[k].node});
                values.push_back(std::move(c));
            }
            return values;
        };
        const auto boundaryCurves=curves(input.chart.boundaryCurves),embeddedCurves=curves(input.chart.embeddedCurves);
        if(initial.route==FaceChartRoute::General) {
            stage("periodic_recovery");PeriodicBdsRecoveryInput recoveryInput;
            recoveryInput.boundaryLoops=chart.boundary.loops;recoveryInput.boundaryCurves=boundaryCurves;
            for(const auto& c:chart.embedded.constraints)recoveryInput.embedded.push_back({c.from,c.to});
            BdsMesh recovered;const auto recoveredResult=recoverPeriodicBdsConstraints(bds,recoveryInput,recovered,limits());require(recoveredResult.status);
            if(recoveredResult.embeddedFailed)throw OperationFailure{{MeshingStatus::NonConverged,"periodic embedded recovery failed"}};
            if(recoveredResult.needsRetry) {
                // 把真正失败的局部缝副本转换回共享物理边，交模型重建入口细分。
                for(const auto& a:recoveredResult.attempts)if(a.recovery&&a.recovery->recoveredEdge<0&&a.curveEntity) {
                    const auto from=bds.points.at(a.first).physicalId,to=bds.points.at(a.second).physicalId;
                    if(from&&to)result.unrecovered.push_back({*a.curveEntity,from,to});
                }
                throw OperationFailure{{MeshingStatus::NonConverged,"periodic boundary recovery requires refinement"}};
            }
            stage("periodic_domain");BdsMesh classified;require(classifyPeriodicBdsDomain(recovered,classified,limits()).status);
            stage("periodic_interior");BdsMesh prepared;require(prepareBdsInterior(classified,prepared,limits()).status);
            stage("periodic_legalization");const auto metric=legalizeInitialMesh(prepared,input.initialMetric,bds,limits());spend(metric.metricQueries);require(metric.status);
        }
        std::vector<FaceVertex> originals;
        for(const auto& n:boundary.nodes)if(n.alive){FaceVertex v;v.id=v.physicalId=n.id;v.point=n.point;v.dimension=n.dimension;v.entity=n.entity;v.parameters={n.parameter,0};v.storedSize=n.storedSize;originals.push_back(v);}
        stage("periodic_seam");FaceMesh original,reconstructed,mesh;
        require(initializeFaceMeshVertices(bds,input.chart.entity,originals,original,limits()));
        require(reconstructPeriodicSeamVertices(bds,original,reconstructed,limits()).status);
        if(!reconstructed.unreconstructedSeams.empty())throw OperationFailure{{MeshingStatus::InvalidTopology,"periodic seam identity could not be reconstructed"}};
        require(convertBdsToFaceMesh(bds,reconstructed,mesh,limits()).status);
        NodeSizeInitializationInput nodeInput;std::set<std::uint64_t> referenced;
        for(const auto& t:mesh.triangles)for(auto id:t)referenced.insert(id);
        for(const auto& c:boundaryCurves) {nodeInput.boundaryCurves.push_back({c.meshDegenerated,c.segments});if(!c.meshDegenerated)for(const auto& s:c.segments)referenced.insert(s.begin(),s.end());}
        for(const auto& c:embeddedCurves) {nodeInput.embeddedCurves.push_back({c.meshDegenerated,c.segments});if(!c.meshDegenerated)for(const auto& s:c.segments)referenced.insert(s.begin(),s.end());}
        for(auto id:input.chart.embeddedVertices){nodeInput.embeddedVertices.push_back({id,1.e22});referenced.insert(id);}
        nodeInput.vertexOrder.assign(referenced.begin(),referenced.end());
        stage("periodic_front_sizes");NodeSizeState nodes;
        const auto sized=initializeNodeSizes(mesh,nodeInput,input.reparameterize,modelLength,options,sizes,nodes,limits());spend(sized.reparameterizations.size()+sized.embeddedQueries.size());require(sized.status);
        observe("initial",mesh);
        stage("periodic_front");FrontMesh front;require(initializeFrontTriangles(mesh,nodes,options.extendBoundarySize!=0,front,limits()).status);
        FrontQueueState queue;require(initializeFrontQueue(front,queue,frontalRadiusLimit(),limits()));
        auto source=input.front;source.backgroundSize=[&](const MeshSizeQuery& q){return evaluateMeshSize(q,modelLength,options,sizes);};
        FrontGenerationOptions frontOptions;frontOptions.extendBoundarySize=options.extendBoundarySize!=0;frontOptions.trueBoundary=&chart.trueBoundary;
        const auto generated=generateFrontMesh(front,queue,mesh,nodes,source,frontOptions,limits());spend(generated.geometryEvaluations);require(generated.status);result.statistics.inserted=generated.inserted;
        stage("periodic_transfer");FaceMesh uncollapsed,folded;
        require(transferFrontMesh(front,queue,mesh,nodes,{},uncollapsed,limits()).status);
        FrontMeshTransferOptions transfer;transfer.equivalence=&mesh.equivalence;
        require(transferFrontMesh(front,queue,mesh,nodes,transfer,folded,limits()).status);
        for(const auto& t:uncollapsed.triangles) {
            auto physical=t;std::array<Point2D,3> uv;
            for(int k=0;k<3;++k){uv[k]=nodes.nodes.at(nodes.indices.at(t[k])).parameters;const auto e=mesh.equivalence.find(t[k]);if(e!=mesh.equivalence.end())physical[k]=e->second;}
            if(physical[0]!=physical[1]&&physical[0]!=physical[2]&&physical[1]!=physical[2])folded.triangleCornerUV.push_back(uv);
        }
        mesh=std::move(folded);observe("pre_smoothing",mesh);
        stage("periodic_smoothing");SurfaceSmoothingOptions smooth;smooth.iterations=options.smoothingIterations;
        const auto smoothed=smoothSurfaceMesh(mesh,input.smoothing,smooth,limits());spend(smoothed.geometryEvaluations);require(smoothed.status);
        for(std::size_t i=0;i<mesh.triangles.size();++i)for(int k=0;k<3;++k){const auto& v=mesh.vertices.at(mesh.triangles[i][k]);if(v.dimension==2)mesh.triangleCornerUV[i][k]=v.parameters;}
        // 原算法平滑最后使用缝副本；此后仅保留物理连接与独立每角参数。
        for(const auto& e:mesh.equivalence){mesh.vertices.erase(e.first);mesh.parametricCoordinates.erase(e.first);}
        mesh.equivalence.clear();mesh.bdsToVertex.clear();
        stage("final_coordinates");spend(finalizeFaceMeshCoordinates(mesh,input,limits()));
        check();observe("final",mesh);result.mesh=std::move(mesh);result.failedStage.clear();
    }catch(const OperationFailure& f){result.status=f.status;}
     catch(const std::exception& e){result.status={MeshingStatus::GeometryQueryFailed,e.what()};}
     catch(...){result.status={MeshingStatus::GeometryQueryFailed,"non-standard geometry exception during "+result.failedStage};}
    return result;
}
}
