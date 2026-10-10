#pragma once
/** @file GmshFrontPointJson.h
 * @brief 前沿候选叶/初始排空及可选首插点衔接；首提交隔离，不执行完整动态反馈。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontPointPlacement.h"
#include "mesh/surface/algorithm6/front/FrontQueue.h"
#include "GmshFrontCavityJson.h"
#include "GmshFrontPointInsertionJson.h"
#include <ostream>
#include <stdexcept>
inline mesher::gmshReference::FrontPointPlacementResult writeFrontPointJson(std::ostream& out,const mesher::gmshReference::FrontTriangle& triangle,int edge,
    const mesher::gmshReference::FaceMesh& mesh,const mesher::gmshReference::NodeSizeState& nodes,bool extension,
    const mesher::gmshReference::FrontMetricSource& metric,const mesher::gmshReference::SurfaceFunction& surface,
    mesher::gmshReference::FrontPointCandidate* published = nullptr)
{
    using namespace mesher::gmshReference; using mesher::Point2D;
    FrontPointCandidate candidate; auto result = computeFrontPoint(triangle,edge,nodes,extension,metric,candidate);
    std::vector<std::array<double,5>> queries;
    if(result.status.complete()) result = correctFrontPointOnSurface(triangle,edge,mesh,candidate,[&](Point2D uv) {
        const auto p = surface(uv); queries.push_back({uv.x,uv.y,p.x,p.y,p.z}); return p;
    },candidate);
    out << "{\"status\":" << int(result.status.code) << ",\"values\":[";
    if(result.status.complete()) {
        const auto& c = candidate;
        const double values[] = {c.metricQuery.x,c.metricQuery.y,c.planarCenter.x,c.planarCenter.y,c.metricCenter.x,c.metricCenter.y,
            c.edgeMidpoint.x,c.edgeMidpoint.y,c.fastPoint.x,c.fastPoint.y,c.metric.gUU,c.metric.gUV,c.metric.gVV,
            c.edgeMeanSizes.meshSize,c.edgeMeanSizes.backgroundSize,c.targetSize,c.distanceToCenter,c.height,c.directionNorm,c.metricRatio,
            c.circle.center.x,c.circle.center.y,c.circle.center.z,c.circle.cosineAxis.x,c.circle.cosineAxis.y,c.circle.cosineAxis.z,
            c.circle.sineAxis.x,c.circle.sineAxis.y,c.circle.sineAxis.z,c.circle.radius,c.normalNorm,double(c.skippedDegenerateNormal),
            double(c.correctionAttempted),double(c.intersectionFound),c.point.x,c.point.y,1.,double(queries.size())};
        for(int j = 0; j < 38; ++j) { if(j) out << ','; out << values[j]; }
    }
    out << "],\"queries\":[";
    for(std::size_t q = 0; q < queries.size(); ++q) {
        if(q) out << ','; out << '['; for(int j = 0; j < 5; ++j) { if(j) out << ','; out << queries[q][j]; } out << ']';
    }
    out << "]}";
    if(result.status.complete() && published) *published = candidate;
    return result;
}
inline void writeInitialFrontPointsJson(std::ostream& out,const mesher::gmshReference::FaceMesh& mesh,
    const mesher::gmshReference::NodeSizeState& nodes,const mesher::gmshReference::FrontMesh& front,bool extension,
    const mesher::gmshReference::FrontMetricSource& metric,const mesher::gmshReference::SurfaceFunction& surface,bool includeCavity = false,
    const mesher::gmshReference::FrontInsertionSource* insertionSource = nullptr,double modelLength = 0.)
{
    using namespace mesher::gmshReference;
    FrontQueueState queue; const auto status = initializeFrontQueue(front,queue);
    FrontMesh cavityMesh; if(includeCavity) cavityMesh = front; // 测试排空只复制一次；核心buildFrontCavity原位标记。
    if(!status.complete()) throw std::runtime_error(status.message);
    out << '['; std::size_t row = 0; bool insertionWritten = false;
    while(!queue.activeTriangles.empty()) {
        const auto pop = popActiveTriangle(front,queue); if(!pop.status.complete()) throw std::runtime_error(pop.status.message);
        if(!pop.requiresInsertion) continue;
        if(row++) out << ','; out << "{\"triangle\":" << pop.triangle << ",\"edge\":" << pop.edge << ",\"candidate\":";
        FrontPointCandidate point;
        const auto placement = writeFrontPointJson(out,front.triangles.at(pop.triangle),pop.edge,mesh,nodes,extension,metric,surface,&point);
        if(includeCavity) {
            if(!placement.status.complete()) throw std::runtime_error("initial front cavity requires finite candidate: "+placement.status.message);
            out << ",\"cavity\":"; writeFrontCavityJson(out,cavityMesh,nodes,queue.allTriangles,pop.triangle,point.point,point.metric);
        }
        if(insertionSource && !insertionWritten) {
            if(!placement.status.complete()) throw std::runtime_error("initial insertion requires finite candidate: "+placement.status.message);
            // 只检查首个实际候选的前缀衔接。测试副本隔离提交，后续初始排空不冒充动态反馈。
            auto insertedMesh = mesh; auto insertedNodes = nodes; auto insertedFront = front; auto insertedQueue = queue;
            const auto result = insertFrontPoint(insertedFront,insertedQueue,insertedMesh,insertedNodes,pop.triangle,point.point,point.metric,*insertionSource,{extension,true,false});
            out << ",\"insertion\":"; writeFrontPointInsertionJson(out,result,insertedFront,insertedQueue,insertedMesh,insertedNodes,modelLength);
            insertionWritten = true;
        }
        out << '}';
    }
    out << ']';
}
