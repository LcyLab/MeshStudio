/** @file GmshFrontGenerationTest.cpp
 * @brief 多轮真实插点反馈、空域/无门控区别、累计查询与中止活动头恢复。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontGeneration.h"
#include "mesh/surface/algorithm6/front/FrontTriangleInitialization.h"
#include <iostream>
#include <stdexcept>
using namespace mesher; using namespace mesher::gmshReference;
int main() {
    int failures = 0; auto check = [&](bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } };
    FaceMesh originalMesh; originalMesh.entity = 1; originalMesh.maximumVertexId = 4;
    NodeSizeState originalNodes; const Point2D p[] = {{0.,0.},{2.,0.},{0.,2.},{2.,2.}};
    for(int i = 0; i < 4; ++i) { FaceVertex v; v.id = i+1; v.point = {p[i].x,p[i].y,0.}; v.dimension = 1; originalMesh.vertices.emplace(v.id,v); originalNodes.indices[v.id] = i; originalNodes.nodes.push_back({v.id,p[i],{.6,.6}}); }
    originalMesh.triangles = {{1,2,3},{2,4,3}};
    FrontMesh originalFront; initializeFrontTriangles(originalMesh,originalNodes,true,originalFront); FrontQueueState originalQueue; initializeFrontQueue(originalFront,originalQueue);
    auto mesh = originalMesh; auto nodes = originalNodes; auto front = originalFront; auto queue = originalQueue;
    int bounds = 0,metric = 0,surface = 0,bgm = 0,observed = 0; bool createdPop = false;
    FrontGenerationSource source;
    source.parameterBounds = [&] { ++bounds; return std::array<Point2D,2>{{{0.,0.},{2.,2.}}}; };
    source.metric = [&](Point2D) { ++metric; return SurfaceMetric{1.,0.,1.}; };
    source.surface = [&](Point2D uv) { ++surface; return Point3D{uv.x,uv.y,0.}; };
    source.surfacePoint = [&](Point2D uv) { ++surface; return FrontSurfacePoint{{uv.x,uv.y,0.},true}; };
    source.backgroundSize = [&](const MeshSizeQuery&) { ++bgm; MeshSizeEvaluation e; e.size = .6; return e; };
    // 四个粗边界点的原生单片拒绝会很快耗尽队列；多轮成功用已经离散的较密边界/初网格。
    FaceMesh denseMesh; denseMesh.entity = 1; denseMesh.maximumVertexId = 16; NodeSizeState denseNodes;
    for(int y = 0; y < 4; ++y) for(int x = 0; x < 4; ++x) {
        FaceVertex v; v.id = y*4+x+1; v.dimension = x == 0 || x == 3 || y == 0 || y == 3 ? 1 : 2; v.point = {x*(2./3.),y*(2./3.),0.};
        denseMesh.vertices.emplace(v.id,v); denseNodes.indices[v.id] = denseNodes.nodes.size(); denseNodes.nodes.push_back({v.id,{v.point.x,v.point.y},{.3,.3}});
    }
    for(int y = 0; y < 3; ++y) for(int x = 0; x < 3; ++x) {
        const std::uint64_t a = y*4+x+1,b = a+1,c = a+4,d = c+1; denseMesh.triangles.push_back({a,b,c}); denseMesh.triangles.push_back({b,d,c});
    }
    mesh = denseMesh; nodes = denseNodes; initializeFrontTriangles(mesh,nodes,true,front); initializeFrontQueue(front,queue);
    FrontGenerationOptions options; options.maximumPops = 10000;
    const auto initialTriangleCount = front.triangles.size();
    auto result = generateFrontMesh(front,queue,mesh,nodes,source,options,{},[&](const FrontGenerationEvent& event) { ++observed; createdPop = createdPop || std::size_t(event.selection.triangle) >= initialTriangleCount; });
    check(result.status.complete() && result.inserted > 2 && result.pops == std::size_t(observed) && createdPop && queue.activeTriangles.empty(),"newly inserted triangles feed the same dynamic queue until empty");
    check(bounds == 1 && result.geometryEvaluations == std::size_t(bounds+metric+surface+bgm) && nodes.nodes.size() >= denseNodes.nodes.size()+result.inserted && mesh.interiorVertices.size() == result.inserted,"one range query, accumulated geometry counts and successful live publication");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; queue = originalQueue; bounds = metric = surface = bgm = 0;
    std::vector<TrueBoundarySegment> empty; options.trueBoundary = &empty;
    result = generateFrontMesh(front,queue,mesh,nodes,source,options);
    check(result.status.complete() && result.domainRejected == 2 && result.inserted == 0 && bgm == 0 && nodes.nodes.size() == 4 && queue.activeTriangles.empty() && front.triangles[0].radius == originalFront.triangles[0].radius,"non-null empty domain only consumes initial heads, leaves radii/slots");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; queue = originalQueue; options.trueBoundary = nullptr;
    RunControl control; control.maximumGeometryEvaluations = 1; bounds = metric = surface = bgm = 0;
    result = generateFrontMesh(front,queue,mesh,nodes,source,options,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 1 && bounds == 1 && metric == 0 && queue.activeTriangles.size() == originalQueue.activeTriangles.size() && front.triangles.size() == 2 && mesh.maximumVertexId == 4,"budget after range query restores uncompleted pop without querying metric");
    control.maximumGeometryEvaluations = 2;
    result = generateFrontMesh(front,queue,mesh,nodes,source,options,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 2 && queue.activeTriangles.size() == 2 && mesh.maximumVertexId == 4,"shared geometry budget also stops circle correction without swallowing control failure");
    control = {}; control.maximumNodes = 4;
    result = generateFrontMesh(front,queue,mesh,nodes,source,options,control);
    check(result.status.code == MeshingStatus::BudgetExceeded && queue.activeTriangles.size() == 2 && nodes.nodes.size() == 4 && mesh.maximumVertexId == 4 && !front.triangles[0].deleted && !front.triangles[1].deleted,"allocation budget restores activity head and insertion state");
    control = {}; control.keepGoing = [&] { return mesh.interiorVertices.empty(); };
    result = generateFrontMesh(front,queue,mesh,nodes,source,options,control);
    check(result.status.code == MeshingStatus::Cancelled && result.inserted == 1 && mesh.interiorVertices.size() == 1 && queue.activeTriangles.size() > 0,"cancel at next iteration keeps prior committed insertion");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; queue = originalQueue;
    result = generateFrontMesh(front,queue,mesh,nodes,source,options,{},[](const FrontGenerationEvent&) { throw std::runtime_error("observer failed"); });
    check(result.status.code == MeshingStatus::NonConverged && result.pops == 1 && result.inserted == 1 && mesh.interiorVertices.size() == 1,"observer exception reports failure while preserving the completed insertion");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; queue = originalQueue; queue.activeTriangles.clear(); bounds = 0;
    result = generateFrontMesh(front,queue,mesh,nodes,source,options);
    check(result.status.complete() && result.pops == 0 && result.geometryEvaluations == 1 && bounds == 1,"empty active set still performs the native one-time range query");
    mesh = originalMesh; nodes = originalNodes; front = originalFront; queue = originalQueue;
    source.surfacePoint = [&](Point2D uv) { ++surface; return FrontSurfacePoint{{uv.x,uv.y,0.},false}; };
    result = generateFrontMesh(front,queue,mesh,nodes,source,options);
    check(result.status.complete() && result.inserted == 0 && mesh.vertices.size() == 4 && nodes.nodes.size() > 4 && mesh.maximumVertexId > 4,"failed CAD point keeps consumed ghost slots but publishes no live vertex in the dynamic loop");
    source.surfacePoint = [&](Point2D uv) { ++surface; return FrontSurfacePoint{{uv.x,uv.y,0.},true}; };
    mesh = originalMesh; nodes = originalNodes; front = originalFront; queue = originalQueue; options.maximumPops = 0; bounds = 0;
    result = generateFrontMesh(front,queue,mesh,nodes,source,options);
    check(result.status.code == MeshingStatus::BudgetExceeded && queue.activeTriangles.size() == 2 && bounds == 1,"pop budget is explicit after native one-time range setup");
    options.maximumPops = 10000; source.metric = [](Point2D)->SurfaceMetric { throw std::runtime_error("metric source failed"); };
    result = generateFrontMesh(front,queue,mesh,nodes,source,options);
    check(result.status.code == MeshingStatus::GeometryQueryFailed && queue.activeTriangles.size() == 2 && mesh.vertices.size() == 4,"source exception restores consumed head");
    return failures ? 1 : 0;
}
