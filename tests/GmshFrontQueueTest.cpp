/** @file GmshFrontQueueTest.cpp
 * @brief 初始扫描/首活动边/pop/失败0与-1/局部入集及失败原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/front/FrontQueue.h"
#include <cmath>
#include <iostream>
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* label) { if(!ok) { ++failures; std::cerr << "FAIL " << label << '\n'; } }
FrontTriangle triangle(std::array<std::uint64_t,3> ids,double radius,std::array<int,3> neighbors = {-1,-1,-1}) {
    FrontTriangle t; t.vertices = ids; t.radius = radius; t.neighbors = neighbors; return t;
}
std::vector<int> ids(const FrontTriangleSet& set) { std::vector<int> values; for(const auto& key : set) values.push_back(key.triangle); return values; }
}
int main() {
    const double limit = frontalRadiusLimit();
    check(limit == .5*std::sqrt(2.)*1.,"fixed native threshold arithmetic");
    FrontMesh mesh; mesh.triangles = {triangle({1,2,3},2.,{1,2,-1}),triangle({2,4,3},limit),triangle({3,4,5},.2)};
    auto activity = findActiveEdge(mesh,0,limit);
    check(activity.status.complete() && activity.active && activity.edge == 1,"first eligible edge: equal-limit neighbor excluded, positive small neighbor accepted before boundary");
    mesh.triangles[2].radius = 0.; activity = findActiveEdge(mesh,0,limit);
    check(activity.status.complete() && activity.active && activity.edge == 2,"zero neighbor excluded, later boundary selected");
    mesh.triangles[2].radius = -1.; mesh.triangles[0].neighbors[2] = 1; activity = findActiveEdge(mesh,0,limit);
    check(activity.status.complete() && !activity.active && activity.edge == 3,"negative and equal-limit neighbor block all edges");
    mesh.triangles[2].radius = .2; mesh.triangles[2].deleted = true; activity = findActiveEdge(mesh,0,limit);
    check(activity.status.complete() && activity.active && activity.edge == 1,"neighbor deleted flag ignored by native activity predicate");
    mesh.triangles[0].deleted = true; activity = findActiveEdge(mesh,0,limit,77);
    check(activity.status.complete() && !activity.active && activity.edge == 77,"deleted target leaves reference edge unchanged");
    mesh.triangles = {triangle({1,2,3},2.),triangle({4,5,6},1.),triangle({7,8,9},.3,{0,0,0}),triangle({10,11,12},.1)};
    mesh.orderedTriangles = {0,1,2,3}; FrontQueueState queue;
    auto status = initializeFrontQueue(mesh,queue);
    check(status.complete() && ids(queue.allTriangles) == mesh.orderedTriangles && ids(queue.activeTriangles) == std::vector<int>{0,1},"initial inactive small radius breaks scan and omits later active boundary triangle");
    if(status.complete()) {
        auto selected = popActiveTriangle(mesh,queue);
        check(selected.status.complete() && selected.triangle == 0 && selected.edge == 0 && selected.requiresInsertion,"pop one native worst/first edge");
        const auto failure = markInsertionFailure(mesh,queue,0,FrontInsertionFailure::ContainingTriangleNotFound);
        check(failure.status.complete() && failure.reinserted && mesh.triangles[0].radius == 0. && ids(queue.allTriangles) == std::vector<int>{1,2,3,0},"not-found moves AllTris key to radius zero without auto-enqueue");
        selected = popActiveTriangle(mesh,queue);
        const auto rejected = markInsertionFailure(mesh,queue,1,FrontInsertionFailure::PointInsertionRejected);
        check(selected.triangle == 1 && rejected.status.complete() && mesh.triangles[1].radius == -1. && queue.activeTriangles.empty(),"rejection radius minus one preserved separately");
        selected = popActiveTriangle(mesh,queue);
        check(selected.status.complete() && selected.triangle == -1,"empty pop normal completion");
    }
    mesh.triangles = {triangle({4,5,6},limit),triangle({1,2,3},limit),triangle({7,8,9},.2),triangle({10,11,12},0.)}; mesh.orderedTriangles = {1,0,2,3};
    status = initializeFrontQueue(mesh,queue);
    check(status.complete() && ids(queue.activeTriangles) == mesh.orderedTriangles,"initial active boundary triangles included even radius at/below threshold");
    if(status.complete()) {
        mesh.triangles[1].deleted = true;
        auto selected = popActiveTriangle(mesh,queue);
        check(selected.triangle == 1 && selected.deleted && !selected.activityEvaluated && !selected.requiresInsertion,"stale deleted entry consumed without activity call");
        selected = popActiveTriangle(mesh,queue);
        check(selected.triangle == 0 && selected.active && !selected.requiresInsertion,"equal-limit active entry consumed without insertion");
        mesh.triangles.push_back(triangle({13,14,15},2.));
        auto update = registerFrontTriangles(mesh,{4,4},queue);
        check(update.status.complete() && update.inserted == std::vector<int>{4} && update.rejectedDuplicates == std::vector<int>{4},"AllTris registration preserves repeated set rejection");
        update = updateActiveTriangles(mesh,{2,4,4,1},queue);
        check(update.status.complete() && update.inserted == std::vector<int>{4} && ids(queue.activeTriangles) == std::vector<int>{4,2,3},"local success update only active radius above limit; keep old small entries");
    }
    FrontQueueState sentinel; sentinel.limit = 99.; RunControl control; control.keepGoing = [] { return false; };
    status = initializeFrontQueue(mesh,sentinel,limit,control);
    check(status.code == MeshingStatus::Cancelled && sentinel.limit == 99. && sentinel.allTriangles.empty(),"initial cancellation atomic");
    auto invalid = mesh; invalid.triangles[0].neighbors[0] = 999;
    activity = findActiveEdge(invalid,0,limit);
    check(activity.status.code == MeshingStatus::InvalidTopology,"neighbor index validated before unsafe lookup");
    invalid = mesh; invalid.orderedTriangles = {0,1};
    status = initializeFrontQueue(invalid,sentinel,limit);
    check(status.code == MeshingStatus::InvalidTopology && sentinel.limit == 99.,"invalid initial set ordering rejected");
    invalid = mesh; invalid.triangles[0].radius = std::numeric_limits<double>::quiet_NaN();
    activity = findActiveEdge(invalid,0,limit);
    check(activity.status.code == MeshingStatus::InvalidInput,"nonfinite radius rejected");
    mesh.triangles = {triangle({1,2,3},2.,{1,1,1}),triangle({4,5,6},.2),triangle({7,8,9},3.)}; mesh.orderedTriangles = {0,1};
    status = initializeFrontQueue(mesh,queue);
    if(status.complete()) {
        const auto before = ids(queue.activeTriangles);
        auto selected = popActiveTriangle(mesh,queue,control);
        check(selected.status.code == MeshingStatus::Cancelled && ids(queue.activeTriangles) == before,"pop cancellation preserves active set");
        auto failure = markInsertionFailure(mesh,queue,0,FrontInsertionFailure::ContainingTriangleNotFound);
        check(failure.status.code == MeshingStatus::InvalidTopology && mesh.triangles[0].radius == 2.,"do not mutate comparator key while target still queued");
        const auto allBefore = ids(queue.allTriangles);
        auto update = registerFrontTriangles(mesh,{2,999},queue);
        check(update.status.code == MeshingStatus::InvalidTopology && update.inserted.empty() && ids(queue.allTriangles) == allBefore,"invalid local registration commits no prefix");
        int calls = 0; RunControl late; late.keepGoing = [&] { return ++calls < 3; };
        update = updateActiveTriangles(mesh,{2,2},queue,late);
        check(update.status.code == MeshingStatus::Cancelled && update.inserted.empty() && ids(queue.activeTriangles) == before,"late cancel commits no activity prefix");
        update = registerFrontTriangles(mesh,{2},queue);
        mesh.triangles[0].neighbors = {2,2,2}; // 局部连接变化不触碰本片set键，pop时重查活动资格。
        selected = popActiveTriangle(mesh,queue);
        check(update.status.complete() && selected.status.complete() && selected.triangle == 0 && !selected.active && selected.edge == 3 && !selected.requiresInsertion,"stale active entry rechecks replaced large-radius neighbors");
        failure = markInsertionFailure(mesh,queue,0,static_cast<FrontInsertionFailure>(99));
        check(failure.status.code == MeshingStatus::InvalidInput && mesh.triangles[0].radius == 2.,"invalid failure kind preserves radius and AllTris");
    }
    FrontMesh empty; status = initializeFrontQueue(empty,sentinel);
    check(status.complete() && sentinel.allTriangles.empty() && sentinel.activeTriangles.empty(),"empty initial set preserved");
    return failures ? 1 : 0;
}
