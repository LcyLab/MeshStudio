/** @file FrontQueue.cpp
 * @brief 固定算法6前沿活动调度。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontQueue.h"
#include <cmath>
namespace mesher::gmshReference {
namespace {
/// 查询调用方是否已请求取消。
bool cancelled(const RunControl& control) { return control.keepGoing && !control.keepGoing(); }
/// 检查前沿三角形编号和半径是否可用于队列操作。
OperationStatus validateTriangle(const FrontMesh& mesh,int triangle)
{
    if(triangle < 0 || static_cast<std::size_t>(triangle) >= mesh.triangles.size())
        return {MeshingStatus::InvalidTopology,"front triangle index out of range"};
    if(!std::isfinite(mesh.triangles[triangle].radius))
        return {MeshingStatus::InvalidInput,"nonfinite front triangle radius"};
    return {};
}
/// 根据前沿队列的排序规则判断两个键是否等价。
bool sameKey(const FrontTriangleKey& a,const FrontTriangleKey& b)
{
    FrontTriangleKeyLess less; return !less(a,b) && !less(b,a);
}
}
/// 返回算法6初始活动半径阈值。
double frontalRadiusLimit() { return .5 * std::sqrt(2.) * 1.; }
/// 检查前沿三角形的邻片状态并返回首条活动边。
FrontActivity findActiveEdge(const FrontMesh& mesh,int triangle,double limit,int initialEdge)
{
    FrontActivity result; result.edge = initialEdge; result.status = validateTriangle(mesh,triangle);
    if(!result.status.complete()) return result;
    if(!std::isfinite(limit)) { result.status = {MeshingStatus::InvalidInput,"nonfinite front radius limit"}; return result; }
    const auto& t = mesh.triangles[triangle];
    if(t.deleted) return result;
    for(result.edge = 0; result.edge < 3; ++result.edge) {
        const int neighbor = t.neighbors[result.edge];
        if(neighbor == -1) { result.active = true; return result; }
        result.status = validateTriangle(mesh,neighbor);
        if(!result.status.complete()) return result;
        const double radius = mesh.triangles[neighbor].radius;
        if(radius < limit && radius > 0.) { result.active = true; return result; }
    }
    return result;
}
/// 建立前沿总集合及初始活动集合。
OperationStatus initializeFrontQueue(const FrontMesh& mesh,FrontQueueState& output,double limit,const RunControl& control)
{
    if(cancelled(control)) return {MeshingStatus::Cancelled,"front queue initialization cancelled"};
    if(!std::isfinite(limit)) return {MeshingStatus::InvalidInput,"nonfinite front radius limit"};
    FrontQueueState prepared; prepared.limit = limit; FrontTriangleKey previous; bool hasPrevious = false;
    for(int index : mesh.orderedTriangles) {
        if(cancelled(control)) return {MeshingStatus::Cancelled,"front queue initialization cancelled"};
        auto status = validateTriangle(mesh,index); if(!status.complete()) return status;
        const auto key = makeFrontTriangleKey(mesh.triangles[index],index);
        if(hasPrevious && !FrontTriangleKeyLess()(previous,key))
            return {MeshingStatus::InvalidTopology,"initial AllTris snapshot has invalid ordering or duplicate key"};
        prepared.allTriangles.insert(prepared.allTriangles.end(),key); previous = key; hasPrevious = true;
    }
    for(const auto& key : prepared.allTriangles) {
        if(cancelled(control)) return {MeshingStatus::Cancelled,"front queue initialization cancelled"};
        const auto activity = findActiveEdge(mesh,key.triangle,limit);
        if(!activity.status.complete()) return activity.status;
        if(activity.active) prepared.activeTriangles.insert(key);
        else if(key.radius < limit) break;
    }
    if(cancelled(control)) return {MeshingStatus::Cancelled,"front queue initialization cancelled"};
    output = std::move(prepared); return {};
}
/// 取出一个队首条目并检查是否需要继续插点。
FrontSelection popActiveTriangle(const FrontMesh& mesh,FrontQueueState& queue,const RunControl& control)
{
    FrontSelection result;
    if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front queue pop cancelled"}; return result; }
    if(queue.activeTriangles.empty()) return result;
    const auto position = queue.activeTriangles.begin(); const auto& key = *position;
    result.status = validateTriangle(mesh,key.triangle); if(!result.status.complete()) return result;
    const auto& t = mesh.triangles[key.triangle];
    if(!sameKey(key,makeFrontTriangleKey(t,key.triangle))) {
        result.status = {MeshingStatus::InvalidTopology,"queued triangle key changed without removal"}; return result;
    }
    result.triangle = key.triangle; result.deleted = t.deleted;
    if(!t.deleted) {
        const auto activity = findActiveEdge(mesh,key.triangle,queue.limit);
        if(!activity.status.complete()) { result.status = activity.status; return result; }
        result.activityEvaluated = true; result.active = activity.active; result.edge = activity.edge;
        result.requiresInsertion = activity.active && t.radius > queue.limit;
    }
    if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front queue pop cancelled"}; return result; }
    queue.activeTriangles.erase(position); return result;
}
/// 将新建三角形登记到前沿总集合并记录重复键。
FrontQueueUpdate registerFrontTriangles(const FrontMesh& mesh,const std::vector<int>& created,FrontQueueState& queue,const RunControl& control)
{
    FrontQueueUpdate result; FrontTriangleSet additions;
    for(int index : created) {
        if(cancelled(control)) return {{MeshingStatus::Cancelled,"front registration cancelled"},{},{}};
        const auto status = validateTriangle(mesh,index); if(!status.complete()) return {status,{},{}};
        const auto key = makeFrontTriangleKey(mesh.triangles[index],index);
        if(queue.allTriangles.find(key) != queue.allTriangles.end() || !additions.insert(key).second)
            result.rejectedDuplicates.push_back(index);
        else result.inserted.push_back(index);
    }
    if(cancelled(control)) return {{MeshingStatus::Cancelled,"front registration cancelled"},{},{}};
    queue.allTriangles.merge(additions); return result;
}
/// 根据插点后的邻域状态登记新的活动三角形。
FrontQueueUpdate updateActiveTriangles(const FrontMesh& mesh,const std::vector<int>& neighborhood,FrontQueueState& queue,const RunControl& control)
{
    FrontQueueUpdate result; FrontTriangleSet additions;
    for(int index : neighborhood) {
        if(cancelled(control)) return {{MeshingStatus::Cancelled,"front activity update cancelled"},{},{}};
        const auto activity = findActiveEdge(mesh,index,queue.limit);
        if(!activity.status.complete()) return {activity.status,{},{}};
        const auto& t = mesh.triangles[index];
        if(activity.active && t.radius > queue.limit) {
            const auto key = makeFrontTriangleKey(t,index);
            if(queue.activeTriangles.find(key) == queue.activeTriangles.end() && additions.insert(key).second)
                result.inserted.push_back(index);
        }
    }
    if(cancelled(control)) return {{MeshingStatus::Cancelled,"front activity update cancelled"},{},{}};
    queue.activeTriangles.merge(additions); return result;
}
/// 按失败原因更新目标三角形半径及队列状态。
FrontFailureUpdate markInsertionFailure(FrontMesh& mesh,FrontQueueState& queue,int triangle,FrontInsertionFailure reason,const RunControl& control)
{
    FrontFailureUpdate result;
    if(cancelled(control)) { result.status = {MeshingStatus::Cancelled,"front failure update cancelled"}; return result; }
    result.status = validateTriangle(mesh,triangle); if(!result.status.complete()) return result;
    if(reason != FrontInsertionFailure::ContainingTriangleNotFound && reason != FrontInsertionFailure::PointInsertionRejected) {
        result.status = {MeshingStatus::InvalidInput,"unknown front insertion failure"}; return result;
    }
    const auto key = makeFrontTriangleKey(mesh.triangles[triangle],triangle);
    const auto position = queue.allTriangles.find(key); const auto active = queue.activeTriangles.find(key);
    if(position == queue.allTriangles.end() || position->triangle != triangle ||
       (active != queue.activeTriangles.end() && active->triangle == triangle)) {
        result.status = {MeshingStatus::InvalidTopology,"failed triangle missing from AllTris or not popped from active set"}; return result;
    }
    queue.allTriangles.erase(position);
    mesh.triangles[triangle].radius = reason == FrontInsertionFailure::ContainingTriangleNotFound ? 0. : -1.;
    result.reinserted = queue.allTriangles.insert(makeFrontTriangleKey(mesh.triangles[triangle],triangle)).second;
    return result;
}
}
