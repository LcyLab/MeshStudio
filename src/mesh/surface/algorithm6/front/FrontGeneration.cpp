/** @file FrontGeneration.cpp
 * @brief 动态前沿编排。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontGeneration.h"
#include <exception>
namespace mesher::gmshReference {
/// 按活动队列逐轮放置候选并插点，返回结束状态和统计。
FrontGenerationResult generateFrontMesh(FrontMesh& front,FrontQueueState& queue,FaceMesh& mesh,NodeSizeState& nodes,
    const FrontGenerationSource& source,const FrontGenerationOptions& options,const RunControl& control,const FrontGenerationObserver& observer)
{
    FrontGenerationResult result;
    auto require = [](OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; };
    auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"front generation cancelled"}}; };
    auto query = [&] {
        check();
        if(result.geometryEvaluations >= control.maximumGeometryEvaluations) throw OperationFailure{{MeshingStatus::BudgetExceeded,"front generation query budget exceeded"}};
        ++result.geometryEvaluations;
    };
    FrontTriangleSet savedHead; bool restoreHead = false;
    auto restore = [&] { if(restoreHead) { queue.activeTriangles.insert(savedHead.extract(savedHead.begin())); restoreHead = false; } };
    try {
        check();
        if(!source.parameterBounds || !source.metric || !source.surface || !source.surfacePoint || !source.backgroundSize)
            throw OperationFailure{{MeshingStatus::InvalidInput,"front generation geometry source missing"}};
        if(queue.limit != frontalRadiusLimit()) throw OperationFailure{{MeshingStatus::InvalidInput,"algorithm6 requires its fixed front radius limit"}};
        query(); std::array<Point2D,2> bounds;
        try { bounds = source.parameterBounds(); }
        catch(const OperationFailure&) { throw; }
        catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
        catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"front range source failed"}}; }
        require(computeParametricFarPoint(bounds[0],bounds[1],result.farPoint));
        // 所有源入口共享总计数；内部阶段自己的局部计数不重置本次运行预算。
        RunControl stageControl = control; stageControl.maximumGeometryEvaluations = std::numeric_limits<std::size_t>::max();
        const FrontMetricSource metric = [&](Point2D uv) { query(); return source.metric(uv); };
        const SurfaceFunction surface = [&](Point2D uv) { query(); return source.surface(uv); };
        FrontInsertionSource insertion;
        insertion.surfacePoint = [&](Point2D uv) { query(); return source.surfacePoint(uv); };
        insertion.backgroundSize = [&](const MeshSizeQuery& q) { query(); return source.backgroundSize(q); };
        while(!queue.activeTriangles.empty()) {
            // 每轮只消费一个活动片；接受新点后局部登记新片，下一轮继续使用更新的队列。
            check();
            if(result.pops >= options.maximumPops) throw OperationFailure{{MeshingStatus::BudgetExceeded,"front generation pop budget exceeded"}};
            // 保存一个预分配set节点；失败恢复活动头无需分配，不复制全队列。
            savedHead.clear(); savedHead.insert(*queue.activeTriangles.begin()); restoreHead = true;
            FrontGenerationEvent event; event.selection = popActiveTriangle(front,queue,control); require(event.selection.status);
            if(event.selection.requiresInsertion) {
                // 先构造度量候选，再用 CAD 圆/曲面求交修正；此时尚未修改网格连接。
                FrontPointCandidate candidate;
                const auto& triangle = front.triangles[event.selection.triangle];
                require(computeFrontPoint(triangle,event.selection.edge,nodes,options.extendBoundarySize,metric,candidate,stageControl).status);
                require(correctFrontPointOnSurface(triangle,event.selection.edge,mesh,candidate,surface,candidate,stageControl).status);
                event.candidate = candidate;
                if(options.trueBoundary) {
                    event.domainTested = true; event.domain = testParametricDomain(*options.trueBoundary,candidate.point,result.farPoint,control); require(event.domain.status);
                }
                if(!event.domainTested || event.domain.inside) {
                    // 只有通过域门控才进入插点事务；定位、空腔和连接提交由插入模块负责。
                    event.insertionAttempted = true;
                    event.insertion = insertFrontPoint(front,queue,mesh,nodes,event.selection.triangle,candidate.point,candidate.metric,insertion,{options.extendBoundarySize,true,false},stageControl);
                    require(event.insertion.status);
                }
            }
            // 原正常拒绝/门控跳过也是完成轮次；此前成功轮次不受后续取消影响。
            restoreHead = false; ++result.pops;
            if(event.candidate) ++result.candidates;
            if(event.domainTested && !event.domain.inside) ++result.domainRejected;
            if(event.insertionAttempted) { ++result.insertionAttempts; if(event.insertion.inserted) ++result.inserted; }
            if(observer) observer(event);
        }
    // 取消或异常只恢复尚未完成轮次的活动头，保留此前已完成轮次的合法插点。
    } catch(const OperationFailure& error) { restore(); result.status = error.status; }
      catch(const std::exception& error) { restore(); result.status = {MeshingStatus::NonConverged,error.what()}; }
      catch(...) { restore(); result.status = {MeshingStatus::NonConverged,"unknown front generation failure"}; }
    return result;
}
}
