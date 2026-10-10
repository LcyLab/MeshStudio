#pragma once
/** @file FrontQueue.h
 * @brief 算法6活动集/首活动边/pop及失败半径转换；不计算候选或实施插点。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontTriangleOrdering.h"
#include <set>
namespace mesher::gmshReference {
using FrontTriangleSet = std::set<FrontTriangleKey,FrontTriangleKeyLess>;
struct FrontQueueState {
    double limit = 0.;
    FrontTriangleSet allTriangles,activeTriangles; // 运行期集合；键变化须先移除再插入。
};
struct FrontActivity {
    OperationStatus status;
    bool active = false;
    int edge = 0; // deleted保留调用者initialEdge；无活动边时为3。
};
struct FrontSelection {
    OperationStatus status;
    int triangle = -1,edge = -1;
    bool deleted = false,activityEvaluated = false,active = false,requiresInsertion = false;
};
enum class FrontInsertionFailure { ContainingTriangleNotFound,PointInsertionRejected };
struct FrontQueueUpdate {
    OperationStatus status;
    std::vector<int> inserted,rejectedDuplicates;
};
struct FrontFailureUpdate { OperationStatus status; bool reinserted = false; };
double frontalRadiusLimit(); // 固定算法6初始LIMIT_=0.5*sqrt(2.0)*1。
/// 检查前沿三角形的邻片状态并返回首条活动边。
FrontActivity findActiveEdge(const FrontMesh& mesh,int triangle,double limit,int initialEdge = 0);
/** 原AllTris遍历：活动片无条件加入（即使本片R<=limit），只有非活动且R<limit才break。
 * 输入orderedTriangles是初始化快照；运行期使用本结构allTriangles，不修改该快照。
 */
OperationStatus initializeFrontQueue(const FrontMesh& mesh,FrontQueueState& output,
    double limit = frontalRadiusLimit(),const RunControl& control = {});
/** 消耗一个set头，保留deleted/非活动/本片半径不足的跳过事件；不循环找下一合格片。 */
FrontSelection popActiveTriangle(const FrontMesh& mesh,FrontQueueState& queue,const RunControl& control = {});
/// 将新建三角形登记到前沿总集合并记录重复键。
FrontQueueUpdate registerFrontTriangles(const FrontMesh& mesh,const std::vector<int>& created,
    FrontQueueState& queue,const RunControl& control = {});
/** 对应成功插点后的new_cavity遍历，仅活动且本片R>limit时加入；不会移除已有陈旧项。 */
FrontQueueUpdate updateActiveTriangles(const FrontMesh& mesh,const std::vector<int>& neighborhood,
    FrontQueueState& queue,const RunControl& control = {});
/** 仅在目标已pop后调用：从AllTris移除、R置0/-1、再插入；不改deleted/邻接或活动邻片。
 * 不复刻空腔标记回滚；该职责由后续FrontCavity承担。
 */
FrontFailureUpdate markInsertionFailure(FrontMesh& mesh,FrontQueueState& queue,int triangle,
    FrontInsertionFailure reason,const RunControl& control = {});
}
