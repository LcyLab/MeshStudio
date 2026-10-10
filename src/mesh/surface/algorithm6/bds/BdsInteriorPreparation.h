#pragma once
/** @file BdsInteriorPreparation.h
 * @brief 普通/周期BDS域后删空域面、两次非稳定池清理、分类传播与外围点删除。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
namespace mesher::gmshReference {
struct BdsPoolRemapping {
    std::vector<int> triangles, edges; // 旧池索引->新池索引，已删项为-1。
};
struct BdsInteriorPreparationResult {
    OperationStatus status;
    std::vector<int> unclassifiedTriangles, orphanEdges, removedOuterPoints;
    BdsPoolRemapping firstCleanup, secondCleanup;
};
/** 普通尺寸初始化后、度量合法化前的原生顺序：删空分类面/cleanup，
 * 删除孤边或给无分类边赋临时CLASS_F，端点只降分类维数/空分类填充，
 * 第二次cleanup，最后删-1..-4点（编号上限保持，其它孤点保持）。
 * cleanup按std::partition保留编译器的非稳定排列，所有池引用同步映射，
 * 节点入射边/边邻面相对顺序保持；不求尺寸、不恢复边、不翻边。
 * 成功原子替换output；取消/预算/非法拓扑保留output。若外围点仍有活边
 * 显式InvalidTopology，避免复刻原版删除点后悬空指针。
 */
BdsInteriorPreparationResult prepareOrdinaryBdsInterior(const BdsMesh& input,
    BdsMesh& output,const RunControl& control = {});
/** 普通和周期源码一致的内域准备操作；不决定调用时机，周期尺寸由图表持有，
 * 普通在节点BGM后调用。保留CLASS_E/CLASS_F来源与池映射，失败输出保持。
 */
BdsInteriorPreparationResult prepareBdsInterior(const BdsMesh& input,
    BdsMesh& output,const RunControl& control = {});
}
