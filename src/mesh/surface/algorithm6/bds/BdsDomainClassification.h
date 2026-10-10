#pragma once
/** @file BdsDomainClassification.h
 * @brief 普通/周期BDS域标记，保留各自种子规则/临时身份/LIFO；不恢复或清理。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
namespace mesher::gmshReference {
struct BdsDomainFill {
    int seed = -1;
    BdsClassification classification;
    std::vector<std::pair<int,bool>> visits; // pop顺序与实际是否从空分类展开，包含重复pop。
};
struct BdsDomainClassificationResult {
    OperationStatus status;
    std::vector<BdsDomainFill> fills;
    std::vector<int> clearedExteriorTriangles;
    std::size_t interiorTriangles = 0;
};
/** ordinary meshGenerator原域阶段：清所有面分类，第一负点面填外域(1,3)，
 * 第一分类双面边跨外域填内域(1,2)，清外域分类，再按负对点补一次内域种子。
 * recur_tag为pop时空分类才展开的LIFO，三条未分类双面边按e1/e2/e3入栈。
 * 原池/删除记录/重复访问/临时分类对象身份保持，不按CAD几何补填未知域。
 * Complete表示阶段结束，零interiorTriangles明确留给上层判定，不能伪称有效面。
 * 不恢复embedded、不设置双尺寸、不删除面/边/外围点或执行cleanup。
 * 控制/非法失败原子保留output，成功替换完整工作池，无CAD/Gmsh调用。
 */
BdsDomainClassificationResult classifyOrdinaryBdsDomain(const BdsMesh& input,BdsMesh& output,
    const RunControl& control = {});
/** 周期meshGeneratorPeriodic域阶段：清分类、跳过deleted负点种子面，
 * 外域临时(3,2)、首个分类双面边填内域(1,2)，清外域分类；
 * 不执行普通路径的负对点补填。LIFO/重复pop/边序沿用原recur_tag。
 * CLASS_E来源保持，不清理或求尺寸；控制/非法失败保留output。
 */
BdsDomainClassificationResult classifyPeriodicBdsDomain(const BdsMesh& input,BdsMesh& output,
    const RunControl& control = {});
}
