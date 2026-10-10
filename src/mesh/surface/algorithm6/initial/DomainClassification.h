#pragma once
/** @file DomainClassification.h
 * @brief 普通初始 PolyMesh 的原生 Color 两阶段域着色。
 * @date 2026-10-07
 * @modified 2026-10-08
 */
#include "WalkTriangulation.h"
namespace mesher::gmshReference {
struct DomainClassificationResult {
    OperationStatus status;
    int interiorSeedHalfEdge = -1, finalOppositeHalfEdge = -1;
    std::vector<int> exteriorVisits, interiorVisits; // 原生 LIFO pop 顺序，保留重复访问。
};
/** 先从外围点0的入射边着外域-2，再从原生返回的other_side着faceEntity。
 * 输入所有面须为未知域-1；残余-1按原生保留，不额外填充孔或断开区域。
 * CAD半边标签阻断泛洪；pop时着色，保留重复入栈/访问和最后other_side覆盖。
 * 缺失内域种子（原版空指针）、取消/预算/非法输入原子保留output。
 * 不删除外围点/孔内片，不判方向或执行质量优化。
 */
DomainClassificationResult classifyFaceDomain(const WalkTriangulation& input, int faceEntity,
    WalkTriangulation& output, const RunControl& control = {});
}
