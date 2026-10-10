#pragma once
/** @file InitialUvLegalization.h
 * @brief 普通 PolyMesh 恢复/着色后的原生 UV 翻边；与 BDS 度量合法化分开。
 * @date 2026-10-07
 * @modified 2026-10-08
 */
#include "WalkTriangulation.h"
namespace mesher::gmshReference {
struct InitialUvLegalizationResult {
    OperationStatus status;
    std::vector<std::vector<int>> flippedHalfEdges; // 每次整池扫描，保留最后一次空扫描。
    bool passLimitReached = false; // 原生100次后停止；不是RunControl预算或收敛。
};
/** 对应 GFaceInitialMesh(recover=1) 的最多100次半边池序扫描。
 * 仅两面domain==faceEntity、raw incircle<0且原生严格凸性相交时翻边。
 * 不额外锁CAD标签；标签仍留在池对象，后续BDS恢复属于另一阶段。
 * 原生达到次数上限仍发布结果，同时显式标记；不宣称其收敛。
 * 取消/预算/非法输入原子保留output，不调用CAD或BDS度量规则。
 */
InitialUvLegalizationResult legalizeInitialUvMesh(const WalkTriangulation& input, int faceEntity,
    WalkTriangulation& output, const RunControl& control = {});
}
