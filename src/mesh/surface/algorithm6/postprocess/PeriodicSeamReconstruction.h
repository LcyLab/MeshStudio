#pragma once
/** @file PeriodicSeamReconstruction.h
 * @brief 周期前沿前的MVertex临时副本/UV/等价映射；不改BDS或生成三角形。
 * @date 2026-10-08
 */
#include "FaceMeshConversion.h"
namespace mesher::gmshReference {
struct PeriodicSeamReconstructionResult {
    OperationStatus status;
    std::vector<int> duplicatedLocalNodes;
};
/** 按recoverMap局部ID序：首次原身份记录BDS UV，重复0D/1D复制原实体、
 * 物理XYZ、曲线参数/lc，记录fake->original与该副本UV，并改局部映射。
 * 即使节点没被当前活三角形引用，也按原recoverMap规则处理。
 * 重复dim>=2保留原身份/第一次UV，记录unreconstructedSeams而不假造副本。
 * BDS/config/双尺寸不改，临时副本保留至前沿后；控制/非法失败保持output。
 */
PeriodicSeamReconstructionResult reconstructPeriodicSeamVertices(const BdsMesh& bds,
    const FaceMesh& input,FaceMesh& output,const RunControl& control = {});
}
