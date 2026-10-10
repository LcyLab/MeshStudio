#pragma once
/** @file FaceMeshConversion.h
 * @brief recoverMap身份视图初始化与BDS到前沿面连接转换；不重建缝或生成前沿。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
#include "mesh/surface/algorithm6/data/FaceMeshTypes.h"
namespace mesher::gmshReference {
/** 根据BDS物理身份绑定原MVertex元数据，只选recoverMap引用的原节点。
 * 原节点id=physicalId；临时/新节点从所选原ID上限后分配本阶段稳定身份。
 * 不按XYZ合并，不用BDS求值XYZ替换原物理XYZ，不设置UV等价映射。
 */
OperationStatus initializeFaceMeshVertices(const BdsMesh& bds,int faceEntity,
    const std::vector<FaceVertex>& originals,FaceMesh& output,const RunControl& control = {});
/// 核对 BDS 节点与转换后的面网格节点是否一致。
OperationStatus validateFaceMeshVertices(const BdsMesh& bds,const FaceMesh& mesh,const RunControl& control = {});
struct FaceMeshConversionResult {
    OperationStatus status;
    std::vector<int> skippedDeletedTriangles, collapsedTriangles, createdLocalNodes;
};
/** BDS2GMSH三角分支：按活面池及getNodes顺序复用recoverMap或创建带UV的
 * FaceCreated节点，原点XYZ/UV直接保存，不查CAD；创建完成后仅按节点身份
 * 相同过滤三角形，不按坐标、物理ID、面积或形状过滤。保留原追加顺序。
 * 此数据合同是一阶三角形路线；不接受四边形池。不求尺寸/翻边/去掉临时副本。
 * 成功替换output，取消/预算/非法数据失败保持output。
 */
FaceMeshConversionResult convertBdsToFaceMesh(const BdsMesh& bds,const FaceMesh& input,
    FaceMesh& output,const RunControl& control = {});
}
