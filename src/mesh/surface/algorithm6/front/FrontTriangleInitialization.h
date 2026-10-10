#pragma once
/** @file FrontTriangleInitialization.h
 * @brief MTri3初始尺寸/R/h、AllTris成员及邻接；不选择活动边或插点。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/FrontMeshTypes.h"
#include "NodeSizeInitialization.h"
namespace mesher::gmshReference {
struct FrontTriangleInitializationResult {
    OperationStatus status;
    std::vector<int> rejectedDuplicateTriangles;
};
/** buildMeshGenerationDataStructures三角部分：字面0.3333333333平均双尺寸，
 * 按已解析Extend1dMeshIn2dSurfaces选择min(mesh,BGM)或BGM，再构造物理R/h。
 * 以MFaceLessThan的排序节点ID决胜同半径，保留等价键set拒绝及原创建序。
 * 邻接只处理AllTris成员，按原边编号/非稳定std::sort扫描相邻相同边成对连接。
 * 不按物理等价或XYZ合并、不应用质量门槛、不构建活动队列/候选或修改输入。
 * 当前合同为固定一阶Algorithm6、radiusNorm=2、无boundary layer。
 * 非有限尺寸/归一半径显式失败；保留circumCenterXYZ有限退化哨兵行为。
 * 取消/预算/非法输入或几何失败不发布output。
 */
FrontTriangleInitializationResult initializeFrontTriangles(const FaceMesh& mesh,
    const NodeSizeState& nodes,bool extendBoundarySize,FrontMesh& output,const RunControl& control = {});
}
