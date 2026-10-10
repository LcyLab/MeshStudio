#pragma once
/** @file BdsMeshConversion.h
 * @brief 普通 PolyMesh/周期分治初始网格到 BDS；不恢复约束或裁域。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
namespace mesher::gmshReference {
struct BdsConversionResult {
    OperationStatus status;
    std::size_t geometryEvaluations = 0;
    std::vector<int> skippedTriangles; // 普通 aaa 查找缺失时按原版跳片，显式记录。
};

/** 普通 meshGenerator 的 PolyMesh→BDS。
 * chart.vertices 提供先前建立的独立 BDS UV/XYZ/分类，不能拿 PolyMesh UV 覆盖。
 * 输入为 recover=1/Color/后置UV已完成的池；四外围点按 -1..-4 在曲面求值。
 * 全部 PolyMesh 面按池/可变首边序转换，不传域/半边标签，不删孔内或外域片。
 * 未找到物理身份的面按原版跳过并返回索引；不伪称完全转换。
 * General 专用；Skip/Direct 由上层早退，PeriodicRequired 归周期入口。
 * 每个外围点计一次几何查询；取消/预算/非法查询失败原子保留 output。
 */
BdsConversionResult convertOrdinaryInitialMeshToBds(const FaceChart& chart,
    const WalkTriangulation& initial, const std::function<Point3D(Point2D)>& surfacePoint,
    BdsMesh& output, const RunControl& control = {});

/** 周期入口将 raw BDS 点和已完成 DocRecord 三角形按原顺序转换。
 * 保留尺寸/分类/counterpart/编号空洞和负外围点，不重算 raw UV 或 CAD XYZ。
 * General 专用；早退由编排层处理；不依据扰动后 UV 重排三角形。
 * 成功原子替换 output，不调用几何查询。
 */
BdsConversionResult convertPeriodicInitialMeshToBds(const InitialTriangulation& initial,
    BdsMesh& output, const RunControl& control = {});
}
