#pragma once
/** @file InitialTriangulation.h
 * @brief 两种原版初始剖分入口的扩盒/早退编排，不恢复约束或域。
 * @date 2026-10-07
 * @modified 2026-10-08
 */
#include "DivideAndConquerTriangulation.h"
#include "mesh/surface/algorithm6/chart/PeriodicFaceChart.h"
#include "WalkTriangulation.h"
namespace mesher::gmshReference {
struct OrdinaryInitialTriangulation {
    FaceChartRoute route = FaceChartRoute::General;
    WalkTriangulation mesh; // GFaceInitialMesh(recover=0)，含四外围点/所有半边。
    std::vector<std::array<int,3>> directTriangles; // Direct 早退的 BDS localId，不调用 Walk。
    Point2D outerLower,outerUpper;
};
/** 普通 GFaceInitialMesh 的 getNodeCopies 原次序及 *=1.1 矩形入口。
 * 不排序/扰动/调用 CAD；Skip/Direct 保留 meshGenerator 的早退。
 * General 保留 PolyMesh 池和面首边；recover=1 的约束/着色/翻边尚属后续模块。
 */
OperationStatus buildInitialTriangulation(const FaceChart& chart, OrdinaryInitialTriangulation& output,
    const RunControl& control = {});
struct InitialTriangulation {
    std::vector<BdsClassification> geometry; // 图表中未必被点引用的登记，传到BDS转换。
    FaceChartRoute route = FaceChartRoute::General;
    std::vector<PeriodicBoundaryVertex> vertices; // 原图表点后接 -1..-4；raw UV/尺寸不改变。
    DivideAndConquerTriangulation triangulation;
    std::vector<std::array<int,3>> triangles; // BDS localId，保留 DocRecord 次序；含外围三角形。
    Point2D outerLower,outerUpper;
    bool cubicBounds = false;
};
struct InitialTriangulationResult {
    OperationStatus status;
    std::size_t geometryEvaluations = 0;
};

/** 对应 meshGeneratorPeriodic 的外围四点→DocRecord→BDS 三角形阶段。
 * raw 盒来自 boundary，是否 makeCube 使用 CAD 参数跨度，原版 cube 边长为 raw 盒对角线。
 * makeCube/scale 的算术次序与 SBoundingBox3d 相同，随后 *=3.5，外围点无随机扰动。
 * 四次面求值，外围分类为 face/2、双尺寸初值 1e22，ID 顺序 -1,-2,-3,-4。
 * 不删除外围点/域外片，不恢复边，不按 raw UV 重新判三角形方向。
 * General 输入必须来自已完成图表，保留每个 DocRecord 位置与 BDS 身份的分别计数。
 * Skip/Direct 路线不剖分或调用 CAD；Direct 使用 BDS 编号序三点。
 * maximumNodes 同时约束 BDS 唯一点及 DocRecord 位置；失败原子保留 output。
 * 普通入口由独立 buildInitialTriangulation 调用 Walk，不使用旧 CDT。
 */
InitialTriangulationResult buildPeriodicInitialTriangulation(const PeriodicFaceChart& chart,
    const PeriodicFaceBoundaryInput& geometry, InitialTriangulation& output, const RunControl& control = {});
}
