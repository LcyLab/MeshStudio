#pragma once
/** @file FaceChart.h
 * @brief CAD 物理身份到面内 UV 的图表输入；不生成三角形或修改共享边。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/boundary/BoundaryRegistry.h"

namespace mesher::gmshReference {
struct FaceCurveUse {
    int entity = -1;
    bool seam = false;
};
struct FaceChartInput {
    int entity = -1;
    std::vector<FaceCurveUse> boundaryCurves; // 保留 GFace::edges 顺序和重复 use。
    std::vector<FaceCurveUse> embeddedCurves;
    std::vector<std::uint64_t> embeddedVertices; // 显式 0D 物理身份。
    std::function<Point2D(const BoundaryNode&)> nodeCoordinates;
    std::function<Point3D(Point2D)> surfacePoint;
};
enum class FaceChartRoute { General, SkipInsufficientNodes, DirectTriangle, PeriodicRequired };
struct FaceChartVertex {
    int localId = -1; // 普通 meshGenerator 的 BDS 编号，从 0 开始。
    std::uint64_t physicalId = 0;
    int dimension = -1, entity = -1;
    Point2D uv;
    Point3D surfacePoint; // 普通 BDS::add_point 在 UV 处求值，独立于原物理节点 XYZ。
};
struct FaceChartConstraint {
    int curveEntity = -1;
    int from = -1, to = -1;
    bool embedded = false;
};
struct InitialChartVertex {
    std::uint64_t physicalId = 0;
    Point2D uv;
};
struct FaceChart {
    int entity = -1;
    FaceChartRoute route = FaceChartRoute::General;
    std::vector<FaceChartVertex> vertices;
    std::vector<FaceChartConstraint> constraints;
    // 默认 GFaceInitialMesh 的 getNodeCopies unordered_map 枚举顺序。
    // 不能替换为 Hilbert/坐标排序；初始四外围节点由 InitialTriangulation 创建。
    std::vector<InitialChartVertex> initialVertices;
    std::vector<std::pair<int,std::uint64_t>> boundaryVersions;
};
struct FaceChartBuildResult {
    OperationStatus status;
    FaceChartRoute route = FaceChartRoute::General;
    std::size_t geometryEvaluations = 0;
};

/** meshGFace.cpp::meshGenerator 普通面输入和 getNodeCopies：
 * 边界按物理身份检查奇偶闭合；meshDegenerate 边跳过；embedded 不算外环。
 * 少于/等于 3 个节点保留原生早退路线，未调用 UV 或曲面查询。
 * General 路线生成稳定分类/UV/约束和默认初始插入顺序；不做质量/翻边。
 * 输入查询必须精确且本次运行不可变。成功原子替换 output；失败保留原值。
 * 边界 seam 返回 PeriodicRequired 和 NonConverged，交给独立周期路径。
 * embedded seam 的 getNodeCopies 多 UV 分支也尚未实现，显式返回同一路线。
 * 不能将未支持的多 UV 图表暗压成普通面。
 * maximumNodes 限制图表节点数；每次 UV/曲面查询计一次预算。
 */
FaceChartBuildResult buildFaceChart(const BoundaryMesh& boundary, const FaceChartInput& input,
    FaceChart& output, const RunControl& control = {});

struct PeriodicLoopVertex {
    std::uint64_t physicalId = 0;
    Point2D uv;
};
struct PeriodicCurveChart {
    int entity = -1;
    bool seam = false;
    // CAD 边原方向：begin / 活内点 / end，含退化边的 0D 端点。
    // 几何层使用 parBoundsOnFace 和 reparamOnFace(direction=1) 精确求值。
    std::vector<PeriodicLoopVertex> vertices;
    // seam 时 direction=-1；与 vertices 一一对应，物理身份不另复制。
    std::vector<Point2D> alternateCoordinates;
};
struct PeriodicBoundaryLoop {
    std::vector<PeriodicLoopVertex> vertices; // 已去掉最后一个闭合端点。
    int initialDirection = 0;
};

/** buildConsecutiveListOfVertices 的纯 UV 环拼接部分。
 * 输入保留 GEdgeLoop 边序；尝试首边正/反向，其余边选择严格唯一最近的
 * 正/反向及 seam 分支。连接和闭合都用 distance < tolerance；不平均 UV。
 * 首次续接失败才反向重试，闭合失败直接返回 NonConverged。
 * seamFirst 仅选择首边的另一 UV；容差六级重试归周期图表编排层。
 * 不生成 BDS 点/分类/尺寸，不处理跨环复用或 embedded，不调用 CAD。
 * 成功原子替换 output；非闭环、取消、预算和非法输入保留原值。
 */
OperationStatus assemblePeriodicBoundaryLoop(const std::vector<PeriodicCurveChart>& curves,
    double tolerance, bool seamFirst, PeriodicBoundaryLoop& output, const RunControl& control = {});
} // namespace mesher::gmshReference
