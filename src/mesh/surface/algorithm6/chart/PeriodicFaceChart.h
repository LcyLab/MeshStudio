#pragma once
/** @file PeriodicFaceChart.h
 * @brief 周期面边界/嵌入图表和 BDS 初始节点；不剖分或组装物理网格。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "FaceChart.h"
#include "mesh/surface/algorithm6/data/BdsTypes.h"
#include <cstdint>

namespace mesher::gmshReference {
// 固定Windows CRT算术的运行私有流。全模型只初始化一次，面/恢复重试继续消费；
// 单面独立调用拥有自己的流。仅初始剖分坐标扰动，原CAD UV不改变。
class PeriodicInitialPerturbation {
public:
    /// 推进本次运行的随机状态并对周期图表 UV 作微小扰动。
    Point2D perturb(Point2D,double parameterSpan,double factor);
private:
    std::uint32_t state=1;
};
struct PeriodicFaceBoundaryInput {
    int entity = -1;
    Point2D parameterLower, parameterUpper;
    std::vector<std::vector<PeriodicCurveChart>> edgeLoops; // 保留 GFace::edgeLoops 及环内边序。
    double modelCharacteristicLength = 1.;
    std::function<Point3D(Point2D)> surfacePoint;
};
struct PeriodicBoundaryVertex {
    FaceChartVertex vertex;
    NodeSize sizes; // 新 BDS 点的 lc 和 lcBGM 均为分类实体处 BGM_MeshSize。
    int periodicCounterpart = -1; // BDS ID；三个以上复制点按原版覆盖前一对应。
};
struct PeriodicLoopSelection {
    double tolerance = 0.;
    bool seamFirst = false;
    int initialDirection = 0;
};
struct PeriodicBoundaryChart {
    int entity = -1;
    FaceChartRoute route = FaceChartRoute::General;
    std::vector<PeriodicBoundaryVertex> vertices; // 按 BDS ID 排列；ID 可有空洞。
    std::vector<std::vector<int>> loops; // 每个环位置引用 BDS ID，允许跨环复用。
    std::vector<PeriodicLoopSelection> selections;
    std::size_t loopPositionCount = 0; // 原生 nbPointsTotal，计复用位置而非唯一点数。
    int maximumLocalId = 0; // BDS_Mesh::MAXPOINTNUMBER 的原生初值。
    bool hasBounds = false;
    Point2D lower, upper; // 原始 UV 包围盒，尚未 makeCube 或 *=3.5。
    std::size_t invalidSizeFallbacks = 0;
};

/** meshGeneratorPeriodic 的边界环/BDS 初始化及早退阶段。
 * 每个环按原生 LC2D 六档容差先普通首缝再 alternate，失败不创建该环点。
 * 仅在此前完成环中复用同物理 0D 且 UV 距离 <tol 的节点；不复用当前环或 1D。
 * 每个环位置递增编号计数，即使复用；不压缩编号，不按 XYZ 合并身份。
 * 新点 surface XYZ 在 UV 求值，尺寸在原物理 XYZ 的分类实体和参数求值。
 * 完成全部环后按 BDS ID 更新 counterpart；按环位置数选择 <3/==3 早退路线。
 * 本函数不处理 trueBoundary/embedded/随机扰动/初始剖分/等价点恢复。
 * 输入为本次不可变的精确边 pcurve；仍需单独验证自己的 CAD 适配器。
 * maximumNodes 限制全部环位置数（含复用），每次新点 surface/BGM 各计一次查询。
 * 仅成功原子替换 output；几何/预算/取消/不闭合失败保留原值。
 */
FaceChartBuildResult buildPeriodicBoundaryChart(const BoundaryMesh& boundary,
    const PeriodicFaceBoundaryInput& input, const SurfaceMeshingOptions& options,
    const GeometrySizeSource& sizes, PeriodicBoundaryChart& output, const RunControl& control = {});

struct TrueBoundaryCurveInput {
    int entity = -1;
    double first = 0., last = 1.; // CAD parBoundsOnFace，不来自 1D 网格节点。
    bool seam = false;
    std::function<Point2D(double,int)> onFace; // 精确 reparamOnFace，分支为 0/1。
};
struct TrueBoundarySegment {
    int curveEntity = -1, branch = 0;
    Point2D from, to;
};

/** trueBoundary 的固定参数采样；不依赖实际 1D 网格，也不自适应细化。
 * 输入由拓扑适配器提供去重 CAD 边及明确遍历顺序；不在核心按 tag 排序。
 * 原生顺序为 std::set<GEdge*> 指针序，不能与 GEdgeLoop 边序混淆。
 * 每条边/分支固定 300 个点，参数为 first+(last-first)*(k/299)。
 * 非 seam 也用分支 0；seam 先 0 后 1，不能替换为连续环的 +1/-1。
 * 包括零参数跨度和退化边，输出每分支 299 对相邻 UV，不做质量判据。
 * 成功原子替换 output；取消、预算或 CAD 失败保留原值。
 * maximumNodes 限制全部采样点数（每分支 300），每次 pcurve 查询计一次。
 * debug 文件导出不属于本计算函数，由观察层另行负责。
 */
BoundaryOperationResult sampleTrueBoundary(const std::vector<TrueBoundaryCurveInput>& orderedCurves,
    std::vector<TrueBoundarySegment>& output, const RunControl& control = {});

struct PeriodicEmbeddedCurveInput {
    int entity = -1;
    std::vector<std::pair<std::uint64_t,std::uint64_t>> lines; // 原生 GEdge::lines 及每线端点顺序。
    double first = 0., last = 1.; // 精确 parBoundsOnFace。
    std::function<Point3D(double)> point;
    std::function<Point2D(double)> onFace; // reparamOnFace(dir=1)，仅多副本选择时查询。
};
struct PeriodicEmbeddedInput {
    std::vector<std::uint64_t> vertices; // 原生 getEmbeddedVertices 顺序；仅 0D。
    std::vector<PeriodicEmbeddedCurveInput> curves;
    double modelCharacteristicLength = 1.;
    std::function<Point2D(Point3D)> closestSurfaceCoordinates; // 原生 closestPoint，种子固定 (0,0)。
    std::function<Point3D(Point2D)> surfacePoint;
    // 调用方的 BDS 副本指针序，以 localId 表达；非空时须为全部边界节点的排列。
    // 空时使用本次 boundary.vertices 对象的指针序，不擅自按 UV/距离排序。
    std::vector<int> boundaryCopyOrder;
    // 调用方提供本次运行的随机扰动流；在每个新嵌入点的尺寸查询之后立即调用。
    // 叶阶段允许空回调以记录 raw UV；完整 buildPeriodicFaceChart 的 General 路线必须提供。
    // 即使扰动系数为零，仍需保留原生随机数调用；不计作 CAD 查询。
    std::function<Point2D(Point2D)> perturbInitialPoint;
};
struct PeriodicInitialPoint {
    int localId = -1;
    Point2D coordinates; // DocRecord 的扰动坐标，不覆盖 BDS raw UV。
};
struct PeriodicEmbeddedSelectionFailure {
    int curveEntity = -1;
    std::uint64_t physicalId = 0;
};
struct PeriodicEmbeddedChart {
    std::vector<BdsClassification> geometry; // 每条嵌入边均登记，全部端点复用时也保留。
    std::vector<PeriodicBoundaryVertex> vertices; // 仅新增节点；BDS 分类 tag 为负 CAD 点/边 tag。
    std::vector<FaceChartConstraint> constraints;
    std::vector<PeriodicInitialPoint> initialPoints; // CAD 点先、按嵌入边首次访问的新点后。
    std::vector<PeriodicEmbeddedSelectionFailure> seamSelectionFailures;
    std::size_t expectedInitialPointCount = 0; // 原生预分配的嵌入点数，和实际新增数分别保留。
    bool initialPointCountMismatch = false; // 原生 seam 选择失败可使实际数量超过 DocRecord 预分配。
    int maximumLocalId = 0;
    std::size_t invalidSizeFallbacks = 0;
};

/** meshGeneratorPeriodic 的嵌入 BDS/约束阶段；不创建外围盒或三角形。
 * 边界 inverse map 在加入嵌入点前冻结；嵌入 CAD 点始终新建，边端点可再次新建。
 * 多边界副本按原生端点物理距离选择参数（等距取上限），再按指针序选 UV 距离 <1e-10 的首副本。
 * 找不到副本时记录原生错误，再保留其首次投影/后续 facile 空覆盖行为；不擅自修约束。
 * 新嵌入边点的 BDS 分类属于首次创建它的边，尺寸仍按原物理节点分类查询。
 * maximumNodes 计已有边界唯一点加本次分配；失败原子保留 output。
 * expectedInitialPointCount 与实际数不符显式标记，初始剖分不得静默修补原生预分配错误。
 * 跳过/直接三角形路线不处理嵌入点；完整编排和边界 DocRecord 点由 buildPeriodicFaceChart 接齐。
 */
BoundaryOperationResult appendPeriodicEmbeddedChart(const BoundaryMesh& physical,
    const PeriodicBoundaryChart& boundary, const PeriodicEmbeddedInput& input,
    const SurfaceMeshingOptions& options, const GeometrySizeSource& sizes,
    PeriodicEmbeddedChart& output, const RunControl& control = {});

struct PeriodicFaceChartInput {
    std::vector<TrueBoundaryCurveInput> trueBoundaryCurves;
    PeriodicFaceBoundaryInput boundary;
    PeriodicEmbeddedInput embedded; // 其 modelLength/surfacePoint 统一使用 boundary 的同一面查询。
};
struct PeriodicFaceChart {
    std::vector<TrueBoundarySegment> trueBoundary;
    PeriodicBoundaryChart boundary;
    PeriodicEmbeddedChart embedded;
    std::vector<PeriodicInitialPoint> initialPoints; // 嵌入点前缀，然后逐环所有位置，保留重复引用。
    std::size_t expectedInitialPointCount = 0;
};

/** 顺序对应 meshGeneratorPeriodic 的 trueBoundary → 边界 BDS/早退 → 嵌入 → DocRecord 点序。
 * 各叶模块保留职责；几何查询预算跨阶段累计，取消/失败不发布半图表。
 * DocRecord 扰动由调用方的运行上下文提供，按原生调用时机保留，raw UV 不改变。
 * General 路线必须显式提供回调；零扰动仍消耗原生随机流。边界早退路线不要求回调。
 * 外围四点、扩盒、divide/conquer、边恢复/域标记均属于 InitialTriangulation 等后续模块。
 * 原生 seam 失败导致预分配数量不符时显式拒绝，避免复刻越界写；不补点或忽略约束。
 */
FaceChartBuildResult buildPeriodicFaceChart(const BoundaryMesh& physical,
    const PeriodicFaceChartInput& input, const SurfaceMeshingOptions& options,
    const GeometrySizeSource& sizes, PeriodicFaceChart& output, const RunControl& control = {});
} // namespace mesher::gmshReference
